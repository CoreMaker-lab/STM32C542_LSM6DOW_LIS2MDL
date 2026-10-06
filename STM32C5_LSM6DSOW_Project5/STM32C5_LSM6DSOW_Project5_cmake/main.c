/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  *                  Calls target system initialization then loop in main.
  ******************************************************************************
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions prototype -----------------------------------------------*/
#include "mx_usart1.h"
#include <stdio.h>
#include <string.h>

#include "lsm6dso_reg.h"

int _write(int file, char *ptr, int len)
{
    hal_uart_handle_t *huart1 = mx_usart1_uart_gethandle();
    if (huart1 != NULL) {
        HAL_UART_Transmit(huart1, ptr, len, 1000);
    }
    return len;
}

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
/*
 * Wake-up threshold:
 * WU_INACT_THS_W = 0
 * 1 LSB = 7.8125 mg
 * 2 = 15.625 mg
 * 4 = 31.25  mg
 * 8 = 62.5   mg
 */
#define    WAKEUP_THRESHOLD     4
/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 */
static int32_t platform_write(void *handle,uint8_t reg,const uint8_t *bufp,uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg,uint8_t *bufp,uint16_t len);
static void platform_delay(uint32_t ms);
static stmdev_ctx_t dev_ctx;
static volatile uint8_t thread_wake = 0;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  if (HAL_I2C_MASTER_MemWrite((hal_i2c_handle_t *)handle, LSM6DSO_I2C_ADD_L, reg,
                              HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  /* NOTE: the 0x80 read bit is only required on the SPI interface.
   * On I2C the register address must be sent as-is. */
  if (HAL_I2C_MASTER_MemRead((hal_i2c_handle_t *)handle, LSM6DSO_I2C_ADD_L, reg,
                             HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}


/**
 * @brief  EXTI trigger callback
 */
void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti, hal_exti_trigger_t trigger) {
    if (HAL_EXTI_GetInstance(hexti) == HAL_EXTI_GPIO_0)
    {
        thread_wake = 1;
    }
}

/**
  * brief:  The application entry point.
  * retval: none but we specify int to comply with C99 standard
  */
int main(void)
{
  /** System Init: this code placed in targets folder initializes your system.
    * It calls the initialization (and sets the initial configuration) of the peripherals.
    * You can use STM32CubeMX to generate and call this code or not in this project.
    * It also contains the HAL initialization and the initial clock configuration.
    */
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }
  else
  {
    /*
      * You can start your application code here
      */

      printf("HELLO\n");
      HAL_GPIO_WritePin(CS1_PORT, CS1_PIN, HAL_GPIO_PIN_SET);
      HAL_GPIO_WritePin(SA0_PORT, SA0_PIN, HAL_GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CS2_PORT, CS2_PIN, HAL_GPIO_PIN_SET);


      lsm6dso_pin_int1_route_t pin_int = {0};
      uint8_t rst = 0;

      /* Initialize mems driver interface */
      dev_ctx.write_reg = platform_write;
      dev_ctx.read_reg = platform_read;
      dev_ctx.mdelay = platform_delay;
      dev_ctx.handle = mx_i2c1_i2c_gethandle();


      /* Wait sensor boot time */
      platform_delay(BOOT_TIME);


      /* Check device ID */
      lsm6dso_device_id_get(&dev_ctx, &whoamI);

      printf("LSM6DSO_ID=0x%x,id=0x%x\n",
             LSM6DSO_ID, whoamI);

      if (whoamI != LSM6DSO_ID)
        while (1);

      /* Perform device power-on-reset and wait for completion */
      lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
      do {
        lsm6dso_reset_get(&dev_ctx, &rst);
      } while (rst);

      /* Disable I3C interface (I2C bus) */
      lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);


      /* Enable Block Data Update */
      lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	  /*
       * Set Low-G accelerometer.
       *
       * ODR = 417 Hz
       * FS  = +/-2 g
       */
      lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_417Hz);
      lsm6dso_xl_power_mode_set(&dev_ctx, LSM6DSO_HIGH_PERFORMANCE_MD);
      lsm6dso_xl_full_scale_set(&dev_ctx,LSM6DSO_2g);


      /*
       * INT1/INT2:
       * open-drain + active low
       */
      lsm6dso_pin_mode_set(&dev_ctx, LSM6DSO_OPEN_DRAIN);
      lsm6dso_pin_polarity_set(&dev_ctx, LSM6DSO_ACTIVE_LOW);

      /*
       * Enable basic interrupts (wake-up) and latched mode.
       * Without this the event is detected but never driven on INT1.
       */
      lsm6dso_int_notification_set(&dev_ctx, LSM6DSO_ALL_INT_LATCHED);

      /*
       * Enable XL Filter fast settling
       */
      lsm6dso_xl_fast_settling_set(&dev_ctx, PROPERTY_ENABLE);
        
      /*
       * Mask interrupt trigger during
       * accelerometer filter settling.
       */
      lsm6dso_filter_settling_mask_set(&dev_ctx,PROPERTY_ENABLE);

      /*
       * Use High-Pass filter for
       * Wake-up detection.
       */
      lsm6dso_xl_hp_path_internal_set(&dev_ctx,LSM6DSO_USE_SLOPE);

      /*
       * Wake-up threshold configuration.
       *
       * wake_ths_w = 1 (LSM6DSO_LSb_FS_DIV_256):
       * 1 LSB = FS_XL / 256 = 2 g / 256 = 7.8125 mg
       *
       * threshold = 4:
       * 4 x 7.8125 mg = 31.25 mg
       */
      lsm6dso_wkup_ths_weight_set(&dev_ctx, LSM6DSO_LSb_FS_DIV_256);
      lsm6dso_wkup_threshold_set(&dev_ctx, WAKEUP_THRESHOLD);
      lsm6dso_wkup_dur_set(&dev_ctx, 0);

      /*
       * Route Wake-up event to INT1
       */
      pin_int.wake_up = PROPERTY_ENABLE;
      lsm6dso_pin_int1_route_set(&dev_ctx,pin_int);


      printf("Wake-up Motion Detection start...\r\n");
      printf("Threshold = %d x 7.8125 mg\r\n",WAKEUP_THRESHOLD);

      while (1) {
        if (thread_wake)
        {
          lsm6dso_all_sources_t status = {0};
          thread_wake = 0;

          /* Read status so the latched interrupt is cleared
           * and INT1 can be asserted again on the next event. */
          lsm6dso_all_sources_get(&dev_ctx, &status);

          printf("Motion detected\r\n");
        }
    }
  }
} /* end main */

