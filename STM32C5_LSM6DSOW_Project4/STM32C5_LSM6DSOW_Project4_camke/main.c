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
/* Private functions prototype -----------------------------------------------*/

#include "mx_usart1.h"
#include <stdio.h>
#include <string.h>

#include "lsm6dso_reg.h"

int _write(int file, char *ptr, int len)
{
    hal_uart_handle_t *huart1 = mx_usart1_uart_gethandle();

    if (huart1 != NULL)
    {
        HAL_UART_Transmit(huart1, ptr, len, 1000);
    }

    return len;
}

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
/* Extern variables ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
static stmdev_ctx_t dev_ctx;
static volatile uint8_t thread_wake = 0;
/* STM32 HAL expects the 8-bit I2C address.
 * SA0 = 0 -> 0xD5, SA0 = 1 -> 0xD7 (see LSM6DSO_I2C_ADD_L/H). */
#define LSM6DSO_I2C_ADD  LSM6DSO_I2C_ADD_L

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  if (HAL_I2C_MASTER_MemWrite((hal_i2c_handle_t *)handle, LSM6DSO_I2C_ADD, reg,
                              HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  /* NOTE: the 0x80 read bit is only required on the SPI interface.
   * On I2C the register address must be sent as-is. */
  if (HAL_I2C_MASTER_MemRead((hal_i2c_handle_t *)handle, LSM6DSO_I2C_ADD, reg,
                             HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

/* EXTI line (INT1 on PB0) trigger callback: wake up the main loop. */
void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti, hal_exti_trigger_t trigger)
{
  (void)hexti;
  (void)trigger;
  thread_wake = 1;
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

	  /*
	   * INT1 (PB0) is open-drain + active-low from the sensor.
	   * Enable the MCU internal pull-up so the line idles high and the
	   * step-detector pulse generates a clean falling edge on EXTI0.
	   */
	  hal_gpio_config_t int1_cfg = {0};
	  int1_cfg.mode  = HAL_GPIO_MODE_INPUT;
	  int1_cfg.pull  = HAL_GPIO_PULL_UP;
	  int1_cfg.speed = HAL_GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(HAL_GPIOB, HAL_GPIO_PIN_0, &int1_cfg);

      lsm6dso_emb_sens_t emb_sens = {0};
      lsm6dso_pin_int1_route_t pin_int = {0};
      uint8_t rst = 0;
      uint8_t debounce = 3;
      uint16_t step_count = 0;

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

      /* Restore default configuration (software reset) */
      lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
      do {
        lsm6dso_reset_get(&dev_ctx, &rst);
      } while (rst);

      /* Disable I3C interface (I2C bus) */
      lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);

      /* Enable Block Data Update */
      lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

      /*
       * Pedometer works internally at 26 Hz.
       * Accelerometer ODR must be set to 26 Hz.
       */
      lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_26Hz);
      lsm6dso_xl_power_mode_set(&dev_ctx, LSM6DSO_HIGH_PERFORMANCE_MD);

      /* Set accelerometer full scale */
      lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);

      /* INT1/INT2: open-drain + active low */
      lsm6dso_pin_mode_set(&dev_ctx, LSM6DSO_OPEN_DRAIN);
      lsm6dso_pin_polarity_set(&dev_ctx, LSM6DSO_ACTIVE_LOW);

      /* Enable pedometer and step counter (advanced false step rejection) */
      lsm6dso_pedo_sens_set(&dev_ctx, LSM6DSO_FALSE_STEP_REJ_ADV_MODE);
      emb_sens.step = PROPERTY_ENABLE;
      emb_sens.step_adv = PROPERTY_ENABLE;
      lsm6dso_embedded_sens_set(&dev_ctx, &emb_sens);

      /*
       * Set pedometer debounce.
       *
       * Default value = 10 steps.
       * Set to 3 steps for easier demonstration.
       */
      lsm6dso_pedo_debounce_steps_set(&dev_ctx, &debounce);

      /* Reset step counter */
      lsm6dso_steps_reset(&dev_ctx);

      /* Latch embedded-function events so a short step pulse is not missed */
      lsm6dso_int_notification_set(&dev_ctx, LSM6DSO_ALL_INT_LATCHED);

      /* Route Step Detector event to INT1 */
      lsm6dso_pin_int1_route_get(&dev_ctx, &pin_int);
      pin_int.step_detector = PROPERTY_ENABLE;
      lsm6dso_pin_int1_route_set(&dev_ctx, pin_int);

      printf("Pedometer start...\r\n");

      while (1) {

          if (thread_wake)
          {
            lsm6dso_all_sources_t status = {0};

            thread_wake = 0;

            /* Read interrupt source */
            lsm6dso_all_sources_get(&dev_ctx,
                                    &status);

            /* Check Step Detector event */
            if (status.step_detector)
            {
              /* Read Step Counter */
              lsm6dso_number_of_steps_get(&dev_ctx,
                                          &step_count);

              printf("Step detected, Steps = %d\r\n",
                     step_count);
            }
          }
      }
  }
} /* end main */

