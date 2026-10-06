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

#define BOOT_TIME 10U
#define BOARD_EXPECTED_ID LSM6DSO_ID
#define FREE_FALL_DURATION 12U

static stmdev_ctx_t dev_ctx;
static volatile uint8_t thread_wake = 0;
static int32_t platform_write(void *handle,uint8_t reg,const uint8_t *bufp,uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg,uint8_t *bufp,uint16_t len);
static void platform_delay(uint32_t ms);

/**
 * @brief  EXTI trigger callback
 */
void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti, hal_exti_trigger_t trigger) {
    (void)trigger;
    if (HAL_EXTI_GetInstance(hexti) == HAL_EXTI_GPIO_0)
    {
        thread_wake = 1;
    }
}


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions prototype -----------------------------------------------*/

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

    lsm6dso_pin_int1_route_t pin_int1 = {0};
    uint8_t whoamI;
    uint8_t rst;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = mx_i2c1_i2c_gethandle();

    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);

    /* Check device ID */
    lsm6dso_device_id_get(&dev_ctx, &whoamI);

    printf("LSM6DSO_ID=0x%x,id=0x%x\n", LSM6DSO_ID, whoamI);

    if (whoamI != BOARD_EXPECTED_ID)
        while (1);

    /* Reset Device and wait end */
    lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
      lsm6dso_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Enable Block Data Update */
    lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Avoid events during accelerometer filter settling. */
	lsm6dso_xl_fast_settling_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dso_filter_settling_mask_set(&dev_ctx, PROPERTY_ENABLE);

    /* Low-G accelerometer: 417 Hz, +/-2 g, high performance. */
    lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_417Hz);
    lsm6dso_xl_power_mode_set(&dev_ctx, LSM6DSO_HIGH_PERFORMANCE_MD);
    lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);

    /* INT1: open-drain + active low. */
    lsm6dso_pin_mode_set(&dev_ctx, LSM6DSO_OPEN_DRAIN);
    lsm6dso_pin_polarity_set(&dev_ctx, LSM6DSO_ACTIVE_LOW);

    /* Enable latched interrupts. */
    lsm6dso_int_notification_set(&dev_ctx, LSM6DSO_ALL_INT_LATCHED);

    /* Free-fall threshold = 312 mg. */
    lsm6dso_ff_threshold_set(&dev_ctx, LSM6DSO_FF_TSH_312mg);

    /* Duration = 12 / 417 Hz ~= 29 ms (unit = 1/ODR_XL, max 63). */
    lsm6dso_ff_dur_set(&dev_ctx, FREE_FALL_DURATION);

    /* Route Free-fall event to INT1. */
    pin_int1.free_fall = PROPERTY_ENABLE;
    lsm6dso_pin_int1_route_set(&dev_ctx, pin_int1);

    while (1)
    {
        if (thread_wake)
        {
            uint8_t src = 0;
            thread_wake = 0;
            if (platform_read(dev_ctx.handle, LSM6DSO_ALL_INT_SRC, &src, 1) != 0)
            {
                thread_wake = 1;
                platform_delay(100);
                continue;
            }

            if (src & 0x01)
            {
                printf("Free-fall detected\r\n");
            }
        }
    }
  }
} /* end main */



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
