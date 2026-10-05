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
#define BOOT_TIME 10 // ms

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);

static volatile uint8_t thread_wake = 0;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len) {
  if (HAL_I2C_MASTER_MemWrite((hal_i2c_handle_t *)handle, LSM6DSO_I2C_ADD_L,
                              reg, HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK) {
    return -1;
  }
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len) {
  if (HAL_I2C_MASTER_MemRead((hal_i2c_handle_t *)handle, LSM6DSO_I2C_ADD_L,
                             reg, HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK) {
    return -1;
  }
  return 0;
}

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti,
                              hal_exti_trigger_t trigger)
{
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

    stmdev_ctx_t dev_ctx;
    lsm6dso_pin_int1_route_t pin_int = {0};  /* 单双击中断路由到 INT1 */
    lsm6dso_int_mode_t       irq_mode = {0};  /* 中断引脚极性/锁存 */
    lsm6dso_pin_conf_t       pin_conf = {0};  /* 中断引脚推挽/开漏 */

	  /* Initialize mems driver interface */
	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.mdelay = platform_delay;
	  dev_ctx.handle = mx_i2c1_i2c_gethandle();

	  /* Init test platform */
      // platform_init(dev_ctx.handle);

	  /* Wait sensor boot time */
	  platform_delay(BOOT_TIME);

	  /* Check device ID */
	  lsm6dso_device_id_get(&dev_ctx, &whoamI);
	  printf("LSM6DSO_ID=0x%x,id=0x%x\n",LSM6DSO_ID,whoamI);
	  if (whoamI != LSM6DSO_ID)
	    while (1);

	  /* Restore default configuration */
	  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
	  platform_delay(10);

	  /* Enable Block Data Update */
	  lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	  /* Set Output Data Rate. */
	  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_417Hz);
	  lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);

	  /* Set full scale */
	  lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
	  lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);

    /* INT1/INT2: open-drain + active low (matches external pull-up + PB0 falling edge) */
    pin_conf.int1_int2_push_pull = PROPERTY_DISABLE;  /* 0 = open-drain */
    lsm6dso_pin_conf_set(&dev_ctx, pin_conf);
    irq_mode.active_low = PROPERTY_ENABLE;            /* active low */
    lsm6dso_interrupt_mode_set(&dev_ctx, irq_mode);

      /* Enable Z-axis Tap detection */
      lsm6dso_tap_detection_on_x_set(&dev_ctx, PROPERTY_DISABLE);
      lsm6dso_tap_detection_on_y_set(&dev_ctx, PROPERTY_DISABLE);
      lsm6dso_tap_detection_on_z_set(&dev_ctx, PROPERTY_ENABLE);

      /* Set Tap threshold */
      lsm6dso_tap_threshold_x_set(&dev_ctx, 0);
      lsm6dso_tap_threshold_y_set(&dev_ctx, 0);
      lsm6dso_tap_threshold_z_set(&dev_ctx, 2);

      /* Set Tap time windows */
      lsm6dso_tap_shock_set(&dev_ctx, 2);
      lsm6dso_tap_quiet_set(&dev_ctx, 3);
      lsm6dso_tap_dur_set(&dev_ctx, 4);

      /* Enable Single Tap and Double Tap */
      lsm6dso_tap_mode_set(&dev_ctx, LSM6DSO_BOTH_SINGLE_DOUBLE);

      /* Route Single Tap and Double Tap to INT1 */
      pin_int.single_tap = PROPERTY_ENABLE;
      pin_int.double_tap = PROPERTY_ENABLE;
      pin_int.drdy_xl    = PROPERTY_DISABLE;
      lsm6dso_pin_int1_route_set(&dev_ctx, pin_int);


	  while (1) {

		if (thread_wake)
		{
		  lsm6dso_all_sources_t status = {0};

		  thread_wake = 0;

		  /* Read interrupt source */
		  lsm6dso_all_sources_get(&dev_ctx, &status);

		  if (status.double_tap)
		  {
			printf("Double Tap\r\n");
		  }
		  else if (status.single_tap)
		  {
			printf("Single Tap\r\n");
		  }
	    }
	  }
  }
} /* end main */



