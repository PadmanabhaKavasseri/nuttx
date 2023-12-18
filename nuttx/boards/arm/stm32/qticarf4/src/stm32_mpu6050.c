/****************************************************************************
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <nuttx/sensors/mpu60x0.h>

#include "stm32_i2c.h"
#include "qcomcarf4.h"

#define DEVNODE_MPU6050   "/dev/imu0"
#define MPU_SLAVE_ADDR      0x68
#define I2CPORT_MPU6050   2


int stm32_mpu6050_initialize(void)
{
  int port = I2CPORT_MPU6050;
  int minor = SPIMINOR_MPU6000;
  struct i2c_master_s *i2c_master;
  struct mpu_config_s config;
  

  /* Get the i2c bus instance. */
 i2c_master = stm32_i2cbus_initialize(port);

  if (i2c_master == NULL)
  {
	  return -ENODEV;
  }

  config.i2c = i2c_master;
  config.addr = MPU_SLAVE_ADDR;
  /* TODO: configure EXTI pin */

  /* Register the chip with the device driver. */

  int ret = mpu60x0_register(DEVNODE_MPU6050, &config);
  return ret;
}