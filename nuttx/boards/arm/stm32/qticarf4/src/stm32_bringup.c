/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_bringup.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "stm32.h"
#include "stm32_romfs.h"

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#include "qcomcarf4.h"

/* Conditional logic in qcomcarf4.h will determine if certain features
 * are supported.  Tests for these features need to be made after including
 * qcomcarf4.h.
 */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *lower;
#endif
  int ret = OK;

#ifdef CONFIG_DEV_GPIO
  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
   syslog(LOG_INFO, " GPIOs have initialized\n");
#endif

#ifdef CONFIG_SENSORS_QENCODER
  ret = stm32f4_qencoder_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
  syslog(LOG_INFO, " Encoder have initialized\n");
#endif

#ifdef CONFIG_ADC
  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the ADC: %d\n",
             ret);
      return ret;
    }
   syslog(LOG_INFO, " ADC have initialized\n");
#endif

#ifdef CONFIG_PWM
  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
  syslog(LOG_INFO, " PWM have initialized\n");
#endif

#ifdef CONFIG_CAN
  /* Initialize CAN and register the CAN driver. */

  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_can_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAPTURE
  /* Initialize Capture and register the Capture driver. */

  ret = stm32_capture_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_capture_setup failed: %d\n", ret);
      return ret;
    }
  syslog(LOG_INFO, "Capture have initialized\n");
#endif

#ifdef CONFIG_SENSORS_MPU60X0
  /* Initialize the MPU6000 device. */

  ret = stm32_mpu6050_initialize();
  if (ret < 0)
  {
      syslog(LOG_ERR, "ERROR: stm32_mpu6050_initialize() failed: %d\n", ret);
  }
  syslog(LOG_INFO, " IMU have initialized\n");
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_STM32_ROMFS
  /* Initialize and mount ROMFS. */

  ret = stm32_romfs_initialize();
  if (ret < 0)
    {
      serr("ERROR: Failed to mount romfs at %s: %d\n",
           CONFIG_STM32_ROMFS_MOUNTPOINT, ret);
    }
 syslog(LOG_INFO, " romfs have initialized\n");
#endif

  return ret;
}
