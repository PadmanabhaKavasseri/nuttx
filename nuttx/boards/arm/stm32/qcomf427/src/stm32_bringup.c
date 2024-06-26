/****************************************************************************
 * boards/arm/stm32/qcomf427/src/stm32_bringup.c
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

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#include "qcomf427.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif
  int ret = OK;

printf("stm32 bringup\n");
#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "failed to mount procfs at %s %d\n",
             STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = stm32_gpio_initialize();
  if (ret < 0)
  {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
     return ret;
  }
  syslog(LOG_INFO, " GPIOs have initialized\n");
printf("stm32_gpio_initialize done \n");

#endif

#ifdef CONFIG_SENSORS_ICM42688
  /* Initialize the ICM42688  device. */

  ret = stm32_icm42688_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ICM42688_initialize() failed: %d\n", ret);
    }
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

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */
  ret = stm32_adc_setup();
  if (ret <0)
  {
    syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    return ret;
  }
  syslog(LOG_INFO, "ADC  have initialized\n");
#endif

#ifdef CONFIG_PWM
  ret = stm32_pwm_setup();
  if (ret < 0){
    syslog(LOG_ERR, "ERROR: stm32_pwm_setup failed: %d\n", ret);
    return ret;
  }
  syslog(LOG_INFO, "PWM  have initialized --nuttx/boards/arm/stm32/qcomf427/src/stm32_bringup.c\n");
}
#endif


// #ifdef CONFIG_TIMER
//   ret = stm32_timer_driver_setup("/dev/timer0", 9);
//   if (ret < 0){
//     syslog(LOG_ERR, "ERROR: stm32_timer_driver_setup failed: %d\n", ret);
//     return ret;
//   }
//   syslog(LOG_INFO, "Timer initialized --nuttx/boards/arm/stm32/qcomf427/src/stm32_bringup.c\n");
// #endif
