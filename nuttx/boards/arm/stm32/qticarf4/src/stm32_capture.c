/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_capture.c
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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * 
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <nuttx/timers/capture.h>
#include <arch/board/board.h>

#include "chip.h"

#include "stm32.h"
#include "stm32_capture.h"
#include "arm_internal.h"

#include "qcomcarf4.h"

#if defined(CONFIG_CAPTURE)
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Capture
 *
 * The stm32f4 use timer1 capture function to capture RC receiver PWM
 */

#ifdef CONFIG_CAPTURE
#define RC_CAPTURE_TIMER 1
#define RC_CAPTURE_TIMER_1 5
#endif

#define HAVE_CAPTURE 1

#ifndef CONFIG_CAPTURE
#  undef HAVE_CAPTURE
#endif

#ifndef CONFIG_STM32_TIM1
#  undef HAVE_CAPTURE
#endif

#ifndef CONFIG_STM32_TIM1_CAP
#  undef HAVE_CAPTURE
#endif

#if !defined(CONFIG_STM32_TIM1_CHANNEL)
#  undef HAVE_CAPTURE
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_capture_setup
 *
 * Description:
 *   Initialize and register the pwm capture driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/capture0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_capture_setup(void)
{
#ifdef HAVE_CAPTURE
  struct cap_lowerhalf_s *capture;
  int ret;

  capture = stm32_cap_initialize(RC_CAPTURE_TIMER);

  /* register capture0 */
  
  ret = cap_register("/dev/capture0", capture);
  if (ret < 0) {
	syslog(LOG_ERR, "ERROR: Error registering /dev/capture0\n");      
  }

  /* register capture1 */
  capture = stm32_cap_initialize(RC_CAPTURE_TIMER_1);
  ret = cap_register("/dev/capture1", capture);
  if (ret < 0) {
        syslog(LOG_ERR, "ERROR: Error registering /dev/capture1\n");
  }

  return ret;

#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAPTURE */
