/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_pwm.c
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

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <limits.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>
#include <syslog.h>

#include "chip.h"
#include "stm32_pwm.h"
#include "qcomcarf4.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *
 *   Initialize PWM and register Omnibus F4's TIM2 and TIM3 PWM devices:
 *
 *    TIM3 CH3 PB0 S1_OUT
 *    TIM3 CH4 PB1 S2_OUT
 *    TIM2 CH4 PA3 S3_OUT
 *    TIM2 CH3 PA2 S4_OUT
 *
 ****************************************************************************/

static void stm32_timer_setup(int num)
{
  int npwm = num;
  const char *ppwm = NULL;
  struct pwm_lowerhalf_s *pwm = NULL;

  pwm = stm32_pwminitialize(npwm);
  /* Translate the peripheral number to a device name. */
  if (!pwm)
    syslog(LOG_ERR, "ERROR: stm32_timer_setup() failed \n");

   switch (npwm)
   {
        case 0:
          ppwm = "/dev/pwm0";
          break;
        case 1:
          ppwm = "/dev/pwm1";
          break;
        case 2:
          ppwm = "/dev/pwm2";
          break;
        case 3:
          ppwm = "/dev/pwm3";
          break;
        case 4:
          ppwm = "/dev/pwm4";
          break;
        case 5:
          ppwm = "/dev/pwm5";
          break;
        case 6:
          ppwm = "/dev/pwm6";
          break;
        case 7:
          ppwm = "/dev/pwm7";
          break;
        case 8:
          ppwm = "/dev/pwm8";
          break;

        /* Skip missing names. */

        default:
          return;
   }

  pwm_register(ppwm, pwm);

}

int stm32_pwm_setup(void)
{
  stm32_timer_setup(8);
  stm32_timer_setup(4);
  return 0;
}
