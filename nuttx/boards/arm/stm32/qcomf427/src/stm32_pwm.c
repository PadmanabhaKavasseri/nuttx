/****************************************************************************
 * boards/arm/stm32/nucleo-f429zi/src/stm32_pwm.c
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

#include <errno.h>
#include <debug.h>
#include <sys/types.h>

#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_pwm.h"
#include "qcomf427.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#ifndef CONFIG_PWM
#  undef HAVE_PWM
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm0;
  struct pwm_lowerhalf_s *pwm1;
  struct pwm_lowerhalf_s *pwm2;
  struct pwm_lowerhalf_s *pwm3;
  struct pwm_lowerhalf_s *pwm4;
  struct pwm_lowerhalf_s *pwm5;
  int ret;

  /* Have we already initialized? */

	if (!initialized){
		/* Call stm32_pwminitialize() to get an instance of the PWM interface */
	
		// pwm0 = stm32_pwminitialize(11);
		// if (!pwm0){
		// 	aerr("ERROR: Failed to get the STM32F4 PWM lower half\n");
		// 	return -ENODEV;
		// }

		// ret = pwm_register("/dev/pwm0", pwm0);
		// if (ret < 0){
		// 	aerr("ERROR: pwm_register failed: %d\n", ret);
		// 	return ret;
		// }
		// syslog(LOG_INFO, "PWM0 initialized\n");
		//--------------------------------------

		pwm1 = stm32_pwminitialize(13);
		if (!pwm1){
			aerr("ERROR: Failed to get the STM32F4 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register("/dev/pwm1", pwm1);
		if (ret < 0){
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
		syslog(LOG_INFO, "PWM1 initialized\n");
		//--------------------------------------
		
		pwm2 = stm32_pwminitialize(5); 
		if (!pwm2){
			aerr("ERROR: Failed to get the STM32F4 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register("/dev/pwm2", pwm2);
		if (ret < 0){
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
		syslog(LOG_INFO, "PWM2 initialized\n");
		//--------------------------------------

		pwm3 = stm32_pwminitialize(4);
		if (!pwm3){
			aerr("ERROR: Failed to get the STM32F4 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register("/dev/pwm3", pwm3);
		if (ret < 0){
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
		syslog(LOG_INFO, "PWM3 initialized\n");
		//--------------------------------------

		pwm4 = stm32_pwminitialize(3);
		if (!pwm4){
			aerr("ERROR: Failed to get the STM32F4 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register("/dev/pwm4", pwm4);
		if (ret < 0){
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
		syslog(LOG_INFO, "PWM4 initialized\n");
		//--------------------------------------

		pwm5 = stm32_pwminitialize(9);
		if (!pwm5){
			aerr("ERROR: Failed to get the STM32F4 PWM lower half\n");
			return -ENODEV;
		}

		ret = pwm_register("/dev/pwm5", pwm5);
		if (ret < 0){
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}
		syslog(LOG_INFO, "PWM5 initialized\n");
		//--------------------------------------
		
		/* Now we are initialized */
		initialized = true;
	}

  return OK;
}
