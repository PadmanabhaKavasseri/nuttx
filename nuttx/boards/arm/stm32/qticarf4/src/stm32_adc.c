/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_adc.c
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
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_adc.h"
#include "qcomcarf4.h"


#ifdef CONFIG_ADC

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)
#ifndef CONFIG_STM32_ADC1
#  warning "Channel information only available for ADC1"
#endif

#define GPIO_ADC_SERVO_ZERO    GPIO_ADC1_IN1         
#define GPIO_ADC_VOLTAGE       GPIO_ADC1_IN5
#define GPIO_ADC_CAR_MODE      GPIO_ADC2_IN13 

#define DEV_ADC_SERVO   "/dev/adc_servo"
//#define DEV_VOLTAGE   "/dev/adc_power"
#define DEV_ADC_CAR   "/dev/adc_car_mode"

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS  2
#define ADC2_NCHANNELS  1
#define ADC_NCHANNELS  (ADC1_NCHANNELS + ADC1_NCHANNELS)

static const uint8_t  g_chanlist1[ADC1_NCHANNELS] =
{
  1,
  5,
};

static const uint8_t  g_chanlist2[ADC2_NCHANNELS] =
{
  13
};

/* Configurations of pins used byte each ADC channels */

static const uint32_t g_pinlist[ADC_NCHANNELS] =
{
	GPIO_ADC_SERVO_ZERO,
	GPIO_ADC_VOLTAGE,
	GPIO_ADC_CAR_MODE,
};

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
	static bool initialized = false;
	struct adc_dev_s *adc1;
	struct adc_dev_s *adc2;
	int ret1, ret2;
	int i;

	/* Check if we have already initialized */
	if (!initialized)
    {
		/* Configure the pins as analog inputs for the selected channels */
		for (i = 0; i < ADC_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist[i]);
        }

		/* Call stm32_adcinitialize() to get an instance of the ADC interface */
		adc1 = stm32_adcinitialize(1, g_chanlist1, ADC1_NCHANNELS);
		adc2 = stm32_adcinitialize(2, g_chanlist2, ADC2_NCHANNELS);
		if (adc1 == NULL || adc2 == NULL ) {
			syslog(LOG_ERR, "ERROR: Failed to get ADC interface\n");
			return -ENODEV;
		}
		/* Register the ADC driver at "/dev/adcxxx" */
		ret1 = adc_register(DEV_ADC_SERVO, adc1);
		ret2 = adc_register(DEV_ADC_CAR, adc2);
		if (ret1 < 0) {
			syslog(LOG_ERR, "ERROR: Failed to get ADC1 interface\n");
			return ret1;
		}
		if (ret2 < 0) {
			syslog(LOG_ERR, "ERROR: Failed to get ADC2 interface\n");
			return ret2;
		}
		initialized = true;
	}

	return OK;
}

#endif /* CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 || CONFIG_STM32_ADC3 */
#endif /* CONFIG_ADC */
