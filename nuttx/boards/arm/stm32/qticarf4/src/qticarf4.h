/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32f4discovery.h
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

#ifndef __BOARDS_ARM_STM32_QTICARF4_SRC_QTICARF4_H
#define __BOARDS_ARM_STM32_QTICARF4_SRC_QTICARF4_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* QCOMCARF4 GPIOs **********************************************************/

#define SPIPORT_MPU6000   1
#define SPIMINOR_MPU6000  0
#define DEVNODE_MPU6000   "/dev/imu0"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
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
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_CAN
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_timer_driver_setup
 *
 * Description:
 *   Configure the timer driver.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *             form /dev/timer0
 *   timer   - The timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32_timer_driver_setup(FAR const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32_mpu6050_initialize
 *
 * Description:
 *  Initialize the MPU6000 device.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MPU60X0
int stm32_mpu6050_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#endif
/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    9 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* gpio in */
#define GPIO_IN1         (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
			   GPIO_OPENDRAIN | GPIO_PORTC | GPIO_PIN1)

/*gpio int */
#define GPIO_INT1         (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN2)

/* led */
#define GPIO_OUT1         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN13)
/*************** motor A/B direction *******/
/* MOTOR A */
#define GPIO_OUT2         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN5)
#define GPIO_OUT3         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                           GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN12)
/* motor B */
#define GPIO_OUT4         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                           GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN2)
#define GPIO_OUT5         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN4)
/******************* oled used 4 gpios ****************/
/* PC0 for OLED_DC */
#define GPIO_OUT6	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
				GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN0)
/* PC13 for OLED_SCLK */
#define GPIO_OUT7	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                                GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN13)
/* PC14 for OLED_SDIN */
#define GPIO_OUT8       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                                GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN14)
/* PC15 for OLED_RST */
#define GPIO_OUT9       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                                GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN15)

/* Quard encoder */
#ifdef CONFIG_SENSORS_QENCODER
int stm32f4_qencoder_initialize(void);
#endif

/* ADC */
#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/* TIMER1 capture for RC */
#if defined(CONFIG_CAPTURE)
int stm32_capture_setup(void);
#endif


#endif /* __ASSEMBLY__ */
#endif
