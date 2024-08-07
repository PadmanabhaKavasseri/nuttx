/****************************************************************************
 * boards/arm/stm32/qcomf427/src/qcomf427.h
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



#ifndef __BOARDS_ARM_STM32_QCOMF427_SRC_QCOMF427_H
#define __BOARDS_ARM_STM32_QCOMF427_SRC_QCOMF427_H

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

/****************************************************************************
 * configuration
 */


/****************************************************************************
 * PROC File System Configuration
 */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * LEDs
 */

#define GPIO_LED1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                   GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)

#define GPIO_LED2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                   GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)

/* GPIOs */

#define BOARD_NGPIOIN     1
#define BOARD_NGPIOOUT    7
#define BOARD_NGPIOINT    0 

#define GPIO_CAN_STBY	      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN13)

#define GPIO_IMU_CS	     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN15)


//GPIO definitions
#define GPIO_TIM11_CH1OUT   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN9)
#define GPIO_TIM9_CH2OUT    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN6)
#define GPIO_TIM9_CH1OUT    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN5)
#define GPIO_TIM1_CH2OUT    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN11)
#define GPIO_TIM1_CH1OUT    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN9)

//GPIO Input definitions
#define GPIO_TIM2_CH3IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN10)




//PWM definitions
#define GPIO_TIM13_CH1OUT   GPIO_TIM13_CH1OUT_1
#define GPIO_TIM5_CH4OUT    GPIO_TIM5_CH4OUT_1
#define GPIO_TIM5_CH1OUT    GPIO_TIM5_CH1OUT_1
#define GPIO_TIM4_CH4OUT    GPIO_TIM4_CH4OUT_2
#define GPIO_TIM4_CH3OUT    GPIO_TIM4_CH3OUT_2 
#define GPIO_TIM4_CH2OUT    GPIO_TIM4_CH2OUT_2
#define GPIO_TIM3_CH4OUT    GPIO_TIM3_CH4OUT_1
#define GPIO_TIM3_CH3OUT    GPIO_TIM3_CH3OUT_1 


// #define GPIO_TIM4   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz| GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN6)

//Serial Definitions
#define GPIO_UART7_TX       GPIO_UART7_TX_1 //uartS4
#define GPIO_UART7_RX       GPIO_UART7_RX_1


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
int stm32_icm42688_initialize(void);

int stm32_bringup(void);
int stm32_gpio_initialize(void);
int stm32_can_setup(void);
int stm32_adc_setup(void);
int stm32_timer_driver_setup(const char *devpath, int timer);

/* TIMER1 capture for RC */
#ifdef CONFIG_CAPTURE
int stm32_capture_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_QCOMF427_SRC_QCOMF427_H */
