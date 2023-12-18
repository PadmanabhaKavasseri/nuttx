/****************************************************************************
 * boards/arm/stm32/omnibusf4/include/board.h
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

#ifndef __BOARDS_ARM_STM32_QCOMCARF4_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_QCOMCARF4_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/*
 * This is the canonical configuration:
 *   System Clock source    : PLL (HSE)
 *   SYSCLK(Hz)             : 168000000    Determined by PLL configuration
 *   HCLK(Hz)               : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler          : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler         : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler         : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)      : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                   : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                   : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                   : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                   : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator
 *   output voltage         : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)      : 5
 *   Prefetch Buffer        : OFF
 *   Instruction cache      : ON
 *   Data cache             : ON
 *   Require 48MHz for
 *   USB OTG FS,            : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (8,000,000 / 8) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* Pin configurations *******************************************************/

/**** PWM DC motor A/B ****/
/* GPIO PC6 */
#define GPIO_TIM8_CH1OUT  GPIO_TIM8_CH1OUT_1
/* GPIO PC7 */
#define GPIO_TIM8_CH2OUT  GPIO_TIM8_CH2OUT_1

/**** timer encoder A/B ****/
/* GPIO A15,B3 */
#define GPIO_TIM2_CH1IN  GPIO_TIM2_CH1IN_2
#define GPIO_TIM2_CH2IN  GPIO_TIM2_CH2IN_2
/* GPIO A6.A7  */
#define GPIO_TIM3_CH1IN  GPIO_TIM3_CH1IN_1
#define GPIO_TIM3_CH2IN  GPIO_TIM3_CH2IN_1

/*** PWM servo  ****/
#define GPIO_TIM4_CH2OUT GPIO_TIM4_CH2OUT_1

/*******RC timer***********/
#ifdef CONFIG_STM32_TIM1
#define GPIO_TIM1_CH1IN  GPIO_TIM1_CH1IN_1
#define GPIO_TIM1_CH2IN  GPIO_TIM1_CH2IN_1
#define GPIO_TIM1_CH3IN  GPIO_TIM1_CH3IN_1
#define GPIO_TIM1_CH4IN  GPIO_TIM1_CH4IN_1
#endif
#ifdef CONFIG_STM32_TIM5
#define GPIO_TIM5_CH1IN  GPIO_TIM5_CH1IN_1
#define GPIO_TIM5_CH2IN  GPIO_TIM5_CH2IN_1
#endif
/***************************************************************************/

#define BOARD_NLEDS     1                      /* One literal LED, one beeper */
#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |\
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN13)
#define GPIO_BEEPER1    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |\
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB|GPIO_PIN4)

/* USART1: */
#ifdef CONFIG_STM32_USART1
#define GPIO_USART1_RX  GPIO_USART1_RX_1       /* PA10 */
#define GPIO_USART1_TX  GPIO_USART1_TX_1       /* PA9  */
#endif

/* USART2:
 * function as nsh
 */
#ifdef CONFIG_STM32_USART2
#define GPIO_USART2_RX  GPIO_USART2_RX_1       /* PA3 */
#define GPIO_USART2_TX  GPIO_USART2_TX_1       /* PA2  */
#endif

/* USART3:
 *function as ROS
 */
#ifdef CONFIG_STM32_USART3
#define GPIO_USART3_TX    GPIO_USART3_TX_2     /* PC10 */
#define GPIO_USART3_RX    GPIO_USART3_RX_2     /* PC11 */
#endif

/* I2C for MPU */

#define GPIO_I2C2_SCL     GPIO_I2C2_SCL_1     /* PB10 */
#define GPIO_I2C2_SDA     GPIO_I2C2_SDA_1     /* PB11 */

#ifdef CONFIG_ADC
/* multi channels need DMA */
#define ADC1_DMA_CHAN DMAMAP_ADC1_1
#endif

#endif /* __BOARDS_ARM_STM32_QCOMCARF4_INCLUDE_BOARD_H */
