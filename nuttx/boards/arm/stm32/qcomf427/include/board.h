
#ifndef __BOARDS_ARM_STM32_QCOMF427_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_QCOMF427_INCLUDE_BOARD_H

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

/****************************************************************************
 * Clocking
 * Configured at that system clock rate, so the core clock is 168MHz.
 *
 * This is the canonical configuration:
 *   System Clock source  : PLL (HSE)
 *   SYSCLK(Hz)           : 168000000    Determined by PLL configuration
 *   HCLK(Hz)             : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler        : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler       : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler       : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)    : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                 : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                 : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                 : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                 : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator
 *   output voltage       : Scale1 mode  Needed for high speed SYSCLK
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

/* MCB board is 25MHZ  */
#define STM32_BOARD_XTAL        25000000ul

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

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK    /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4   /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2   /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

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
#define BOARD_TIM9_FREQUENCY    STM32_HCLK_FREQUENCY


/*******RC timer***********/
#ifdef CONFIG_STM32_TIM1
#define GPIO_TIM1_CH1IN  GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN  GPIO_TIM1_CH2IN_2

#endif
#ifdef CONFIG_STM32_TIM9
#define GPIO_TIM9_CH1IN  GPIO_TIM9_CH1IN_2
#define GPIO_TIM9_CH2IN  GPIO_TIM9_CH2IN_2
#endif

/****************************************************************************
 * LED Definitions
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2
#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_RED     BOARD_LED2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

#define LED_STARTED       0  /* LED_STATUS on */
#define LED_HEAPALLOCATE  1  /* no change */
#define LED_IRQSENABLED   2  /* no change */
#define LED_STACKCREATED  3  /* no change */
#define LED_INIRQ         4  /* no change */
#define LED_SIGNAL        5  /* no change */
#define LED_ASSERTION     6  /* LED_STATUS off */
#define LED_PANIC         7  /* LED_STATUS blinking */



/* USART1 - console on header pins */

#define GPIO_USART1_RX GPIO_USART1_RX_1 /* AF7, PA10 */
#define GPIO_USART1_TX GPIO_USART1_TX_1 /* AF7, PA9 */


/* USART2 - To control motor controller(ZLAC8015D) */

#define GPIO_USART2_RX GPIO_USART2_RX_2 /* AF7, PD6 */
#define GPIO_USART2_TX GPIO_USART2_TX_2 /* AF7, PD5 */


#define GPIO_USART6_RX GPIO_USART6_RX_1  /* PC7 for 485*/
#define GPIO_USART6_TX GPIO_USART6_TX_1 /* PC6 for 485 */ 


/* CAN1 */
#define GPIO_CAN1_RX GPIO_CAN1_RX_3
#define GPIO_CAN1_TX GPIO_CAN1_TX_3

/* CAN2 */
#define GPIO_CAN2_RX GPIO_CAN2_RX_1
#define GPIO_CAN2_TX GPIO_CAN2_TX_1

/* SPI3 - Used for IMU icm42688  */
#define GPIO_SPI3_MISO    GPIO_SPI3_MISO_1  /* PB4 */
#define GPIO_SPI3_MOSI    GPIO_SPI3_MOSI_1  /* PB5 */
#define GPIO_SPI3_NSS     GPIO_SPI3_NSS_1   /* PA15 */ /* Used as chip select pin */
#define GPIO_SPI3_SCK     GPIO_SPI3_SCK_1   /* PB3 */

#define GPIO_CS_ICM42688  GPIO_SPI3_NSS

#define GPIO_USART6_RS485_DIR         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                                       GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN7)



#define GPIO_TIM11_CH1OUT    GPIO_TIM11_CH1OUT_1 

#endif /* __BOARDS_ARM_STM32_AXOLOTI_INCLUDE_BOARD_H */




