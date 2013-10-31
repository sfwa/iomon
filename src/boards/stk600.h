/*****************************************************************************
 *
 * \file
 *
 * \brief AT32UC3C UC3C_EK board header file.
 *
 * This file contains definitions and services related to the features of the
 * UC3C_EK board.
 *
 * To use this board, define BOARD=UC3C_EK.
 *
 * Copyright (C) 2009 - 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/


#ifndef _STK600_H_
#define _STK600_H_

#include "compiler.h"

#ifdef AVR32_SCIF_101_H_INCLUDED
#define AVR32_SCIF_OSCCTRL0_STARTUP_2048_RCOSC  0x00000003u
#define AVR32_SCIF_OSCCTRL0_STARTUP_16384_RCOSC 0x00000006u
#define AVR32_SCIF_OSCCTRL32_STARTUP_8192_RCOSC 0x00000002u
#endif

/* Oscillator Definitions */
#define FOSC32          AVR32_SCIF_OSC32_FREQUENCY              /* Osc32 frequency: Hz. */
#define OSC32_STARTUP   AVR32_SCIF_OSCCTRL32_STARTUP_8192_RCOSC /* Osc32 startup time: RCOsc periods. */

/* Osc0 crystal is not mounted by default. Set the following definitions to the
   appropriate values if a custom Osc0 crystal is mounted on your STK600 */
#define FOSC0           16000000u                               /* Osc0 frequency: Hz. */
#define OSC0_STARTUP    AVR32_SCIF_OSCCTRL0_STARTUP_2048_RCOSC  /* Osc0 startup time: RCOsc periods. */

#define BOARD_OSC0_HZ           16000000u
#define BOARD_OSC0_STARTUP_US   2000u
#define BOARD_OSC0_IS_XTAL      true
#define BOARD_OSC32_HZ          32768u
#define BOARD_OSC32_STARTUP_US  71000u
#define BOARD_OSC32_IS_XTAL     true

/* Clock setup */
#define CONFIG_SYSCLK_SOURCE          SYSCLK_SRC_PLL0
#define CONFIG_PLL0_SOURCE            PLL_SRC_OSC0
#define CONFIG_USBCLK_SOURCE          USBCLK_SRC_PLL0
#define CONFIG_USBCLK_DIV             1 /* Fusb = Fsys/(2 ^ USB_div) */

/* Set up PLL0 for 48MHz operation; Fpll = (Fclk * PLL_mul) / PLL_div */
#define CONFIG_PLL0_MUL               4u
#define CONFIG_PLL0_DIV               1u


/* UC3C1512 - TQFP100 / STK600 / Function pin assignments
 *
 * 001: GPIO000: PA00 / PA0,TCK
 * 002: GPIO001: PA01 / PA1,TDI
 * 003: GPIO002: PA02 / PA2,TDO
 * 004: GPIO003: PA03 / PA3,TMS
 * 005:
 * 006:
 * 007: GPIO036: PB04 / PE4
 * 008: GPIO037: PB05 / PE5
 * 009: GPIO038: PB06 / PE6
 * 010: GPIO004: PA04 / PA4       / ADCIN0
 * 011: GPIO005: PA05 / PA5       / ADCIN1
 * 012: GPIO006: PA06 / PA6       / ADCIN2
 * 013: GPIO007: PA07 / PA7       / ADCIN3
 * 014: GPIO008: PA08 / PB0       / LED0_GPIO
 * 015: GPIO009: PA09 / PB1       / LED1_GPIO
 * 016: GPIO010: PA10 / PB2       / LED2_GPIO
 * 017: GPIO011: PA11 / PB3,AREF1 / LED3_GPIO
 * 018: GPIO012: PA12 / PB4
 * 019: GPIO013: PA13 / PB5
 * 020: GPIO014: PA14 / PB6
 * 021: GPIO015: PA15 / PB7
 * 022: GPIO016: PA16 / PC0,AREF0
 * 023: ADC REFP
 * 024: ADC REFN
 * 025: GPIO019: PA19 / PC3
 * 028: GPIO020: PA20 / PC4
 * 029: GPIO021: PA21 / PC5
 * 030: GPIO022: PA22 / PC6
 * 031: GPIO023: PA23 / PC7
 * 032: GPIO024: PA24 / PD0
 * 033: GPIO025: PA25 / PD1
 * 042: GPIO051: PB19 / PG3
 * 043: GPIO052: PB20 / PG4
 * 044: GPIO053: PB21 / PG5
 * 045: GPIO054: PB22 / PG6
 * 046: GPIO055: PB23 / PG7
 * 047: GPIO062: PB30 / PH6,XTAL1
 * 048: GPIO063: PB31 / PH7,XTAL2
 * 049: GPIO064: PC00 / PJ0
 * 050: GPIO065: PC01 / PJ1,UVCON
 * 051: GPIO066: PC02 / PJ2       / HMC5883_TWI_TWD_PIN[0]
 * 052: GPIO067: PC03 / PJ3       / HMC5883_TWI_TWCK_PIN[0]
 * 055: GPIO068: PC04 / PJ4       / MPU6050_TWI_TWD_PIN[0]
 * 056: GPIO069: PC05 / PJ5       / MPU6050_TWD_TWCK_PIN[0]
 * 057: GPIO070: PC06 / PJ6       / MS5611_TWI_TWD_PIN[4]
 * 058: GPIO071: PC07 / PJ7       / MS5611_TWI_TWCK_PIN[4]
 * 059: GPIO075: PC11 / PK3       / PWM-PWMH3[0]
 * 060: GPIO076: PC12 / PK4       / GPS_ENABLE_PIN
 * 061: GPIO077: PC13 / PK5       / PWM-PWMH2[0]
 * 062: GPIO078: PC14 / PK6       / MS5611_ENABLE_PIN
 * 063: GPIO079: PC15 / PK7       / PWM-PWMH1[0]
 * 064: GPIO080: PC16 / PL0       / HMC5883_ENABLE_PIN
 * 065: GPIO081: PC17 / PL1       / PWM-PWMH0[0]
 * 066: GPIO082: PC18 / PL2       / MPU6050_ENABLE_PIN
 * 067: GPIO083: PC19 / PL3
 * 068: GPIO084: PC20 / PL4
 * 069: GPIOO85: PC21 / PL5
 * 070: GPIO086: PC22 / PL6
 * 071: GPIO087: PC23 / PL7
 * 072: GPIO088: PC24 / PM0
 * 073: GPIO095: PC31 / PM7
 * 074: GPIO096: PD00 / PN0       / GPS_USART_TXD_PIN[4]
 * 075: GPIO097: PD01 / PN1       / GPS_USART_RXD_PIN[4]
 * 076: GPIO098: PD02 / PN2
 * 077: GPIO099: PD03 / PN3
 * 078: GPIO103: PD07 / PN7
 * 079: GPIO104: PD08 / PP0       / GPOUT0
 * 080: GPIO105: PD09 / PP1       / GPOUT1
 * 081: GPIO106: PD10 / PP2       / GPOUT2
 * 084: GPIO107: PD11 / PP3       / GPOUT3
 * 085: GPIO108: PD12 / PP4
 * 086: GPIO109: PD13 / PP5
 * 087: GPIO110: PD14 / PP6
 * 088: GPIO117: PD21 / PQ5
 * 089: GPIO118: PD22 / PQ6
 * 090: GPIO119: PD23 / PQ7
 * 091: GPIO120: PD24 / PDATA0
 * 092: GPIO123: PD27 / PDATA3
 * 093: GPIO124: PD28 / PDATA4
 * 094: GPIO125: PD29 / PDATA5
 * 095: GPIO126: PD30 / PDATA6
 * 096: GPIO032: PB00 / PE0,TOSC1 / GPIN0
 * 097: GPIO033: PB01 / PE1,TOSC2 / GPIN1
 * 099: GPIO034: PB02 / PE2       / GPIN2
 * 100: GPIO035: PB03 / PE3       / GPIN3
 */

/* LED assignments for GPIOs -- active low */
#define LED_ON(x) gpio_local_clr_gpio_pin(x)
#define LED_OFF(x) gpio_local_set_gpio_pin(x)

#define LED0_GPIO                        8
#define LED1_GPIO                        9
#define LED2_GPIO                        10
#define LED3_GPIO                        11

/* USART connection to the GPS */
#define GPS_USART                        (&AVR32_USART0)
#define GPS_USART_RXD_PIN                97
#define GPS_USART_RXD_FUNCTION           4
#define GPS_USART_TXD_PIN                96
#define GPS_USART_TXD_FUNCTION           4
#define GPS_USART_IRQ                    AVR32_USART0_IRQ
#define GPS_USART_IRQ_GROUP              AVR32_USART0_IRQ_GROUP
#define GPS_USART_SYSCLK                 SYSCLK_USART0

#define PDCA_CHANNEL_GPS_TX              0
#define PDCA_CHANNEL_GPS_RX              1
#define GPS_USART_PDCA_PID_TX            AVR32_PDCA_PID_USART0_TX
#define GPS_USART_PDCA_PID_RX            AVR32_PDCA_PID_USART0_RX

#define GPS_ENABLE_PIN                   76

/* I2C connection to the MS5611 barometer */
#define MS5611_DEVICE_ADDR               0x76u /* 0x77 if CS tied to GND */

#define MS5611_TWI                       (&AVR32_TWIM2)
#define MS5611_TWI_TWD_PIN               70
#define MS5611_TWI_TWD_FUNCTION          4
#define MS5611_TWI_TWCK_PIN              71
#define MS5611_TWI_TWCK_FUNCTION         4
#define MS5611_TWI_SYSCLK                SYSCLK_TWIM2

#define PDCA_CHANNEL_MS5611_TX           2
#define PDCA_CHANNEL_MS5611_RX           3
#define MS5611_TWI_PDCA_PID_TX           AVR32_TWIM2_PDCA_ID_TX
#define MS5611_TWI_PDCA_PID_RX           AVR32_TWIM2_PDCA_ID_RX

#define MS5611_ENABLE_PIN                78

/* I2C connection to the MPU6050 accelerometer/gyro unit */
#define MPU6050_DEVICE_ADDR              0x68u /* 0x69 if AD0 high */

#define MPU6050_TWI                      (&AVR32_TWIM1)
#define MPU6050_TWI_TWD_PIN              68
#define MPU6050_TWI_TWD_FUNCTION         0
#define MPU6050_TWI_TWCK_PIN             69
#define MPU6050_TWI_TWCK_FUNCTION        0
#define MPU6050_TWI_SYSCLK               SYSCLK_TWIM1

#define PDCA_CHANNEL_MPU6050_TX          4
#define PDCA_CHANNEL_MPU6050_RX          5
#define MPU6050_TWI_PDCA_PID_TX          AVR32_TWIM1_PDCA_ID_TX
#define MPU6050_TWI_PDCA_PID_RX          AVR32_TWIM1_PDCA_ID_RX

#define MPU6050_ENABLE_PIN               82

/* I2C connection to the HMC5883 magnetometer */
#define HMC5883_DEVICE_ADDR              0x1Eu

#define HMC5883_TWI                      (&AVR32_TWIM0)
#define HMC5883_TWI_TWD_PIN              66
#define HMC5883_TWI_TWD_FUNCTION         0
#define HMC5883_TWI_TWCK_PIN             67
#define HMC5883_TWI_TWCK_FUNCTION        0
#define HMC5883_TWI_SYSCLK               SYSCLK_TWIM0

#define PDCA_CHANNEL_HMC5883_TX          6
#define PDCA_CHANNEL_HMC5883_RX          7
#define HMC5883_TWI_PDCA_PID_TX          AVR32_TWIM0_PDCA_ID_TX
#define HMC5883_TWI_PDCA_PID_RX          AVR32_TWIM0_PDCA_ID_RX

#define HMC5883_ENABLE_PIN               80

/* GPIO and ADC definitions */
#define GPIN_0_PIN                       32
#define GPIN_1_PIN                       33
#define GPIN_2_PIN                       34
#define GPIN_3_PIN                       35

#define GPOUT_0_PIN                      104
#define GPOUT_1_PIN                      105
#define GPOUT_2_PIN                      106
#define GPOUT_3_PIN                      107

#define ADC_PITOT_PIN                    4
#define ADC_PITOT_FUNCTION               0
#define ADC_BATTERY_V_PIN                5
#define ADC_BATTERY_V_FUNCTION           0
#define ADC_BATTERY_I_PIN                6
#define ADC_BATTERY_I_FUNCTION           0
#define ADC_AUX_PIN                      7
#define ADC_AUX_FUNCTION                 0

#define GP_ADC                           (&AVR32_ADCIFA)
#define GP_ADC_SYSCLK                    SYSCLK_ADCIFA
#define PDCA_CHANNEL_ADC_RX              8
#define ADC_PDCA_PID_RX                  AVR32_PDCA_PID_ADCIFA_CH0_RX

/* PWM pin definitions */
#define PWM_0_PIN                        81
#define PWM_0_FUNCTION                   0
#define PWM_1_PIN                        79
#define PWM_1_FUNCTION                   0
#define PWM_2_PIN                        77
#define PWM_2_FUNCTION                   0
#define PWM_3_PIN                        75
#define PWM_3_FUNCTION                   0
#define PWM_SYSCLK                       SYSCLK_PWM

#define PWM_ENABLE_PIN                   0

/* CPU board interface */
#define CPU_USART                        (&AVR32_USART1)
#define CPU_USART_TXD_PIN                0
#define CPU_USART_TXD_FUNCTION           0
#define CPU_USART_RXD_PIN                0
#define CPU_USART_RXD_FUNCTION           0
#define CPU_USART_IRQ                    AVR32_USART1_IRQ
#define CPU_USART_IRQ_GROUP              AVR32_USART1_IRQ_GROUP
#define CPU_USART_SYSCLK                 SYSCLK_USART1

#define PDCA_CHANNEL_CPU_TX              9
#define PDCA_CHANNEL_CPU_RX              10
#define CPU_USART_PDCA_PID_TX            AVR32_PDCA_PID_USART1_TX
#define CPU_USART_PDCA_PID_RX            AVR32_PDCA_PID_USART1_RX

#define CPU_RESET_PIN                    0

/* AUX USART interface */
#define AUX_USART                        (&AVR32_USART2)
#define AUX_USART_TXD_PIN                0
#define AUX_USART_TXD_FUNCTION           0
#define AUX_USART_RXD_PIN                0
#define AUX_USART_RXD_FUNCTION           0
#define AUX_USART_IRQ                    AVR32_USART2_IRQ
#define AUX_USART_IRQ_GROUP              AVR32_USART2_IRQ_GROUP
#define AUX_USART_SYSCLK                 SYSCLK_USART2

#define PDCA_CHANNEL_AUX_TX              11
#define PDCA_CHANNEL_AUX_RX              12
#define AUX_USART_PDCA_PID_TX            AVR32_PDCA_PID_USART2_TX
#define AUX_USART_PDCA_PID_RX            AVR32_PDCA_PID_USART2_RX

#endif
