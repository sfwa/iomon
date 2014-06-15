/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _IOBOARD_H_
#define _IOBOARD_H_

#include "compiler.h"

/* 12MHz oscillator connected to XIN0 */
#define BOARD_OSC0_HZ                  12000000u
#define BOARD_OSC0_STARTUP_US          10000u
#define BOARD_OSC0_IS_XTAL             false

/* Clock setup */
#define CONFIG_SYSCLK_SOURCE           SYSCLK_SRC_PLL0
#define CONFIG_PLL0_SOURCE             PLL_SRC_OSC0
#define CONFIG_USBCLK_SOURCE           USBCLK_SRC_PLL0
#define CONFIG_USBCLK_DIV              1 /* Fusb = Fsys/(2 ^ USB_div) */

/* Set up PLL0 for 52MHz operation; Fpll = (Fclk * PLL_mul) / PLL_div
   All we need to do is multiply by 13/3. */
#define CONFIG_PLL0_MUL                13u
#define CONFIG_PLL0_DIV                3u

/* UC3C1512 - TQFP100 / IOBOARD      / Software function pin assignments
 *
 * 001: GPIO000: PA00 / JTAG TCK
 * 002: GPIO001: PA01 / JTAG TDI
 * 003: GPIO002: PA02 / JTAG TDO
 * 004: GPIO003: PA03 / JTAG TMS
 * 005:
 * 006:
 * 007: GPIO036: PB04
 * 008: GPIO037: PB05 / RESET_OUT     / CPU_RESET_PIN
 * 009: GPIO038: PB06
 * 010: GPIO004: PA04 / ADC0,P7.1     / ADC_PITOT_PIN[0]
 * 011: GPIO005: PA05 / ADC1,P8.1     / ADC_AUX_PIN[0]
 * 012: GPIO006: PA06 / ADC2,P9.1     / ADC_BATTERY_V_PIN[0]
 * 013: GPIO007: PA07 / ADC3,P9.2     / ADC_BATTERY_I_PIN[0]
 * 014: GPIO008: PA08
 * 015: GPIO009: PA09
 * 016: GPIO010: PA10
 * 017: GPIO011: PA11 / AREF1 (+VIN)
 * 018: GPIO012: PA12 / LED0          / LED0_GPIO
 * 019: GPIO013: PA13 / LED1          / LED1_GPIO
 * 020: GPIO014: PA14 / LED2          / LED2_GPIO
 * 021: GPIO015: PA15 / LED3          / LED3_GPIO
 * 022: GPIO016: PA16
 * 023: ADC REFP
 * 024: ADC REFN
 * 025: GPIO019: PA19
 * 028: GPIO020: PA20
 * 029: GPIO021: PA21
 * 030: GPIO022: PA22
 * 031: GPIO023: PA23
 * 032: GPIO024: PA24
 * 033: GPIO025: PA25
 * 042: GPIO051: PB19 / SPI1.MOSI     / MPU6000_MOSI[1]
 * 043: GPIO052: PB20 / SPI1.MISO     / MPU6000_MISO[1]
 * 044: GPIO053: PB21 / SPI1.CLK      / MPU6000_CLK[1]
 * 045: GPIO054: PB22 / SPI1.CS3#     / MPU6000_CS[1]
 * 046: GPIO055: PB23 / SPI1.EN       / MPU6000_ENABLE_PIN
 * 047: GPIO062: PB30 / CLK_12        / OSC0
 * 048: GPIO063: PB31
 * 049: GPIO064: PC00 / I2C1.EN       / MS5611_ENABLE_PIN
 * 050: GPIO065: PC01 / I2C0.EN       / HMC5883_ENABLE_PIN
 * 051: GPIO066: PC02 / I2C0.SDA      / HMC5883_TWI_TWD_PIN[0]
 * 052: GPIO067: PC03 / I2C0.SCL      / HMC5883_TWD_TWCK_PIN[0]
 * 055: GPIO068: PC04 / I2C1.SDA      / MS5611_TWI_TWD_PIN[0]
 * 056: GPIO069: PC05 / I2C1.SCL      / MS5611_TWI_TWCK_PIN[0]
 * 057: GPIO070: PC06 / I2C2.SDA      / MS4525_TWI_TWD_PIN[4]
 * 058: GPIO071: PC07 / I2C2.SCL      / MS4525_TWI_TWCK_PIN[4]
 * 059: GPIO075: PC11 / MCU_PWM.PWM3  / PWM_3_PIN[0]
 * 060: GPIO076: PC12 / I2C2.EN       / MS4525_ENABLE_PIN
 * 061: GPIO077: PC13 / MCU_PWM.PWM2  / PWM_2_PIN[0]
 * 062: GPIO078: PC14
 * 063: GPIO079: PC15 / MCU_PWM.PWM1  / PWM_1_PIN[0]
 * 064: GPIO080: PC16
 * 065: GPIO081: PC17 / MCU_PWM.PWM0  / PWM_0_PIN[0]
 * 066: GPIO082: PC18 / MCU_PWM.EN    / PWM_ENABLE_PIN
 * 067: GPIO083: PC19
 * 068: GPIO084: PC20
 * 069: GPIOO85: PC21
 * 070: GPIO086: PC22
 * 071: GPIO087: PC23
 * 072: GPIO088: PC24
 * 073: GPIO095: PC31
 * 074: GPIO096: PD00 / UART0.TX      / CPU_USART_TXD_PIN[4]
 * 075: GPIO097: PD01 / UART0.RX      / CPU_USART_RXD_PIN[4]
 * 076: GPIO098: PD02
 * 077: GPIO099: PD03
 * 078: GPIO103: PD07 / UART4.TX      / AUX_USART_TXD_PIN[4]
 * 079: GPIO104: PD08 / UART4.RX      / AUX_USART_RXD_PIN[4]
 * 080: GPIO105: PD09
 * 081: GPIO106: PD10
 * 084: GPIO107: PD11 / UART1.TX      / GPS_USART_TXD_PIN[0]
 * 085: GPIO108: PD12 / UART1.RX      / GPS_USART_RXD_PIN[0]
 * 086: GPIO109: PD13 / UART1.EN      / GPS_ENABLE_PIN
 * 087: GPIO110: PD14
 * 088: GPIO117: PD21 / MCU_PWM_IN.PWM0 / PWM_IN_0_PIN[0]
 * 089: GPIO118: PD22 / MCU_PWM_IN.PWM1 / PWM_IN_1_PIN[0]
 * 090: GPIO119: PD23 / MCU_PWM_IN.PWM2 / PWM_IN_2_PIN[0]
 * 091: GPIO120: PD24 / MCU_PWM_IN.PWM3 / PWM_IN_3_PIN[0]
 * 092: GPIO123: PD27 / GP_OUT0       / GPOUT_0_PIN,PAYLOAD_TRIGGER
 * 093: GPIO124: PD28 / GP_OUT1,P11.4 / GPOUT_1_PIN,LL_OUT0
 * 094: GPIO125: PD29 / GP_OUT2,P11.3 / GPOUT_2_PIN,LL_OUT1
 * 095: GPIO126: PD30 / GP_OUT3,P11.2 / GPOUT_3_PIN,LL_OUT2
 * 096: GPIO032: PB00 / GP_IN0,P2.1   / GPIN_0_PIN,PAYLOAD_DETECT
 * 097: GPIO033: PB01 / GP_IN1,P3.4   / GPIN_1_PIN,AUX_INPUT0
 * 099: GPIO034: PB02 / GP_IN2,P3.3   / GPIN_2_PIN,AUX_INPUT1
 * 100: GPIO035: PB03 / GP_IN3,P3.2   / GPIN_3_PIN,AUX_INPUT2
 */


/* LED assignments -- active high */
#define LED_ON(x) gpio_local_set_gpio_pin(x)
#define LED_OFF(x) gpio_local_clr_gpio_pin(x)

#define LED0_GPIO                      12
#define LED1_GPIO                      13
#define LED2_GPIO                      14
#define LED3_GPIO                      15

/* USART connection to the GPS */
#define GPS_USART                      (&AVR32_USART1)
#define GPS_USART_TXD_PIN              107
#define GPS_USART_TXD_FUNCTION         0
#define GPS_USART_RXD_PIN              108
#define GPS_USART_RXD_FUNCTION         0
#define GPS_USART_IRQ                  AVR32_USART1_IRQ
#define GPS_USART_IRQ_GROUP            AVR32_USART1_IRQ_GROUP
#define GPS_USART_SYSCLK               SYSCLK_USART1

#define PDCA_CHANNEL_GPS_TX            0
#define PDCA_CHANNEL_GPS_RX            1
#define GPS_USART_PDCA_PID_TX          AVR32_PDCA_PID_USART1_TX
#define GPS_USART_PDCA_PID_RX          AVR32_PDCA_PID_USART1_RX

#define GPS_ENABLE_PIN                 109

/* I2C connection to the MS4525 pitot sensor */
#define MS4525_DEVICE_ADDR             0x28u

#define MS4525_TWI                     (&AVR32_TWIM2)
#define MS4525_TWI_TWD_PIN             70
#define MS4525_TWI_TWD_FUNCTION        4
#define MS4525_TWI_TWCK_PIN            71
#define MS4525_TWI_TWCK_FUNCTION       4
#define MS4525_TWI_SYSCLK              SYSCLK_TWIM2

#define PDCA_CHANNEL_MS4525_TX         6
#define PDCA_CHANNEL_MS4525_RX         7
#define MS4525_TWI_PDCA_PID_TX         AVR32_TWIM2_PDCA_ID_TX
#define MS4525_TWI_PDCA_PID_RX         AVR32_TWIM2_PDCA_ID_RX

#define MS4525_ENABLE_PIN              76

/* I2C connection to the HMC5883 magnetometer */
#define HMC5883_DEVICE_ADDR            0x1Eu

#define HMC5883_TWI                    (&AVR32_TWIM0)
#define HMC5883_TWI_TWD_PIN            66
#define HMC5883_TWI_TWD_FUNCTION       0
#define HMC5883_TWI_TWCK_PIN           67
#define HMC5883_TWI_TWCK_FUNCTION      0
#define HMC5883_TWI_SYSCLK             SYSCLK_TWIM0

#define PDCA_CHANNEL_HMC5883_TX        2
#define PDCA_CHANNEL_HMC5883_RX        3
#define HMC5883_TWI_PDCA_PID_TX        AVR32_TWIM0_PDCA_ID_TX
#define HMC5883_TWI_PDCA_PID_RX        AVR32_TWIM0_PDCA_ID_RX

#define HMC5883_ENABLE_PIN             65

/* I2C connection to the MS5611 barometric pressure sensor */
#define MS5611_DEVICE_ADDR             0x77u /* 0x76 if CS tied to VDD */

#define MS5611_TWI                     (&AVR32_TWIM1)
#define MS5611_TWI_TWD_PIN             68
#define MS5611_TWI_TWD_FUNCTION        0
#define MS5611_TWI_TWCK_PIN            69
#define MS5611_TWI_TWCK_FUNCTION       0
#define MS5611_TWI_SYSCLK              SYSCLK_TWIM1

#define PDCA_CHANNEL_MS5611_TX         4
#define PDCA_CHANNEL_MS5611_RX         5
#define MS5611_TWI_PDCA_PID_TX         AVR32_TWIM1_PDCA_ID_TX
#define MS5611_TWI_PDCA_PID_RX         AVR32_TWIM1_PDCA_ID_RX

#define MS5611_ENABLE_PIN              64

/* SPI connection to the MPU6000 accelerometer/gyroscope */
#define MPU6000_SPI                    (&AVR32_SPI1)
#define MPU6000_SPI_MISO_PIN           52
#define MPU6000_SPI_MISO_FUNCTION      1
#define MPU6000_SPI_MOSI_PIN           51
#define MPU6000_SPI_MOSI_FUNCTION      1
#define MPU6000_SPI_CLK_PIN            53
#define MPU6000_SPI_CLK_FUNCTION       1
#define MPU6000_SPI_CS_PIN             54
#define MPU6000_SPI_CS_FUNCTION        1
#define MPU6000_SPI_SYSCLK             SYSCLK_SPI1

#define PDCA_CHANNEL_MPU6000_TX        14
#define PDCA_CHANNEL_MPU6000_RX        13
#define MPU6000_SPI_PDCA_PID_TX        AVR32_SPI1_PDCA_ID_TX
#define MPU6000_SPI_PDCA_PID_RX        AVR32_SPI1_PDCA_ID_RX

#define MPU6000_ENABLE_PIN             55

/* GPIO and ADC definitions */
#define GPIN_0_PIN                     32
#define GPIN_1_PIN                     33
#define GPIN_2_PIN                     34
#define GPIN_3_PIN                     35

#define GPOUT_0_PIN                    123
#define GPOUT_1_PIN                    124
#define GPOUT_2_PIN                    125
#define GPOUT_3_PIN                    126

#define ADC_PITOT_PIN                  4
#define ADC_PITOT_FUNCTION             0
#define ADC_AUX_PIN                    5
#define ADC_AUX_FUNCTION               0
#define ADC_BATTERY_V_PIN              6
#define ADC_BATTERY_V_FUNCTION         0
#define ADC_BATTERY_I_PIN              7
#define ADC_BATTERY_I_FUNCTION         0

#define GP_ADC                         (&AVR32_ADCIFA)
#define GP_ADC_SYSCLK                  SYSCLK_ADCIFA
#define PDCA_CHANNEL_ADC_RX            8
#define ADC_PDCA_PID_RX                AVR32_PDCA_PID_ADCIFA_CH0_RX

/* PWM pin definitions */
#define PWM_0_PIN                      81
#define PWM_0_FUNCTION                 0
#define PWM_1_PIN                      79
#define PWM_1_FUNCTION                 0
#define PWM_2_PIN                      77
#define PWM_2_FUNCTION                 0
#define PWM_3_PIN                      75
#define PWM_3_FUNCTION                 0
#define PWM_SYSCLK                     SYSCLK_PWM

#define PWM_ENABLE_PIN                 82

/* PWM input pin definitions */

#define PWM_IN_0_PIN                   117
#define PWM_IN_1_PIN                   118
#define PWM_IN_2_PIN                   119
#define PWM_IN_3_PIN                   120

/* CPU board interface */
#define CPU_USART                      (&AVR32_USART0)
#define CPU_USART_TXD_PIN              96
#define CPU_USART_TXD_FUNCTION         4
#define CPU_USART_RXD_PIN              97
#define CPU_USART_RXD_FUNCTION         4
#define CPU_USART_IRQ                  AVR32_USART0_IRQ
#define CPU_USART_IRQ_GROUP            AVR32_USART0_IRQ_GROUP
#define CPU_USART_SYSCLK               SYSCLK_USART0

#define PDCA_CHANNEL_CPU_TX            9
#define PDCA_CHANNEL_CPU_RX            10
#define CPU_USART_PDCA_PID_TX          AVR32_PDCA_PID_USART0_TX
#define CPU_USART_PDCA_PID_RX          AVR32_PDCA_PID_USART0_RX

#define CPU_RESET_PIN                  37

/* AUX USART interface */
#define AUX_USART                      (&AVR32_USART4)
#define AUX_USART_TXD_PIN              103
#define AUX_USART_TXD_FUNCTION         4
#define AUX_USART_RXD_PIN              104
#define AUX_USART_RXD_FUNCTION         4
#define AUX_USART_IRQ                  AVR32_USART4_IRQ
#define AUX_USART_IRQ_GROUP            AVR32_USART4_IRQ_GROUP
#define AUX_USART_SYSCLK               SYSCLK_USART4

#define PDCA_CHANNEL_AUX_TX            11
#define PDCA_CHANNEL_AUX_RX            12
#define AUX_USART_PDCA_PID_TX          AVR32_PDCA_PID_USART4_TX
#define AUX_USART_PDCA_PID_RX          AVR32_PDCA_PID_USART4_RX

#endif
