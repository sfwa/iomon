/*****************************************************************************
*
* \file
*
* \brief GPIO software driver interface for AVR UC3.
*
* Copyright (c) 2010-2012 Atmel Corporation. All rights reserved.
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
*****************************************************************************/

#ifndef _GPIO_H_
#define _GPIO_H_

/**
 * \defgroup group_avr32_drivers_gpio GPIO - General-Purpose Input/Output
 *
 * GPIO gives access to the MCU pins.
 *
 * @{
 */

#include <avr32/io.h>
#include <compiler.h>
#include <board.h>

/** \name Return Values of the GPIO API
 * @{ */
#define GPIO_SUCCESS            0 /**< Function successfully completed. */
#define GPIO_INVALID_ARGUMENT   1 /**< Input parameters are out of range. */
/** @} */

/** \name Interrupt Trigger Modes
 * @{ */
#define GPIO_PIN_CHANGE         0 /**< Interrupt triggered upon pin change. */
#define GPIO_RISING_EDGE        1 /**< Interrupt triggered upon rising edge. */
#define GPIO_FALLING_EDGE       2 /**< Interrupt triggered upon falling edge. */
/** @} */

/** \name Common defines for GPIO_FLAGS parameter
 * @{ */
#define GPIO_DIR_INPUT  (0 << 0) /**< Pin is Input */
#define GPIO_DIR_OUTPUT (1 << 0) /**< Pin is Output */
#define GPIO_INIT_LOW   (0 << 1) /**< Initial Output State is Low */
#define GPIO_INIT_HIGH  (1 << 1) /**< Initial Output State is High */
#define GPIO_PULL_UP    (1 << 2) /**< Pull-Up (when input) */
#define GPIO_PULL_DOWN  (2 << 2) /**< Pull-Down (when input) */
#define GPIO_BUSKEEPER  (3 << 2) /**< Bus Keeper */
#define GPIO_DRIVE_MIN  (0 << 4) /**< Drive Min Configuration */
#define GPIO_DRIVE_LOW  (1 << 4) /**< Drive Low Configuration */
#define GPIO_DRIVE_HIGH (2 << 4) /**< Drive High Configuration */
#define GPIO_DRIVE_MAX  (3 << 4) /**< Drive Max Configuration */
#define GPIO_OPEN_DRAIN (1 << 6) /**< Open-Drain (when output) */
#define GPIO_INTERRUPT  (1 << 7) /**< Enable Pin/Group Interrupt */
#define GPIO_BOTHEDGES  (3 << 7) /**< Sense Both Edges */
#define GPIO_RISING     (5 << 7) /**< Sense Rising Edge */
#define GPIO_FALLING    (7 << 7) /**< Sense Falling Edge */
/** @} */

/** \name Peripheral Bus Interface
 *
 * Low-speed interface with a non-deterministic number of clock cycles per
 * access.
 *
 * This interface operates with lower clock frequencies (fPB <= fCPU), and its
 * timing is not deterministic since it needs to access a shared bus which may
 * be heavily loaded.
 *
 * \note This interface is immediately available without initialization.
 *
 * @{
 */

uint32_t gpio_enable_module_pin(uint32_t pin, uint32_t function);

void gpio_configure_pin(uint32_t pin, uint32_t flags);

uint32_t gpio_enable_pin_interrupt(uint32_t pin, uint32_t mode);

/** @} */

#if (defined AVR32_GPIO_LOCAL_ADDRESS) || defined(__DOXYGEN__)

/** \name Local Bus Interface
 *
 * High-speed interface with only one clock cycle per access.
 *
 * This interface operates with high clock frequency (fCPU), and its timing is
 * deterministic since it does not need to access a shared bus which may be
 * heavily loaded.
 *
 * \warning To use this interface, the clock frequency of the peripheral bus on
 *          which the GPIO peripheral is connected must be set to the CPU clock
 *          frequency (fPB = fCPU).
 *
 * \note This interface has to be initialized in order to be available.
 *
 * @{
 */

/** \brief Enables the local bus interface for GPIO.
 *
 * \note This function must have been called at least once before using other
 *       functions in this interface.
 */
inline static void gpio_local_init(void)
{
	Set_system_register(AVR32_CPUCR,
			Get_system_register(AVR32_CPUCR) | AVR32_CPUCR_LOCEN_MASK);
}

/** \brief Returns the value of a pin.
 *
 * \param pin The pin number.
 *
 * \return The pin value.
 *
 * \note \ref gpio_local_init must have been called beforehand.
 */
inline static bool gpio_local_get_pin_value(uint32_t pin)
{
	return (AVR32_GPIO_LOCAL.port[pin >> 5].pvr >> (pin & 0x1F)) & 1;
}

/** \brief Drives a GPIO pin to 1.
 *
 * \param pin The pin number.
 *
 * \note \ref gpio_local_init must have been called beforehand.
 *
 * \note This function does not enable the GPIO mode of the pin nor its output
 *       driver. \ref gpio_enable_gpio_pin and
 *       \ref gpio_local_enable_pin_output_driver can be called for this
 *       purpose.
 */
inline static void gpio_local_set_gpio_pin(uint32_t pin)
{
	AVR32_GPIO_LOCAL.port[pin >> 5].ovrs = 1 << (pin & 0x1F);
}

/** \brief Drives a GPIO pin to 0.
 *
 * \param pin The pin number.
 *
 * \note \ref gpio_local_init must have been called beforehand.
 *
 * \note This function does not enable the GPIO mode of the pin nor its output
 *       driver. \ref gpio_enable_gpio_pin and
 *       \ref gpio_local_enable_pin_output_driver can be called for this
 *       purpose.
 */
inline static void gpio_local_clr_gpio_pin(uint32_t pin)
{
	AVR32_GPIO_LOCAL.port[pin >> 5].ovrc = 1 << (pin & 0x1F);
}

/** \brief Toggles a GPIO pin.
 *
 * \param pin The pin number.
 *
 * \note \ref gpio_local_init must have been called beforehand.
 *
 * \note This function does not enable the GPIO mode of the pin nor its output
 *       driver. \ref gpio_enable_gpio_pin and
 *       \ref gpio_local_enable_pin_output_driver can be called for this
 *       purpose.
 */
inline static void gpio_local_tgl_gpio_pin(uint32_t pin)
{
	AVR32_GPIO_LOCAL.port[pin >> 5].ovrt = 1 << (pin & 0x1F);
}

/** @} */
#endif /* AVR32_GPIO_LOCAL_ADDRESS */


/** @} */

#endif  /* _GPIO_H_ */
