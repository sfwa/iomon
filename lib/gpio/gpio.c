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

#include "gpio.h"

/** \brief Enables a specific module mode for a pin.
 *
 * \note A pin and pin function index can be found in the device part header
 *       file. The \c AVR32_*_PIN constants map a GPIO number from the device
 *       datasheet to the appropriate pin function, while the corresponding
 *       \c AVR32_*_FUNCTION macro contains the appropriate function index.
 *       \n\n
 *       For example, the constants \c AVR32_PWM_3_PIN and
 *       \c AVR32_PWM_3_FUNCTION contain the pin and function index of the PWM
 *       module, channel 3, for the current device (if available).
 *
 * \param pin The pin number.
 * \param function The pin function.
 *
 * \return \ref GPIO_SUCCESS or \ref GPIO_INVALID_ARGUMENT.
 */
uint32_t gpio_enable_module_pin(uint32_t pin, uint32_t function)
{
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> 5];

	/* Enable the correct function. */
	switch (function) {
	case 0: /* A function. */
		gpio_port->pmr0c = 1 << (pin & 0x1F);
		gpio_port->pmr1c = 1 << (pin & 0x1F);
#if (AVR32_GPIO_H_VERSION >= 210)
		gpio_port->pmr2c = 1 << (pin & 0x1F);
#endif
		break;

	case 1: /* B function. */
		gpio_port->pmr0s = 1 << (pin & 0x1F);
		gpio_port->pmr1c = 1 << (pin & 0x1F);
#if (AVR32_GPIO_H_VERSION >= 210)
		gpio_port->pmr2c = 1 << (pin & 0x1F);
#endif
		break;

	case 2: /* C function. */
		gpio_port->pmr0c = 1 << (pin & 0x1F);
		gpio_port->pmr1s = 1 << (pin & 0x1F);
#if (AVR32_GPIO_H_VERSION >= 210)
		gpio_port->pmr2c = 1 << (pin & 0x1F);
#endif
		break;

	case 3: /* D function. */
		gpio_port->pmr0s = 1 << (pin & 0x1F);
		gpio_port->pmr1s = 1 << (pin & 0x1F);
#if (AVR32_GPIO_H_VERSION >= 210)
		gpio_port->pmr2c = 1 << (pin & 0x1F);
#endif
		break;

#if (AVR32_GPIO_H_VERSION >= 210)
	case 4: /* E function. */
		gpio_port->pmr0c = 1 << (pin & 0x1F);
		gpio_port->pmr1c = 1 << (pin & 0x1F);
		gpio_port->pmr2s = 1 << (pin & 0x1F);
		break;

	case 5: /* F function. */
		gpio_port->pmr0s = 1 << (pin & 0x1F);
		gpio_port->pmr1c = 1 << (pin & 0x1F);
		gpio_port->pmr2s = 1 << (pin & 0x1F);
		break;

	case 6: /* G function. */
		gpio_port->pmr0c = 1 << (pin & 0x1F);
		gpio_port->pmr1s = 1 << (pin & 0x1F);
		gpio_port->pmr2s = 1 << (pin & 0x1F);
		break;

	case 7: /* H function. */
		gpio_port->pmr0s = 1 << (pin & 0x1F);
		gpio_port->pmr1s = 1 << (pin & 0x1F);
		gpio_port->pmr2s = 1 << (pin & 0x1F);
		break;
#endif

	default:
		return GPIO_INVALID_ARGUMENT;
	}

	/* Disable GPIO control. */
	gpio_port->gperc = 1 << (pin & 0x1F);

	return GPIO_SUCCESS;
}

/** \brief Configuration functionality on a pin.
 *
 * \param pin The pin number.
 * \param flags The configuration.
 */
void gpio_configure_pin(uint32_t pin, uint32_t flags)
{
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> 5];

	/* Both pull-up and pull-down set means buskeeper */
#if defined(AVR32_GPIO_200_H_INCLUDED) || defined(AVR32_GPIO_210_H_INCLUDED) ||	\
	defined(AVR32_GPIO_212_H_INCLUDED)
	if (flags & GPIO_PULL_DOWN) {
		gpio_port->pders = 1 << (pin & 0x1F);
	} else {
		gpio_port->pderc = 1 << (pin & 0x1F);
	}

#endif
	if (flags & GPIO_PULL_UP) {
		gpio_port->puers = 1 << (pin & 0x1F);
	} else {
		gpio_port->puerc = 1 << (pin & 0x1F);
	}

	/* Enable open-drain mode if requested */
#if defined(AVR32_GPIO_200_H_INCLUDED) || defined(AVR32_GPIO_210_H_INCLUDED) ||	\
	defined(AVR32_GPIO_212_H_INCLUDED)
	if (flags & GPIO_OPEN_DRAIN) {
		gpio_port->odmers = 1 << (pin & 0x1F);
	} else {
		gpio_port->odmerc = 1 << (pin & 0x1F);
	}

#endif

#if defined(AVR32_GPIO_200_H_INCLUDED) || defined(AVR32_GPIO_210_H_INCLUDED) ||	\
	defined(AVR32_GPIO_212_H_INCLUDED)
	/* Select drive strength */
	if (flags & GPIO_DRIVE_LOW) {
		gpio_port->odcr0s = 1 << (pin & 0x1F);
	} else {
		gpio_port->odcr0c = 1 << (pin & 0x1F);
	}

	if (flags & GPIO_DRIVE_HIGH) {
		gpio_port->odcr1s = 1 << (pin & 0x1F);
	} else {
		gpio_port->odcr1c = 1 << (pin & 0x1F);
	}

#endif

	/* Select interrupt level for group */
	if (flags & GPIO_INTERRUPT) {
		if (flags & GPIO_BOTHEDGES) {
			gpio_port->imr0c = 1 << (pin & 0x1F);
			gpio_port->imr1c = 1 << (pin & 0x1F);
		} else if (flags & GPIO_RISING) {
			gpio_port->imr0s = 1 << (pin & 0x1F);
			gpio_port->imr1c = 1 << (pin & 0x1F);
		} else if (flags & GPIO_FALLING) {
			gpio_port->imr0c = 1 << (pin & 0x1F);
			gpio_port->imr1s = 1 << (pin & 0x1F);
		}
	}

	/* Select direction and initial pin state */
	if (flags & GPIO_DIR_OUTPUT) {
		if (flags & GPIO_INIT_HIGH) {
			gpio_port->ovrs = 1 << (pin & 0x1F);
		} else {
			gpio_port->ovrc = 1 << (pin & 0x1F);
		}

		gpio_port->oders = 1 << (pin & 0x1F);
	} else {
		gpio_port->oderc = 1 << (pin & 0x1F);
	}

	/* Enable GPIO */
	gpio_port->gpers = 1 << (pin & 0x1F);
}

/** \brief Configure the edge detector of an input pin
 *
 * \param pin The pin number.
 * \param mode The edge detection mode (\ref GPIO_PIN_CHANGE,
 *             \ref GPIO_RISING_EDGE or \ref GPIO_FALLING_EDGE).
 *
 * \return \ref GPIO_SUCCESS or \ref GPIO_INVALID_ARGUMENT.
 */
static uint32_t gpio_configure_edge_detector(uint32_t pin, uint32_t mode)
{
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> 5];

	/* Configure the edge detector. */
	switch (mode) {
	case GPIO_PIN_CHANGE:
		gpio_port->imr0c = 1 << (pin & 0x1F);
		gpio_port->imr1c = 1 << (pin & 0x1F);
		break;

	case GPIO_RISING_EDGE:
		gpio_port->imr0s = 1 << (pin & 0x1F);
		gpio_port->imr1c = 1 << (pin & 0x1F);
		break;

	case GPIO_FALLING_EDGE:
		gpio_port->imr0c = 1 << (pin & 0x1F);
		gpio_port->imr1s = 1 << (pin & 0x1F);
		break;

	default:
		return GPIO_INVALID_ARGUMENT;
	}

	return GPIO_SUCCESS;
}

/** \brief Enables the interrupt of a pin with the specified settings.
 *
 * \param pin The pin number.
 * \param mode The trigger mode (\ref GPIO_PIN_CHANGE, \ref GPIO_RISING_EDGE or
 *             \ref GPIO_FALLING_EDGE).
 *
 * \return \ref GPIO_SUCCESS or \ref GPIO_INVALID_ARGUMENT.
 */
uint32_t gpio_enable_pin_interrupt(uint32_t pin, uint32_t mode)
{
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> 5];

	/* Enable the glitch filter. */
	gpio_port->gfers = 1 << (pin & 0x1F);

	/* Configure the edge detector. */
	if (GPIO_INVALID_ARGUMENT == gpio_configure_edge_detector(pin, mode)) {
		return(GPIO_INVALID_ARGUMENT);
	}

	/* Enable interrupt. */
	gpio_port->iers = 1 << (pin & 0x1F);

	return GPIO_SUCCESS;
}

/** @} */
