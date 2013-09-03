/*****************************************************************************
 *
 * \file
 *
 * \brief USART driver for AVR32 UC3.
 *
 * This file contains basic functions for the AVR32 USART, with support for all
 * modes, settings and clock speeds.
 *
 * Copyright (c) 2009-2012 Atmel Corporation. All rights reserved.
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


#include "compiler.h"
#include "usart.h"


/*! \brief Calculates a clock divider (\e CD) and a fractional part (\e FP) for
 *         the USART asynchronous modes to generate a baud rate as close as
 *         possible to the baud rate set point.
 *
 * Baud rate calculation:
 * \f$ Baudrate = \frac{SelectedClock}{Over \times (CD + \frac{FP}{8})} \f$, \e Over being 16 or 8.
 * The maximal oversampling is selected if it allows to generate a baud rate close to the set point.
 *
 * \param usart     Base address of the USART instance.
 * \param baudrate  Baud rate set point.
 * \param pba_hz    USART module input clock frequency (PBA clock, Hz).
 *
 * \retval USART_SUCCESS        Baud rate successfully initialized.
 * \retval USART_INVALID_INPUT  Baud rate set point is out of range for the given input clock frequency.
 */
static int usart_set_async_baudrate(volatile avr32_usart_t *usart,
        unsigned int baudrate, unsigned long pba_hz) {
    unsigned int over = (pba_hz >= 16 * baudrate) ? 16 : 8;
    unsigned int cd_fp = ((1 << AVR32_USART_BRGR_FP_SIZE) *
        pba_hz + (over * baudrate) / 2) / (over * baudrate);
    unsigned int cd = cd_fp >> AVR32_USART_BRGR_FP_SIZE;
    unsigned int fp = cd_fp & ((1 << AVR32_USART_BRGR_FP_SIZE) - 1);

    if (cd < 1 || cd > (1 << AVR32_USART_BRGR_CD_SIZE) - 1) {
        return USART_INVALID_INPUT;
    }

    usart->mr = (usart->mr & ~(AVR32_USART_MR_USCLKS_MASK |
        AVR32_USART_MR_SYNC_MASK | AVR32_USART_MR_OVER_MASK)) |
        AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS_OFFSET |
        ((over == 16) ? AVR32_USART_MR_OVER_X16 : AVR32_USART_MR_OVER_X8)
        << AVR32_USART_MR_OVER_OFFSET;

    usart->brgr = cd << AVR32_USART_BRGR_CD_OFFSET |
        fp << AVR32_USART_BRGR_FP_OFFSET;

    return USART_SUCCESS;
}

void usart_reset(volatile avr32_usart_t *usart) {
    bool global_interrupt_enabled = cpu_irq_is_enabled();

    /* Disable all USART interrupts.
       Interrupts needed should be set explicitly on every reset. */
    if (global_interrupt_enabled) {
        cpu_irq_disable();
    }
    usart->idr = 0xFFFFFFFF;
    usart->csr;
    if (global_interrupt_enabled) {
        cpu_irq_enable();
    }

    /* Reset mode and other registers that could cause unpredictable behavior after reset. */
    usart->mr = 0;
    usart->rtor = 0;
    usart->ttgr = 0;

    /* Shutdown TX and RX (will be re-enabled when setup has successfully completed),
       reset status bits and turn off DTR and RTS. */
    usart->cr = AVR32_USART_CR_RSTRX_MASK | AVR32_USART_CR_RSTTX_MASK |
        AVR32_USART_CR_RSTSTA_MASK | AVR32_USART_CR_RSTIT_MASK |
        AVR32_USART_CR_RSTNACK_MASK | AVR32_USART_CR_RTSDIS_MASK;
}


int usart_init_rs232(volatile avr32_usart_t *usart,
        const usart_options_t *opt, long pba_hz) {
    /* Reset the USART and shutdown TX and RX. */
    usart_reset(usart);

    /* Check input values. */
    if (!opt || opt->charlength < 5 || opt->charlength > 9 ||
            opt->paritytype > 7 || opt->stopbits > 2 + 255 || opt->channelmode > 3 ||
            usart_set_async_baudrate(usart, opt->baudrate, pba_hz) == USART_INVALID_INPUT) {
        return USART_INVALID_INPUT;
    }

    if (opt->charlength == 9) {
        /* Character length set to 9 bits. MODE9 dominates CHRL. */
        usart->mr |= AVR32_USART_MR_MODE9_MASK;
    } else {
        /* CHRL gives the character length (- 5) when MODE9 = 0. */
        usart->mr |= (opt->charlength - 5) << AVR32_USART_MR_CHRL_OFFSET;
    }

    usart->mr |= opt->paritytype << AVR32_USART_MR_PAR_OFFSET |
        opt->channelmode << AVR32_USART_MR_CHMODE_OFFSET;

    if (opt->stopbits > USART_2_STOPBITS) {
        /* Set two stop bits */
        usart->mr |= AVR32_USART_MR_NBSTOP_2 << AVR32_USART_MR_NBSTOP_OFFSET;
        /* and a timeguard period gives the rest. */
        usart->ttgr = opt->stopbits - USART_2_STOPBITS;
    } else {
        /* Insert 1, 1.5 or 2 stop bits. */
        usart->mr |= opt->stopbits << AVR32_USART_MR_NBSTOP_OFFSET;
    }

    /* Set normal mode. */
    usart->mr = (usart->mr & ~AVR32_USART_MR_MODE_MASK) |
        AVR32_USART_MR_MODE_NORMAL << AVR32_USART_MR_MODE_OFFSET;

    /* Setup complete; enable communication.
       Enable input and output. */
    usart->cr = AVR32_USART_CR_RXEN_MASK | AVR32_USART_CR_TXEN_MASK;

    return USART_SUCCESS;
}
