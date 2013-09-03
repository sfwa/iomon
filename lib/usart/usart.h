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


#ifndef _USART_H_
#define _USART_H_

/**
 * \defgroup group_avr32_drivers_usart USART - Univ. Sync/Async Serial Rec/Trans
 *
 * Driver for the USART (Universal Synchronous Asynchronous Receiver Transmitter).
 * The driver supports the following  modes: RS232, RS485, SPI, LIN and ISO7816.
 *
 * \{
 */

#include <avr32/io.h>
#include "compiler.h"


/*! \name Return Values
 */
#define USART_SUCCESS                 0 /* Successful completion. */
#define USART_FAILURE                -1 /* Failure because of some unspecified reason. */
#define USART_INVALID_INPUT           1 /* Input value out of range. */
#define USART_INVALID_ARGUMENT       -1 /* Argument value out of range. */
#define USART_TX_BUSY                 2 /* Transmitter was busy. */
#define USART_RX_EMPTY                3 /* Nothing was received. */
#define USART_RX_ERROR                4 /* Transmission error occurred. */
#define USART_MODE_FAULT              5 /* USART not in the appropriate mode. */

/* Default time-out value (number of attempts). */
#define USART_DEFAULT_TIMEOUT         10000

/*! \name Parity Settings
 */
#define USART_EVEN_PARITY             AVR32_USART_MR_PAR_EVEN   /* Use even parity on character transmission. */
#define USART_ODD_PARITY              AVR32_USART_MR_PAR_ODD    /* Use odd parity on character transmission. */
#define USART_SPACE_PARITY            AVR32_USART_MR_PAR_SPACE  /* Use a space as parity bit. */
#define USART_MARK_PARITY             AVR32_USART_MR_PAR_MARK   /* Use a mark as parity bit. */
#define USART_NO_PARITY               AVR32_USART_MR_PAR_NONE   /* Don't use a parity bit. */
#define USART_MULTIDROP_PARITY        AVR32_USART_MR_PAR_MULTI  /* Parity bit is used to flag address characters. */

/*! \name Stop Bits Settings
 */
#define USART_1_STOPBIT               AVR32_USART_MR_NBSTOP_1   /* Use 1 stop bit. */
#define USART_1_5_STOPBITS            AVR32_USART_MR_NBSTOP_1_5 /* Use 1.5 stop bits. */
#define USART_2_STOPBITS              AVR32_USART_MR_NBSTOP_2   /* Use 2 stop bits (for more, just give the number of bits). */

/*! \name Channel Modes
 */
#define USART_NORMAL_CHMODE           AVR32_USART_MR_CHMODE_NORMAL      /* Normal communication. */
#define USART_AUTO_ECHO               AVR32_USART_MR_CHMODE_ECHO        /* Echo data. */
#define USART_LOCAL_LOOPBACK          AVR32_USART_MR_CHMODE_LOCAL_LOOP  /* Local loopback. */
#define USART_REMOTE_LOOPBACK         AVR32_USART_MR_CHMODE_REMOTE_LOOP /* Remote loopback. */


/* Input parameters when initializing RS232 and similar modes. */
typedef struct
{
  /* Set baud rate of the USART (unused in slave modes). */
  unsigned long baudrate;

  /* Number of bits to transmit as a character (5 to 9). */
  unsigned char charlength;

  /* How to calculate the parity bit: \ref USART_EVEN_PARITY, \ref USART_ODD_PARITY,
     \ref USART_SPACE_PARITY, \ref USART_MARK_PARITY, \ref USART_NO_PARITY or
     \ref USART_MULTIDROP_PARITY. */
  unsigned char paritytype;

  /* Number of stop bits between two characters: \ref USART_1_STOPBIT,
     \ref USART_1_5_STOPBITS, \ref USART_2_STOPBITS or any number from 3 to 257
     which will result in a time guard period of that length between characters.
     \note \ref USART_1_5_STOPBITS is supported in asynchronous modes only. */
  unsigned short stopbits;

  /* Run the channel in testmode: \ref USART_NORMAL_CHMODE, \ref USART_AUTO_ECHO,
     \ref USART_LOCAL_LOOPBACK or \ref USART_REMOTE_LOOPBACK. */
  unsigned char channelmode;
} usart_options_t;

/*! \name Initialization Functions
 */

/*! \brief Resets the USART and disables TX and RX.
 *
 * \param usart   Base address of the USART instance.
 */
void usart_reset(volatile avr32_usart_t *usart);

/*! \brief Sets up the USART to use the standard RS232 protocol.
 *
 * \param usart   Base address of the USART instance.
 * \param opt     Options needed to set up RS232 communication (see \ref usart_options_t).
 * \param pba_hz  USART module input clock frequency (PBA clock, Hz).
 *
 * \retval USART_SUCCESS        Mode successfully initialized.
 * \retval USART_INVALID_INPUT  One or more of the arguments is out of valid range.
 */
int usart_init_rs232(volatile avr32_usart_t *usart,
    const usart_options_t *opt, long pba_hz);


/*! \name Read and Reset Error Status Bits
 */

/*! \brief Resets the error status.
 *
 * This function resets the status bits indicating that a parity error,
 * framing error or overrun has occurred. The RXBRK bit, indicating
 * a start/end of break condition on the RX line, is also reset.
 *
 * \param usart   Base address of the USART instance.
 */
inline static void usart_reset_status(volatile avr32_usart_t *usart) {
    usart->cr = AVR32_USART_CR_RSTSTA_MASK;
}

/*! \brief Checks if a parity error has occurred since last status reset.
 *
 * \param usart   Base address of the USART instance.
 *
 * \return \c 1 if a parity error has been detected, otherwise \c 0.
 */
inline static int usart_parity_error(volatile avr32_usart_t *usart) {
    return (usart->csr & AVR32_USART_CSR_PARE_MASK) != 0;
}

/*! \brief Checks if a framing error has occurred since last status reset.
 *
 * \param usart   Base address of the USART instance.
 *
 * \return \c 1 if a framing error has been detected, otherwise \c 0.
 */
inline static int usart_framing_error(volatile avr32_usart_t *usart) {
    return (usart->csr & AVR32_USART_CSR_FRAME_MASK) != 0;
}

/*! \brief Checks if an overrun error has occurred since last status reset.
 *
 * \param usart   Base address of the USART instance.
 *
 * \return \c 1 if a overrun error has been detected, otherwise \c 0.
 */
inline static int usart_overrun_error(volatile avr32_usart_t *usart) {
    return (usart->csr & AVR32_USART_CSR_OVRE_MASK) != 0;
}

/*! \name Transmit/Receive Functions
 */

/*! \brief Tests if all requested USART transmissions are over.
 *
 * \param usart   Base address of the USART instance.
 *
 * \return \c 1 if the USART Transmit Shift Register and the USART Transmit
 *         Holding Register are free, otherwise \c 0.
 */
inline static int usart_tx_empty(volatile avr32_usart_t *usart) {
    return (usart->csr & AVR32_USART_CSR_TXEMPTY_MASK) != 0;
}

/*! \brief Tests if the USART contains a received character.
 *
 * \param usart   Base address of the USART instance.
 *
 * \return \c 1 if the USART Receive Holding Register is full, otherwise \c 0.
 */
inline static int usart_test_hit(volatile avr32_usart_t *usart) {
    return (usart->csr & AVR32_USART_CSR_RXRDY_MASK) != 0;
}

#endif
