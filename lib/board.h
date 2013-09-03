/**
 * \file
 *
 * \brief Standard board header file.
 *
 * This file includes the appropriate board header file according to the
 * defined board (parameter BOARD).
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
 */

#ifndef _BOARD_H_
#define _BOARD_H_

/**
 * \defgroup group_common_boards Generic board support
 *
 * The generic board support module includes board-specific definitions
 * and function prototypes, such as the board initialization function.
 *
 * \{
 */

#include <compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup part_macros_group Atmel part identification macros
 *
 * This collection of macros identify which series and families that the various
 * Atmel parts belong to. These can be used to select part-dependent sections of
 * code at compile time.
 *
 * @{
 */

/**
 * \name Convenience macros for part checking
 * @{
 */

/* ! Check GCC and IAR part definition for 32-bit AVR */
#define AVR32_PART_IS_DEFINED(part) \
    (defined(__AT32 ## part ## __) || defined(__AVR32_ ## part ## __))

/** @} */

/**
 * \defgroup uc3_part_macros_group AVR UC3 parts
 * @{
 */

/**
 * \name AVR UC3 C series
 * @{
 */
#define UC3C0 ( \
        AVR32_PART_IS_DEFINED(UC3C064C)  || \
        AVR32_PART_IS_DEFINED(UC3C0128C) || \
        AVR32_PART_IS_DEFINED(UC3C0256C) || \
        AVR32_PART_IS_DEFINED(UC3C0512C) \
        )

#define UC3C1 ( \
        AVR32_PART_IS_DEFINED(UC3C164C)  || \
        AVR32_PART_IS_DEFINED(UC3C1128C) || \
        AVR32_PART_IS_DEFINED(UC3C1256C) || \
        AVR32_PART_IS_DEFINED(UC3C1512C) \
        )

#define UC3C2 ( \
        AVR32_PART_IS_DEFINED(UC3C264C)  || \
        AVR32_PART_IS_DEFINED(UC3C2128C) || \
        AVR32_PART_IS_DEFINED(UC3C2256C) || \
        AVR32_PART_IS_DEFINED(UC3C2512C) \
        )
/** @} */

#define UC3L 0
#define UC3D 0

/**
 * \name AVR UC3 families
 * @{
 */

/** AVR UC3 C family */
#define UC3C (UC3C0 || UC3C1 || UC3C2)

/** @} */

/** AVR UC3 product line */
#define UC3  (UC3A || UC3B || UC3C || UC3D || UC3L)

/** @} */

/** @} */

/*! \name Base Boards
 */
#define STK600_RCUC3C0        17

#include "conf_board.h"


#ifdef __cplusplus
}
#endif

/**
 * \}
 */

#endif
