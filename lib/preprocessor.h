/*****************************************************************************
 *
 * \file
 *
 * \brief Preprocessor utils.
 *
 * Copyright (c) 2009 Atmel Corporation. All rights reserved.
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


#ifndef _PREPROCESSOR_H_
#define _PREPROCESSOR_H_

/*! \name Token Paste
 *
 * Paste N preprocessing tokens together, these tokens being allowed to be \#defined.
 *
 * May be used only within macros with the tokens passed as arguments if the tokens are \#defined.
 *
 * For example, writing TPASTE2(U, WIDTH) within a macro \#defined by
 * UTYPE(WIDTH) and invoked as UTYPE(UL_WIDTH) with UL_WIDTH \#defined as 32 is
 * equivalent to writing U32.
 */
#define TPASTE2( a, b)                            a##b
#define TPASTE3( a, b, c)                         a##b##c
#define TPASTE4( a, b, c, d)                      a##b##c##d
#define TPASTE5( a, b, c, d, e)                   a##b##c##d##e

/*! \name Absolute Token Paste
 *
 * Paste N preprocessing tokens together, these tokens being allowed to be \#defined.
 *
 * No restriction of use if the tokens are \#defined.
 *
 * For example, writing ATPASTE2(U, UL_WIDTH) anywhere with UL_WIDTH \#defined
 * as 32 is equivalent to writing U32.
 */
#define ATPASTE2( a, b)                           TPASTE2( a, b)


/* Maximal number of repetitions supported by MREPEAT. */
#define MREPEAT_LIMIT   64

/*! \brief Macro repeat.
 *
 * This macro represents a horizontal repetition construct.
 *
 * \param count  The number of repetitious calls to macro. Valid values range from 0 to MREPEAT_LIMIT.
 * \param macro  A binary operation of the form macro(n, data). This macro is expanded by MREPEAT with
 *               the current repetition number and the auxiliary data argument.
 * \param data   Auxiliary data passed to macro.
 *
 * \return       <tt>macro(0, data) macro(1, data) ... macro(count - 1, data)</tt>
 */
#define MREPEAT(count, macro, data)         TPASTE2(MREPEAT, count)(macro, data)

#define MREPEAT0(  macro, data)
#define MREPEAT1(  macro, data)       MREPEAT0(  macro, data)   macro(  0, data)
#define MREPEAT2(  macro, data)       MREPEAT1(  macro, data)   macro(  1, data)
#define MREPEAT3(  macro, data)       MREPEAT2(  macro, data)   macro(  2, data)
#define MREPEAT4(  macro, data)       MREPEAT3(  macro, data)   macro(  3, data)
#define MREPEAT5(  macro, data)       MREPEAT4(  macro, data)   macro(  4, data)
#define MREPEAT6(  macro, data)       MREPEAT5(  macro, data)   macro(  5, data)
#define MREPEAT7(  macro, data)       MREPEAT6(  macro, data)   macro(  6, data)
#define MREPEAT8(  macro, data)       MREPEAT7(  macro, data)   macro(  7, data)
#define MREPEAT9(  macro, data)       MREPEAT8(  macro, data)   macro(  8, data)
#define MREPEAT10( macro, data)       MREPEAT9(  macro, data)   macro(  9, data)
#define MREPEAT11( macro, data)       MREPEAT10( macro, data)   macro( 10, data)
#define MREPEAT12( macro, data)       MREPEAT11( macro, data)   macro( 11, data)
#define MREPEAT13( macro, data)       MREPEAT12( macro, data)   macro( 12, data)
#define MREPEAT14( macro, data)       MREPEAT13( macro, data)   macro( 13, data)
#define MREPEAT15( macro, data)       MREPEAT14( macro, data)   macro( 14, data)
#define MREPEAT16( macro, data)       MREPEAT15( macro, data)   macro( 15, data)
#define MREPEAT17( macro, data)       MREPEAT16( macro, data)   macro( 16, data)
#define MREPEAT18( macro, data)       MREPEAT17( macro, data)   macro( 17, data)
#define MREPEAT19( macro, data)       MREPEAT18( macro, data)   macro( 18, data)
#define MREPEAT20( macro, data)       MREPEAT19( macro, data)   macro( 19, data)
#define MREPEAT21( macro, data)       MREPEAT20( macro, data)   macro( 20, data)
#define MREPEAT22( macro, data)       MREPEAT21( macro, data)   macro( 21, data)
#define MREPEAT23( macro, data)       MREPEAT22( macro, data)   macro( 22, data)
#define MREPEAT24( macro, data)       MREPEAT23( macro, data)   macro( 23, data)
#define MREPEAT25( macro, data)       MREPEAT24( macro, data)   macro( 24, data)
#define MREPEAT26( macro, data)       MREPEAT25( macro, data)   macro( 25, data)
#define MREPEAT27( macro, data)       MREPEAT26( macro, data)   macro( 26, data)
#define MREPEAT28( macro, data)       MREPEAT27( macro, data)   macro( 27, data)
#define MREPEAT29( macro, data)       MREPEAT28( macro, data)   macro( 28, data)
#define MREPEAT30( macro, data)       MREPEAT29( macro, data)   macro( 29, data)
#define MREPEAT31( macro, data)       MREPEAT30( macro, data)   macro( 30, data)
#define MREPEAT32( macro, data)       MREPEAT31( macro, data)   macro( 31, data)
#define MREPEAT33( macro, data)       MREPEAT32( macro, data)   macro( 32, data)
#define MREPEAT34( macro, data)       MREPEAT33( macro, data)   macro( 33, data)
#define MREPEAT35( macro, data)       MREPEAT34( macro, data)   macro( 34, data)
#define MREPEAT36( macro, data)       MREPEAT35( macro, data)   macro( 35, data)
#define MREPEAT37( macro, data)       MREPEAT36( macro, data)   macro( 36, data)
#define MREPEAT38( macro, data)       MREPEAT37( macro, data)   macro( 37, data)
#define MREPEAT39( macro, data)       MREPEAT38( macro, data)   macro( 38, data)
#define MREPEAT40( macro, data)       MREPEAT39( macro, data)   macro( 39, data)
#define MREPEAT41( macro, data)       MREPEAT40( macro, data)   macro( 40, data)
#define MREPEAT42( macro, data)       MREPEAT41( macro, data)   macro( 41, data)
#define MREPEAT43( macro, data)       MREPEAT42( macro, data)   macro( 42, data)
#define MREPEAT44( macro, data)       MREPEAT43( macro, data)   macro( 43, data)
#define MREPEAT45( macro, data)       MREPEAT44( macro, data)   macro( 44, data)
#define MREPEAT46( macro, data)       MREPEAT45( macro, data)   macro( 45, data)
#define MREPEAT47( macro, data)       MREPEAT46( macro, data)   macro( 46, data)
#define MREPEAT48( macro, data)       MREPEAT47( macro, data)   macro( 47, data)
#define MREPEAT49( macro, data)       MREPEAT48( macro, data)   macro( 48, data)
#define MREPEAT50( macro, data)       MREPEAT49( macro, data)   macro( 49, data)
#define MREPEAT51( macro, data)       MREPEAT50( macro, data)   macro( 50, data)
#define MREPEAT52( macro, data)       MREPEAT51( macro, data)   macro( 51, data)
#define MREPEAT53( macro, data)       MREPEAT52( macro, data)   macro( 52, data)
#define MREPEAT54( macro, data)       MREPEAT53( macro, data)   macro( 53, data)
#define MREPEAT55( macro, data)       MREPEAT54( macro, data)   macro( 54, data)
#define MREPEAT56( macro, data)       MREPEAT55( macro, data)   macro( 55, data)
#define MREPEAT57( macro, data)       MREPEAT56( macro, data)   macro( 56, data)
#define MREPEAT58( macro, data)       MREPEAT57( macro, data)   macro( 57, data)
#define MREPEAT59( macro, data)       MREPEAT58( macro, data)   macro( 58, data)
#define MREPEAT60( macro, data)       MREPEAT59( macro, data)   macro( 59, data)
#define MREPEAT61( macro, data)       MREPEAT60( macro, data)   macro( 60, data)
#define MREPEAT62( macro, data)       MREPEAT61( macro, data)   macro( 61, data)
#define MREPEAT63( macro, data)       MREPEAT62( macro, data)   macro( 62, data)
#define MREPEAT64( macro, data)       MREPEAT63( macro, data)   macro( 63, data)
/**
 * \}
 */


#endif
