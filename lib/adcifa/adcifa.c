/*****************************************************************************
*
* \file
*
* \brief ADCIFA driver for AVR UC3.
*
* This file defines a useful set of functions for ADC on AVR UC3 devices.
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
*****************************************************************************/

#include <avr32/io.h>
#include "fcsassert.h"
#include "compiler.h"
#include "adcifa.h"

/** \brief Get ADCIFA Calibration Data.
 *
 * Mandatory to call if factory calibration data are wanted to be used.
 * If not called, Calibration Data should be set by the application.
 *
 * \param adcifa       Base address of the ADCIFA
 * \param p_adcifa_opt Structure for the ADCIFA core configuration
 */
void adcifa_get_calibration_data(volatile avr32_adcifa_t *adcifa,
		adcifa_opt_t *p_adcifa_opt)
{
	/* Get Offset Calibration */
	int32_t adc_ocal
		= ((*(volatile signed int *)(AVR32_FLASHC_FACTORY_PAGE_ADDRESS
			+ AVR32_FLASHC_FROW_OCAL_WORD)) &
			AVR32_FLASHC_FROW_OCAL_MASK) >>
			AVR32_FLASHC_FROW_OCAL_OFFSET;

	/* Get Gain Calibration */
	int32_t adc_gcal
		= ((*(volatile signed int *)(AVR32_FLASHC_FACTORY_PAGE_ADDRESS
			+ AVR32_FLASHC_FROW_GCAL_WORD)) &
			AVR32_FLASHC_FROW_GCAL_MASK) >>
			AVR32_FLASHC_FROW_GCAL_OFFSET;

	/*  Get S/H Calibration */
	int32_t adc_gain0
		= ((*(volatile signed int *)(AVR32_FLASHC_FACTORY_PAGE_ADDRESS
			+ AVR32_FLASHC_FROW_GAIN0_WORD)) &
			AVR32_FLASHC_FROW_GAIN0_MASK) >>
			AVR32_FLASHC_FROW_GAIN0_OFFSET;

	int32_t adc_gain1
		= ((*(volatile signed int *)(AVR32_FLASHC_FACTORY_PAGE_ADDRESS
			+ AVR32_FLASHC_FROW_GAIN1_WORD)) &
			AVR32_FLASHC_FROW_GAIN1_MASK) >>
			AVR32_FLASHC_FROW_GAIN1_OFFSET;

	/* Get ADC Offset Calibration */
	p_adcifa_opt->offset_calibration_value = adc_ocal;

	/* Get ADC Gain Calibration */
	p_adcifa_opt->gain_calibration_value = adc_gcal;

	/* Get Sample & Hold Gain Calibration for Seq 0 */
	p_adcifa_opt->sh0_calibration_value = adc_gain0;

	/* Get Sample & Hold Gain Calibration for Seq 1 */
	p_adcifa_opt->sh1_calibration_value = adc_gain1;
}

/** \brief Configure ADCIFA. Mandatory to call.
 *
 * If not called, ADC channels will have side effects
 *
 * \param adcifa       Base address of the ADCIFA
 * \param p_adcifa_opt Structure for the ADCIFA core configuration
 * \param pb_hz        Peripheral Bus frequency
 *
 * \return ADCIFA_CONFIGURATION_REFUSED or ADCIFA_CONFIGURATION_ACCEPTED
 */
uint8_t adcifa_configure(volatile avr32_adcifa_t *adcifa,
		adcifa_opt_t *p_adcifa_opt,
		uint32_t pb_hz)
{
	/* Sanity Check */
	fcs_assert(adcifa != NULL);

	/* Set the ADC configuration */
	AVR32_ADCIFA.cfg
		= (p_adcifa_opt->sleep_mode_enable <<
			AVR32_ADCIFA_CFG_SLEEP)
			| (p_adcifa_opt->single_sequencer_mode <<
			AVR32_ADCIFA_CFG_SSMQ)
			| (p_adcifa_opt->free_running_mode_enable <<
			AVR32_ADCIFA_CFG_FRM)
			| (p_adcifa_opt->reference_source <<
			AVR32_ADCIFA_CFG_RS)
			| (p_adcifa_opt->sample_and_hold_disable <<
			AVR32_ADCIFA_CFG_SHD)
			| (((ADCIFA_START_UP_TIME *
			(p_adcifa_opt->frequency /
			1000)) / 32000) << AVR32_ADCIFA_CFG_SUT);

	/* Configure Clock  (rounded up) */
	adcifa->ckdiv
		= (((pb_hz /
			(2 * p_adcifa_opt->frequency)) - 1) <<
			AVR32_ADCIFA_CKDIV_CNT_OFFSET) &
			AVR32_ADCIFA_CKDIV_CNT_MASK;

	/* Set ADC Offset Calibration */
	ADCIFA_set_offset_calibration(p_adcifa_opt->offset_calibration_value);

	/* Set ADC Gain Calibration */
	ADCIFA_set_gain_calibration(p_adcifa_opt->gain_calibration_value);

	/* Set Sample & Hold Gain Calibration for seq 0 */
	ADCIFA_set_sh0_gain_calibration(p_adcifa_opt->sh0_calibration_value);

	/* Set Sample & Hold Gain Calibration for seq 1 */
	ADCIFA_set_sh1_gain_calibration(p_adcifa_opt->sh1_calibration_value);

	/* Enable ADCIFA */
	ADCIFA_enable();

	/* Wait Startup Time */
	while (1) {
		if (ADCIFA_is_startup_time()) {
			break;
		}
	}

	return ADCIFA_CONFIGURATION_ACCEPTED;
}

/** \brief Configure ADCIFA specific sequencer.
 *         - Sequence, Gain and Mux
 * \param adcifa                  Base address of the ADCIFA
 * \param sequencer               Sequencer index
 * \param p_adcifa_sequencer_opt  Structure for the sequencer configuration
 * \param p_adcifa_sequencer_conversion_opt  Pointer on a buffer for each
 *                                           conversion on a sequencer
 * \return ADCIFA_CONFIGURATION_REFUSED or ADCIFA_CONFIGURATION_ACCEPTED
 */
uint8_t adcifa_configure_sequencer(volatile avr32_adcifa_t *adcifa,
		uint8_t sequencer,
		adcifa_sequencer_opt_t *p_adcifa_sequencer_opt,
		adcifa_sequencer_conversion_opt_t *p_adcifa_sequencer_conversion_opt)
{
	uint8_t g[8] = {0};
	uint8_t mp[8] = {0};
	uint8_t mn[8] = {0};
	uint8_t i;

	/* Sanity Check */
	fcs_assert( adcifa != NULL );

	/* Switch case with sequencer */
	switch (sequencer) {
	/* Sequencer 0 */
	case 0:
		/* Configure Sequencer 0 */
		ADCIFA_configure_sequencer_0(
				(p_adcifa_sequencer_opt->convnb - 1),
				(p_adcifa_sequencer_opt->resolution),
				(p_adcifa_sequencer_opt->trigger_selection),
				(p_adcifa_sequencer_opt->start_of_conversion),
#ifdef AVR32_ADCIFA_100_H_INCLUDED
				(p_adcifa_sequencer_opt->oversampling),
#else
				(p_adcifa_sequencer_opt->sh_mode),
#endif
				(p_adcifa_sequencer_opt->half_word_adjustment),
				(p_adcifa_sequencer_opt->software_acknowledge));
		/* Configure Gain for Sequencer 0 */
		for (i = 0; i < p_adcifa_sequencer_opt->convnb; i++) {
			g[i] = p_adcifa_sequencer_conversion_opt[i].gain;
		}
		ADCIFA_configure_sh0gain(g[7], g[6], g[5], g[4], g[3], g[2],
				g[1], g[0]);

		/* Configure Mux for Sequencer 0 */
		for (i = 0; i < p_adcifa_sequencer_opt->convnb; i++) {
			mp[i] = p_adcifa_sequencer_conversion_opt[i].channel_p;
			mn[i] = p_adcifa_sequencer_conversion_opt[i].channel_n;
		}
		ADCIFA_configure_muxsel0p(mp[7], mp[6], mp[5], mp[4], mp[3],
				mp[2], mp[1], mp[0]);
		ADCIFA_configure_muxsel0n(mn[7], mn[6], mn[5], mn[4], mn[3],
				mn[2], mn[1], mn[0]);
		break;

	/* Sequencer 1 */
	case 1:
		/* Configure Sequencer 1 */
		ADCIFA_configure_sequencer_1(
				(p_adcifa_sequencer_opt->convnb - 1),
				(p_adcifa_sequencer_opt->resolution),
				(p_adcifa_sequencer_opt->trigger_selection),
				(p_adcifa_sequencer_opt->start_of_conversion),
#ifdef AVR32_ADCIFA_100_H_INCLUDED
				(p_adcifa_sequencer_opt->oversampling),
#else
				(p_adcifa_sequencer_opt->sh_mode),
#endif
				(p_adcifa_sequencer_opt->half_word_adjustment),
				(p_adcifa_sequencer_opt->software_acknowledge));

		/* Configure Gain for Sequencer 1 */
		for (i = 0; i < p_adcifa_sequencer_opt->convnb; i++) {
			g[i] = p_adcifa_sequencer_conversion_opt[i].gain;
		}
		ADCIFA_configure_sh1gain(g[7], g[6], g[5], g[4], g[3], g[2],
				g[1], g[0]);

		/* Configure Mux for Sequencer 1 */
		for (i = 0; i < p_adcifa_sequencer_opt->convnb; i++) {
			mp[i] = p_adcifa_sequencer_conversion_opt[i].channel_p;
			mn[i] = p_adcifa_sequencer_conversion_opt[i].channel_n;
		}
		ADCIFA_configure_muxsel1p(mp[7], mp[6], mp[5], mp[4], mp[3],
				mp[2], mp[1], mp[0]);
		ADCIFA_configure_muxsel1n(mn[7], mn[6], mn[5], mn[4], mn[3],
				mn[2], mn[1], mn[0]);
		break;

	default:
		break;
	}
	return ADCIFA_CONFIGURATION_ACCEPTED;
}
