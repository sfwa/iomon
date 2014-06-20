/**
 * \file
 *
 * \brief PLL management
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
 */
#ifndef CLK_PLL_H_INCLUDED
#define CLK_PLL_H_INCLUDED

#include "conf_clock.h"
#include <board.h>
#include <osc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bugzilla #11801 */
#ifdef AVR32_SCIF_PLL_VCO_RANGE0_MAX_FREQ
#undef AVR32_SCIF_PLL_VCO_RANGE0_MAX_FREQ
#endif
#define AVR32_SCIF_PLL_VCO_RANGE0_MAX_FREQ   240000000
#ifdef AVR32_SCIF_PLL_VCO_RANGE0_MIN_FREQ
#undef AVR32_SCIF_PLL_VCO_RANGE0_MIN_FREQ
#endif
#define AVR32_SCIF_PLL_VCO_RANGE0_MIN_FREQ   160000000
#ifdef AVR32_SCIF_PLL_VCO_RANGE1_MAX_FREQ
#undef AVR32_SCIF_PLL_VCO_RANGE1_MAX_FREQ
#endif
#define AVR32_SCIF_PLL_VCO_RANGE1_MAX_FREQ   180000000
#ifdef AVR32_SCIF_PLL_VCO_RANGE1_MIN_FREQ
#undef AVR32_SCIF_PLL_VCO_RANGE1_MIN_FREQ
#endif
#define AVR32_SCIF_PLL_VCO_RANGE1_MIN_FREQ   80000000
/**
 * \weakgroup pll_group
 * @{
 */

#define PLL_MAX_STARTUP_CYCLES    ((1 << AVR32_SCIF_PLL_PLLCOUNT_SIZE) - 1)
#define NR_PLLS                   2

/**
 * \brief Number of milliseconds to wait for PLL lock
 */
#define PLL_TIMEOUT_MS    \
	div_ceil(1000 * (PLL_MAX_STARTUP_CYCLES * 2), OSC_RCSYS_MIN_HZ)

/**
 * \note The PLL must run at twice this frequency internally, but the
 * output frequency may be divided by two by setting the PLLOPT[1] bit.
 */
#define PLL_MIN_HZ                40000000
#define PLL_MAX_HZ                240000000

/* \name Chip-specific PLL options */
/* VCO frequency range is 80-180 MHz (160-240 MHz if unset). */
#define PLL_OPT_VCO_RANGE_LOW     0
/* Divide output frequency by two */
#define PLL_OPT_OUTPUT_DIV        1
/* Disable wide-bandwidth mode */
#define PLL_OPT_WBM_DISABLE       2
/* Number of PLL options */
#define PLL_NR_OPTIONS            AVR32_SCIF_PLL_PLLOPT_SIZE
/* The threshold under which to set the #PLL_OPT_VCO_RANGE_LOW option */
#define PLL_VCO_LOW_THRESHOLD     ((AVR32_SCIF_PLL_VCO_RANGE0_MIN_FREQ         \
		+ AVR32_SCIF_PLL_VCO_RANGE1_MAX_FREQ) / 2)

#ifndef __ASSEMBLY__

#include <avr32/io.h>
#include <stdbool.h>
#include <stdint.h>

#define PLL_SRC_OSC0            0    /* Oscillator 0 */
#define PLL_SRC_OSC1            1    /* Oscillator 1 */
#define PLL_SRC_RC8M            2    /* 8MHz/1MHz RC oscillator */

enum pll_source {
	pll_src_osc0            = PLL_SRC_OSC0,
	pll_src_osc1            = PLL_SRC_OSC1,
	pll_src_rc8m            = PLL_SRC_RC8M,
	PLL_NR_SOURCES,                 /* Number of PLL sources */
};

struct pll_config {
	uint32_t ctrl;
};

static inline void pll_config_set_option(struct pll_config *cfg,
unsigned int option) {
       Assert(option < PLL_NR_OPTIONS);

       cfg->ctrl |= 1U << (AVR32_SCIF_PLLOPT + option);
}

/**
 * The PLL options #PLL_OPT_VCO_RANGE_LOW and #PLL_OPT_OUTPUT_DIV will
 * be set automatically based on the calculated target frequency.
 */
static inline void pll_config_init(struct pll_config *cfg,
enum pll_source src, unsigned int div, unsigned int mul) {
	uint32_t vco_hz;

	Assert(src < PLL_NR_SOURCES);

	/* Calculate internal VCO frequency */
	vco_hz = osc_get_rate(src) * mul;
	vco_hz /= div;
	Assert(vco_hz >= PLL_MIN_HZ);
	Assert(vco_hz <= PLL_MAX_HZ);

	cfg->ctrl = 0;

	/* Bring the internal VCO frequency up to the minimum value */
	if ((vco_hz < PLL_MIN_HZ * 2) && (mul <= 8)) {
		mul *= 2;
		vco_hz *= 2;
		pll_config_set_option(cfg, PLL_OPT_OUTPUT_DIV);
	}

	/* Set VCO frequency range according to calculated value */
	if (vco_hz < PLL_VCO_LOW_THRESHOLD)
		pll_config_set_option(cfg, PLL_OPT_VCO_RANGE_LOW);

	Assert(mul > 2 && mul <= 16);
	Assert(div > 0 && div <= 15);

	cfg->ctrl |= ((mul - 1) << AVR32_SCIF_PLLMUL)
		| (div << AVR32_SCIF_PLLDIV)
		| (PLL_MAX_STARTUP_CYCLES << AVR32_SCIF_PLLCOUNT)
		| (src << AVR32_SCIF_PLLOSC);
}

static inline void pll_enable(const struct pll_config *cfg,
unsigned int pll_id) {
	Assert(pll_id < NR_PLLS);

	AVR32_SCIF.unlock = 0xaa000000 | (AVR32_SCIF_PLL + (4 * pll_id));
	AVR32_SCIF.pll[pll_id] = cfg->ctrl | (1U << AVR32_SCIF_PLLEN);
}

static inline void pll_disable(unsigned int pll_id) {
	Assert(pll_id < NR_PLLS);

	AVR32_SCIF.unlock = 0xaa000000 | (AVR32_SCIF_PLL + (4 * pll_id));
	AVR32_SCIF.pll[pll_id] = 0;
}

static inline bool pll_is_locked(unsigned int pll_id)
{
	Assert(pll_id < NR_PLLS);

	return !!(AVR32_SCIF.pclksr & (1U << (AVR32_SCIF_PLL0_LOCK + pll_id)));
}

#endif /* __ASSEMBLY__ */

#ifdef __cplusplus
}
#endif

#endif /* CLK_PLL_H_INCLUDED */
