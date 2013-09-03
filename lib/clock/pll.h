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

#define pll_get_default_rate(pll_id)                                           \
	((osc_get_rate(CONFIG_PLL##pll_id##_SOURCE)                            \
			* CONFIG_PLL##pll_id##_MUL)                            \
			/ CONFIG_PLL##pll_id##_DIV)

static inline void pll_config_set_option(struct pll_config *cfg,
		unsigned int option)
{
	Assert(option < PLL_NR_OPTIONS);

	cfg->ctrl |= 1U << (AVR32_SCIF_PLLOPT + option);
}

static inline void pll_config_clear_option(struct pll_config *cfg,
		unsigned int option)
{
	Assert(option < PLL_NR_OPTIONS);

	cfg->ctrl &= ~(1U << (AVR32_SCIF_PLLOPT + option));
}

/**
 * The PLL options #PLL_OPT_VCO_RANGE_LOW and #PLL_OPT_OUTPUT_DIV will
 * be set automatically based on the calculated target frequency.
 */
static inline void pll_config_init(struct pll_config *cfg,
		enum pll_source src, unsigned int div, unsigned int mul)
{
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

#define pll_config_defaults(cfg, pll_id)                                       \
	pll_config_init(cfg,                                                   \
			CONFIG_PLL##pll_id##_SOURCE,                           \
			CONFIG_PLL##pll_id##_DIV,                              \
			CONFIG_PLL##pll_id##_MUL)

static inline void pll_config_read(struct pll_config *cfg, unsigned int pll_id)
{
	Assert(pll_id < NR_PLLS);

	cfg->ctrl = AVR32_SCIF.pll[pll_id];
}

extern void pll_config_write(const struct pll_config *cfg, unsigned int pll_id);
extern void pll_enable(const struct pll_config *cfg, unsigned int pll_id);
extern void pll_disable(unsigned int pll_id);

static inline bool pll_is_locked(unsigned int pll_id)
{
	Assert(pll_id < NR_PLLS);

	return !!(AVR32_SCIF.pclksr & (1U << (AVR32_SCIF_PLL0_LOCK + pll_id)));
}

static inline void pll_enable_source(enum pll_source src)
{
	switch (src) {
		case PLL_SRC_OSC0:
			if (!osc_is_ready(OSC_ID_OSC0)) {
				osc_enable(OSC_ID_OSC0);
				osc_wait_ready(OSC_ID_OSC0);
			}
			break;

		case PLL_SRC_OSC1:
			if (!osc_is_ready(OSC_ID_OSC1)) {
				osc_enable(OSC_ID_OSC1);
				osc_wait_ready(OSC_ID_OSC1);
			}
			break;

		case PLL_SRC_RC8M:
			if (!osc_is_ready(OSC_ID_RC8M)) {
				osc_enable(OSC_ID_RC8M);
				osc_wait_ready(OSC_ID_RC8M);
			}
			break;

		default:
			Assert(false);
			break;
	}
}

static inline void pll_enable_config_defaults(unsigned int pll_id)
{
	struct pll_config pllcfg;

	if (pll_is_locked(pll_id)) {
		return; /* Pll already running */
	}
	switch (pll_id) {
#ifdef CONFIG_PLL0_SOURCE
		case 0:
			pll_enable_source(CONFIG_PLL0_SOURCE);
			pll_config_init(&pllcfg,
					CONFIG_PLL0_SOURCE,
					CONFIG_PLL0_DIV,
					CONFIG_PLL0_MUL);
			break;
#endif
#ifdef CONFIG_PLL1_SOURCE
		case 1:
			pll_enable_source(CONFIG_PLL1_SOURCE);
			pll_config_init(&pllcfg,
					CONFIG_PLL1_SOURCE,
					CONFIG_PLL1_DIV,
					CONFIG_PLL1_MUL);
			break;
#endif
		default:
			Assert(false);
			break;
	}
	pll_enable(&pllcfg, pll_id);
	while (!pll_is_locked(pll_id));
}

#endif /* __ASSEMBLY__ */

#ifdef __cplusplus
}
#endif


/**
 * \ingroup clk_group
 * \defgroup pll_group PLL Management
 *
 * This group contains functions and definitions related to configuring
 * and enabling/disabling on-chip PLLs. A PLL will take an input signal
 * (the \em source), optionally divide the frequency by a configurable
 * \em divider, and then multiply the frequency by a configurable \em
 * multiplier.
 *
 * Some devices don't support input dividers; specifying any other
 * divisor than 1 on these devices will result in an assertion failure.
 * Other devices may have various restrictions to the frequency range of
 * the input and output signals.
 *
 * \par Example: Setting up PLL0 with default parameters
 *
 * The following example shows how to configure and enable PLL0 using
 * the default parameters specified using the configuration symbols
 * listed above.
 * \code
	pll_enable_config_defaults(0); \endcode
 *
 * To configure, enable PLL0 using the default parameters and to disable
 * a specific feature like Wide Bandwidth Mode (a UC3A3-specific
 * PLL option.), you can use this initialization process.
 * \code
	struct pll_config pllcfg;
	if (pll_is_locked(pll_id)) {
		return;
	}
	pll_enable_source(CONFIG_PLL0_SOURCE);
	pll_config_defaults(&pllcfg, 0);
	pll_config_set_option(&pllcfg, PLL_OPT_WBM_DISABLE);
	pll_enable(&pllcfg, 0);
	pll_wait_for_lock(0); \endcode
 *
 * When the last function call returns, PLL0 is ready to be used as the
 * main system clock source.
 *
 * \section pll_group_config Configuration Symbols
 *
 * Each PLL has a set of default parameters determined by the following
 * configuration symbols in the application's configuration file:
 *   - \b CONFIG_PLLn_SOURCE: The default clock source connected to the
 *     input of PLL \a n. Must be one of the values defined by the
 *     #pll_source enum.
 *   - \b CONFIG_PLLn_MUL: The default multiplier (loop divider) of PLL
 *     \a n.
 *   - \b CONFIG_PLLn_DIV: The default input divider of PLL \a n.
 *
 * These configuration symbols determine the result of calling
 * pll_config_defaults() and pll_get_default_rate().
 *
 * @{
 */

/* \name Chip-specific PLL characteristics */
/**
 * \def PLL_MAX_STARTUP_CYCLES
 * \brief Maximum PLL startup time in number of slow clock cycles
 */
/**
 * \def NR_PLLS
 * \brief Number of on-chip PLLs
 */

/**
 * \def PLL_MIN_HZ
 * \brief Minimum frequency that the PLL can generate
 */
/**
 * \def PLL_MAX_HZ
 * \brief Maximum frequency that the PLL can generate
 */
/**
 * \def PLL_NR_OPTIONS
 * \brief Number of PLL option bits
 */

/**
 * \enum pll_source
 * \brief PLL clock source
 */

/* \name PLL configuration */

/**
 * \struct pll_config
 * \brief Hardware-specific representation of PLL configuration.
 *
 * This structure contains one or more device-specific values
 * representing the current PLL configuration. The contents of this
 * structure is typically different from platform to platform, and the
 * user should not access any fields except through the PLL
 * configuration API.
 */

/**
 * \fn void pll_config_init(struct pll_config *cfg,
 *              enum pll_source src, unsigned int div, unsigned int mul)
 * \brief Initialize PLL configuration from standard parameters.
 *
 * \note This function may be defined inline because it is assumed to be
 * called very few times, and usually with constant parameters. Inlining
 * it will in such cases reduce the code size significantly.
 *
 * \param cfg The PLL configuration to be initialized.
 * \param src The oscillator to be used as input to the PLL.
 * \param div PLL input divider.
 * \param mul PLL loop divider (i.e. multiplier).
 *
 * \return A configuration which will make the PLL run at
 * (\a mul / \a div) times the frequency of \a src
 */
/**
 * \def pll_config_defaults(cfg, pll_id)
 * \brief Initialize PLL configuration using default parameters.
 *
 * After this function returns, \a cfg will contain a configuration
 * which will make the PLL run at (CONFIG_PLLx_MUL / CONFIG_PLLx_DIV)
 * times the frequency of CONFIG_PLLx_SOURCE.
 *
 * \param cfg The PLL configuration to be initialized.
 * \param pll_id Use defaults for this PLL.
 */
/**
 * \def pll_get_default_rate(pll_id)
 * \brief Get the default rate in Hz of \a pll_id
 */
/**
 * \fn void pll_config_set_option(struct pll_config *cfg,
 *              unsigned int option)
 * \brief Set the PLL option bit \a option in the configuration \a cfg.
 *
 * \param cfg The PLL configuration to be changed.
 * \param option The PLL option bit to be set.
 */
/**
 * \fn void pll_config_clear_option(struct pll_config *cfg,
 *              unsigned int option)
 * \brief Clear the PLL option bit \a option in the configuration \a cfg.
 *
 * \param cfg The PLL configuration to be changed.
 * \param option The PLL option bit to be cleared.
 */
/**
 * \fn void pll_config_read(struct pll_config *cfg, unsigned int pll_id)
 * \brief Read the currently active configuration of \a pll_id.
 *
 * \param cfg The configuration object into which to store the currently
 * active configuration.
 * \param pll_id The ID of the PLL to be accessed.
 */
/**
 * \fn void pll_config_write(const struct pll_config *cfg,
 *              unsigned int pll_id)
 * \brief Activate the configuration \a cfg on \a pll_id
 *
 * \param cfg The configuration object representing the PLL
 * configuration to be activated.
 * \param pll_id The ID of the PLL to be updated.
 */

/* \name Interaction with the PLL hardware */
/**
 * \fn void pll_enable(const struct pll_config *cfg,
 *              unsigned int pll_id)
 * \brief Activate the configuration \a cfg and enable PLL \a pll_id.
 *
 * \param cfg The PLL configuration to be activated.
 * \param pll_id The ID of the PLL to be enabled.
 */
/**
 * \fn void pll_disable(unsigned int pll_id)
 * \brief Disable the PLL identified by \a pll_id.
 *
 * After this function is called, the PLL identified by \a pll_id will
 * be disabled. The PLL configuration stored in hardware may be affected
 * by this, so if the caller needs to restore the same configuration
 * later, it should either do a pll_config_read() before disabling the
 * PLL, or remember the last configuration written to the PLL.
 *
 * \param pll_id The ID of the PLL to be disabled.
 */
/**
 * \fn bool pll_is_locked(unsigned int pll_id)
 * \brief Determine whether the PLL is locked or not.
 *
 * \param pll_id The ID of the PLL to check.
 *
 * \retval true The PLL is locked and ready to use as a clock source
 * \retval false The PLL is not yet locked, or has not been enabled.
 */
/**
 * \fn void pll_enable_source(enum pll_source src)
 * \brief Enable the source of the pll.
 * The source is enabled, if the source is not already running.
 *
 * \param src The ID of the PLL source to enable.
 */
/**
 * \fn void pll_enable_config_defaults(unsigned int pll_id)
 * \brief Enable the pll with the default configuration.
 * PLL is enabled, if the PLL is not already locked.
 *
 * \param pll_id The ID of the PLL to enable.
 */

/**
 * \brief Wait for PLL \a pll_id to become locked
 *
 * \todo Use a timeout to avoid waiting forever and hanging the system
 *
 * \param pll_id The ID of the PLL to wait for.
 *
 * \retval STATUS_OK The PLL is now locked.
 * \retval ERR_TIMEOUT Timed out waiting for PLL to become locked.
 */
static inline int pll_wait_for_lock(unsigned int pll_id)
{
	Assert(pll_id < NR_PLLS);

	while (!pll_is_locked(pll_id)) {
		/* Do nothing */
	}

	return 0;
}


#endif /* CLK_PLL_H_INCLUDED */
