/**
 * \file
 *
 * \brief Generic clock management
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
#ifndef CLK_GENCLK_H_INCLUDED
#define CLK_GENCLK_H_INCLUDED

#include "conf_clock.h"
#include <board.h>

/**
 * \weakgroup genclk_group
 * @{
 */

#define GENCLK_DIV_MAX  256

#ifndef __ASSEMBLY__

#include <stdint.h>
#include <avr32/io.h>

/* Get around http://asf.atmel.com/bugzilla/show_bug.cgi?id=2662 */

#define GENCLK_SRC_RCSYS        0    /* System RC oscillator */
#define GENCLK_SRC_OSC32K       1    /* 32 kHz oscillator */
#define GENCLK_SRC_RC8M         2    /* 8 MHz RC oscillator */
#define GENCLK_SRC_OSC0         3    /* Oscillator 0 */
#define GENCLK_SRC_OSC1         4    /* Oscillator 1 */
#define GENCLK_SRC_PLL0         5    /* PLL 0 */
#define GENCLK_SRC_PLL1         6    /* PLL 1 */
#define GENCLK_SRC_CLK_CPU      7    /* CPU clock */
#define GENCLK_SRC_CLK_HSB      8    /* High Speed Bus clock */
#define GENCLK_SRC_CLK_PBA      9    /* Peripheral Bus A clock */
#define GENCLK_SRC_CLK_PBB      10   /* Peripheral Bus B clock */
#define GENCLK_SRC_CLK_PBC      11   /* Peripheral Bus C clock */

enum genclk_source {
    genclk_src_rcsys        = GENCLK_SRC_RCSYS,
    genclk_src_osc32k       = GENCLK_SRC_OSC32K,
    genclk_src_rc8m         = GENCLK_SRC_RC8M,
    genclk_src_osc0         = GENCLK_SRC_OSC0,
    genclk_src_osc1         = GENCLK_SRC_OSC1,
    genclk_src_pll0         = GENCLK_SRC_PLL0,
    genclk_src_pll1         = GENCLK_SRC_PLL1,
    genclk_src_clk_cpu      = GENCLK_SRC_CLK_CPU,
    genclk_src_clk_hsb      = GENCLK_SRC_CLK_HSB,
    genclk_src_clk_pba      = GENCLK_SRC_CLK_PBA,
    genclk_src_clk_pbb      = GENCLK_SRC_CLK_PBB,
    genclk_src_clk_pbc      = GENCLK_SRC_CLK_PBC
};


#include <osc.h>
#include <pll.h>

struct genclk_config {
    uint32_t ctrl;
};

static inline void genclk_config_defaults(struct genclk_config *cfg,
        unsigned int id)
{
    cfg->ctrl = 0;
}

static inline void genclk_config_read(struct genclk_config *cfg,
        unsigned int id)
{
    cfg->ctrl = AVR32_SCIF.gcctrl[id];
}

static inline void genclk_config_write(const struct genclk_config *cfg,
        unsigned int id)
{
    AVR32_SCIF.gcctrl[id] = cfg->ctrl;
}

static inline void genclk_config_set_source(struct genclk_config *cfg,
        enum genclk_source src)
{
    cfg->ctrl = (cfg->ctrl & ~AVR32_SCIF_GCCTRL_OSCSEL_MASK)
            | (src << AVR32_SCIF_GCCTRL_OSCSEL);
}

static inline void genclk_config_set_divider(struct genclk_config *cfg,
        unsigned int divider)
{
    Assert(divider > 0 && divider <= GENCLK_DIV_MAX);

    /* Clear all the bits we're about to modify */
    cfg->ctrl &= ~(AVR32_SCIF_GCCTRL_DIVEN_MASK
            | AVR32_SCIF_GCCTRL_DIV_MASK);

    if (divider > 1) {
        cfg->ctrl |= 1U << AVR32_SCIF_GCCTRL_DIVEN;
        cfg->ctrl |= (((divider + 1) / 2) - 1) << AVR32_SCIF_GCCTRL_DIV;
    }
}

static inline void genclk_enable(const struct genclk_config *cfg,
        unsigned int id)
{
    AVR32_SCIF.gcctrl[id] = cfg->ctrl | (1U << AVR32_SCIF_GCCTRL_CEN);
}

static inline void genclk_disable(unsigned int id)
{
    AVR32_SCIF.gcctrl[id] = 0;
}

static inline void genclk_enable_source(enum genclk_source src)
{
    switch (src) {
        case GENCLK_SRC_RCSYS:
        case GENCLK_SRC_CLK_CPU:
        case GENCLK_SRC_CLK_HSB:
        case GENCLK_SRC_CLK_PBA:
        case GENCLK_SRC_CLK_PBB:
        case GENCLK_SRC_CLK_PBC:
            /* Nothing to do */
            break;

        case GENCLK_SRC_OSC32K:
            if (!osc_is_ready(OSC_ID_OSC32)) {
                osc_enable(OSC_ID_OSC32);
                osc_wait_ready(OSC_ID_OSC32);
            }
            break;

        case GENCLK_SRC_RC8M:
            if (!osc_is_ready(OSC_ID_RC8M)) {
                osc_enable(OSC_ID_RC8M);
                osc_wait_ready(OSC_ID_RC8M);
            }
            break;

#ifdef BOARD_OSC0_HZ
        case GENCLK_SRC_OSC0:
            if (!osc_is_ready(OSC_ID_OSC0)) {
                osc_enable(OSC_ID_OSC0);
                osc_wait_ready(OSC_ID_OSC0);
            }
            break;
#endif

#ifdef BOARD_OSC1_HZ
        case GENCLK_SRC_OSC1:
            if (!osc_is_ready(OSC_ID_OSC1)) {
                osc_enable(OSC_ID_OSC1);
                osc_wait_ready(OSC_ID_OSC1);
            }
            break;
#endif

#ifdef CONFIG_PLL0_SOURCE
        case GENCLK_SRC_PLL0:
            pll_enable_config_defaults(0);
            break;
#endif

#ifdef CONFIG_PLL1_SOURCE
        case GENCLK_SRC_PLL1:
            pll_enable_config_defaults(1);
            break;
#endif

        default:
            Assert(false);
            break;
    }
}

#endif /* __ASSEMBLY__ */


/**
 * \ingroup clk_group
 * \defgroup genclk_group Generic Clock Management
 *
 * Generic clocks are configurable clocks which run outside the system
 * clock domain. They are often connected to peripherals which have an
 * asynchronous component running independently of the bus clock, e.g.
 * USB controllers, low-power timers and RTCs, etc.
 *
 * Note that not all platforms have support for generic clocks; on such
 * platforms, this API will not be available.
 *
 * @{
 */

/**
 * \def GENCLK_DIV_MAX
 * \brief Maximum divider supported by the generic clock implementation
 */
/**
 * \enum genclk_source
 * \brief Generic clock source ID
 *
 * Each generic clock may be generated from a different clock source.
 * These are the available alternatives provided by the chip.
 */

/**
 * \struct genclk_config
 * \brief Hardware representation of a set of generic clock parameters
 */
/**
 * \fn void genclk_config_defaults(struct genclk_config *cfg,
 *              unsigned int id)
 * \brief Initialize \a cfg to the default configuration for the clock
 * identified by \a id.
 */
/**
 * \fn void genclk_config_read(struct genclk_config *cfg, unsigned int id)
 * \brief Read the currently active configuration of the clock
 * identified by \a id into \a cfg.
 */
/**
 * \fn void genclk_config_write(const struct genclk_config *cfg,
 *              unsigned int id)
 * \brief Activate the configuration \a cfg on the clock identified by
 * \a id.
 */
/**
 * \fn void genclk_config_set_source(struct genclk_config *cfg,
 *              enum genclk_source src)
 * \brief Select a new source clock \a src in configuration \a cfg.
 */
/**
 * \fn void genclk_config_set_divider(struct genclk_config *cfg,
 *              unsigned int divider)
 * \brief Set a new \a divider in configuration \a cfg.
 */
/**
 * \fn void genclk_enable_source(enum genclk_source src)
 * \brief Enable the source clock \a src used by a generic clock.
 */

/**
 * \fn void genclk_enable(const struct genclk_config *cfg, unsigned int id)
 * \brief Activate the configuration \a cfg on the clock identified by
 * \a id and enable it.
 */
/**
 * \fn void genclk_disable(unsigned int id)
 * \brief Disable the generic clock identified by \a id.
 */

/**
 * \brief Enable the configuration defined by \a src and \a divider
 * for the generic clock identified by \a id.
 *
 * \param id      The ID of the generic clock.
 * \param src     The source clock of the generic clock.
 * \param divider The divider used to generate the generic clock.
 */
static inline void genclk_enable_config(unsigned int id, enum genclk_source src, unsigned int divider)
{
	struct genclk_config gcfg;

	genclk_config_defaults(&gcfg, id);
	genclk_enable_source(src);
	genclk_config_set_source(&gcfg, src);
	genclk_config_set_divider(&gcfg, divider);
	genclk_enable(&gcfg, id);
}

#endif /* CLK_GENCLK_H_INCLUDED */
