/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include <asf.h>
#include <avr32/io.h>
#include "twim_pdca.h"

#ifndef CONTINUE_ON_ASSERT
#define TWIAssert(x) Assert(x)
#else
#define TWIAssert(x) if (!(x)) { wdt_reset_mcu(); }
#endif

inline static void twim_pdca_enable(volatile avr32_pdca_channel_t *pdca,
    void *buffer, uint32_t nbytes, uint32_t pid);

inline static void twim_pdca_enable(volatile avr32_pdca_channel_t *pdca,
        void *buffer, uint32_t nbytes, uint32_t pid) {
	TWIAssert(pdca);
	TWIAssert(nbytes <= 255u);
	TWIAssert(!nbytes || buffer);

    pdca->cr = AVR32_PDCA_TDIS_MASK;
    pdca->cr = AVR32_PDCA_ECLR_MASK;
    pdca->mar = (uint32_t)buffer;
    pdca->tcr = nbytes;
    pdca->psr = pid;
    pdca->marr = 0;
    pdca->tcrr = 0;
    pdca->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
    pdca->isr;
}

void twim_pdca_init(struct twim_pdca_cfg_t *cfg, uint32_t speed_hz) {
    TWIAssert(cfg);
    TWIAssert(100000u <= speed_hz && speed_hz <= 400000u);

	/* Clear PDCAs */
	twim_pdca_enable(&AVR32_PDCA.channel[cfg->tx_pdca_num], NULL, 0,
		cfg->tx_pid);
	twim_pdca_enable(&AVR32_PDCA.channel[cfg->rx_pdca_num], NULL, 0,
		cfg->rx_pid);

    /* Set up the TWI registers */
    irqflags_t flags = cpu_irq_save();
    cfg->twim->idr = 0xffffffffu;
    cfg->twim->cr = AVR32_TWIM_CR_MEN_MASK;
	cfg->twim->cr = AVR32_TWIM_CR_SWRST_MASK;
	/* Clear SR */
    cfg->twim->scr = 0xffffffffu;
	cfg->twim->cr = AVR32_TWIM_CR_MDIS_MASK;

    /* Initialize the TWI device clock -- PBA for TWIM0-1, PBC for TWIM2 */
	uint32_t f_pb = cfg->twim != &AVR32_TWIM2 ? sysclk_get_pba_hz() : sysclk_get_pbc_hz();
    uint32_t f_prescaled = (f_pb / speed_hz / 2u);
    uint8_t cwgr_exp = 0;
    /* f_prescaled must fit in 8 bits, cwgr_exp must fit in 3 bits */
    while ((f_prescaled > 0xffu) && (cwgr_exp <= 0x7u)) {
        /* increase clock divider and divide f_prescaled value */
        cwgr_exp++;
        f_prescaled >>= 1u;
    }
    TWIAssert(f_prescaled <= 0xffu && cwgr_exp <= 0x08u);

	uint32_t min_hz = (speed_hz == 100000u) ? ((10000000u / 47u) + 1u) :
		((100000000u / 133u) + 1u);
	while ((f_prescaled > 0) && ((f_pb / f_prescaled) <  min_hz)) {
		f_prescaled--;
	}

	TWIAssert(f_prescaled && cwgr_exp <= 0x7u);

	/* set clock waveform generator register */
	cfg->twim->CWGR.exp = cwgr_exp;
	cfg->twim->CWGR.data = f_prescaled / 4;
	cfg->twim->CWGR.stasto = f_prescaled;
	cfg->twim->CWGR.high = f_prescaled / 2;
	cfg->twim->CWGR.low = f_prescaled / 2;

	cpu_irq_restore(flags);
}

void twim_pdca_transact(struct twim_pdca_cfg_t *cfg,
        struct twim_transaction_t *txn) {
    TWIAssert(cfg);
    TWIAssert(txn);

    /* AVR32 datasheet, 27.8.5.1 */
    /* 1. Initialize PDCA */
    TWIAssert(cfg->tx_pdca_num < AVR32_PDCA_CHANNEL_LENGTH);
    TWIAssert(cfg->rx_pdca_num < AVR32_PDCA_CHANNEL_LENGTH);
    volatile avr32_pdca_channel_t *tx_pdca =
        &AVR32_PDCA.channel[cfg->tx_pdca_num];
    volatile avr32_pdca_channel_t *rx_pdca =
        &AVR32_PDCA.channel[cfg->rx_pdca_num];
    uint32_t tx_cmd = 0, rx_cmd = 0;

    irqflags_t flags = cpu_irq_save();

    /* Reset the TWIM module */
    cfg->twim->idr = 0xffffffffu;
    cfg->twim->cr = AVR32_TWIM_CR_MEN_MASK;
	cfg->twim->cr = AVR32_TWIM_CR_SWRST_MASK;
	/* Clear SR */
    cfg->twim->scr = 0xffffffffu;
	cfg->twim->cr = AVR32_TWIM_CR_MDIS_MASK;

    /* Configure TX and RX PDCAs */
    if (txn->tx_len) {
        twim_pdca_enable(tx_pdca, txn->tx_buf, txn->tx_len, cfg->tx_pid);
        tx_cmd = (txn->dev_addr << AVR32_TWIM_CMDR_SADR_OFFSET)
            | (txn->tx_len << AVR32_TWIM_CMDR_NBYTES_OFFSET)
            | AVR32_TWIM_CMDR_VALID_MASK
			| AVR32_TWIM_CMDR_START_MASK
			| (txn->rx_len ? 0 : AVR32_TWIM_CMDR_STOP_MASK);
    }
    if (txn->rx_len) {
        TWIAssert(txn->rx_buf);

        twim_pdca_enable(rx_pdca, txn->rx_buf, txn->rx_len, cfg->rx_pid);
        rx_cmd = (txn->dev_addr << AVR32_TWIM_CMDR_SADR_OFFSET)
            | (txn->rx_len << AVR32_TWIM_CMDR_NBYTES_OFFSET)
            | AVR32_TWIM_CMDR_VALID_MASK
			| AVR32_TWIM_CMDR_START_MASK
			| AVR32_TWIM_CMDR_STOP_MASK
            | AVR32_TWIM_CMDR_READ_MASK;
    }

    /* 2. Configure TWIM (ADR, NBYTES etc)
       set the command to start the transfer */
    if (txn->tx_len && txn->rx_len) {
        cfg->twim->cmdr = tx_cmd;
        cfg->twim->ncmdr = rx_cmd;
    } else if (txn->tx_len && !txn->rx_len) {
        cfg->twim->cmdr = tx_cmd;
		cfg->twim->ncmdr = 0;
    } else if (!txn->tx_len && txn->rx_len) {
        cfg->twim->cmdr = rx_cmd;
		cfg->twim->ncmdr = 0;
    } else {
        /* nothing to do */
    }

	/* Enable master transfer */
	cfg->twim->cr = AVR32_TWIM_CR_MEN_MASK;

	/* 3. Start transfer by enabling PDCA */
    if (txn->tx_len) {
        tx_pdca->cr = AVR32_PDCA_TEN_MASK;
    }
    if (txn->rx_len) {
        rx_pdca->cr = AVR32_PDCA_TEN_MASK;
    }

    cpu_irq_restore(flags);
}

enum twim_transaction_result_t twim_run_sequence(struct twim_pdca_cfg_t *cfg,
		struct twim_transaction_t seq[], uint32_t idx) {
	TWIAssert(cfg);
	TWIAssert(cfg->twim);
	TWIAssert(seq);

    enum twim_transaction_result_t result = TWIM_TRANSACTION_NOTREADY;

	if (!seq[idx].dev_addr) {
		result = TWIM_TRANSACTION_SEQDONE;
	} else {
        bool tx_ready = (cfg->twim->SR.txrdy &&
                !AVR32_PDCA.channel[cfg->tx_pdca_num].tcr) ||
                !cfg->twim->CMDR.valid;
		bool tx_err = cfg->twim->SR.arblst || cfg->twim->SR.dnak ||
				cfg->twim->SR.anak;

		if (tx_err) {
            seq[idx].txn_status = TWIM_TRANSACTION_STATUS_NONE;
			result = TWIM_TRANSACTION_ERROR;
		} else if (tx_ready && seq[idx].txn_status == TWIM_TRANSACTION_STATUS_NONE) {
            twim_pdca_transact(cfg, &(seq[idx]));
            if (seq[idx].rx_len) {
                /* Sent the request, so the command isn't complete yet */
                seq[idx].txn_status = TWIM_TRANSACTION_STATUS_SENT;
                result = TWIM_TRANSACTION_PENDING;
            } else {
                /* Mark transaction as complete if there's nothing else to do */
                seq[idx].txn_status = TWIM_TRANSACTION_STATUS_NONE;
				seq[idx + 1].txn_status = TWIM_TRANSACTION_STATUS_NONE;
                result = TWIM_TRANSACTION_EXECUTED;
            }
        } else if (!AVR32_PDCA.channel[cfg->rx_pdca_num].tcr &&
                cfg->twim->SR.ccomp &&
                seq[idx].txn_status == TWIM_TRANSACTION_STATUS_SENT) {
            /* Checked for read command and PDCA transfer completion */
            seq[idx].txn_status = TWIM_TRANSACTION_STATUS_NONE;
			seq[idx + 1].txn_status = TWIM_TRANSACTION_STATUS_NONE;
            result = TWIM_TRANSACTION_EXECUTED;
        } else {
            /* Nothing ready */
        }
    }

    return result;
}
