/*
Copyright (C) 2014 Ben Dyer

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
#include "spim_pdca.h"

#ifndef CONTINUE_ON_ASSERT
#define SPIAssert(x) Assert(x)
#else
#define SPIAssert(x) if (!(x)) { wdt_reset_mcu(); }
#endif

inline static void spim_pdca_enable(volatile avr32_pdca_channel_t *pdca,
void *buffer, uint32_t nbytes, uint32_t pid);

inline static void spim_pdca_enable(volatile avr32_pdca_channel_t *pdca,
void *buffer, uint32_t nbytes, uint32_t pid) {
    SPIAssert(pdca);
    SPIAssert(nbytes <= 255u);
    SPIAssert(!nbytes || buffer);

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

void spim_pdca_init(struct spim_pdca_cfg_t *cfg, uint32_t speed_hz) {
    SPIAssert(cfg);
    SPIAssert(1000000u <= speed_hz && speed_hz <= 20000000u);

    /* Clear PDCAs */
    spim_pdca_enable(&AVR32_PDCA.channel[cfg->tx_pdca_num], NULL, 0,
        cfg->tx_pid);
    spim_pdca_enable(&AVR32_PDCA.channel[cfg->rx_pdca_num], NULL, 0,
        cfg->rx_pid);

    /* Initialize the SPI device clock -- PBC for SPI0, PBA for SPI1 */
    uint32_t f_pb = sysclk_get_pba_hz();
    uint32_t f_prescaled = f_pb / speed_hz;
    SPIAssert(f_prescaled <= 0xffu);

    /* Set up the SPI registers */
    irqflags_t flags = cpu_irq_save();
    cfg->spim->idr = 0xffffffffu;
    cfg->spim->cr = AVR32_SPI_CR_SPIEN_MASK;
    cfg->spim->cr = AVR32_SPI_CR_SWRST_MASK;
    cfg->spim->cr = AVR32_SPI_CR_FLUSHFIFO_MASK;
    /* Master mode, with fixed peripheral select set to 0b0111 (for CS3) */
    cfg->spim->mr = AVR32_SPI_MR_MSTR_MASK | AVR32_SPI_MR_PS_MASK |
                    (0x7u << AVR32_SPI_MR_PCS_OFFSET);
    /* Configure CSR3, which is the control register for CS3. Set the clock
       divisor and set NCPHA = 1 to capture data on rising edge and change on
       falling edge. */
    cfg->spim->csr3 = (f_prescaled << AVR32_SPI_CSR3_SCBR_OFFSET) |
                      AVR32_SPI_CSR3_NCPHA_MASK;
    /* Clear SR */
    cfg->spim->cr = AVR32_SPI_CR_SPIDIS_MASK;

    cpu_irq_restore(flags);
}

void spim_pdca_transact(struct spim_pdca_cfg_t *cfg,
struct spim_transaction_t *txn) {
    SPIAssert(cfg);
    SPIAssert(txn);

    /* AVR32 datasheet, 27.8.5.1 */
    /* 1. Initialize PDCA */
    SPIAssert(cfg->tx_pdca_num < AVR32_PDCA_CHANNEL_LENGTH);
    SPIAssert(cfg->rx_pdca_num < AVR32_PDCA_CHANNEL_LENGTH);
    volatile avr32_pdca_channel_t *tx_pdca =
        &AVR32_PDCA.channel[cfg->tx_pdca_num];
    volatile avr32_pdca_channel_t *rx_pdca =
        &AVR32_PDCA.channel[cfg->rx_pdca_num];

    irqflags_t flags = cpu_irq_save();

    /* Reset the SPIM module */
    cfg->spim->idr = 0xffffffffu;
    cfg->spim->cr = AVR32_SPI_CR_SPIEN_MASK;
    cfg->spim->cr = AVR32_SPI_CR_SWRST_MASK;
    cfg->spim->cr = AVR32_SPI_CR_FLUSHFIFO_MASK;
    /* Clear SR */
    cfg->spim->cr = AVR32_SPI_CR_SPIDIS_MASK;

    /* Configure TX and RX PDCAs */
    if (txn->txn_len) {
        spim_pdca_enable(tx_pdca, txn->tx_buf, txn->txn_len, cfg->tx_pid);
        spim_pdca_enable(rx_pdca, txn->rx_buf, txn->txn_len, cfg->rx_pid);
    }

    /* Enable master transfer */
    cfg->spim->cr = AVR32_SPI_CR_SPIEN_MASK;

    /* 2. Start transfer by enabling PDCA */
    if (txn->txn_len) {
        tx_pdca->cr = AVR32_PDCA_TEN_MASK;
        rx_pdca->cr = AVR32_PDCA_TEN_MASK;
    }

    cpu_irq_restore(flags);
}

enum spim_transaction_result_t spim_run_sequence(struct spim_pdca_cfg_t *cfg,
struct spim_transaction_t seq[], uint32_t idx) {
    SPIAssert(cfg);
    SPIAssert(cfg->spim);
    SPIAssert(seq);

    enum spim_transaction_result_t result = SPIM_TRANSACTION_NOTREADY;

    if (!seq[idx].txn_len) {
        result = SPIM_TRANSACTION_SEQDONE;
    } else if (seq[idx].txn_status == SPIM_TRANSACTION_STATUS_NONE) {
        spim_pdca_transact(cfg, &(seq[idx]));
        /* Sent the request, so the command isn't complete yet */
        seq[idx].txn_status = SPIM_TRANSACTION_STATUS_SENT;
        result = SPIM_TRANSACTION_PENDING;
    } else if (!AVR32_PDCA.channel[cfg->rx_pdca_num].tcr) {
        /* Checked for read command and PDCA transfer completion */
        seq[idx].txn_status = SPIM_TRANSACTION_STATUS_NONE;
        seq[idx + 1].txn_status = SPIM_TRANSACTION_STATUS_NONE;
        result = SPIM_TRANSACTION_EXECUTED;
    } else {
        /* Nothing ready */
    }

    return result;
}
