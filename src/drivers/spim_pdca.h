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

#ifndef _SPIM_PDCA_H_
#define _SPIM_PDCA_H_

/*
SPIM transactions consist of an optional write command and an optional read
command. If a write command is present, it is always sent before the read is
initiated.

Transactions store their write data inline, but any data read is stored to
a buffer pointed to by rx_buf. Up to 4 bytes may be written, and 255 bytes
read.

When passed in an array to twim_run_sequence, each transaction is executed in
the order defined. A transaction result is one of:
- SPIM_TRANSACTION_EXECUTED: indicates completion of both write and read
  phases (if present) of the transaction;
- SPIM_TRANSACTION_NOTREADY: indicates the transaction is currently waiting on
  hardware to execute the write or read components;
- SPIM_TRANSACTION_PENDING: indicates the transaction has been initiated bu
  is not yet complete;
- SPIM_TRANSACTION_SEQDONE: returned if a transaction sequence has a
  terminating SPIM_TRANSACTION_SENTINEL value, and the sequence index points
  to this value.
*/

enum spim_transaction_status_t {
    SPIM_TRANSACTION_STATUS_NONE = 0,
    SPIM_TRANSACTION_STATUS_SENT
};

struct spim_transaction_t {
    uint8_t txn_len;
    uint8_t tx_buf[16];
    uint8_t rx_buf[16];
    enum spim_transaction_status_t txn_status;
};

enum spim_transaction_result_t {
    SPIM_TRANSACTION_EXECUTED = 0,
    SPIM_TRANSACTION_NOTREADY,
    SPIM_TRANSACTION_PENDING,
    SPIM_TRANSACTION_SEQDONE,
    SPIM_TRANSACTION_ERROR
};

#define SPIM_TRANSACTION_SENTINEL {0, {0}, {0}, 0}

/*
spim_pdca_cfg_t stores relevant pointers and channel IDs for a SPIM/PDCA
channel combination.
- spim must point to a SPIM instance (TODO);
- tx_pdca_num must be a unique PDCA channel ID (0-31);
- rx_pdca_num must be a unique PDCA channle ID (0-31);
- tx_pid is the PDCA peripheral ID of the SPIM instance TX register
  (TODO)
- rx_pid is the PDCA peripheral ID of the SPIM instance RX register
  (TODO)
*/

struct spim_pdca_cfg_t {
    volatile avr32_spi_t *spim;
    uint8_t tx_pdca_num;
    uint8_t rx_pdca_num;
    uint32_t tx_pid;
    uint32_t rx_pid;
};

/*
spim_pdca_init performs basic configuration of a SPIM instance, including
setting the clock waveform generator (CWGR) to the correct parameters for
the desired SPI clock rate (must be 1MHz-20MHz inclusive).
*/
void spim_pdca_init(struct spim_pdca_cfg_t *cfg, uint32_t speed_hz);

/*
spim_pdca_write executes a write[/read] transaction specified by txn on the
SPIM identified by cfg.
*/
void spim_pdca_transact(struct spim_pdca_cfg_t *cfg,
struct spim_transaction_t *txn);

/*
spim_run_sequence executes the next transaction in seq (indexed by seq_idx);
if both write and read components of the transaction have been successfully
completed it returns SPIM_TRANSACTION_EXECUTED.
*/
enum spim_transaction_result_t spim_run_sequence(struct spim_pdca_cfg_t *cfg,
struct spim_transaction_t seq[], uint32_t seq_idx);

#endif