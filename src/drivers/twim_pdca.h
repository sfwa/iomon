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

#ifndef _TWIM_PDCA_H_
#define _TWIM_PDCA_H_

/*
TWIM transactions consist of an optional write command and an optional read
command. If a write command is present, it is always sent before the read is
initiated.

Transactions store their write data inline, but any data read is stored to
a buffer pointed to by rx_buf. Up to 4 bytes may be written, and 255 bytes
read.

When passed in an array to twim_run_sequence, each transaction is executed in
the order defined. A transaction result is one of:
- TWIM_TRANSACTION_EXECUTED: indicates completion of both write and read
  phases (if present) of the transaction;
- TWIM_TRANSACTION_NOTREADY: indicates the transaction is currently waiting on
  hardware to execute the write or read components;
- TWIM_TRANSACTION_PENDING: indicates the transaction has been initiated bu
  is not yet complete;
- TWIM_TRANSACTION_SEQDONE: returned if a transaction sequence has a
  terminating TWIM_TRANSACTION_SENTINEL value, and the sequence index points
  to this value.
*/

enum twim_transaction_status_t {
    TWIM_TRANSACTION_STATUS_NONE = 0,
    TWIM_TRANSACTION_STATUS_SENT
};

struct twim_transaction_t {
    uint8_t dev_addr;
    uint8_t tx_len;
    uint8_t tx_buf[4];
    uint8_t rx_len;
    volatile void *rx_buf;
    enum twim_transaction_status_t txn_status;
};

enum twim_transaction_result_t {
    TWIM_TRANSACTION_EXECUTED = 0,
    TWIM_TRANSACTION_NOTREADY,
    TWIM_TRANSACTION_PENDING,
    TWIM_TRANSACTION_SEQDONE,
    TWIM_TRANSACTION_ERROR
};

#define TWIM_TRANSACTION_SENTINEL {0, 0, {0}, 0, NULL, 0}


/*
twim_pdca_cfg_t stores relevant pointers and channel IDs for a TWIM/PDCA
channel combination.
- twim must point to a TWIM instance (AVR32_TWIM0-AVR32_TWIM2);
- tx_pdca_num must be a unique PDCA channel ID (0-31);
- rx_pdca_num must be a unique PDCA channle ID (0-31);
- tx_pid is the PDCA peripheral ID of the TWIM instance TX register
  (AVR32_TWIM0_PDCA_ID_TX-AVR32_TWIM2_PDCA_ID_TX)
- rx_pid is the PDCA peripheral ID of the TWIM instance RX register
  (AVR32_TWIM0_PDCA_ID_RX-AVR32_TWIM2_PDCA_ID_RX)
*/

struct twim_pdca_cfg_t {
    volatile avr32_twim_t *twim;
    uint8_t tx_pdca_num;
    uint8_t rx_pdca_num;
    uint32_t tx_pid;
    uint32_t rx_pid;
};

/*
twim_pdca_init performs basic configuration of a TWIM instance, including
setting the clock waveform generator (CWGR) to the correct parameters for
the desired I2C speed (must be 100-400kHz inclusive).
*/
void twim_pdca_init(struct twim_pdca_cfg_t *cfg, uint32_t speed_hz);


/*
twim_pdca_write executes a write[/read] transaction specified by txn on the
TWIM identified by cfg.
*/
void twim_pdca_transact(struct twim_pdca_cfg_t *cfg,
struct twim_transaction_t *txn);

/*
twim_run_sequence executes the next transaction in seq (indexed by seq_idx);
if both write and read components of the transaction have been successfully
completed it returns TWIM_TRANSACTION_EXECUTED.
*/
enum twim_transaction_result_t twim_run_sequence(struct twim_pdca_cfg_t *cfg,
struct twim_transaction_t seq[], uint32_t seq_idx);

#endif
