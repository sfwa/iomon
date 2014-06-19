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

#ifndef _SPIDEVICE_H_
#define _SPIDEVICE_H_

#include "spim_pdca.h"

enum spi_state_t {
    SPI_POWERING_UP = 0,
    SPI_INIT_SEQUENCE,
    SPI_READ_SEQUENCE,
    SPI_POWERING_DOWN
};

struct spi_device_t {
    /* SPI device configuration data */
    uint32_t speed; /* Hz */
    uint16_t power_delay; /* ticks (ms) */
    uint16_t init_timeout; /* ticks (ms) */
    uint16_t read_timeout; /* ticks (ms) */

    /* Hardware configuration data */
    uint8_t miso_pin_id;
    uint8_t miso_function;
    uint8_t mosi_pin_id;
    uint8_t mosi_function;
    uint8_t cs_pin_id;
    uint8_t cs_function;
    uint8_t clk_pin_id;
    uint8_t clk_function;
    uint8_t enable_pin_id;
    uint8_t sysclk_id;

    /* SPIM/PDCA configuration */
    struct spim_pdca_cfg_t spim_cfg;

    /* Current device state */
    enum spi_state_t state;
    uint32_t sequence_idx;
    uint32_t state_timer;

    /* Transaction sequence definitions */
    struct spim_transaction_t *init_sequence;
    struct spim_transaction_t *read_sequence;
};

static inline void spi_device_state_transition(struct spi_device_t *dev,
        enum spi_state_t new_state) {
    fcs_assert(new_state <= SPI_POWERING_DOWN);

    dev->state = new_state;
    dev->state_timer = 0;
    dev->sequence_idx = 0;
}

/*
Initialize SPI device pin configurations and SPIM.
*/
void spi_device_init(struct spi_device_t *dev);

/*
Handle periodic SPI update tasks, including timeouts and management of the
init_sequence commands.
*/
void spi_device_tick(struct spi_device_t *dev);

#endif
