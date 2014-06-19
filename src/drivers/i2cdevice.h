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

#ifndef _I2CDEVICE_H_
#define _I2CDEVICE_H_

#include "twim_pdca.h"

enum i2c_state_t {
    I2C_POWERING_UP = 0,
    I2C_INIT_SEQUENCE,
    I2C_READ_SEQUENCE,
    I2C_POWERING_DOWN
};

struct i2c_device_t {
    /* I2C device configuration data */
    uint32_t speed; /* bits/sec */
    uint16_t power_delay; /* ticks (ms) */
    uint16_t init_timeout; /* ticks (ms) */
    uint16_t read_timeout; /* ticks (ms) */

    /* Hardware configuration data */
    uint8_t sda_pin_id;
    uint8_t sda_function;
    uint8_t scl_pin_id;
    uint8_t scl_function;
    uint8_t enable_pin_id;
    uint8_t sysclk_id;

    /* TWIM/PDCA configuration */
    struct twim_pdca_cfg_t twim_cfg;

    /* Current device state */
    enum i2c_state_t state;
    uint32_t sequence_idx;
    uint32_t state_timer;

    /* Transaction sequence definitions */
    struct twim_transaction_t *init_sequence;
    struct twim_transaction_t *read_sequence;
};

static inline void i2c_device_state_transition(struct i2c_device_t *dev,
        enum i2c_state_t new_state) {
    fcs_assert(new_state <= I2C_POWERING_DOWN);

    dev->state = new_state;
    dev->state_timer = 0;
    dev->sequence_idx = 0;
}

/*
Initialize I2C device pin configurations and TWIM.
*/
void i2c_device_init(struct i2c_device_t *dev);

/*
Handle periodic I2C update tasks, including timeouts and management of the
init_sequence commands.
*/
void i2c_device_tick(struct i2c_device_t *dev);

#endif
