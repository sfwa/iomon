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
#include <string.h>
#include "comms.h"
#include "drivers/i2cdevice.h"
#include "ms4525.h"
#include "plog/parameter.h"

static uint8_t data_buf[4];

static struct twim_transaction_t read_sequence[] = {
    {MS4525_DEVICE_ADDR, 0u, {0x00u}, 0, NULL},      /* READ_MR */
    {MS4525_DEVICE_ADDR, 0u, {0x00u}, 4u, data_buf},   /* READ_DF4 */
    TWIM_TRANSACTION_SENTINEL
};

static struct i2c_device_t ms4525 = {
    .speed = 100000u,
    .power_delay = 100u,
    .init_timeout = 30u,
    .read_timeout = 30u,

    .sda_pin_id = MS4525_TWI_TWD_PIN,
    .sda_function = MS4525_TWI_TWD_FUNCTION,
    .scl_pin_id = MS4525_TWI_TWCK_PIN,
    .scl_function = MS4525_TWI_TWCK_FUNCTION,
    .enable_pin_id = MS4525_ENABLE_PIN,
    .sysclk_id = MS4525_TWI_SYSCLK,

    .twim_cfg = {
        .twim = MS4525_TWI,
        .tx_pdca_num = PDCA_CHANNEL_MS4525_TX,
        .rx_pdca_num = PDCA_CHANNEL_MS4525_RX,
        .tx_pid = MS4525_TWI_PDCA_PID_TX,
        .rx_pid = MS4525_TWI_PDCA_PID_RX
    },

    .init_sequence = read_sequence,
    .read_sequence = read_sequence
};

#ifndef CONTINUE_ON_ASSERT
#define MS4525Assert(x) Assert(x)
#else
#define MS4525Assert(x) if (!(x)) { ms4525.state_timer = 0xffffu; }
#endif

void ms4525_init(void) {
    i2c_device_init(&ms4525);
}

void ms4525_tick(void) {
    uint16_t pressure, temp;
    uint8_t status;
    struct fcs_parameter_t param;

    i2c_device_tick(&ms4525);
    if (ms4525.state != I2C_READ_SEQUENCE) {
        return;
    }

    enum twim_transaction_result_t result;
    result = twim_run_sequence(&ms4525.twim_cfg, ms4525.read_sequence,
                               ms4525.sequence_idx);
    if (result != TWIM_TRANSACTION_EXECUTED) {
        return;
    }

    /* Convert the result and update the comms module */
    if (ms4525.sequence_idx == 1u) {
        ms4525.sequence_idx = 0;

        status = (data_buf[0] >> 6u) & 0x3u;
        pressure = ((data_buf[0] << 8u) + data_buf[1]) & 0x3FFFu;
        temp = ((data_buf[2] << 8u) + data_buf[3]) & 0x3FFFu;

        if (status == 0) {
            fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 16u, 2u);
            fcs_parameter_set_type(&param, FCS_PARAMETER_PITOT);
            fcs_parameter_set_device_id(&param, 0);
            param.data.i16[0] = swap16(pressure);
            param.data.i16[1] = swap16(temp);
            (void)fcs_log_add_parameter(&comms_out_log, &param);
        } else {
            /* Something went wrong */
            i2c_device_state_transition(&ms4525, I2C_POWERING_DOWN);
        }
    } else {
        ms4525.sequence_idx++;
    }
}
