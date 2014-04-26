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
#include <string.h>
#include "drivers/i2cdevice.h"
#include "comms.h"
#include "hmc5883.h"
#include "plog/parameter.h"

static uint8_t hmc5883_inbuf[6];

static struct twim_transaction_t init_sequence[] = {
    /*
    Device address, TX byte count, TX bytes (0-4), RX byte count, RX buffer
    With this configuration, full-scale is 2Ga (1090LSB/Ga sensitivity),
    8 samples are averaged per measurement, and measurements are done in
    single mode at ~125Hz.
    */

    /* Write 0x78 to CRA -- 8 samples per measurement, 75Hz nominal, no bias */
    {HMC5883_DEVICE_ADDR, 2u, {0x00u, 0x78u}, 0, NULL},
    /* Write 0x00 to CRB -- gain = 2 (1090LSB/Ga) */
    {HMC5883_DEVICE_ADDR, 2u, {0x01u, 0x20u}, 0, NULL},
    TWIM_TRANSACTION_SENTINEL
};

static struct twim_transaction_t read_sequence[] = {
    /* Write single-measurement start to MODE register (0x01) */
    {HMC5883_DEVICE_ADDR, 2u, {0x02u, 0x01u}, 0, NULL},
    /*
    Read 6 bytes from DXRA -- returns:
    DXRA, DXRB, DZRA, DZRB, DYRA, DYRB (A=MSB, B=LSB)
    */
    {HMC5883_DEVICE_ADDR, 1u, {0x03u}, 6u, hmc5883_inbuf},
    TWIM_TRANSACTION_SENTINEL
};

static struct i2c_device_t hmc5883 = {
    .speed = 100000u,
    .power_delay = 500u,
    .init_timeout = 600u,
    .read_timeout = 15u,

    .sda_pin_id = HMC5883_TWI_TWD_PIN,
    .sda_function = HMC5883_TWI_TWD_FUNCTION,
    .scl_pin_id = HMC5883_TWI_TWCK_PIN,
    .scl_function = HMC5883_TWI_TWCK_FUNCTION,
    .enable_pin_id = HMC5883_ENABLE_PIN,
    .sysclk_id = HMC5883_TWI_SYSCLK,

    .twim_cfg = {
        .twim = HMC5883_TWI,
        .tx_pdca_num = PDCA_CHANNEL_HMC5883_TX,
        .rx_pdca_num = PDCA_CHANNEL_HMC5883_RX,
        .tx_pid = HMC5883_TWI_PDCA_PID_TX,
        .rx_pid = HMC5883_TWI_PDCA_PID_RX
    },

    .init_sequence = init_sequence,
    .read_sequence = read_sequence
};

#ifndef CONTINUE_ON_ASSERT
#define HMC5883Assert(x) Assert(x)
#else
#define HMC5883Assert(x) if (!(x)) { hmc5883.state_timer = 0xffffu; }
#endif

void hmc5883_measure(void);

void hmc5883_init(void) {
    i2c_device_init(&hmc5883);
}

void hmc5883_tick(void) {
    i2c_device_tick(&hmc5883);

    if (hmc5883.state == I2C_READ_SEQUENCE) {
        hmc5883_measure();
    }
}

void hmc5883_measure(void) {
    struct fcs_parameter_t param;
    enum twim_transaction_result_t read_result;
    int16_t measurement[3];

    if (hmc5883.state_timer >= 8u) {
        /*
        Wait until the 8th tick (~7ms after measurement command), then execute
        a read operation to get the latest magnetometer measurement. If the
        command completes, start another measurement.
        */
        read_result = twim_run_sequence(&hmc5883.twim_cfg,
                                        hmc5883.read_sequence, 1u);

        if (read_result == TWIM_TRANSACTION_EXECUTED) {
            /* Convert the result and update the comms module */
            memcpy(measurement, hmc5883_inbuf, sizeof(measurement));

            /*
            Magnetic field over-/underflow -- should maybe adjust sensitivity
            automatically?
            */
            if (!  (-2048 <= measurement[0] && measurement[0] <= 2047 &&
                    -2048 <= measurement[1] && measurement[1] <= 2047 &&
                    -2048 <= measurement[2] && measurement[2] <= 2047)) {
                /* Power the device down */
                gpio_local_clr_gpio_pin(hmc5883.enable_pin_id);
                i2c_device_state_transition(&hmc5883, I2C_POWERING_DOWN);
                return;
            }

            /* Registers are ordered X, Z, Y */
            fcs_parameter_set_header(&param, FCS_VALUE_SIGNED, 16u, 3u);
            fcs_parameter_set_type(&param, FCS_PARAMETER_MAGNETOMETER_XYZ);
            fcs_parameter_set_device_id(&param, 0);
            param.data.i16[0] = swap16(measurement[0]);
            param.data.i16[1] = swap16(measurement[2]);
            param.data.i16[2] = swap16(measurement[1]);
            (void)fcs_log_add_parameter(&comms_out_log, &param);

            hmc5883.state_timer = 0;
        }
    }

    if (hmc5883.state_timer == 0) {
        /*
        At the start of the read state, or after each successful read,
        generate a measurement command.
        */
        hmc5883.read_sequence[0].txn_status = TWIM_TRANSACTION_STATUS_NONE;
        hmc5883.read_sequence[1].txn_status = TWIM_TRANSACTION_STATUS_NONE;
        twim_run_sequence(&hmc5883.twim_cfg, hmc5883.read_sequence, 0);
    }
}
