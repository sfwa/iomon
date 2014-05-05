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
#include "fcsassert.h"
#include "comms.h"
#include "drivers/spidevice.h"
#include "mpu6000.h"
#include "plog/parameter.h"

static struct spim_transaction_t init_sequence[] = {
    /*
    TX byte count, TX bytes (0-4), RX byte count, RX buffer

    With this configuration, accel and gyro are sampled at 8kHz with
    accelerometer LPF off (260Hz), gyro LPF off (256Hz), accel latency at 0ms
    and gyro latency at 0.98ms.

    Full-scale on accel is 8g, and gyro is 500deg/s.
    */

    /* Write 0x15 to USER_CTRL -- disables I2C interface and resets FIFO and
       signal path. */
    {2u, {0x6au, 0x15u}, {0, 0}},
    /* Write 0x02 to RA_PWR_MGMT_1 -- sets clock source to gyro w/ PLL */
    {2u, {0x6bu, 0x02u}, {0, 0}},
    /* Write 0x00 to RA_SMPLRT_DIV -- 8000/(1+0) = 8kHz */
    {2u, {0x19u, 0x00u}, {0, 0}},
    /* Write 0x00 to RA_CONFIG -- disable FSync, no/256Hz low-pass */
    {2u, {0x1au, 0x00u}, {0, 0}},
    /* Write 0x08 to RA_GYRO_CONFIG -- no self test, scale 500deg/s */
    {2u, {0x1bu, 0x08u}, {0, 0}},
    /* Write 0x10 to RA_ACCEL_CONFIG -- no self test, scale of +-8g, no HPF */
    {2u, {0x1cu, 0x10u}, {0, 0}},
    /* Write 0x00 to RA_SIGNAL_PATH_RESET -- reset sensor signal paths */
    {2u, {0x68u, 0x00u}, {0, 0}},
    SPIM_TRANSACTION_SENTINEL
};

static struct spim_transaction_t read_sequence[] = {
    /* Read 14 bytes from RA_ACCEL_XOUT_H -- returns:
       AX.H, AX.L, AY.H, AY.L, AZ.H, AZ.L,
       TEMP.H, TEMP.L,
       GX.H, GX.L, GY.H, GY.L, GZ.H, GZ.L */
    {15u, {0x3bu | 0x80, 0}, {0}},
    SPIM_TRANSACTION_SENTINEL
};

static struct spi_device_t mpu6000 = {
    .speed = 1000000u,
    .power_delay = 100u,
    .init_timeout = 150u,
    .read_timeout = 5u,

    .miso_pin_id = MPU6000_SPI_MISO_PIN,
    .miso_function = MPU6000_SPI_MISO_FUNCTION,
    .mosi_pin_id = MPU6000_SPI_MOSI_PIN,
    .mosi_function = MPU6000_SPI_MOSI_FUNCTION,
    .cs_pin_id = MPU6000_SPI_CS_PIN,
    .cs_function = MPU6000_SPI_CS_FUNCTION,
    .clk_pin_id = MPU6000_SPI_CLK_PIN,
    .clk_function = MPU6000_SPI_CLK_FUNCTION,
    .enable_pin_id = MPU6000_ENABLE_PIN,
    .sysclk_id = MPU6000_SPI_SYSCLK,

    .spim_cfg = {
        .spim = MPU6000_SPI,
        .tx_pdca_num = PDCA_CHANNEL_MPU6000_TX,
        .rx_pdca_num = PDCA_CHANNEL_MPU6000_RX,
        .tx_pid = MPU6000_SPI_PDCA_PID_TX,
        .rx_pid = MPU6000_SPI_PDCA_PID_RX
    },

    .init_sequence = init_sequence,
    .read_sequence = read_sequence
};

void mpu6000_init(void) {
    spi_device_init(&mpu6000);
}

void mpu6000_tick(void) {
    struct fcs_parameter_t param;
    int16_t data[7];

    spi_device_tick(&mpu6000);

    if (mpu6000.state == SPI_READ_SEQUENCE &&
            spim_run_sequence(&mpu6000.spim_cfg, mpu6000.read_sequence, 0) ==
            SPIM_TRANSACTION_EXECUTED) {
        /*
        Convert the result and update the comms module.
        Accel XYZ is in data[0:3], temp is in data [3], and gyro XYZ is in
        data[4:7] (Python slice notation).
        */
        memcpy(data, &read_sequence[0].rx_buf[1], sizeof(data));

        fcs_parameter_set_header(&param, FCS_VALUE_SIGNED, 16u, 3u);
        fcs_parameter_set_type(&param, FCS_PARAMETER_ACCELEROMETER_XYZ);
        fcs_parameter_set_device_id(&param, 0);
        param.data.i16[0] = swap16(data[0]);
        param.data.i16[1] = swap16(data[1]);
        param.data.i16[2] = swap16(data[2]);
        (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);

        fcs_parameter_set_type(&param, FCS_PARAMETER_GYROSCOPE_XYZ);
        fcs_parameter_set_device_id(&param, 0);
        param.data.i16[0] = swap16(data[4]);
        param.data.i16[1] = swap16(data[5]);
        param.data.i16[2] = swap16(data[6]);
        (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);

        mpu6000.state_timer = 0;
        /*
        Start the next read to make sure there are values ready next tick
        */
        spim_run_sequence(&mpu6000.spim_cfg, mpu6000.read_sequence, 0);
    }
}
