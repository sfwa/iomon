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
#include <compiler.h>
#include <string.h>
#include <board.h>
#include "comms.h"
#include "cobsr.h"
#include "peripherals/pwm.h"
#include "peripherals/ubx_gps.h"
#include "plog/parameter.h"


#ifndef CONTINUE_ON_ASSERT
#define CommsAssert(x) Assert(x)
#else
#define CommsAssert(x) if (!(x)) { wdt_reset_mcu(); }
#endif


/*
See https://github.com/bendyer/uav/wiki/IO-Board-Design
*/

/* Wait 10ms after receipt of last message before killing CPU, then
   hold reset high for 0.2s. */
#define CPU_TIMEOUT_TICKS 10u
#define CPU_RESET_TICKS 200u

#define UPDATED_ACCEL 0x01u
#define UPDATED_BAROMETER 0x02u
#define UPDATED_MAG 0x04u
#define UPDATED_GPS 0x08u
#define UPDATED_PITOT 0x10u


struct connection_t cpu_conn;
struct connection_t gcs_conn;

static uint16_t cpu_reset_countdown_tick;

static enum fcs_parameter_type_t telemetry_feed_params[] = {
    FCS_PARAMETER_ESTIMATED_POSITION_LLA,
    FCS_PARAMETER_ESTIMATED_VELOCITY_NED,
    FCS_PARAMETER_ESTIMATED_ATTITUDE_Q,
    FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED,
    FCS_PARAMETER_ESTIMATED_POSITION_SD,
    FCS_PARAMETER_ESTIMATED_VELOCITY_SD,
    FCS_PARAMETER_ESTIMATED_ATTITUDE_SD,
    FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_SD,
    FCS_PARAMETER_AHRS_MODE,
    FCS_PARAMETER_GP_IN,
    FCS_PARAMETER_CONTROL_STATUS,
    FCS_PARAMETER_AHRS_STATUS,
    FCS_PARAMETER_NAV_VERSION,
    FCS_PARAMETER_NAV_PATH,
    FCS_PARAMETER_LAST  /* terminator */
};


static void comms_process_conn_rx(struct connection_t *conn);
static bool comms_process_conn_read(struct connection_t *conn,
uint32_t bytes_avail);


void comms_set_cpu_status(uint32_t cycles_used) {
    CommsAssert(cycles_used < 1000000);

    uint32_t cycles_per_tick = sysclk_get_cpu_hz() / 1000,
             proportion_used = (255u * cycles_used) / cycles_per_tick;
    if (proportion_used > 255u) {
        proportion_used = 255u;
    }

    /* TODO: save this to measurement log */
}

void comms_init(void) {
    /* Set up serial comms + CPU board interface */
    gpio_enable_module_pin(CPU_USART_RXD_PIN, CPU_USART_RXD_FUNCTION);
    gpio_enable_module_pin(CPU_USART_TXD_PIN, CPU_USART_TXD_FUNCTION);
    sysclk_enable_pba_module(CPU_USART_SYSCLK);

    /* Configure AUX USART interfaces */
    gpio_enable_module_pin(AUX_USART_RXD_PIN, AUX_USART_RXD_FUNCTION);
    gpio_enable_module_pin(AUX_USART_TXD_PIN, AUX_USART_TXD_FUNCTION);
    sysclk_enable_pba_module(AUX_USART_SYSCLK);

    /* Configure CPU board reset output */
    gpio_configure_pin(CPU_RESET_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);


    memset(&cpu_conn, 0, sizeof(cpu_conn));
    memset(&gcs_conn, 0, sizeof(gcs_conn));

    cpu_reset_countdown_tick = 0;

    /* Configure CPU USART */
    static usart_options_t usart_options;
    usart_options.baudrate = 2604168u;
    usart_options.charlength = 8u;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    CPU_USART->idr = 0xFFFFFFFFu;
    uint32_t result = usart_init_rs232(CPU_USART, &usart_options,
        sysclk_get_pba_hz());
    CommsAssert(result == USART_SUCCESS);

    /* Configure AUX USART */
    usart_options.baudrate = 2604168u;
    usart_options.charlength = 8u;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    AUX_USART->idr = 0xFFFFFFFFu;
    result = usart_init_rs232(AUX_USART, &usart_options,
        sysclk_get_pba_hz());
    CommsAssert(result == USART_SUCCESS);

	fcs_log_init(&cpu_conn.out_log, FCS_LOG_TYPE_MEASUREMENT, 0);
    fcs_log_init(&cpu_conn.in_log, FCS_LOG_TYPE_COMBINED, 0);
}

void comms_tick(void) {
    size_t packet_len, i;
    irqflags_t flags;
    struct fcs_parameter_t param;
    volatile avr32_pdca_channel_t *pdca_channel;

    static uint8_t sensors_updated = 0;

    if (fcs_parameter_find_by_type_and_device(
            &cpu_conn.out_log, FCS_PARAMETER_ACCELEROMETER_XYZ, 0, &param)) {
        sensors_updated |= UPDATED_ACCEL;
    }
    if (fcs_parameter_find_by_type_and_device(
            &cpu_conn.out_log, FCS_PARAMETER_PRESSURE_TEMP, 0, &param)) {
        sensors_updated |= UPDATED_BAROMETER;
    }
    if (fcs_parameter_find_by_type_and_device(
            &cpu_conn.out_log, FCS_PARAMETER_MAGNETOMETER_XYZ, 0, &param)) {
        sensors_updated |= UPDATED_MAG;
    }
    if (fcs_parameter_find_by_type_and_device(
            &cpu_conn.out_log, FCS_PARAMETER_GPS_POSITION_LLA, 0, &param)) {
        sensors_updated |= UPDATED_GPS;
    }
    if (fcs_parameter_find_by_type_and_device(
            &cpu_conn.out_log, FCS_PARAMETER_PITOT, 0, &param)) {
        sensors_updated |= UPDATED_PITOT;
    }

    /*
    Turn LED1 on if we haven't seen each of the sensors updated this second
    */
    if (cpu_conn.last_tx_packet_tick % 500 == 0) {
        if (sensors_updated == (UPDATED_ACCEL | UPDATED_BAROMETER |
                UPDATED_MAG | UPDATED_GPS | UPDATED_PITOT)) {
            LED_OFF(LED1_GPIO);
        } else {
            LED_ON(LED1_GPIO);
        }
        sensors_updated = 0;
    }

    comms_process_conn_rx(&cpu_conn);
    comms_process_conn_rx(&gcs_conn);

    /* Prepare the telemetry packet for transfer over the telemetry UART */
    fcs_log_init(&gcs_conn.out_log, FCS_LOG_TYPE_COMBINED,
                 cpu_conn.last_tx_packet_tick);
    for (i = 0; i < 100u && telemetry_feed_params[i] != FCS_PARAMETER_LAST;
            i++) {
        if (fcs_parameter_find_by_type_and_device(&cpu_conn.in_log,
                                                  telemetry_feed_params[i], 0,
                                                  &param)) {
            fcs_log_add_parameter(&gcs_conn.out_log, &param);
        }
    }

    packet_len = fcs_log_serialize(gcs_conn.tx_buf, sizeof(gcs_conn.tx_buf),
                                   &gcs_conn.out_log);

    flags = cpu_irq_save();
    pdca_channel = &AVR32_PDCA.channel[PDCA_CHANNEL_AUX_TX];
    pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
    pdca_channel->mar = (uint32_t)gcs_conn.tx_buf;
    pdca_channel->tcr = packet_len;
    pdca_channel->marr = 0;
    pdca_channel->tcrr = 0;
    pdca_channel->psr = AUX_USART_PDCA_PID_TX;
    pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
    pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    pdca_channel->isr;
    cpu_irq_restore(flags);

    /* Schedule the transfer over CPU and AUX UARTs */
    packet_len = fcs_log_serialize(cpu_conn.tx_buf, sizeof(cpu_conn.tx_buf),
                                   &cpu_conn.out_log);

    flags = cpu_irq_save();
    pdca_channel = &AVR32_PDCA.channel[PDCA_CHANNEL_CPU_TX];
    pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
    pdca_channel->mar = (uint32_t)cpu_conn.tx_buf;
    pdca_channel->tcr = packet_len;
    pdca_channel->marr = 0;
    pdca_channel->tcrr = 0;
    pdca_channel->psr = CPU_USART_PDCA_PID_TX;
    pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
    pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    pdca_channel->isr;
    cpu_irq_restore(flags);

    cpu_conn.last_tx_packet_tick++;
    fcs_log_init(&cpu_conn.out_log, FCS_LOG_TYPE_MEASUREMENT,
                 cpu_conn.last_tx_packet_tick);

    /* Set the CPU reset line if the last received packet was more than 30s
       ago */
    if (cpu_reset_countdown_tick == 0 &&
            cpu_conn.last_tx_packet_tick - cpu_conn.last_rx_packet_tick >
            CPU_TIMEOUT_TICKS) {
        cpu_reset_countdown_tick = CPU_RESET_TICKS;
        gpio_local_set_gpio_pin(CPU_RESET_PIN);

        /* Turn PWM off until we get another CPU packet */
        pwm_disable();
    } else if (cpu_reset_countdown_tick == 1u) {
        cpu_reset_countdown_tick = 0;
        cpu_conn.last_rx_packet_tick = cpu_conn.last_tx_packet_tick;
        gpio_local_clr_gpio_pin(CPU_RESET_PIN);
    } else if (cpu_reset_countdown_tick > 1u) {
        cpu_reset_countdown_tick--;
    } else {
        /* No timeout conditions to handle */
    }
}

static void comms_process_conn_rx(struct connection_t *conn) {
    irqflags_t flags;
    size_t bytes_read, bytes_avail;
    volatile avr32_pdca_channel_t *pdca_channel;

    /*
    TODO: not the most elegant way of working out which PDCA channel to use
    */
    if (conn == &cpu_conn) {
        pdca_channel = &AVR32_PDCA.channel[PDCA_CHANNEL_CPU_RX];
    } else {
        pdca_channel = &AVR32_PDCA.channel[PDCA_CHANNEL_AUX_RX];
    }

    /* Receive data from the UART */
    bytes_read = RX_BUF_LEN - pdca_channel->tcr;
    bytes_avail = 0;

    CommsAssert(bytes_read <= RX_BUF_LEN);

    /* Work out the number of bytes available in the ring buffer */
    if (bytes_read > conn->rx_buf_idx) {
        bytes_avail = bytes_read - conn->rx_buf_idx;
    } else if (bytes_read < conn->rx_buf_idx) {
        bytes_avail = (RX_BUF_LEN - conn->rx_buf_idx - 1u) + bytes_read;
    } else {
        /* bytes_read == rx_buf_idx, i.e. nothing new has been read */
        bytes_avail = 0;
    }

    if (bytes_avail > RX_BUF_LEN >> 1u) {
        /*
        Either the PDCA isn't initialized, or there's been a possible buffer
        overflow -- either way, re-initialize the RX PDCA channel.
        */
        bytes_read = 0;
        bytes_avail = 0;
        conn->rx_buf_idx = 0;
        conn->rx_parse_state = RX_NO_MSG;

        flags = cpu_irq_save();
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->mar = (uint32_t)conn->rx_buf;
        pdca_channel->tcr = RX_BUF_LEN;
        pdca_channel->marr = (uint32_t)conn->rx_buf;
        pdca_channel->tcrr = RX_BUF_LEN;
        pdca_channel->psr = CPU_USART_PDCA_PID_RX;
        pdca_channel->mr = (AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET)
            | (1 << AVR32_PDCA_RING_OFFSET);
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
        pdca_channel->isr;
        cpu_irq_restore(flags);
    }

    if (bytes_avail && comms_process_conn_read(conn, bytes_avail)) {
        conn->last_rx_packet_tick = conn->last_tx_packet_tick;
        LED_ON(LED0_GPIO);
    } else {
        LED_OFF(LED0_GPIO);
    }
}

static bool comms_process_conn_read(struct connection_t *conn,
uint32_t bytes_avail) {
    CommsAssert(bytes_avail <= RX_BUF_LEN);
    CommsAssert(RX_NO_MSG <= conn->rx_parse_state &&
                conn->rx_parse_state <= RX_END);

    bool result = false;
    uint8_t ch, buf[15];
    struct cobsr_decode_result decode_result;
    struct fcs_parameter_t param;

    for (; bytes_avail; bytes_avail--) {
        ch = conn->rx_buf[conn->rx_buf_idx];
        conn->rx_buf_idx++;

        if (ch == 0x00) {
            if (conn->rx_parse_state == RX_IN_MSG) {
                conn->rx_parse_state = RX_END;
            } else {
                conn->rx_parse_state = RX_START;
            }
        } else {
            if (conn->rx_parse_state == RX_START) {
                conn->rx_parse_state = RX_IN_MSG;
                conn->rx_msg_idx = 0;
            } else if (conn->rx_parse_state != RX_IN_MSG) {
                /*
                Didn't get a valid packet -- we're receiving data without
                having found a start byte.
                */
                conn->rx_parse_state = RX_NO_MSG;
            } else {
                /* rx_parse_state == RX_IN_MSG; this is handled below */
            }
        }

        if (conn->rx_parse_state == RX_IN_MSG) {
            /*
            Currently parsing a message, so copy the data to the message
            buffer
            */
            if (conn->rx_msg_idx < FCS_LOG_SERIALIZED_LENGTH) {
                conn->rx_msg[conn->rx_msg_idx] = ch;
                conn->rx_msg_idx++;
            } else {
                /*
                Buffer overrun -- messages are only allowed to
                be FCS_LOG_SERIALIZED_LENGTH bytes long so something went
                wrong
                */
                conn->rx_parse_state = RX_NO_MSG;
            }
        } else if (conn->rx_parse_state == RX_END) {
            /* Message was fully received; decode it now */
            if (conn->rx_msg[2] == 0x11u && conn->rx_msg[3] == 0xcu) {
                /*
                RFD900 status packet -- decode and if it's valid, add it to
                the CPU output parameter log.
                */
                result = true;

                decode_result = cobsr_decode(buf, sizeof(buf),
                                             &conn->rx_msg[1],
                                             conn->rx_msg_idx - 2u);
                if (decode_result.status == COBSR_DECODE_OK &&
                        decode_result.out_len == 14u) {
                    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 8u,
                                             4u);
                    fcs_parameter_set_type(&param, FCS_PARAMETER_RADIO);
                    fcs_parameter_set_device_id(&param, 0);
                    param.data.u8[0] = buf[3]; /* RSSI */
                    param.data.u8[1] = buf[6]; /* Noise */
                    param.data.u8[2] = buf[8]; /* # errors LSB */
                    param.data.u8[3] = buf[10]; /* # errors fixed LSB */
                    (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);
                }
            } else {
                /* Measurement log packet */
                result = fcs_log_deserialize(&conn->in_log, conn->rx_msg,
                                             conn->rx_msg_idx);
            }
        } else {
            /*
            rx_parse_state != IN_MSG && rx_parse_state != RX_END
            This was handled in the if/else block above, so nothing more
            to do.
            */
        }
    }

    return result;
}
