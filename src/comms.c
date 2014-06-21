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
#include "fcsassert.h"
#include "comms.h"
#include "cobsr.h"
#include "peripherals/pwm.h"
#include "peripherals/ubx_gps.h"
#include "plog/parameter.h"


/*
See https://github.com/bendyer/uav/wiki/IO-Board-Design
*/

#define TELEMETRY_INTERVAL 500u


struct connection_t cpu_conn;
struct connection_t gcs_conn;
struct sensor_status_t sensor_status;

volatile uint32_t g_t[10];

static uint8_t cpu_tx_dma_buf[TX_BUF_LEN];
static uint8_t gcs_tx_dma_buf[TX_BUF_LEN];

static enum fcs_parameter_type_t cpu_feed_params[] = {
    FCS_PARAMETER_DERIVED_REFERENCE_PRESSURE,
    FCS_PARAMETER_DERIVED_REFERENCE_ALT,
    FCS_PARAMETER_NAV_VERSION,
    FCS_PARAMETER_NAV_PATH_ID,
    FCS_PARAMETER_NAV_WAYPOINT_ID,
    FCS_PARAMETER_KEY_VALUE, /* should only be one of these per frame */
    FCS_PARAMETER_LAST  /* terminator */
};

static void comms_process_conn_rx(struct connection_t *conn,
uint32_t channel_id);
static bool comms_process_conn_read(struct connection_t *conn,
uint32_t bytes_avail);

/*
FIXME: Internal fcs_parameter functions -- work out a better way of exposing
performant log scans for sets of parameter types.
*/
static inline size_t _extract_num_values(uint8_t header) {
    size_t num_values;

    if (header & FCS_PARAMETER_HEADER_MODE_MASK) {
        num_values = (
            (header & FCS_PARAMETER_HEADER_DATA_LENGTH_MASK)
            >> FCS_PARAMETER_HEADER_DATA_LENGTH_OFFSET
        );

        if (num_values > FCS_PARAMETER_DATA_LENGTH_MAX) {
            num_values = 0;
        }
    } else {
        num_values = (
            (header & FCS_PARAMETER_HEADER_NUM_VALUES_MASK)
            >> FCS_PARAMETER_HEADER_NUM_VALUES_OFFSET
        ) + 1u;

        if (num_values > FCS_PARAMETER_NUM_VALUES_MAX) {
            num_values = 0;
        }
    }

    return num_values;
}

static inline size_t _extract_precision_bits(uint8_t header) {
    size_t precision_bits;

    if (header & FCS_PARAMETER_HEADER_MODE_MASK) {
        return 8u;
    } else {
        precision_bits = 8u <<
            ((header & FCS_PARAMETER_HEADER_PRECISION_MASK)
             >> FCS_PARAMETER_HEADER_PRECISION_OFFSET);
    }

    return precision_bits;
}

static inline size_t _extract_length(uint8_t header) {
    size_t num_values, precision_bits, length;

    num_values = _extract_num_values(header);
    precision_bits = _extract_precision_bits(header);

    if (!num_values || !precision_bits) {
        length = 0;
    } else {
        /*
        Header length + num values * bytes required to contain precision_bits
        */
        length = 3u + num_values * ((precision_bits + 7u) >> 3u);
    }

    fcs_assert(length < sizeof(struct fcs_parameter_t));

    return length;
}

void comms_set_cpu_status(uint32_t cycles_used) {
    fcs_assert(cycles_used < 1000000);
    fcs_assert(cpu_conn.out_log.length >= 5 &&
               cpu_conn.out_log.length < 1024);

	static uint32_t max_proportion_used = 0;
	struct fcs_parameter_t param;
    uint32_t cycles_per_tick = CONFIG_MAIN_HZ / 1000,
             proportion_used = (255u * cycles_used) / cycles_per_tick;
    if (proportion_used > max_proportion_used) {
        max_proportion_used = proportion_used;
    }

    /* Save this to measurement log */
	fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 16u,
	1u);
	fcs_parameter_set_type(&param, FCS_PARAMETER_IO_STATUS);
	fcs_parameter_set_device_id(&param, 0);
	param.data.u16[0] = max_proportion_used < 0xFFFFu ?
		max_proportion_used : 0xFFFFu;
	(void)fcs_log_add_parameter(&cpu_conn.out_log, &param);
}

void comms_init(void) {
    static usart_options_t usart_options;
    uint32_t result;

    /* Set up serial comms + CPU board interface */
    gpio_enable_module_pin(CPU_USART_RXD_PIN, CPU_USART_RXD_FUNCTION);
    gpio_enable_module_pin(CPU_USART_TXD_PIN, CPU_USART_TXD_FUNCTION);

    /* Configure AUX USART interfaces */
    gpio_enable_module_pin(AUX_USART_RXD_PIN, AUX_USART_RXD_FUNCTION);
    gpio_enable_module_pin(AUX_USART_TXD_PIN, AUX_USART_TXD_FUNCTION);

    /* Configure CPU board reset output */
    gpio_configure_pin(CPU_RESET_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    /* Configure CPU USART */
    usart_options.baudrate = 2604166u;
    usart_options.charlength = 8u;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    result = usart_init_rs232(CPU_USART, &usart_options, CONFIG_MAIN_HZ);
    fcs_assert(result == USART_SUCCESS);

    /* Configure AUX USART */
    usart_options.baudrate = 57600u;
    usart_options.charlength = 8u;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = 5;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    result = usart_init_rs232(AUX_USART, &usart_options, CONFIG_MAIN_HZ);
    fcs_assert(result == USART_SUCCESS);

	fcs_log_init(&cpu_conn.out_log, FCS_LOG_TYPE_MEASUREMENT, 0);
    fcs_log_init(&cpu_conn.in_log, FCS_LOG_TYPE_COMBINED, 0);
	fcs_log_init(&gcs_conn.out_log, FCS_LOG_TYPE_COMBINED, 0);
	fcs_log_init(&gcs_conn.in_log, FCS_LOG_TYPE_COMBINED, 0);
}

void comms_tick(void) {
    size_t packet_len, i, j, param_len;
    enum fcs_parameter_type_t param_type;
    struct fcs_parameter_t param;
    volatile avr32_pdca_channel_t *pdca_channel;

    fcs_assert(FCS_LOG_MIN_LENGTH <= cpu_conn.out_log.length &&
               cpu_conn.out_log.length <= FCS_LOG_MAX_LENGTH);

    g_t[0] = Get_system_register(AVR32_COUNT);

    /*
    Turn LED1 on if we haven't seen each of the sensors updated this second
    */
    if (cpu_conn.last_tx_packet_tick % 150 == 0) {
        if (sensor_status.updated == (UPDATED_ACCEL | UPDATED_BARO |
                                      UPDATED_MAG | UPDATED_GPS |
                                      UPDATED_PITOT)) {
            LED_OFF(LED1_GPIO);
        } else {
            LED_ON(LED1_GPIO);
        }
        sensor_status.updated = 0;
    }

    comms_process_conn_rx(&cpu_conn, PDCA_CHANNEL_CPU_RX);

    g_t[1] = Get_system_register(AVR32_COUNT) - g_t[0];

    comms_process_conn_rx(&gcs_conn, PDCA_CHANNEL_AUX_RX);

    g_t[2] = Get_system_register(AVR32_COUNT) - g_t[0];

    /*
    If there's a waypoint or path update in the telemetry log, add that to the
    measurement log.

    Also add the latest reference pressure and altitude from the telemetry log
    to the measurement log.
    */
    fcs_assert(FCS_LOG_MIN_LENGTH <= gcs_conn.out_log.length &&
               gcs_conn.out_log.length <= FCS_LOG_MAX_LENGTH);

    for (i = 5u; i < gcs_conn.in_log.length - 3u; ) {
        param_len = _extract_length(gcs_conn.in_log.data[i]);
        param_type = (enum fcs_parameter_type_t)cpu_conn.out_log.data[i + 2u];
        if (param_type == FCS_PARAMETER_INVALID || !param_len ||
                param_len > sizeof(struct fcs_parameter_t) ||
                i + param_len > gcs_conn.in_log.length) {
            break;
        }

        for (j = 0; j < 100u && cpu_feed_params[j] != FCS_PARAMETER_LAST;
                j++) {
            if (cpu_feed_params[j] == param_type) {
                memcpy(&param, &gcs_conn.in_log.data[i], param_len);
                (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);
            }
        }

		i += param_len;
    }

    g_t[3] = Get_system_register(AVR32_COUNT) - g_t[0];

    /*
	Only send the telemetry packet every TELEMETRY_INTERVAL ticks. The
	last valid CPU packet is copied directly to the GCS TX buffer so
	leave that there.

    Transmit each set of 240 bytes over 60ms to avoid killing the buffer (100
    bytes).
	*/
	pdca_channel = &AVR32_PDCA.channel[PDCA_CHANNEL_AUX_TX];
    if (pdca_channel->tcr == 0 &&
			cpu_conn.last_tx_packet_tick % TELEMETRY_INTERVAL == 0) {
		pdca_channel->cr = AVR32_PDCA_TDIS_MASK;

		memcpy(gcs_tx_dma_buf, gcs_conn.tx_buf, TX_BUF_LEN);

        pdca_channel->idr = 0xFFFFFFFFu;
        pdca_channel->isr;
        pdca_channel->mar = (uint32_t)gcs_tx_dma_buf;
        pdca_channel->tcr = 60u;
        pdca_channel->marr = 0;
        pdca_channel->tcrr = 0;
        pdca_channel->psr = AUX_USART_PDCA_PID_TX;
        pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    } else if (pdca_channel->tcr == 0 &&
            cpu_conn.last_tx_packet_tick % TELEMETRY_INTERVAL == 50u) {
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->idr = 0xFFFFFFFFu;
        pdca_channel->isr;
        pdca_channel->mar = ((uint32_t)gcs_tx_dma_buf) + 60u;
        pdca_channel->tcr = 60u;
        pdca_channel->marr = 0;
        pdca_channel->tcrr = 0;
        pdca_channel->psr = AUX_USART_PDCA_PID_TX;
        pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    } else if (pdca_channel->tcr == 0 &&
            cpu_conn.last_tx_packet_tick % TELEMETRY_INTERVAL == 100u) {
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->idr = 0xFFFFFFFFu;
        pdca_channel->isr;
        pdca_channel->mar = ((uint32_t)gcs_tx_dma_buf) + 120u;
        pdca_channel->tcr = 60u;
        pdca_channel->marr = 0;
        pdca_channel->tcrr = 0;
        pdca_channel->psr = AUX_USART_PDCA_PID_TX;
        pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    } else if (pdca_channel->tcr == 0 &&
            cpu_conn.last_tx_packet_tick % TELEMETRY_INTERVAL == 150u) {
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->idr = 0xFFFFFFFFu;
        pdca_channel->isr;
        pdca_channel->mar = ((uint32_t)gcs_tx_dma_buf) + 180u;
        pdca_channel->tcr = 60u;
        pdca_channel->marr = 0;
        pdca_channel->tcrr = 0;
        pdca_channel->psr = AUX_USART_PDCA_PID_TX;
        pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    }

    /* Validate the last data buffer */
    fcs_assert(memcmp(cpu_conn.tx_buf, cpu_tx_dma_buf, 192) == 0);

    /* Schedule the measurement log transfer over the CPU UART */
    packet_len = fcs_log_serialize(cpu_conn.tx_buf, 192u, &cpu_conn.out_log);

    g_t[4] = Get_system_register(AVR32_COUNT) - g_t[0];

    for (i = 0; i < packet_len; i++) {
        cpu_conn.tx_buf[191u - i] = cpu_conn.tx_buf[packet_len - i - 1u];
    }
    for (; i < 192u; i++) {
        cpu_conn.tx_buf[191u - i] = 0;
    }

    /*
    Transmission is start by calling comms_start_transmit(); that's done at a
    fixed time every frame to avoid jitter at the DSP end.
    */

    cpu_conn.last_tx_packet_tick++;
    fcs_log_init(&(cpu_conn.out_log), FCS_LOG_TYPE_MEASUREMENT,
                 cpu_conn.last_tx_packet_tick);

	g_t[5] = Get_system_register(AVR32_COUNT) - g_t[0];
}

void comms_start_transmit(void) {
	volatile avr32_pdca_channel_t *pdca_channel;
    size_t i;

    pdca_channel = &AVR32_PDCA.channel[PDCA_CHANNEL_CPU_TX];

    /* Don't start the next transfer until the current one completes */
    if (pdca_channel->tcr) {
        return;
    }

    pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
    pdca_channel->idr = 0xFFFFFFFFu;
    pdca_channel->isr;

    for (i = 0; i < 192u; i++) {
        cpu_tx_dma_buf[i] = cpu_conn.tx_buf[i];
    }

    pdca_channel->mar = (uint32_t)cpu_tx_dma_buf;
    pdca_channel->tcr = 192u;
    pdca_channel->marr = 0;
    pdca_channel->tcrr = 0;
    pdca_channel->psr = CPU_USART_PDCA_PID_TX;
    pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
    pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
}

static void comms_process_conn_rx(struct connection_t *conn,
uint32_t channel_id) {
    size_t bytes_read, bytes_avail;
    bool result;
    volatile avr32_pdca_channel_t *pdca_channel;

    /*
    TODO: not the most elegant way of working out which PDCA channel to use
    */
	pdca_channel = &AVR32_PDCA.channel[channel_id];

    /* Receive data from the UART */
    bytes_read = RX_BUF_LEN - pdca_channel->tcr;
    bytes_avail = 0;
    result = true;

    fcs_assert(bytes_read <= RX_BUF_LEN);

    /* Work out the number of bytes available in the ring buffer */
    if (bytes_read > (conn->rx_buf_idx & 0x1FFu)) {
        bytes_avail = bytes_read - (conn->rx_buf_idx & 0x1FFu);
    } else if (bytes_read < (conn->rx_buf_idx & 0x1FFu)) {
        bytes_avail = (RX_BUF_LEN - (conn->rx_buf_idx & 0x1FFu)) + bytes_read;
    }

    if (bytes_avail > RX_BUF_LEN - 8u) {
        /*
        Either the PDCA isn't initialized, or there's been a possible buffer
        overflow -- either way, re-initialize the RX PDCA channel.
        */
        bytes_read = 0;
        bytes_avail = 0;
        conn->rx_buf_idx = 0;

        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->idr = 0xFFFFFFFFu;
        pdca_channel->isr;

        pdca_channel->mar = (uint32_t)conn->rx_buf;
        pdca_channel->tcr = RX_BUF_LEN;
        pdca_channel->marr = (uint32_t)conn->rx_buf;
        pdca_channel->tcrr = RX_BUF_LEN;
        pdca_channel->psr = CPU_USART_PDCA_PID_RX;
        pdca_channel->mr = (AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET)
            | (1 << AVR32_PDCA_RING_OFFSET);
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    }

    if (bytes_avail) {
        result = comms_process_conn_read(conn, bytes_avail);
    } else if (conn == &cpu_conn) {
		LED_OFF(LED0_GPIO);
		LED_OFF(LED2_GPIO);
	}
}

static bool comms_process_conn_read(struct connection_t *conn,
uint32_t bytes_avail) {
    fcs_assert(bytes_avail <= RX_BUF_LEN);

    bool result = false, got_message = false, did_get_message = false,
         did_get_error = false;
    uint8_t ch;

    while (bytes_avail) {
        ch = conn->rx_buf[conn->rx_buf_idx & 0x1FFu];
        conn->rx_buf_idx++;
        bytes_avail--;

        if (ch == 0x00 || conn->rx_msg_idx) {
            if (ch == 0x00 && conn->rx_msg_idx) {
                if (conn->rx_msg[conn->rx_msg_idx - 1u] == 0x00) {
                    /*
                    This was a false start -- the previous character was
                    actually the end of an incomplete packet rather than the
                    start of a new one.
                    */
                    conn->rx_msg_idx--;
                } else {
                    got_message = true;
                }
            }

            /* in a message */
            conn->rx_msg[conn->rx_msg_idx] = ch;
            conn->rx_msg_idx++;

			if (conn->rx_msg_idx > 510u) {
				conn->rx_msg_idx = 0;
                got_message = false;
			}
        }

        if (got_message) {
			if (conn->rx_msg_idx > 11u &&
                    conn->rx_msg_idx < FCS_LOG_SERIALIZED_LENGTH) {
				/* Message was fully received; decode it now */
				did_get_message = true;

				/* Measurement log packet */
				result = fcs_log_deserialize(&conn->in_log, conn->rx_msg,
				                             conn->rx_msg_idx);
				if (!result) {
				    fcs_log_init(&(conn->in_log), FCS_LOG_TYPE_COMBINED, 0);
					conn->rx_errors++;
				    did_get_error = true;
				} else {
					conn->rx_packets++;
					conn->last_rx_packet_tick = conn->last_tx_packet_tick;

					/* Copy CPU packets straight to the GCS out buffer */
					if (conn == &cpu_conn && conn->rx_msg_idx < TX_BUF_LEN) {
						memcpy(gcs_conn.tx_buf, conn->rx_msg,
						       conn->rx_msg_idx);
					}
				}
			}

            conn->rx_msg_idx = 0;
			got_message = false;
        }
    }

    /*
    Turn on green LED when getting data from the CPU channel, and turn on
    orange LED in response to errors.
    */
    if (conn == &cpu_conn) {
        if (did_get_message) {
            LED_ON(LED0_GPIO);
        } else {
            LED_OFF(LED0_GPIO);
        }

        if (did_get_error) {
            LED_ON(LED2_GPIO);
        } else {
            LED_OFF(LED2_GPIO);
        }
    }

    return result;
}
