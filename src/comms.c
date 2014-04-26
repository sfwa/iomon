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

enum rx_buf_parse_state_t {
    RX_NO_MSG = 0,
    RX_START,
    RX_IN_MSG,
    RX_END
};

struct fcs_log_t comms_out_log;
struct fcs_log_t comms_in_log;

#define RX_BUF_LEN 256u
#define TX_BUF_LEN 256u

static uint8_t tx_buf[TX_BUF_LEN];
static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t rx_buf_idx;
static uint8_t inbuf[FCS_LOG_SERIALIZED_LENGTH];
static uint16_t inbuf_idx;
static enum rx_buf_parse_state_t rx_parse_state = RX_NO_MSG;

static uint16_t last_rx_packet_tick;
static uint16_t last_tx_packet_tick;
static uint16_t cpu_reset_countdown_tick;

static bool comms_process_rx_buf(uint32_t bytes_read);

void comms_set_cpu_status(uint32_t cycles_used) {
    CommsAssert(cycles_used < 1000000);

    uint32_t cycles_per_tick = sysclk_get_cpu_hz() / 1000,
             proportion_used = (255u * cycles_used) / cycles_per_tick;
    if (proportion_used > 255u) {
        proportion_used = 255u;
    }
}

uint32_t comms_init(void) {
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

    last_rx_packet_tick = 0;
	last_tx_packet_tick = 0;
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

    return 0;
}

uint16_t comms_tick(void) {
    size_t packet_len;
    irqflags_t flags;
    uint32_t bytes_read, bytes_avail;

    volatile avr32_pdca_channel_t *pdca_channel =
        &AVR32_PDCA.channel[PDCA_CHANNEL_CPU_RX];

    /* Turn LED1 on if we haven't seen each of the sensors updated this second
       */
    static uint8_t sensors_updated = 0;
    if (last_tx_packet_tick % 1000 == 0) {
        if (sensors_updated == (UPDATED_ACCEL | UPDATED_GYRO |
                UPDATED_BAROMETER | UPDATED_MAG | UPDATED_GPS_POS |
                UPDATED_GPS_INFO | UPDATED_ADC_GPIO)) {
            LED_OFF(LED1_GPIO);
        } else {
            LED_ON(LED1_GPIO);
        }
        sensors_updated = 0;
    }

    /* Schedule the transfer over CPU and AUX UARTs */
    flags = cpu_irq_save();

    packet_len = fcs_log_serialize(tx_buf, sizeof(tx_buf),
                                   &comms_out_log);

    pdca_channel = &AVR32_PDCA.channel[PDCA_CHANNEL_CPU_TX];
    pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
    pdca_channel->mar = (uint32_t)tx_buf;
    pdca_channel->tcr = packet_len;
    pdca_channel->marr = 0;
    pdca_channel->tcrr = 0;
    pdca_channel->psr = CPU_USART_PDCA_PID_TX;
    pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
    pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    pdca_channel->isr;

    pdca_channel = &AVR32_PDCA.channel[PDCA_CHANNEL_AUX_TX];
    pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
    pdca_channel->mar = (uint32_t)tx_buf;
    pdca_channel->tcr = packet_len;
    pdca_channel->marr = 0;
    pdca_channel->tcrr = 0;
    pdca_channel->psr = AUX_USART_PDCA_PID_TX;
    pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
    pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
    pdca_channel->isr;

    cpu_irq_restore(flags);

    last_tx_packet_tick++;
    fcs_log_init(&comms_out_log, FCS_LOG_TYPE_MEASUREMENT,
                 last_tx_packet_tick);

    /* Receive data from primary UART */
    bytes_read = RX_BUF_LEN - pdca_channel->tcr;
    bytes_avail = 0;

    CommsAssert(bytes_read <= RX_BUF_LEN);

    /* Work out the number of bytes available in the ring buffer */
    if (bytes_read > rx_buf_idx) {
        bytes_avail = bytes_read - rx_buf_idx;
    } else if (bytes_read < rx_buf_idx) {
        bytes_avail = (RX_BUF_LEN - rx_buf_idx - 1u) + bytes_read;
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
        rx_buf_idx = 0;
        rx_parse_state = RX_NO_MSG;

        irqflags_t flags = cpu_irq_save();
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->mar = (uint32_t)rx_buf;
        pdca_channel->tcr = RX_BUF_LEN;
        pdca_channel->marr = (uint32_t)rx_buf;
        pdca_channel->tcrr = RX_BUF_LEN;
        pdca_channel->psr = CPU_USART_PDCA_PID_RX;
        pdca_channel->mr = (AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET)
            | (1 << AVR32_PDCA_RING_OFFSET);
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
        pdca_channel->isr;
        cpu_irq_restore(flags);
    }

    if (bytes_avail && comms_process_rx_buf(bytes_avail)) {
        last_rx_packet_tick = last_tx_packet_tick;
        LED_ON(LED0_GPIO);
    } else {
        LED_OFF(LED0_GPIO);
    }

    /* Set the CPU reset line if the last received packet was more than 30s
       ago */
    if (cpu_reset_countdown_tick == 0 &&
            last_tx_packet_tick - last_rx_packet_tick > CPU_TIMEOUT_TICKS) {
        cpu_reset_countdown_tick = CPU_RESET_TICKS;
        gpio_local_set_gpio_pin(CPU_RESET_PIN);

        /* Turn PWM off until we get another CPU packet */
        pwm_disable();
    } else if (cpu_reset_countdown_tick == 1u) {
        cpu_reset_countdown_tick = 0;
        last_rx_packet_tick = last_tx_packet_tick;
        gpio_local_clr_gpio_pin(CPU_RESET_PIN);
    } else if (cpu_reset_countdown_tick > 1u) {
        cpu_reset_countdown_tick--;
    } else {
        /* No timeout conditions to handle */
    }
}

static bool comms_process_rx_buf(uint32_t bytes_avail) {
    CommsAssert(bytes_avail <= RX_BUF_LEN);
    CommsAssert(RX_NO_MSG <= rx_parse_state && rx_parse_state <= RX_END);

    bool result = false;
    uint8_t ch;

    for (; bytes_avail; bytes_avail--) {
        ch = rx_buf[rx_buf_idx];
        rx_buf_idx++;

        if (ch == 0x00) {
            if (rx_parse_state == RX_IN_MSG) {
                rx_parse_state = RX_END;
            } else {
                rx_parse_state = RX_START;
            }
        } else {
            if (rx_parse_state == RX_START) {
                rx_parse_state = RX_IN_MSG;
                inbuf_idx = 0;
            } else if (rx_parse_state != RX_IN_MSG) {
                /*
                Didn't get a valid packet -- we're receiving data without
                having found a start byte.
                */
                rx_parse_state = RX_NO_MSG;
            } else {
                /* rx_parse_state == RX_IN_MSG; this is handled below */
            }
        }

        if (rx_parse_state == RX_IN_MSG) {
            /*
            Currently parsing a message, so copy the data to the message
            buffer
            */
            if (inbuf_idx < FCS_LOG_SERIALIZED_LENGTH) {
                inbuf[inbuf_idx] = ch;
                inbuf_idx++;
            } else {
                /*
                Buffer overrun -- messages are only allowed to
                be FCS_LOG_SERIALIZED_LENGTH bytes long so something went
                wrong
                */
                rx_parse_state = RX_NO_MSG;
            }
        } else if (rx_parse_state == RX_END) {
            /* Message was fully received; decode it now */
            result = fcs_log_deserialize(&comms_in_log, inbuf, inbuf_idx);
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
