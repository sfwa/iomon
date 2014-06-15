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
#include "ubx_gps.h"
#include "plog/parameter.h"

static void ubx_process_latest_msg(void);

inline static int16_t clamp_s16(int32_t v) {
    int16_t result;
    if (v < INT16_MIN) {
        result = INT16_MIN;
    } else if (v > INT16_MAX) {
        result = INT16_MAX;
    } else {
        result = (int16_t)v;
    }

    return result;
}

enum ubx_state_t {
    UBX_POWERING_UP = 0,
    UBX_NAVIGATING,
    UBX_POWERING_DOWN
};

enum ubx_msg_parser_state_t {
    UBX_PARSER_NO_MSG = 0,
    UBX_PARSER_PREFIX_B5,
    UBX_PARSER_PREFIX_62,
    UBX_PARSER_MSG_CLASS,
    UBX_PARSER_MSG_ID,
    UBX_PARSER_LENGTH_0,
    UBX_PARSER_LENGTH_1,
    UBX_PARSER_DONE_MSG
};

struct ubx_nav_pvt_t {
    uint32_t iTOW; /* GPS time of week in ms */
    uint16_t year; /* UTC year */
    uint8_t month; /* UTC month (1-12) */
    uint8_t day; /* UTC day (1-31) */
    uint8_t hour; /* UTC hour (0-23) */
    uint8_t min; /* UTC minute (0-59) */
    uint8_t sec; /* UTC second (0-60, inc leap second) */
    uint8_t valid; /* Validity flags: 0x01 = valid UTC date,
                      0x02 = valid UTC time,
                      0x04 = fully resolved (no seconds uncertainty) */
    uint32_t tAcc; /* Time accuracy estimate (ns) */
    int32_t nano; /* Fraction of a second, -1e9 to 1e9 */
    uint8_t fixType; /* Fix type: 0x00 = no fix, 0x01 = dead reckoning,
                        0x02 = 2D, 0x03 = 3D, 0x04 = GPS + dead reckoning,
                        0x05 = time only */
    uint8_t flags; /* Fix status flags:
                      flags & 0x01 = GPS fix OK,
                      flags & 0x02 = differential fix,
                      (flags & 0x1c) >> 2 = power save mode --
                        * 0 = n/a
                        * 1 = enabled
                        * 2 = acquisition
                        * 3 = tracking
                        * 4 = power optimised tracking
                        * 5 = inactive */
    uint8_t reserved1;
    uint8_t numSV; /* Number of SVs tracked */
    int32_t lon; /* in 1e-7 deg */
    int32_t lat; /* in 1e-7 deg */
    int32_t height; /* in mm above ellipsoid */
    int32_t hMSL; /* in mm above mean sea level */
    uint32_t hAcc; /* horizontal accuracy, in mm error */
    uint32_t vAcc; /* vertical accuracy, in mm error */
    int32_t velN; /* NED north velocity, mm/s */
    int32_t velE; /* NED east velocity, mm/s */
    int32_t velD; /* NED down velocity, mm/s */
    int32_t gSpeed; /* Ground speed, mm/s */
    int32_t heading; /* Heading of motion, in 1e-5 deg */
    uint32_t sAcc; /* Speed accuracy estimate, mm/s */
    uint32_t headingAcc; /* Heading accuracy estimate, 1e-5 deg */
    uint16_t pDOP; /* Position dilution of precision, 1 LSB = 0.01 */
    uint16_t reserved2;
    uint32_t reserved3;
} __attribute__ ((packed));

#define UBX_INBUF_SIZE 256u
#define UBX_MSGBUF_SIZE 256u
/*
2 prefix u8, 1 message class u8, 1 message ID u8, 1 paylod length u16, 1
checksum u16
*/
#define UBX_PREFIX_LEN 6u
#define UBX_SUFFIX_LEN 2u
#define UBX_MSG_OVERHEAD (UBX_PREFIX_LEN + UBX_SUFFIX_LEN)

/* Timeout values */
#define UBX_POWER_DELAY 500u
#define UBX_TIMEOUT 1500u

/* GPS class and message IDs */
#define UBX_MSG_NAV_PVT "\x01\x07"

/* Sanity checks */
#define Ubx_state_is_valid(x) \
    (UBX_POWERING_UP <= (x) && (x) <= UBX_POWERING_DOWN)
#define Ubx_parser_state_is_valid(x) \
    (UBX_PARSER_NO_MSG <= (x) && (x) <= UBX_PARSER_DONE_MSG)

static enum ubx_state_t ubx_state = UBX_POWERING_DOWN;
static uint32_t ubx_state_timer; /* tracks time in current state */

static uint8_t ubx_inbuf[UBX_INBUF_SIZE];
static uint8_t ubx_msgbuf[UBX_MSGBUF_SIZE];

static enum ubx_msg_parser_state_t ubx_inbuf_parse_state =
    UBX_PARSER_NO_MSG;
static uint8_t ubx_inbuf_msg_len_remaining, ubx_inbuf_idx, ubx_inbuf_msg_ck_a,
    ubx_inbuf_msg_ck_b, ubx_msgbuf_idx, ubx_inbuf_msg_class, ubx_inbuf_msg_id;

static enum gps_fix_mode_t ubx_last_fix_mode = GPS_FIX_NONE;

/* Check GPS message types */
#define Ubx_got_msg(msg_id) (ubx_inbuf_msg_class == msg_id[0] && \
    ubx_inbuf_msg_id == msg_id[1])

static inline void ubx_state_transition(enum ubx_state_t new_state) {
    fcs_assert(new_state != ubx_state);
    fcs_assert(Ubx_state_is_valid(new_state));

    ubx_state = new_state;
    ubx_state_timer = 0;
}

void ubx_init(void) {
    ubx_state_timer = 0;
    ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;
    ubx_inbuf_msg_len_remaining = 0;
    ubx_inbuf_idx = 0;
    ubx_msgbuf_idx = 0;

    /* USART GPIO pin configuration and sysclk setup */
    gpio_enable_module_pin(GPS_USART_RXD_PIN, GPS_USART_RXD_FUNCTION);
    gpio_enable_module_pin(GPS_USART_TXD_PIN, GPS_USART_TXD_FUNCTION);
    sysclk_enable_pba_module(GPS_USART_SYSCLK);

    /* Configure GPS power enable */
    gpio_configure_pin(GPS_ENABLE_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    /* Enable USART -- start in 921600 baud mode */
    usart_options_t usart_options;
    usart_options.baudrate = 921600u;
    usart_options.charlength = 8u;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    GPS_USART->idr = 0xFFFFFFFFu;
    uint32_t result = usart_init_rs232(GPS_USART, &usart_options,
        sysclk_get_pba_hz());
    fcs_assert(result == USART_SUCCESS);
}

/*
ubx_tick is called once per millisecond to handle message parsing and periodic
tasks
*/
void ubx_tick(void) {
    fcs_assert(Ubx_state_is_valid(ubx_state));
    fcs_assert(Ubx_parser_state_is_valid(ubx_inbuf_parse_state));

    ubx_state_timer++;

    /* Parse messages appearing in the input buffer */
    volatile avr32_pdca_channel_t *pdca_channel =
        &AVR32_PDCA.channel[PDCA_CHANNEL_GPS_RX];

    /* Reset message done flag */
    if (ubx_inbuf_parse_state == UBX_PARSER_DONE_MSG) {
        ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;
    }

    uint32_t inbuf_bytes_read = UBX_INBUF_SIZE - pdca_channel->tcr;
    uint32_t bytes_avail = 0;

    fcs_assert(inbuf_bytes_read <= UBX_INBUF_SIZE);

    /* Work out the number of bytes available in the ring buffer */
    if (inbuf_bytes_read > ubx_inbuf_idx) {
        bytes_avail = inbuf_bytes_read - ubx_inbuf_idx;
    } else if (inbuf_bytes_read < ubx_inbuf_idx) {
        bytes_avail = (UBX_INBUF_SIZE - ubx_inbuf_idx) + inbuf_bytes_read;
    } else {
        /* inbuf_bytes_read == ubx_inbuf_idx, i.e. no new bytes read */
        bytes_avail = 0;
    }

    if (bytes_avail > UBX_INBUF_SIZE / 2u) {
        /*
        Either the PDCA isn't initialized, or there's been a possible buffer
        overflow -- either way, re-initialize the RX PDCA channel.
        */
        inbuf_bytes_read = 0;
        bytes_avail = 0;
        ubx_inbuf_idx = 0;
        ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;

        irqflags_t flags = cpu_irq_save();
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->mar = (uint32_t)ubx_inbuf;
        pdca_channel->tcr = UBX_INBUF_SIZE;
        pdca_channel->marr = (uint32_t)ubx_inbuf;
        pdca_channel->tcrr = UBX_INBUF_SIZE;
        pdca_channel->psr = GPS_USART_PDCA_PID_RX;
        pdca_channel->mr = (AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET)
            | (1 << AVR32_PDCA_RING_OFFSET);
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
        pdca_channel->isr;
        cpu_irq_restore(flags);
    }

    for (; bytes_avail && ubx_inbuf_parse_state != UBX_PARSER_DONE_MSG;
            bytes_avail--) {
        uint8_t ch = ubx_inbuf[ubx_inbuf_idx];
        ubx_inbuf_idx = (ubx_inbuf_idx + 1) & (UBX_INBUF_SIZE-1);

        if (ubx_inbuf_parse_state == UBX_PARSER_NO_MSG && ch == 0xb5u) {
            /* Got the first byte of a message prefix */
            ubx_inbuf_parse_state = UBX_PARSER_PREFIX_B5;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_PREFIX_B5 && ch == 0x62u) {
            /* Second byte of a message prefix */
            ubx_inbuf_parse_state = UBX_PARSER_PREFIX_62;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_PREFIX_62) {
            /* Got a class ID */
            ubx_inbuf_msg_ck_a = ubx_inbuf_msg_ck_b = 0;
            ubx_inbuf_msg_class = ch;
            ubx_inbuf_parse_state = UBX_PARSER_MSG_CLASS;

            ubx_inbuf_msg_ck_a += ch;
            ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_MSG_CLASS) {
            /* Got a message ID */
            ubx_inbuf_msg_id = ch;
            ubx_inbuf_parse_state = UBX_PARSER_MSG_ID;

            ubx_inbuf_msg_ck_a += ch;
            ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_MSG_ID &&
                ch <= UBX_MSGBUF_SIZE - UBX_SUFFIX_LEN) {
            /* Got the first length byte */
            ubx_inbuf_parse_state = UBX_PARSER_LENGTH_0;

            /* Read the lower byte of the message length */
            ubx_inbuf_msg_len_remaining = ch + UBX_SUFFIX_LEN;
            ubx_msgbuf_idx = 0;

            ubx_inbuf_msg_ck_a += ch;
            ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_LENGTH_0 &&
                ch == 0x00) {
            ubx_inbuf_parse_state = UBX_PARSER_LENGTH_1;

            ubx_inbuf_msg_ck_a += ch;
            ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_LENGTH_1) {
            /*
            Copy the current character into the buffer, and update the CRC
            value if the character isn't part of the message suffix.
            */
            ubx_msgbuf[ubx_msgbuf_idx] = ch;
            ubx_msgbuf_idx++;

            if (ubx_inbuf_msg_len_remaining > UBX_SUFFIX_LEN) {
                ubx_inbuf_msg_ck_a += ch;
                ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
            }
            ubx_inbuf_msg_len_remaining--;

            if (ubx_inbuf_msg_len_remaining == 0u) {
                /*
                Received all the message data -- confirm the checksum bytes
                are correct.
                */
                if (ubx_inbuf_msg_ck_a == ubx_msgbuf[ubx_msgbuf_idx - 2] &&
                        ubx_inbuf_msg_ck_b == ubx_msgbuf[ubx_msgbuf_idx - 1]) {
                    ubx_inbuf_parse_state = UBX_PARSER_DONE_MSG;
                } else {
                    ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;
                }
            }
        } else {
            ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;
        }
    }

    /* GPS driver state processing */
    if (ubx_state_timer > UBX_TIMEOUT) {
        /* Power GPS down */
        gpio_local_clr_gpio_pin(GPS_ENABLE_PIN);
        ubx_state_transition(UBX_POWERING_DOWN);
    } else if (ubx_state_timer > UBX_POWER_DELAY) {
        if (ubx_state == UBX_POWERING_DOWN) {
            /*
            The UBX_POWERING_DOWN state holds for 500ms, powers the GPS unit
            back up, then transfers to UBX_POWERING_UP.
            */
            gpio_local_set_gpio_pin(GPS_ENABLE_PIN);
            ubx_state_transition(UBX_POWERING_UP);
        } else if (ubx_state == UBX_POWERING_UP) {
            /*
            The UBX_POWERING_UP holds for 500ms, then transitions to the
            navigating state.
            */
            ubx_last_fix_mode = GPS_FIX_NONE;
            ubx_state_transition(UBX_NAVIGATING);
        } else {
            /*
            Not in a powering up/down state, so UBX_POWER_DELAY is irrelevant
            */
        }
    } else {
        /* Not timed out, so continue processing normally */
    }

    /* If a valid message has been received, process it */
    if (ubx_inbuf_parse_state == UBX_PARSER_DONE_MSG &&
            ubx_state == UBX_NAVIGATING) {
        ubx_process_latest_msg();
    }
}

static void ubx_process_latest_msg(void) {
    struct fcs_parameter_t param;
    struct ubx_nav_pvt_t msg;
    uint32_t pos_err;

    /*
    UBX_NAVIGATING holds until more than UBX_TIMEOUT ticks elapse between
    received packets.
    */
    if (Ubx_got_msg(UBX_MSG_NAV_PVT)) {
        memcpy(&msg, ubx_msgbuf, sizeof(msg));

        /* Translate fix modes */
        if ((msg.flags & 0x01u) == 0) {
            /* gnssFixOK flag not set -- ignore fix (GPS.G7-SW-12001-B p. 2)*/
            ubx_last_fix_mode = GPS_FIX_NONE;
        } else if (msg.fixType == 0x02u) {
            ubx_last_fix_mode = GPS_FIX_2D;
        } else if (msg.fixType == 0x03u || msg.fixType == 0x04u) {
            ubx_last_fix_mode = GPS_FIX_3D;
        } else {
            ubx_last_fix_mode = GPS_FIX_NONE;
        }

        pos_err = swap_u32(msg.hAcc);
        /* Convert to metres, rounding up */
        pos_err = (pos_err + 500u) / 1000u;
        if (pos_err > 0xffu) {
            pos_err = 0xffu;
        }

        fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 8u, 3u);
        fcs_parameter_set_type(&param, FCS_PARAMETER_GPS_INFO);
        fcs_parameter_set_device_id(&param, 0);
        param.data.u8[0] = ubx_last_fix_mode;
        param.data.u8[1] = (uint8_t)pos_err;
        param.data.u8[2] = msg.numSV;
        (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);

        if (ubx_last_fix_mode == GPS_FIX_3D) {
            fcs_parameter_set_header(&param, FCS_VALUE_SIGNED, 32u, 3u);
            fcs_parameter_set_type(&param, FCS_PARAMETER_GPS_POSITION_LLA);
            fcs_parameter_set_device_id(&param, 0);
            param.data.i32[0] = msg.lat;
            param.data.i32[1] = msg.lon;
            param.data.i32[2] = msg.height;
            (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);

            fcs_parameter_set_header(&param, FCS_VALUE_SIGNED, 16u, 3u);
            fcs_parameter_set_type(&param, FCS_PARAMETER_GPS_VELOCITY_NED);
            fcs_parameter_set_device_id(&param, 0);
            param.data.i16[0] = swap_i16(clamp_s16(swap_i32(msg.velN)));
            param.data.i16[1] = swap_i16(clamp_s16(swap_i32(msg.velE)));
            param.data.i16[2] = swap_i16(clamp_s16(swap_i32(msg.velD)));
            (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);
        }

        ubx_state_timer = 0;
    } else {
        /* Some other message type; ignore */
    }
}
