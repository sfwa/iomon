/**
 * \file
 *
 * \brief USB Device Communication Device Class (CDC) interface.
 *
 * Copyright (c) 2009-2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "conf_usb.h"
#include "usb_protocol.h"
#include "usb_protocol_cdc.h"
#include "udd.h"
#include "udc.h"
#include "udi_cdc.h"
#include <string.h>

#ifdef UDI_CDC_LOW_RATE
#  define UDI_CDC_TX_BUFFERS     (UDI_CDC_DATA_EPS_FS_SIZE)
#  define UDI_CDC_RX_BUFFERS     (UDI_CDC_DATA_EPS_FS_SIZE)
#else
#  define UDI_CDC_TX_BUFFERS     (5*UDI_CDC_DATA_EPS_FS_SIZE)
#  define UDI_CDC_RX_BUFFERS     (5*UDI_CDC_DATA_EPS_FS_SIZE)
#endif

/**
 * \ingroup udi_cdc_group
 * \defgroup udi_cdc_group_udc Interface with USB Device Core (UDC)
 *
 * Structures and functions required by UDC.
 *
 * @{
 */
bool udi_cdc_comm_enable(void);
void udi_cdc_comm_disable(void);
bool udi_cdc_comm_setup(void);
bool udi_cdc_data_enable(void);
void udi_cdc_data_disable(void);
bool udi_cdc_data_setup(void);
uint8_t udi_cdc_getsetting(void);
void udi_cdc_data_sof_notify(void);
UDC_DESC_STORAGE udi_api_t udi_api_cdc_comm = {
	.enable = udi_cdc_comm_enable,
	.disable = udi_cdc_comm_disable,
	.setup = udi_cdc_comm_setup,
	.getsetting = udi_cdc_getsetting,
};
UDC_DESC_STORAGE udi_api_t udi_api_cdc_data = {
	.enable = udi_cdc_data_enable,
	.disable = udi_cdc_data_disable,
	.setup = udi_cdc_data_setup,
	.getsetting = udi_cdc_getsetting,
	.sof_notify = udi_cdc_data_sof_notify,
};

/**
 * \ingroup udi_cdc_group
 * \defgroup udi_cdc_group_internal Implementation of UDI CDC
 *
 * Class internal implementation
 * @{
 */

/**
 * \name Internal routines
 */

/**
 * \name Routines to control serial line
 */

/**
 * \brief Sends line coding to application
 *
 * Called after SETUP request when line coding data is received.
 */
static void udi_cdc_line_coding_received(void);

/**
 * \brief Records new state
 *
 * \param b_set      State is enabled if true, else disabled
 * \param bit_mask   Field to process (see CDC_SERIAL_STATE_ defines)
 */
static void udi_cdc_ctrl_state_change(bool b_set, le16_t bit_mask);

/**
 * \brief Check and eventually notify the USB host of new state
 *
 * \param ep         Port communication endpoint
 */
static void udi_cdc_ctrl_state_notify(udd_ep_id_t ep);

/**
 * \brief Ack sent of serial state message
 * Callback called after serial state message sent
 *
 * \param status     UDD_EP_TRANSFER_OK, if transfer finished
 * \param status     UDD_EP_TRANSFER_ABORT, if transfer aborted
 * \param n          number of data transfered
 */
static void udi_cdc_serial_state_msg_sent(udd_ep_status_t status, iram_size_t n, udd_ep_id_t ep);

/**
 * \name Routines to process data transfer
 */

/**
 * \brief Enable the reception of data from the USB host
 *
 * The value udi_cdc_rx_trans_sel indicate the RX buffer to fill.
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
static bool udi_cdc_rx_start(void);

/**
 * \brief Update rx buffer management with a new data
 * Callback called after data reception on USB line
 *
 * \param status     UDD_EP_TRANSFER_OK, if transfer finish
 * \param status     UDD_EP_TRANSFER_ABORT, if transfer aborted
 * \param n          number of data received
 */
static void udi_cdc_data_received(udd_ep_status_t status, iram_size_t n, udd_ep_id_t ep);

/**
 * \brief Ack sent of tx buffer
 * Callback called after data transfer on USB line
 *
 * \param status     UDD_EP_TRANSFER_OK, if transfer finished
 * \param status     UDD_EP_TRANSFER_ABORT, if transfer aborted
 * \param n          number of data transfered
 */
static void udi_cdc_data_sent(udd_ep_status_t status, iram_size_t n, udd_ep_id_t ep);

/**
 * \brief Send buffer on line or wait a SOF event
 *
 */
static void udi_cdc_tx_send(void);

/**
 * \name Information about configuration of communication line
 */
static usb_cdc_line_coding_t udi_cdc_line_coding;
static bool udi_cdc_serial_state_msg_ongoing;
static volatile le16_t udi_cdc_state;
COMPILER_WORD_ALIGNED static usb_cdc_notify_serial_state_t uid_cdc_state_msg;

/* Status of CDC COMM interfaces */
static volatile uint8_t udi_cdc_nb_comm_enabled = 0;

/**
 * \name Variables to manage RX/TX transfer requests
 * Two buffers for each sense are used to optimize the speed.
 */

/* Status of CDC DATA interfaces */
static volatile uint8_t udi_cdc_nb_data_enabled = 0;
static volatile bool udi_cdc_data_running = false;
/* Buffer to receive data */
COMPILER_WORD_ALIGNED static uint8_t udi_cdc_rx_buf[2][UDI_CDC_RX_BUFFERS];
/* Data available in RX buffers */
static uint16_t udi_cdc_rx_buf_nb[2];
/* Give the current RX buffer used (rx0 if 0, rx1 if 1) */
static volatile uint8_t udi_cdc_rx_buf_sel;
/* Read position in current RX buffer */
static volatile uint16_t udi_cdc_rx_pos;
/* Signal a transfer on-going */
static volatile bool udi_cdc_rx_trans_ongoing;

/* Define a transfer halted */
#define  UDI_CDC_TRANS_HALTED    2

/* Buffer to send data */
COMPILER_WORD_ALIGNED static uint8_t udi_cdc_tx_buf[2][UDI_CDC_TX_BUFFERS];
/* Data available in TX buffers */
static uint16_t udi_cdc_tx_buf_nb[2];
/* Give current TX buffer used (tx0 if 0, tx1 if 1) */
static volatile uint8_t udi_cdc_tx_buf_sel;
/* Value of SOF during last TX transfer */
static uint16_t udi_cdc_tx_sof_num;
/* Signal a transfer on-going */
static volatile bool udi_cdc_tx_trans_ongoing;
/* Signal that both buffer content data to send */
static volatile bool udi_cdc_tx_both_buf_to_send;

bool udi_cdc_comm_enable(void)
{
	udi_cdc_nb_comm_enabled = 0;

	/* Initialize control signal management */
	udi_cdc_state = CPU_TO_LE16(0);

	uid_cdc_state_msg.header.bmRequestType =
			USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
			USB_REQ_RECIP_INTERFACE;
	uid_cdc_state_msg.header.bNotification = USB_REQ_CDC_NOTIFY_SERIAL_STATE;
	uid_cdc_state_msg.header.wValue = LE16(0);
	uid_cdc_state_msg.header.wIndex = LE16(UDI_CDC_COMM_IFACE_NUMBER_0);
	uid_cdc_state_msg.header.wLength = LE16(2);
	uid_cdc_state_msg.value = CPU_TO_LE16(0);

	udi_cdc_line_coding.dwDTERate = CPU_TO_LE32(UDI_CDC_DEFAULT_RATE);
	udi_cdc_line_coding.bCharFormat = UDI_CDC_DEFAULT_STOPBITS;
	udi_cdc_line_coding.bParityType = UDI_CDC_DEFAULT_PARITY;
	udi_cdc_line_coding.bDataBits = UDI_CDC_DEFAULT_DATABITS;

	/* Call application callback to initialize memories or indicate that
	interface is enabled */
	UDI_CDC_SET_CODING_EXT((&udi_cdc_line_coding));
	if (!UDI_CDC_ENABLE_EXT()) {
		return false;
	}

	udi_cdc_nb_comm_enabled = 1;
	return true;
}

bool udi_cdc_data_enable(void)
{
	udi_cdc_nb_data_enabled = 0;

	/* Initialize TX management */
	udi_cdc_tx_trans_ongoing = false;
	udi_cdc_tx_both_buf_to_send = false;
	udi_cdc_tx_buf_sel = 0;
	udi_cdc_tx_buf_nb[0] = 0;
	udi_cdc_tx_buf_nb[1] = 0;
	udi_cdc_tx_sof_num = 0;
	udi_cdc_tx_send();

	/* Initialize RX management */
	udi_cdc_rx_trans_ongoing = false;
	udi_cdc_rx_buf_sel = 0;
	udi_cdc_rx_buf_nb[0] = 0;
	udi_cdc_rx_pos = 0;
	if (!udi_cdc_rx_start()) {
		return false;
	}

	udi_cdc_nb_data_enabled = 1;
	udi_cdc_data_running = true;

	return true;
}

void udi_cdc_comm_disable(void)
{
	Assert(udi_cdc_nb_comm_enabled == 1);
	udi_cdc_nb_comm_enabled = 0;
}

void udi_cdc_data_disable(void)
{
	Assert(udi_cdc_nb_data_enabled == 1);
	udi_cdc_nb_data_enabled = 0;
	UDI_CDC_DISABLE_EXT();
	udi_cdc_data_running = false;
}

bool udi_cdc_comm_setup(void)
{
	if (Udd_setup_is_in()) {
		/* GET Interface Requests */
		if (Udd_setup_type() == USB_REQ_TYPE_CLASS) {
			/* Requests Class Interface Get */
			switch (udd_g_ctrlreq.req.bRequest) {
			case USB_REQ_CDC_GET_LINE_CODING:
				/* Get configuration of CDC line */
				if (sizeof(usb_cdc_line_coding_t) !=
						udd_g_ctrlreq.req.wLength)
					return false; /* Error for USB host */
				udd_g_ctrlreq.payload =
						(uint8_t *) &
						udi_cdc_line_coding;
				udd_g_ctrlreq.payload_size =
						sizeof(usb_cdc_line_coding_t);
				return true;
			}
		}
	}
	if (Udd_setup_is_out()) {
		/* SET Interface Requests */
		if (Udd_setup_type() == USB_REQ_TYPE_CLASS) {
			/* Requests Class Interface Set */
			switch (udd_g_ctrlreq.req.bRequest) {
			case USB_REQ_CDC_SET_LINE_CODING:
				/* Change configuration of CDC line */
				if (sizeof(usb_cdc_line_coding_t) !=
						udd_g_ctrlreq.req.wLength)
					return false; /* Error for USB host */
				udd_g_ctrlreq.callback =
						udi_cdc_line_coding_received;
				udd_g_ctrlreq.payload =
						(uint8_t *) &
						udi_cdc_line_coding;
				udd_g_ctrlreq.payload_size =
						sizeof(usb_cdc_line_coding_t);
				return true;
			case USB_REQ_CDC_SET_CONTROL_LINE_STATE:
				/* According cdc spec 1.1 chapter 6.2.14 */
				UDI_CDC_SET_DTR_EXT((0 !=
						(udd_g_ctrlreq.req.wValue
						 & CDC_CTRL_SIGNAL_DTE_PRESENT)));
				UDI_CDC_SET_RTS_EXT((0 !=
						(udd_g_ctrlreq.req.wValue
						 & CDC_CTRL_SIGNAL_ACTIVATE_CARRIER)));
				return true;
			}
		}
	}
	return false;  /* request Not supported */
}

bool udi_cdc_data_setup(void)
{
	return false;  /* request Not supported */
}

uint8_t udi_cdc_getsetting(void)
{
	return 0;      /* CDC don't have multiple alternate setting */
}

void udi_cdc_data_sof_notify(void)
{
	udi_cdc_tx_send();
}


/*Internal routines to control serial line */

static void udi_cdc_line_coding_received(void)
{
	UDI_CDC_SET_CODING_EXT((&udi_cdc_line_coding));
}

static void udi_cdc_ctrl_state_change(bool b_set, le16_t bit_mask)
{
	irqflags_t flags;

	/* Update state */
	flags = cpu_irq_save(); /* Protect udi_cdc_state */
	if (b_set) {
		udi_cdc_state |= bit_mask;
	} else {
		udi_cdc_state &= ~(unsigned)bit_mask;
	}
	cpu_irq_restore(flags);

	/* Send it if possible and state changed */
	udi_cdc_ctrl_state_notify(UDI_CDC_COMM_EP_0);
}


static void udi_cdc_ctrl_state_notify(udd_ep_id_t ep)
{
	/* Send it if possible and state changed */
	if ((!udi_cdc_serial_state_msg_ongoing)
			&& (udi_cdc_state != uid_cdc_state_msg.value)) {
		/* Fill notification message */
		uid_cdc_state_msg.value = udi_cdc_state;
		/* Send notification message */
		udi_cdc_serial_state_msg_ongoing =
				udd_ep_run(ep,
				false,
				(uint8_t *) & uid_cdc_state_msg,
				sizeof(uid_cdc_state_msg),
				udi_cdc_serial_state_msg_sent);
	}
}


static void udi_cdc_serial_state_msg_sent(udd_ep_status_t status, iram_size_t n, udd_ep_id_t ep)
{
	UNUSED(n);
	UNUSED(status);

	udi_cdc_serial_state_msg_ongoing = false;

	/* For the irregular signals like break, the incoming ring signal,
	   or the overrun error state, this will reset their values to zero
	   and again will not send another notification until their state changes. */
	udi_cdc_state &= ~(CDC_SERIAL_STATE_BREAK |
			CDC_SERIAL_STATE_RING |
			CDC_SERIAL_STATE_FRAMING |
			CDC_SERIAL_STATE_PARITY | CDC_SERIAL_STATE_OVERRUN);
	uid_cdc_state_msg.value &= ~(CDC_SERIAL_STATE_BREAK |
			CDC_SERIAL_STATE_RING |
			CDC_SERIAL_STATE_FRAMING |
			CDC_SERIAL_STATE_PARITY | CDC_SERIAL_STATE_OVERRUN);
	/* Send it if possible and state changed */
	udi_cdc_ctrl_state_notify(ep);
}


/* Internal routines to process data transfer */


static bool udi_cdc_rx_start(void)
{
	irqflags_t flags;
	uint8_t buf_sel_trans;

	flags = cpu_irq_save();
	buf_sel_trans = udi_cdc_rx_buf_sel;
	if (udi_cdc_rx_trans_ongoing ||
		(udi_cdc_rx_pos < udi_cdc_rx_buf_nb[buf_sel_trans])) {
		/* Transfer already on-going or current buffer no empty */
		cpu_irq_restore(flags);
		return false;
	}

	/* Change current buffer */
	udi_cdc_rx_pos = 0;
	udi_cdc_rx_buf_sel = (buf_sel_trans==0)?1:0;

	/* Start transfer on RX */
	udi_cdc_rx_trans_ongoing = true;
	cpu_irq_restore(flags);

	if (udi_cdc_is_rx_ready()) {
		UDI_CDC_RX_NOTIFY();
	}
	/* Send the buffer with enable of short packet */
	return udd_ep_run(UDI_CDC_DATA_EP_OUT_0,
			true,
			udi_cdc_rx_buf[buf_sel_trans],
			UDI_CDC_RX_BUFFERS,
			udi_cdc_data_received);
}


static void udi_cdc_data_received(udd_ep_status_t status, iram_size_t n, udd_ep_id_t ep)
{
	uint8_t buf_sel_trans;

	if (UDD_EP_TRANSFER_OK != status) {
		/* Abort reception */
		return;
	}
	buf_sel_trans = (udi_cdc_rx_buf_sel==0)?1:0;
	if (!n) {
		udd_ep_run( ep,
				true,
				udi_cdc_rx_buf[buf_sel_trans],
				UDI_CDC_RX_BUFFERS,
				udi_cdc_data_received);
		return;
	}
	udi_cdc_rx_buf_nb[buf_sel_trans] = n;
	udi_cdc_rx_trans_ongoing = false;
	udi_cdc_rx_start();
}


static void udi_cdc_data_sent(udd_ep_status_t status, iram_size_t n, udd_ep_id_t ep)
{
	UNUSED(n);

	if (UDD_EP_TRANSFER_OK != status) {
		/* Abort transfer */
		return;
	}
	udi_cdc_tx_buf_nb[(udi_cdc_tx_buf_sel==0)?1:0] = 0;
	udi_cdc_tx_both_buf_to_send = false;
	udi_cdc_tx_trans_ongoing = false;
	udi_cdc_tx_send();
}


static void udi_cdc_tx_send(void)
{
	irqflags_t flags;
	uint8_t buf_sel_trans;
	bool b_short_packet;

	if (udi_cdc_tx_trans_ongoing) {
		return; /* Already on going or wait next SOF to send next data */
	}
	if (udi_cdc_tx_sof_num == udd_get_frame_number()) {
		return; /* Wait next SOF to send next data */
	}

	flags = cpu_irq_save(); /* to protect udi_cdc_tx_buf_sel */
	buf_sel_trans = udi_cdc_tx_buf_sel;
	if (!udi_cdc_tx_both_buf_to_send) {
		/* Send current Buffer and switch the current buffer */
		udi_cdc_tx_buf_sel = (buf_sel_trans==0)?1:0;
	}else{
		/* Send the other Buffer and no switch the current buffer */
		buf_sel_trans = (buf_sel_trans==0)?1:0;
	}
	udi_cdc_tx_trans_ongoing = true;
	cpu_irq_restore(flags);

	b_short_packet = (udi_cdc_tx_buf_nb[buf_sel_trans] != UDI_CDC_TX_BUFFERS);
	if (b_short_packet) {
		udi_cdc_tx_sof_num = udd_get_frame_number();
	}else{
		udi_cdc_tx_sof_num = 0; /* Force next transfer without wait SOF */
	}

	/* Send the buffer with enable of short packet */
	udd_ep_run( UDI_CDC_DATA_EP_IN_0,
			b_short_packet,
			udi_cdc_tx_buf[buf_sel_trans],
			udi_cdc_tx_buf_nb[buf_sel_trans],
			udi_cdc_data_sent);
}


/* Application interface */

void udi_cdc_ctrl_signal_dcd(bool b_set)
{
	udi_cdc_ctrl_state_change(b_set, CDC_SERIAL_STATE_DCD);
}

void udi_cdc_ctrl_signal_dsr(bool b_set)
{
	udi_cdc_ctrl_state_change(b_set, CDC_SERIAL_STATE_DSR);
}

void udi_cdc_signal_framing_error(void)
{
	udi_cdc_ctrl_state_change(true, CDC_SERIAL_STATE_FRAMING);
}

void udi_cdc_signal_parity_error(void)
{
	udi_cdc_ctrl_state_change(true, CDC_SERIAL_STATE_PARITY);
}

void udi_cdc_signal_overrun(void)
{
	udi_cdc_ctrl_state_change(true, CDC_SERIAL_STATE_OVERRUN);
}

bool udi_cdc_is_rx_ready()
{
	irqflags_t flags;
	uint16_t pos;
	bool ready;

	flags = cpu_irq_save();
	pos = udi_cdc_rx_pos;
	ready = pos < udi_cdc_rx_buf_nb[udi_cdc_rx_buf_sel];
	cpu_irq_restore(flags);
	return ready;
}

iram_size_t udi_cdc_read_buf(void* buf, iram_size_t size)
{
	irqflags_t flags;
	uint8_t *ptr_buf = (uint8_t *)buf;
	iram_size_t copy_nb;
	uint16_t pos;
	uint8_t buf_sel;

	/* Check available data */
	flags = cpu_irq_save();
	pos = udi_cdc_rx_pos;
	buf_sel = udi_cdc_rx_buf_sel;
	cpu_irq_restore(flags);
	if (!udi_cdc_data_running || pos >= udi_cdc_rx_buf_nb[buf_sel]) {
		return size;
	}

	/* Read data */
	copy_nb = udi_cdc_rx_buf_nb[buf_sel] - pos;
	if (copy_nb>size) {
		copy_nb = size;
	}
	memcpy(ptr_buf, &udi_cdc_rx_buf[buf_sel][pos], copy_nb);
	udi_cdc_rx_pos += copy_nb;
	ptr_buf += copy_nb;
	size -= copy_nb;
	udi_cdc_rx_start();

	return size;
}

bool udi_cdc_is_tx_ready()
{
	irqflags_t flags;

	if (udi_cdc_tx_buf_nb[udi_cdc_tx_buf_sel]!=UDI_CDC_TX_BUFFERS) {
		return true;
	}
	if (!udi_cdc_tx_both_buf_to_send) {
		flags = cpu_irq_save(); /* to protect udi_cdc_tx_buf_sel */
		if (!udi_cdc_tx_trans_ongoing) {
			/* No transfer on-going then use the other buffer to store data */
			udi_cdc_tx_both_buf_to_send = true;
			udi_cdc_tx_buf_sel = (udi_cdc_tx_buf_sel==0)?1:0;
		}
		cpu_irq_restore(flags);
	}
	return (udi_cdc_tx_buf_nb[udi_cdc_tx_buf_sel]!=UDI_CDC_TX_BUFFERS);
}

iram_size_t udi_cdc_write_buf(const void* buf, iram_size_t size)
{
	irqflags_t flags;
	uint8_t buf_sel;
	uint16_t buf_nb;
	iram_size_t copy_nb;
	uint8_t *ptr_buf = (uint8_t *)buf;

	if (9 == udi_cdc_line_coding.bDataBits) {
		size *=2;
	}

udi_cdc_write_buf_loop_wait:
	/* Check available space */
	if (!udi_cdc_is_tx_ready() || !udi_cdc_data_running) {
		return size;
	}

	/* Write values */
	flags = cpu_irq_save();
	buf_sel = udi_cdc_tx_buf_sel;
	buf_nb = udi_cdc_tx_buf_nb[buf_sel];
	copy_nb = UDI_CDC_TX_BUFFERS - buf_nb;
	if (copy_nb>size) {
		copy_nb = size;
	}
	memcpy(&udi_cdc_tx_buf[buf_sel][buf_nb], ptr_buf, copy_nb);
	udi_cdc_tx_buf_nb[buf_sel] = buf_nb + copy_nb;
	cpu_irq_restore(flags);

	/* Update buffer pointer */
	ptr_buf = ptr_buf + copy_nb;
	size -= copy_nb;

	if (size) {
		goto udi_cdc_write_buf_loop_wait;
	}

	return 0;
}
