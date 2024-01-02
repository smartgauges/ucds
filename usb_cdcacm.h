#ifndef USB_CDCACM_H
#define USB_CDCACM_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include <libopencm3/include/libopencm3/usb/dwc/otg_fs.h>
#include "usb_private.h"

#include "usb.h"
#include "led.h"
#include "ring.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct cdcacm_t
{
	uint8_t iface;
	volatile bool configured;
	volatile bool zlp;

	ring_t tx_ring;
	uint8_t tx_ring_buffer[2000];
	struct ring_t rx_ring;
	uint8_t rx_ring_buffer[2000];

	struct usb_cdc_line_coding line_coding;
};

#define USB_REBASE(x) MMIO32((x) + (USB_OTG_FS_BASE))

#define CDC_RTS_MASK 0x02
#define CDC_DTR_MASK 0x01

static enum usbd_request_return_codes cdcacm_control_request(struct usb_setup_data *req, uint8_t **buf, uint16_t *len, struct cdcacm_t * cdcacm)
{
	const uint8_t type = USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE;
	const uint8_t mask = USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT;

	if ((req->bmRequestType & mask) != type)
		return USBD_REQ_NEXT_CALLBACK;

	uint8_t iface = req->wIndex & 0xff;
	if (iface != cdcacm->iface)
		return USBD_REQ_NEXT_CALLBACK;

	switch (req->bRequest) {
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
			{
				/* D0 - DTR, D1 - RTS */
				/* Check DTR bit */
				cdcacm->configured = (req->wValue & 1) ? true : false;
				return USBD_REQ_HANDLED;
			}
			break;

		case USB_CDC_REQ_SET_LINE_CODING: 
			if (*len < sizeof(struct usb_cdc_line_coding))
				return USBD_REQ_NOTSUPP;
			{
				memcpy(&cdcacm->line_coding, *buf, sizeof(cdcacm->line_coding));
				return USBD_REQ_HANDLED;
			}
			break;

		case USB_CDC_REQ_GET_LINE_CODING: 
			if (*len < sizeof(struct usb_cdc_line_coding))
				return USBD_REQ_NOTSUPP;
			{
				memcpy(*buf, &cdcacm->line_coding, sizeof(cdcacm->line_coding));
				return USBD_REQ_HANDLED;
			}
	}

	return USBD_REQ_NEXT_CALLBACK;
}

static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep, struct cdcacm_t * cdcacm)
{
	char buf[64];
	uint8_t ch = 0;
	uint8_t len = 0;

	if (!cdcacm->configured)
		return;

	ep = ep & 0x7f;

	/* Return if endpoint is already enabled for transmit(fifo packet is not null) */
	if (USB_REBASE(OTG_DIEPTSIZ(ep)) & OTG_DIEPSIZ0_PKTCNT)
		return;
	
	for (uint8_t i = 0; i < sizeof(buf); i++) {

		if (!ring_read(&cdcacm->tx_ring, &ch))
			break;

		buf[i] = ch;
		len++;
	}

	if (len == 64)
		cdcacm->zlp = true;

	if (len || cdcacm->zlp) {

		//led4_on();

		if (usbd_ep_write_packet(usbd_dev, ep, buf, len) != len) {

			cdcacm->configured = false;
		}

		cdcacm->zlp = false;
	}
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep, struct cdcacm_t * cdcacm)
{
	char buf[64];

	ep = ep & 0x7f;
	int len = usbd_ep_read_packet(usbd_dev, ep, buf, 64);

	if (len) {

		cdcacm->configured = true;

		//led2_on();

		for (int i = 0; i < len; i++)
			ring_write(&cdcacm->rx_ring, &buf[i]);
	}
}

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif

