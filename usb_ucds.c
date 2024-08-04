#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include "usb_private.h"

#include "led.h"
#include "ring.h"
#include "usb.h"

static const struct usb_device_descriptor desc =
{
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x1234,
	.bcdDevice = 0x0310,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor endpoints[] =
{
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x82,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x2,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x81,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x1,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	},
};

static const struct usb_interface_descriptor iface =
{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 4,
	.bInterfaceClass = 0xFF, /* Vendor Specific */
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 5,
	.endpoint = endpoints,
};

static const struct usb_interface ifaces[] =
{
	{
		.num_altsetting = 1,
		.altsetting = &iface,
	}
};

static const struct usb_config_descriptor config =
{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 4,
	.bmAttributes = 0xc0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] =
{
	"Dimon Microelectronics",
	"UCDS System",
	"000000000000",
	"UCDS Config",
	"UCDS Interface",
};

#define MSGS_SIZE 100
//to host
struct ring_t ucds_tx_ring;
static uint8_t tx_ring_buffer[MSGS_SIZE];

//from host
struct ring_t ucds_rx_ring;
static uint8_t rx_ring_buffer[MSGS_SIZE];

static char usb_serial_number[25] = { '9', '1', '1', '\0' };

static void usb_ucds_preinit(void)
{
	desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));

	ring_init(&ucds_tx_ring, tx_ring_buffer, 1, sizeof(tx_ring_buffer));
	ring_init(&ucds_rx_ring, rx_ring_buffer, 1, sizeof(rx_ring_buffer));
}

static volatile bool ucds_is_configured = false;

static enum usbd_request_return_codes usbucds_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)usbd_dev;
	(void)buf;
	(void)len;
	(void)complete;

	if ((req->bmRequestType & 0x7F) != 0x21)
		return USBD_REQ_NOTSUPP; /* Only accept class request. */

	return USBD_REQ_NOTSUPP;
}

uint8_t cmd2[265];
uint16_t cmd2_idx = 0;

static void usb_ucds_rx2_cb(usbd_device *usbd_dev, uint8_t ep)
{
	char buf[64];

	ep = ep & 0x7f;
	int len = usbd_ep_read_packet(usbd_dev, ep, buf, 64);

	if (!len)
		return;

	uint8_t bidx = buf[0];
	uint8_t bnum = buf[1];

	if (bidx == 1) {

		cmd2_idx = 0;
		memset(cmd2, 0, sizeof(cmd2));
	}

	if ((bnum + cmd2_idx) < 265) {
		for (int i = 0; i < bnum; i++)
			cmd2[cmd2_idx + i] = buf[i + 2];
		cmd2_idx += bnum;
	}

	if (bidx < 5)
		return;

//	uint16_t cmd2_chksum = 0;
//	for (uint16_t i = 0; i < 262; i++)
//		cmd2_chksum += cmd2[i];

	//if (((cmd2_chksum >> 8 & 0xff) != cmd2[263]) || ((cmd2_chksum & 0xff) != cmd2[264]) || (cmd2[5] != 0))
	//	return;


	led2_blink();

	switch (cmd2[1]) {

		case 0xe:
			{

				uint8_t ans[] = { 6 };
				for (uint8_t i = 0; i < sizeof(ans); i++)
					ring_write(&ucds_tx_ring, &ans[i]);
			}
			break;

		case 0x9:
			{

				uint8_t ans[] = { 7, 5, 3, 1, 4 };
				for (uint8_t i = 0; i < sizeof(ans); i++)
					ring_write(&ucds_tx_ring, &ans[i]);
			}
			break;

		case 0x15:
			{

				uint8_t ans[] = { 0x4a, 0x1b, 0xef, 0x5a, 0x1d, 0xad, 0x12, 0x68,
						0x00, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c };

				for (uint8_t i = 0; i < sizeof(ans); i++)
					ring_write(&ucds_tx_ring, &ans[i]);
			}
			break;

		default:
			{

				uint8_t ans[] = { cmd2[1] };
				for (uint8_t i = 0; i < sizeof(ans); i++)
					ring_write(&ucds_tx_ring, &ans[i]);
			}
	}
}

#define USB_REBASE(x) MMIO32((x) + (USB_OTG_FS_BASE))

static void usb_ucds_tx2_cb(usbd_device *usbd_dev, uint8_t ep)
{
	char buf[64];
	uint8_t len = 0;
	uint8_t ch = 0;

	ep = ep & 0x7f;

	/* Return if endpoint is already enabled for transmit(fifo packet is not null) */
	if (USB_REBASE(OTG_DIEPTSIZ(ep)) & OTG_DIEPSIZ0_PKTCNT)
		return;

	for (uint8_t i = 0; i < sizeof(buf); i++) {

		if (!ring_read(&ucds_tx_ring, &ch))
			break;

		buf[i] = ch;
		len++;
	}

	if (len) {

		led2_blink();

		if (usbd_ep_write_packet(usbd_dev, ep, buf, len) != len) {

			ucds_is_configured = false;
		}
	}
}

static void usb_ucds_tx1_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)usbd_dev;
	(void)ep;
}

static void usb_ucds_rx1_cb(usbd_device *usbd_dev, uint8_t ep)
{
	char buf[64];

	ep = ep & 0x7f;
	int len = usbd_ep_read_packet(usbd_dev, ep, buf, 64);

	(void)len;
}

static void usb_ucds_iface_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, endpoints[0].bEndpointAddress, endpoints[0].bmAttributes, endpoints[0].wMaxPacketSize, usb_ucds_tx2_cb);
	usbd_ep_setup(usbd_dev, endpoints[1].bEndpointAddress, endpoints[1].bmAttributes, endpoints[1].wMaxPacketSize, usb_ucds_rx2_cb);
	usbd_ep_setup(usbd_dev, endpoints[2].bEndpointAddress, endpoints[2].bmAttributes, endpoints[2].wMaxPacketSize, usb_ucds_tx1_cb);
	usbd_ep_setup(usbd_dev, endpoints[3].bEndpointAddress, endpoints[3].bmAttributes, endpoints[3].wMaxPacketSize, usb_ucds_rx1_cb);

	usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, usbucds_control_request);

	ucds_is_configured = true;
}

static void usb_ucds_alt_set_config(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;
	(void)wIndex;
	(void)wValue;
}

static void usb_ucds_process(usbd_device * usbd_dev)
{
	if (ucds_is_configured) {

		uint8_t ep = endpoints[0].bEndpointAddress & 0x7f;
		usb_ucds_tx2_cb(usbd_dev, ep);
	}
}

static void usb_ucds_reset(void)
{
	ucds_is_configured = false;
}

static void usb_ucds_disable(void)
{
	ucds_is_configured = false;
}

struct usb_cb_t ucds_cb =
{
	.desc = &desc,
	.strings = usb_strings,
	.num_strings = 5,
	.config = &config,
	.preinit = usb_ucds_preinit,
	.init = NULL,
	.iface_set_config = usb_ucds_iface_set_config,
	.alt_set_config = usb_ucds_alt_set_config,
	.process = usb_ucds_process,
	.reset = usb_ucds_reset,
	.disable = usb_ucds_disable,
};

