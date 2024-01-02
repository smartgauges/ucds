#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/usb/usbd.h>

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
	.bcdDevice = 0x0318,
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
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x83,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	}
};

static const struct usb_interface_descriptor iface =
{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 5,
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
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] =
{
	"Dimon Microelectronics",
	"UCDS System",
	"UCDS_System_V3",
	"UCDS Config",
	"UCDS Interface",
};

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

static void usb_ucds_iface_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbucds_control_request);
}

static void usb_ucds_alt_set_config(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;
	(void)wIndex;
	(void)wValue;
}

static void usb_ucds_process(usbd_device * usbd_dev)
{
	(void)usbd_dev;
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
	.preinit = NULL,
	.init = NULL,
	.iface_set_config = usb_ucds_iface_set_config,
	.alt_set_config = usb_ucds_alt_set_config,
	.process = usb_ucds_process,
	.reset = usb_ucds_reset,
	.disable = usb_ucds_disable,
};

