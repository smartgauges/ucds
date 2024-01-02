#include <libopencm3/stm32/desig.h>

#include "usb_ch.h"
#include "usb_cdcacm.h"

struct cdcacm_t cdcacm =
{
	.iface = 0,
	.configured = false,
	.line_coding = { .dwDTERate = 115200, .bCharFormat = USB_CDC_1_STOP_BITS, .bParityType = USB_CDC_NO_PARITY, .bDataBits = 8 },
};

struct ring_t * get_tx_ring()
{
	return &cdcacm.tx_ring;
}

struct ring_t * get_rx_ring()
{
	return &cdcacm.rx_ring;
}

void set_dtr()
{
	cdcacm.configured = true;
}

void clr_dtr()
{
	cdcacm.configured = false;
}

static const struct usb_device_descriptor desc_ch =
{
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0x0,
	.bDeviceProtocol = 0x0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static char usb_serial_number_ch[25] = { '9', '1', '1', '\0' };

static const char * usb_strings_ch[] =
{
	"STMicroelectronics",
	"STM32 Virtual COM Port",
	usb_serial_number_ch,
};

static void usb_ch_preinit(void)
{
	desig_get_unique_id_as_string(usb_serial_number_ch, sizeof(usb_serial_number_ch));

	ring_init(&cdcacm.tx_ring, cdcacm.tx_ring_buffer, 1, sizeof(cdcacm.tx_ring_buffer));
	ring_init(&cdcacm.rx_ring, cdcacm.rx_ring_buffer, 1, sizeof(cdcacm.rx_ring_buffer));
}

static const struct
{
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
	struct usb_cdc_call_management_descriptor call_mgmt;
} __attribute__((packed)) cdcacm_functional_descriptors =
{
	.header =
	{
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.acm =
	{
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0x3,
	},
	.cdc_union =
	{
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	},
	.call_mgmt =
	{
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	}
};

static const struct usb_endpoint_descriptor comm_endp =
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 8,
	.bInterval = 255,
};

static const struct usb_endpoint_descriptor data_endp[] =
{
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x01,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x82,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	} 
};

const struct usb_interface_descriptor cdcacm_comm_iface[] =
{
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_CDC,
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
		.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
		.iInterface = 0,

		.endpoint = &comm_endp,

		.extra = &cdcacm_functional_descriptors,
		.extralen = sizeof(cdcacm_functional_descriptors)
	} 
};

const struct usb_interface_descriptor cdcacm_data_iface[] = 
{
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 1,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,

		.endpoint = data_endp,
	} 	
};

const struct usb_interface ifaces[] = 
{
	{
		.num_altsetting = 1,
		.altsetting = cdcacm_comm_iface,
	},
	{
		.num_altsetting = 1,
		.altsetting = cdcacm_data_iface,
	},
};

static const struct usb_config_descriptor config =
{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,/* CDC-ACM(control + streaming) */
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, /* bus powered */
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static enum usbd_request_return_codes ch_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	return cdcacm_control_request(req, buf, len, &cdcacm);
}

static void ch_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	cdcacm_data_tx_cb(usbd_dev, ep, &cdcacm);
}

static void ch_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	cdcacm_data_rx_cb(usbd_dev, ep, &cdcacm);
}

static void usb_cdcacm_iface_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	//com0
	usbd_ep_setup(usbd_dev, data_endp[0].bEndpointAddress, data_endp[0].bmAttributes, data_endp[0].wMaxPacketSize, ch_data_rx_cb);
	usbd_ep_setup(usbd_dev, data_endp[1].bEndpointAddress, data_endp[1].bmAttributes, data_endp[1].wMaxPacketSize, ch_data_tx_cb);
	usbd_ep_setup(usbd_dev, comm_endp.bEndpointAddress, comm_endp.bmAttributes, comm_endp.wMaxPacketSize, 0);

	usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, ch_control_request);
}

static void usb_ch_alt_set_config(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;
	(void)wIndex;
	(void)wValue;
}

static void usb_ch_reset(void)
{
	cdcacm.configured = false;
}

static void usb_ch_disable(void)
{
	cdcacm.configured = false;
}

static void usb_ch_process(usbd_device * usbd_dev)
{
	//flush tx
	if (cdcacm.configured) {

		uint8_t ep = data_endp[1].bEndpointAddress & 0x7f;
		ch_data_tx_cb(usbd_dev, ep);
	}
}

struct usb_cb_t ch_cb =
{
	.desc = &desc_ch,
	.strings = usb_strings_ch,
	.num_strings = 3,
	.config = &config,
	.preinit = usb_ch_preinit,
	.init = NULL,
	.iface_set_config = usb_cdcacm_iface_set_config,
	.alt_set_config = usb_ch_alt_set_config,
	.process = usb_ch_process,
	.reset = usb_ch_reset,
	.disable = usb_ch_disable,
};

