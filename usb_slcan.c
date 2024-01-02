#include <libopencm3/stm32/desig.h>

#include "usb_cdcacm.h"
#include "usb_slcan.h"

struct cdcacm_t cdcacm0 =
{
	.iface = 0,
	.configured = false,
	.zlp = false,
	.line_coding = { .dwDTERate = 115200, .bCharFormat = USB_CDC_1_STOP_BITS, .bParityType = USB_CDC_NO_PARITY, .bDataBits = 8 },
};

struct cdcacm_t cdcacm1 =
{
	.iface = 2,
	.configured = false,
	.zlp = false,
	.line_coding = { .dwDTERate = 115200, .bCharFormat = USB_CDC_1_STOP_BITS, .bParityType = USB_CDC_NO_PARITY, .bDataBits = 8 },
};

struct ring_t * get_tx_ring0()
{
	return &cdcacm0.tx_ring;
}

struct ring_t * get_rx_ring0()
{
	return &cdcacm0.rx_ring;
}

struct ring_t * get_tx_ring1()
{
	return &cdcacm1.tx_ring;
}

struct ring_t * get_rx_ring1()
{
	return &cdcacm1.rx_ring;
}

static const struct
{
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors0 =
{
	.header =
	{
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt =
	{
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm =
	{
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0x2, /* D0: D1:Set/Get_Line_Coding, D2:Send_Break */
	},
	.cdc_union =
	{
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	}
};

static const struct
{
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors1 =
{
	.header =
	{
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt =
	{
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 3,
	},
	.acm =
	{
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 2,
	},
	.cdc_union =
	{
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 2,
		.bSubordinateInterface0 = 3,
	}
};

static const struct usb_endpoint_descriptor comm_endp0 =
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83/*0x84*/,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
};

static const struct usb_endpoint_descriptor comm_endp1 =
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
};

static const struct usb_endpoint_descriptor data_endp0[] =
{
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x01,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x81,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
	} 
};

static const struct usb_endpoint_descriptor data_endp1[] =
{
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x02,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x82,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
	} 
};

const struct usb_interface_descriptor cdcacm_comm_iface0[] =
{
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_CDC,
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
		.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
		.iInterface = 0,

		.endpoint = &comm_endp0,

		.extra = &cdcacm_functional_descriptors0,
		.extralen = sizeof(cdcacm_functional_descriptors0)
	} 
};

const struct usb_interface_descriptor cdcacm_data_iface0[] = 
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

		.endpoint = data_endp0,
	} 	
};

const struct usb_interface_descriptor cdcacm_comm_iface1[] =
{
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 2,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_CDC,
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
		.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
		.iInterface = 0,

		.endpoint = &comm_endp1,

		.extra = &cdcacm_functional_descriptors1,
		.extralen = sizeof(cdcacm_functional_descriptors1)
	} 
};

const struct usb_interface_descriptor cdcacm_data_iface1[] = 
{
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 3,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,

		.endpoint = data_endp1,
	} 	
};

static enum usbd_request_return_codes cdcacm_2x_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	const uint8_t type = USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE;
	const uint8_t mask = USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT;

	if ((req->bmRequestType & mask) != type)
		return USBD_REQ_NEXT_CALLBACK;

	uint8_t iface = req->wIndex & 0xff;
	if (iface == cdcacm0.iface) { 

		return cdcacm_control_request(req, buf, len, &cdcacm0);
	}
	else if (iface == cdcacm1.iface) {

		return cdcacm_control_request(req, buf, len, &cdcacm1);
	}
	
	return USBD_REQ_NEXT_CALLBACK;
}

static void cdcacm_data0_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	cdcacm_data_tx_cb(usbd_dev, ep, &cdcacm0);
}

static void cdcacm_data1_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	cdcacm_data_tx_cb(usbd_dev, ep, &cdcacm1);
}

static void cdcacm_data0_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	cdcacm_data_rx_cb(usbd_dev, ep, &cdcacm0);
}

static void cdcacm_data1_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	cdcacm_data_rx_cb(usbd_dev, ep, &cdcacm1);
}

static void usb_cdcacm_2x_iface_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	//com0
	usbd_ep_setup(usbd_dev, data_endp0[0].bEndpointAddress, data_endp0[0].bmAttributes, data_endp0[0].wMaxPacketSize, cdcacm_data0_rx_cb);
	usbd_ep_setup(usbd_dev, data_endp0[1].bEndpointAddress, data_endp0[1].bmAttributes, data_endp0[1].wMaxPacketSize, cdcacm_data0_tx_cb);
	usbd_ep_setup(usbd_dev, comm_endp0.bEndpointAddress, comm_endp0.bmAttributes, comm_endp0.wMaxPacketSize, 0);

	//com1
	usbd_ep_setup(usbd_dev, data_endp1[0].bEndpointAddress, data_endp1[0].bmAttributes, data_endp1[0].wMaxPacketSize, cdcacm_data1_rx_cb);
	usbd_ep_setup(usbd_dev, data_endp1[1].bEndpointAddress, data_endp1[1].bmAttributes, data_endp1[1].wMaxPacketSize, cdcacm_data1_tx_cb);
	usbd_ep_setup(usbd_dev, comm_endp1.bEndpointAddress, comm_endp1.bmAttributes, comm_endp1.wMaxPacketSize, 0);

	usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_2x_control_request);
}

const struct usb_iface_assoc_descriptor cdc_iface_assoc0 = 
{
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 0,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,
	.iFunction = 0,
};

const struct usb_iface_assoc_descriptor cdc_iface_assoc1 = 
{
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 2,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,
	.iFunction = 0,
};

const struct usb_interface ifaces[] = 
{
	{
		.num_altsetting = 1,
		.iface_assoc = &cdc_iface_assoc0,
		.altsetting = cdcacm_comm_iface0,
	},
	{
		.num_altsetting = 1,
		.altsetting = cdcacm_data_iface0,
	},
	{
		.num_altsetting = 1,
		.iface_assoc = &cdc_iface_assoc1,
		.altsetting = cdcacm_comm_iface1,
	},
	{
		.num_altsetting = 1,
		.altsetting = cdcacm_data_iface1,
	},
};

static const struct usb_config_descriptor config =
{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 4,/* CDC-ACM0(control + streaming), CDC-ACM1(control + streaming) */
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, /* bus powered */
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const struct usb_device_descriptor desc =
{
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	/*Interface Association Descriptor*/
	.bDeviceClass = 0xef,
	/*Interface Association Descriptor*/
	.bDeviceSubClass = 0x02,
	/*Interface Association Descriptor*/
	.bDeviceProtocol = 0x01,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5742,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static char usb_serial_number[25] = { '9', '1', '1', '\0' };

static const char * usb_strings[] =
{
	"STMicroelectronics",
	"2xVCP-CAN",
	usb_serial_number,
};

static void usb_cdcacm_preinit(void)
{
	desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));

	ring_init(&cdcacm0.tx_ring, cdcacm0.tx_ring_buffer, 1, sizeof(cdcacm0.tx_ring_buffer));
	ring_init(&cdcacm0.rx_ring, cdcacm0.rx_ring_buffer, 1, sizeof(cdcacm0.rx_ring_buffer));

	ring_init(&cdcacm1.tx_ring, cdcacm1.tx_ring_buffer, 1, sizeof(cdcacm1.tx_ring_buffer));
	ring_init(&cdcacm1.rx_ring, cdcacm1.rx_ring_buffer, 1, sizeof(cdcacm1.rx_ring_buffer));
}

static void usb_cdcacm_process(usbd_device * usbd_dev)
{
	//flush tx0
	if (cdcacm0.configured) {

		uint8_t ep = data_endp0[1].bEndpointAddress & 0x7f;
		cdcacm_data_tx_cb(usbd_dev, ep, &cdcacm0);
	}

	//flush tx1
	if (cdcacm1.configured) {

		uint8_t ep = data_endp1[1].bEndpointAddress & 0x7f;
		cdcacm_data_tx_cb(usbd_dev, ep, &cdcacm1);
	}
}

static void usb_slcan_alt_set_config(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;
	(void)wIndex;
	(void)wValue;
}

static void usb_cdcacm_reset(void)
{
	cdcacm0.configured = false;
	cdcacm1.configured = false;
}

static void usb_cdcacm_disable(void)
{
	cdcacm0.configured = false;
	cdcacm1.configured = false;
}

struct usb_cb_t slcan_cb =
{
	.desc = &desc,
	.strings = usb_strings,
	.num_strings = 3,
	.config = &config,
	.preinit = usb_cdcacm_preinit,
	.init = NULL,
	.iface_set_config = usb_cdcacm_2x_iface_set_config,
	.alt_set_config = usb_slcan_alt_set_config,
	.process = usb_cdcacm_process,
	.reset = usb_cdcacm_reset,
	.disable = usb_cdcacm_disable,
};

