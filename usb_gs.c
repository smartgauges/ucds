#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include "usb_private.h"

#include "timer.h"
#include "ring.h"
#include "can.h"
#include "led.h"
#include "usb_gs.h"

/* gs_usb.h */
enum gs_usb_breq {
	GS_USB_BREQ_HOST_FORMAT = 0,
	GS_USB_BREQ_BITTIMING,
	GS_USB_BREQ_MODE,
	GS_USB_BREQ_BERR,
	GS_USB_BREQ_BT_CONST,
	GS_USB_BREQ_DEVICE_CONFIG,
	GS_USB_BREQ_TIMESTAMP,
	GS_USB_BREQ_IDENTIFY,
	GS_USB_BREQ_GET_USER_ID,    //not implemented
	GS_USB_BREQ_SET_USER_ID,    //not implemented
	GS_USB_BREQ_DATA_BITTIMING,
	GS_USB_BREQ_BT_CONST_EXT,
	GS_USB_BREQ_SET_TERMINATION,
	GS_USB_BREQ_GET_TERMINATION,
	GS_USB_BREQ_GET_STATE,
};

enum gs_can_mode {
	/* reset a channel. turns it off */
	GS_CAN_MODE_RESET = 0,
	/* starts a channel */
	GS_CAN_MODE_START
};

#define GS_CAN_MODE_NORMAL		  0
#define GS_CAN_MODE_LISTEN_ONLY	  (1<<0)
#define GS_CAN_MODE_LOOP_BACK	  (1<<1)
#define GS_CAN_MODE_TRIPLE_SAMPLE (1<<2)
#define GS_CAN_MODE_ONE_SHOT	  (1<<3)
#define GS_CAN_MODE_HW_TIMESTAMP  (1<<4)
/* #define GS_CAN_FEATURE_IDENTIFY              (1<<5) */
/* #define GS_CAN_FEATURE_USER_ID               (1<<6) */
#define GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE (1<<7)
#define GS_CAN_MODE_FD						 (1<<8)    /* switch device to CAN-FD mode */
/* #define GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX (1<<9) */
/* #define GS_CAN_FEATURE_BT_CONST_EXT          (1<<10) */
/* #define GS_CAN_FEATURE_TERMINATION           (1<<11) */
#define GS_CAN_MODE_BERR_REPORTING (1<<12)
/* GS_CAN_FEATURE_GET_STATE (1<<13) */

#define GS_CAN_FEATURE_LISTEN_ONLY				(1<<0)
#define GS_CAN_FEATURE_LOOP_BACK				(1<<1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE			(1<<2)
#define GS_CAN_FEATURE_ONE_SHOT					(1<<3)
#define GS_CAN_FEATURE_HW_TIMESTAMP				(1<<4)
#define GS_CAN_FEATURE_IDENTIFY					(1<<5)
#define GS_CAN_FEATURE_USER_ID					(1<<6)
#define GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE (1<<7)
#define GS_CAN_FEATURE_FD						(1<<8) /* device supports CAN-FD */
/* request workaround for LPC546XX erratum USB.15:
 * let host driver add a padding byte to each USB frame
 */
#define GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX	(1<<9)
/* device supports separate bit timing constants for CAN-FD
 * arbitration and data phase, see:
 * GS_USB_BREQ_BT_CONST_EXT and struct gs_device_bt_const_extended
 */
#define GS_CAN_FEATURE_BT_CONST_EXT				(1<<10)
/* device supports switchable termination, see:
 * - GS_USB_BREQ_SET_TERMINATION
 * - GS_USB_BREQ_GET_TERMINATION
 * - struct gs_device_termination_state
 */
#define GS_CAN_FEATURE_TERMINATION				(1<<11)
#define GS_CAN_FEATURE_BERR_REPORTINGBIT		(1<<12)
#define GS_CAN_FEATURE_GET_STATE				(1<<13)

#define CAN_EFF_FLAG							0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG							0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG							0x20000000U /* error message frame */

/* data types passed between host and device */
struct gs_host_config {
	uint32_t byte_order;
} __packed __aligned(4);

/* The firmware on the original USB2CAN by Geschwister Schneider
 * Technologie Entwicklungs- und Vertriebs UG exchanges all data
 * between the host and the device in host byte order. This is done
 * with the struct gs_host_config::byte_order member, which is sent
 * first to indicate the desired byte order.
 *
 * The widely used open source firmware candleLight doesn't support
 * this feature and exchanges the data in little endian byte order.
 */
struct gs_device_config {
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t icount;
	uint32_t sw_version;
	uint32_t hw_version;
} __packed __aligned(4);

struct gs_device_mode {
	uint32_t mode;
	uint32_t flags;
} __packed __aligned(4);

struct gs_device_state {
	uint32_t state;
	uint32_t rxerr;
	uint32_t txerr;
} __packed __aligned(4);

struct gs_device_bittiming {
	uint32_t prop_seg;
	uint32_t phase_seg1;
	uint32_t phase_seg2;
	uint32_t sjw;
	uint32_t brp;
} __packed __aligned(4);

struct gs_device_bt_const {
	uint32_t feature;
	uint32_t fclk_can;
	uint32_t tseg1_min;
	uint32_t tseg1_max;
	uint32_t tseg2_min;
	uint32_t tseg2_max;
	uint32_t sjw_max;
	uint32_t brp_min;
	uint32_t brp_max;
	uint32_t brp_inc;
} __packed __aligned(4);

// device info
static const struct gs_device_config USBD_GS_CAN_dconf = {
	.reserved1 = 0,
	.reserved2 = 0,
	.reserved3 = 0,
	.icount = 1, //(0=1, 1=2..)
	.sw_version = 2,
	.hw_version = 1 
};

// bit timing constraints
static const struct gs_device_bt_const USBD_GS_CAN_btconst = {
	GS_CAN_FEATURE_LISTEN_ONLY  // supported features
	| GS_CAN_FEATURE_LOOP_BACK
	| GS_CAN_FEATURE_HW_TIMESTAMP
	| GS_CAN_FEATURE_IDENTIFY
	| GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE
	,
	36000000, // can timing base clock
	1, // tseg1 min
	16, // tseg1 max
	1, // tseg2 min
	8, // tseg2 max
	4, // sjw max
	1, // brp min
	1024, //brp_max
	1, // brp increment;
};

struct gs_host_frame {
	uint32_t echo_id;
	uint32_t can_id;

	uint8_t can_dlc;
	uint8_t channel;
	uint8_t flags;
	uint8_t reserved;

	uint8_t data[8];

	uint32_t timestamp_us;
} __packed __aligned(4);
/* gs_usb.h */

#define MSGS_SIZE 100
//to host
struct ring_t gs_tx_ring;
static uint8_t tx_ring_buffer[sizeof(struct gs_host_frame) * MSGS_SIZE];

//from host
struct ring_t gs_rx_ring;
static uint8_t rx_ring_buffer[sizeof(struct gs_host_frame) * MSGS_SIZE];

static const struct usb_device_descriptor desc =
{
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1d50,
	.idProduct = 0x606f,
	.bcdDevice = 0x0200,
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
		.bEndpointAddress = 0x81,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 0,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x02,
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
	.bNumEndpoints = 2,
	.bInterfaceClass = 0xFF, /* Vendor Specific */
	.bInterfaceSubClass = 0xFF,
	.bInterfaceProtocol = 0xFF,
	.iInterface = 0,
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
	.iConfiguration = 0/*4*/,
	.bmAttributes = 0x80,
	.bMaxPower = 0x4b,

	.interface = ifaces,
};

struct usbd_msft_sig_descr
{
	uint8_t	bLength;
	uint8_t	bType;
	uint8_t wSignature[14];
	uint8_t bVendorCode;
	uint8_t reserved;
};

const struct usbd_msft_sig_descr usbMsftSigStr =
{
	.bLength = 0x12,
	.bType = USB_DT_STRING,
	.wSignature = { 'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00, '1', 0x00, '0', 0x00, '0', 0x00 },
	.bVendorCode = 0x40,
	.reserved = 0x00
};

struct usbd_msft_comp_id
{
	uint32_t dwLength;
	uint16_t wBCD;
	uint16_t wCompabilityId;
	uint8_t  bSectionNumber;
	uint8_t  bReserved[7];
	uint8_t  bIfaceNo;
	uint8_t  bReseved1;
	uint8_t  bCompatibleId[8];
	uint8_t  bSubCompatibleId[8];
	uint8_t  bReserved2[6];
};

const struct usbd_msft_comp_id usbMsftWinUsb =
{
	.dwLength = 0x28,
	.wBCD = 0x0100,
	.wCompabilityId = 0x0004,
	.bSectionNumber = 0x01,
	.bReserved = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	.bIfaceNo = 0x00,
	.bReseved1 = 0x01,
	.bCompatibleId = { 'W', 'I', 'N', 'U', 'S', 'B', 0, 0 },
	.bSubCompatibleId = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	.bReserved2 = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

const struct usb_device_qualifier_descriptor usbDeviceQulifier =
{
	.bLength = 10,
	.bDescriptorType = USB_DT_DEVICE_QUALIFIER,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xFF,
	.bDeviceSubClass = 0x00,
	.bDeviceProtocol = 0x00,
	.bMaxPacketSize0 = 64,
	.bNumConfigurations = 1,
	.bReserved = 0
};

static char usb_serial_number[25] = { '9', '1', '1', '\0' };

static const char *usb_strings[] =
{
	"budgetcan",
	"budgetcan gs_usb",
	usb_serial_number,
	"gs_usb interface"
};

static volatile bool gs_is_configured = false;

static uint32_t sof_timestamp_us = 0;

struct gs_device_bittiming bt0, bt1;

static enum usbd_request_return_codes usb_gs_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)usbd_dev;
	(void)complete;

	const uint8_t type = USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE;
	const uint8_t mask = USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT;

	struct gs_device_mode mode;
	uint32_t param_u32;

	if ((req->bmRequestType & mask) != type)
		return USBD_REQ_NEXT_CALLBACK;
		//return USBD_REQ_NOTSUPP; /* Only accept class request. */

	switch (req->bRequest) {

		case GS_USB_BREQ_SET_TERMINATION:
		case GS_USB_BREQ_GET_TERMINATION:
			return USBD_REQ_NOTSUPP;

		case GS_USB_BREQ_HOST_FORMAT:
			/* The firmware on the original USB2CAN by Geschwister Schneider
			 * Technologie Entwicklungs- und Vertriebs UG exchanges all data
			 * between the host and the device in host byte order. This is done
			 * with the struct gs_host_config::byte_order member, which is sent
			 * first to indicate the desired byte order.
			 *
			 * The widely used open source firmware candleLight doesn't support
			 * this feature and exchanges the data in little endian byte order.
			 */
			return USBD_REQ_HANDLED;
		
		case GS_USB_BREQ_IDENTIFY:
			memcpy(&param_u32, *buf, sizeof(param_u32));
			if (param_u32) {
				//led_run_sequence(hcan->leds, led_identify_seq, -1);
			} else {
				//ch = &hcan->channels[req->wValue]; // TODO verify wValue input data (implement getChannelData() ?)
				//led_set_mode(hcan->leds, can_is_enabled(ch) ? led_mode_normal : led_mode_off);
			}
			return USBD_REQ_HANDLED;
		
		case GS_USB_BREQ_MODE:
			/*num channels*/
			if (req->wValue < 2) {

				struct can_t * can = (req->wValue == 0) ? can_get_hscan() : can_get_mscan();
				memcpy(&mode, *buf, sizeof(struct gs_device_mode));

				if (mode.mode == GS_CAN_MODE_RESET) {

					can_stop(can);
				}
				else if (mode.mode == GS_CAN_MODE_START) {

					//hcan->timestamps_enabled = (mode->flags & GS_CAN_MODE_HW_TIMESTAMP) != 0;
					//hcan->pad_pkts_to_max_pkt_size = (mode->flags & GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE) != 0;

					struct gs_device_bittiming * bt = (req->wValue == 0) ? &bt0 : &bt1;

					uint32_t sjw = (bt->sjw - 1) << 24;
					uint32_t ts2 = (bt->phase_seg2 - 1) << 20;
					uint32_t ts1 = (bt->prop_seg + bt->phase_seg1 - 1) << 16;
					can_set_bittiming(can, bt->brp, ts1, ts2, sjw);
#if 0
					(mode->flags & GS_CAN_MODE_LOOP_BACK) != 0,
					(mode->flags & GS_CAN_MODE_LISTEN_ONLY) != 0,
					(mode->flags & GS_CAN_MODE_ONE_SHOT) != 0
							);
#endif
				}
			}
			return USBD_REQ_HANDLED;
		
		case GS_USB_BREQ_BITTIMING:
			/*num channels*/
			if (req->wValue < 2) {

				struct gs_device_bittiming * bt = (req->wValue == 0) ? &bt0 : &bt1;
				memcpy(bt, *buf, sizeof(struct gs_device_bittiming));
			}
			return USBD_REQ_HANDLED;
		
		case GS_USB_BREQ_DEVICE_CONFIG:
			memcpy(*buf, &USBD_GS_CAN_dconf, sizeof(USBD_GS_CAN_dconf));
			*len = sizeof(USBD_GS_CAN_dconf);
			return USBD_REQ_HANDLED;

		case GS_USB_BREQ_BT_CONST:
			memcpy(*buf, &USBD_GS_CAN_btconst, sizeof(USBD_GS_CAN_btconst));
			*len = sizeof(USBD_GS_CAN_btconst);
			return USBD_REQ_HANDLED;

		case GS_USB_BREQ_TIMESTAMP:
			memcpy(*buf, &sof_timestamp_us, sizeof(sof_timestamp_us));
			*len = sizeof(sof_timestamp_us);
			return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NOTSUPP;
}

static void usb_gs_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	struct gs_host_frame gs_frame;
	ep = ep & 0x7f;

	int rxlen = usbd_ep_read_packet(usbd_dev, ep, &gs_frame, sizeof(gs_frame));

	// Invalid frame length, just ignore it
	if (rxlen < (int)(sizeof(struct gs_host_frame) - 4))
		return;

	ring_write(&gs_rx_ring, &gs_frame);
}

#define USB_REBASE(x) MMIO32((x) + (USB_OTG_FS_BASE))

static void usb_gs_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	uint8_t len = 0;
	struct gs_host_frame gs_frame;

	ep = ep & 0x7f;

	/* Return if endpoint is already enabled for transmit(fifo packet is not null) */
	if (USB_REBASE(OTG_DIEPTSIZ(ep)) & OTG_DIEPSIZ0_PKTCNT)
		return;

	if (!ring_read(&gs_tx_ring, &gs_frame))
		return;

	len = sizeof(struct gs_host_frame) - 4;
	if (len) {

		led2_on();

		usbd_ep_write_packet(usbd_dev, ep, &gs_frame, len);
	}
}

static void usb_gs_iface_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, endpoints[0].bEndpointAddress, endpoints[0].bmAttributes, endpoints[0].wMaxPacketSize, usb_gs_tx_cb);
	usbd_ep_setup(usbd_dev, endpoints[1].bEndpointAddress, endpoints[1].bmAttributes, endpoints[1].wMaxPacketSize, usb_gs_rx_cb);

	usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, usb_gs_control_request);

	gs_is_configured = true;
}

static void usb_gs_alt_set_config(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;
	(void)wIndex;
	(void)wValue;
}

static void usb_gs_preinit(void)
{
	ring_init(&gs_tx_ring, tx_ring_buffer, sizeof(struct gs_host_frame), MSGS_SIZE);
	ring_init(&gs_rx_ring, rx_ring_buffer, sizeof(struct gs_host_frame), MSGS_SIZE);

	desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));
}

/* Microsoft OS 1.0 descriptors */

/* Extended Compat ID OS Feature Descriptor Specification */
#define WINUSB_REQ_GET_COMPATIBLE_ID_FEATURE_DESCRIPTOR 0x04
#define WINUSB_REQ_GET_EXTENDED_PROPERTIES_OS_FEATURE_DESCRIPTOR 0x05

/* Table 2. Function Section */
struct winusb_compatible_id_function_section {
    uint8_t  bInterfaceNumber;
    uint8_t  reserved0[1];
    const char compatibleId[8];
    const char subCompatibleId[8];
    uint8_t  reserved1[6];
} __attribute__((packed));

#define WINUSB_COMPATIBLE_ID_FUNCTION_SECTION_SIZE 24


/* Table 1. Header Section */
struct winusb_compatible_id_descriptor {
    uint32_t dwLength;
    uint16_t bcdVersion;
    uint16_t wIndex;
    uint8_t  bNumSections;
    uint8_t  reserved[7];
    struct winusb_compatible_id_function_section functions[];
} __attribute__((packed));

#define WINUSB_COMPATIBLE_ID_HEADER_SIZE 16

#if 0
/*  Microsoft Compatible ID Feature Descriptor  */
static const uint8_t USBD_MS_COMP_ID_FEATURE_DESC[] = {
	0x40, 0x00, 0x00, 0x00, /* length */
	0x00, 0x01,             /* version 1.0 */
	0x04, 0x00,             /* descr index (0x0004) */
	0x02,                   /* number of sections */
	0x00, 0x00, 0x00, 0x00, /* reserved */
	0x00, 0x00, 0x00,
	0x00,                   /* interface number */
	0x01,                   /* reserved */
	0x57, 0x49, 0x4E, 0x55, /* compatible ID ("WINUSB\0\0") */
	0x53, 0x42, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, /* sub-compatible ID */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, /* reserved */
	0x00, 0x00,
	0x01,                   /* interface number */
	0x01,                   /* reserved */
	0x57, 0x49, 0x4E, 0x55, /* compatible ID ("WINUSB\0\0") */
	0x53, 0x42, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, /* sub-compatible ID */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, /* reserved */
	0x00, 0x00
};
#endif

static const struct winusb_compatible_id_descriptor winusb_wcid =
{
    .dwLength = (WINUSB_COMPATIBLE_ID_HEADER_SIZE + 1*WINUSB_COMPATIBLE_ID_FUNCTION_SECTION_SIZE),
    .bcdVersion = 0x0100,
    .wIndex = 0x0004,
    .bNumSections = 1,
    .reserved = { 0, 0, 0, 0, 0, 0, 0 },
    .functions = {
        {
            .bInterfaceNumber = 0,
            .reserved0 = { 1 },
            .compatibleId = "WINUSB",
            .subCompatibleId = "",
	    .reserved1 = { 0, 0, 0, 0, 0, 0}
        },
    }
};

struct winusb_extended_id_function_section {
    uint8_t  bInterfaceNumber;
    uint8_t  reserved0[1];
    const char compatibleId[8];
    const char subCompatibleId[8];
    uint8_t  reserved1[6];
} __attribute__((packed));


struct winusb_extended_id_descriptor {
    uint32_t dwLength;
    uint16_t bcdVersion;
    uint16_t wIndex;
    uint8_t  bNumSections;
    uint8_t  reserved[7];
    struct winusb_extended_id_function_section functions[];
} __attribute__((packed));


/* Microsoft Extended Properties Feature Descriptor */
static const uint8_t USBD_MS_EXT_PROP_FEATURE_DESC[] = {
	0x92, 0x00, 0x00, 0x00, /* length */
	0x00, 0x01,             /* version 1.0 */
	0x05, 0x00,             /* descr index (0x0005) */
	0x01, 0x00,             /* number of sections */
	0x88, 0x00, 0x00, 0x00, /* property section size */
	0x07, 0x00, 0x00, 0x00, /* property data type 7: Unicode REG_MULTI_SZ */
	0x2a, 0x00,             /* property name length */

	0x44, 0x00, 0x65, 0x00, /* property name "DeviceInterfaceGUIDs" */
	0x76, 0x00, 0x69, 0x00,
	0x63, 0x00, 0x65, 0x00,
	0x49, 0x00, 0x6e, 0x00,
	0x74, 0x00, 0x65, 0x00,
	0x72, 0x00, 0x66, 0x00,
	0x61, 0x00, 0x63, 0x00,
	0x65, 0x00, 0x47, 0x00,
	0x55, 0x00, 0x49, 0x00,
	0x44, 0x00, 0x73, 0x00,
	0x00, 0x00,

	0x50, 0x00, 0x00, 0x00, /* property data length */

	0x7b, 0x00, 0x63, 0x00, /* property name: "{c15b4308-04d3-11e6-b3ea-6057189e6443}\0\0" */
	0x31, 0x00, 0x35, 0x00,
	0x62, 0x00, 0x34, 0x00,
	0x33, 0x00, 0x30, 0x00,
	0x38, 0x00, 0x2d, 0x00,
	0x30, 0x00, 0x34, 0x00,
	0x64, 0x00, 0x33, 0x00,
	0x2d, 0x00, 0x31, 0x00,
	0x31, 0x00, 0x65, 0x00,
	0x36, 0x00, 0x2d, 0x00,
	0x62, 0x00, 0x33, 0x00,
	0x65, 0x00, 0x61, 0x00,
	0x2d, 0x00, 0x36, 0x00,
	0x30, 0x00, 0x35, 0x00,
	0x37, 0x00, 0x31, 0x00,
	0x38, 0x00, 0x39, 0x00,
	0x65, 0x00, 0x36, 0x00,
	0x34, 0x00, 0x34, 0x00,
	0x33, 0x00, 0x7d, 0x00,
	0x00, 0x00, 0x00, 0x00
};

#define WINUSB_MS_VENDOR_CODE 0x20
static enum usbd_request_return_codes winusb_control_vendor_request(usbd_device *usbd_dev,
                              struct usb_setup_data *req,
                              uint8_t **buf, uint16_t *len,
                              usbd_control_complete_callback* complete) {
	(void)complete;
	(void)usbd_dev;

	if (req->bRequest != WINUSB_MS_VENDOR_CODE) {
		return USBD_REQ_NEXT_CALLBACK;
	}

	enum usbd_request_return_codes status = USBD_REQ_NOTSUPP;
	if (((req->bmRequestType & USB_REQ_TYPE_RECIPIENT) == USB_REQ_TYPE_DEVICE) &&
			(req->wIndex == WINUSB_REQ_GET_COMPATIBLE_ID_FEATURE_DESCRIPTOR)) {
		*buf = (uint8_t*)(&winusb_wcid);
		if (*len > winusb_wcid.dwLength) {
			*len = winusb_wcid.dwLength;
		}
		status = USBD_REQ_HANDLED;
	}
	else if (((req->bmRequestType & USB_REQ_TYPE_RECIPIENT) == USB_REQ_TYPE_INTERFACE) &&
			((req->wIndex == WINUSB_REQ_GET_EXTENDED_PROPERTIES_OS_FEATURE_DESCRIPTOR) || (req->wIndex == 0))) {

		if (req->wValue==0) { // only return our GUID for interface #0

			*buf = (uint8_t*)USBD_MS_EXT_PROP_FEATURE_DESC;
			uint32_t sz = sizeof(USBD_MS_EXT_PROP_FEATURE_DESC);
			if (*len > sz) {
				*len = sz;
			}
			status = USBD_REQ_HANDLED;
		}
	}

	return status;
}

static void winusb_set_config(usbd_device* usbd_dev, uint16_t wValue)
{
	(void)wValue;
	usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_VENDOR/* | USB_REQ_TYPE_INTERFACE*/, USB_REQ_TYPE_TYPE/* | USB_REQ_TYPE_RECIPIENT*/, winusb_control_vendor_request);
}

static void usb_gs_init(usbd_device * usbd_dev)
{
	static const char msft_extra_string[] = {'M', 'S', 'F', 'T', '1', '0', '0', WINUSB_MS_VENDOR_CODE, '\0'};
	usbd_register_extra_string(usbd_dev, 0xEE, msft_extra_string);
	usbd_register_set_config_callback(usbd_dev, winusb_set_config);

	/* Windows probes the compatible ID before setting the configuration, so also register the callback now */
	winusb_set_config(usbd_dev, 0);
}

static void usb_gs_process(usbd_device * usbd_dev)
{
	if (gs_is_configured) {

		uint8_t ep = endpoints[0].bEndpointAddress & 0x7f;
		usb_gs_tx_cb(usbd_dev, ep);
	}
}

static void usb_gs_sof(void)
{
	sof_timestamp_us = timer_get();
}

static void usb_gs_reset(void)
{
	gs_is_configured = false;
}

static void usb_gs_disable(void)
{
	gs_is_configured = false;
}

struct usb_cb_t gs_cb =
{
	.desc = &desc,
	.strings = usb_strings,
	.num_strings = 4,
	.config = &config,
	.preinit = usb_gs_preinit,
	.init = usb_gs_init,
	.iface_set_config = usb_gs_iface_set_config,
	.alt_set_config = usb_gs_alt_set_config,
	.process = usb_gs_process,
	.sof = usb_gs_sof,
	.reset = usb_gs_reset,
	.disable = usb_gs_disable,
};

void usb_gs_msg_push(uint8_t ch, const can_message_t * msg)
{
	struct gs_host_frame frame;

	if (!gs_is_configured)
		return;

	frame.echo_id = msg->echo_id;
	frame.can_id = msg->id;
	frame.can_id |= msg->IDE ? CAN_EFF_FLAG : 0; 
	frame.can_id |= msg->RTR ? CAN_RTR_FLAG : 0; 
	frame.can_dlc = msg->dlc;
	frame.channel = ch;
	frame.flags = 0;
	frame.reserved = 0;
	frame.timestamp_us = timer_get();
	for (uint8_t i = 0; i < msg->dlc; i++)
		frame.data[i] = msg->data[i];

	ring_write(&gs_tx_ring, &frame);
}

bool usb_gs_msg_pop(uint8_t * ch, can_message_t * msg)
{
	struct gs_host_frame gs_frame;

	if (!gs_is_configured)
		return false;

	if (!ring_read(&gs_rx_ring, &gs_frame))
		return false;

	msg->id = gs_frame.can_id & 0x1FFFFFFF;
	msg->echo_id = gs_frame.echo_id;
	msg->dlc = gs_frame.can_dlc;
	msg->IDE = (gs_frame.can_id & CAN_EFF_FLAG) ? 1 : 0;
	msg->RTR = (gs_frame.can_id & CAN_RTR_FLAG) ? 1 : 0;
	for (uint8_t i = 0; i < gs_frame.can_dlc; i++) {
		msg->data[i] = gs_frame.data[i];
	}

	*ch = 2;
	if (gs_frame.channel == 0)
		*ch = 0;
	if (gs_frame.channel == 1)
		*ch = 1;

	return true;
}

