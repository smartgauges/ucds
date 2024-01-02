#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>

#include "usb.h"
#include "usb_dfu.h"
#include "bl.h"

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

bool dfu_in_progress(void)
{
	return (usbdfu_state == STATE_DFU_IDLE) ? false : true;
}

static struct {
	uint8_t buf[USB_CB_SIZE];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog = {
	.len = 0,
	.addr = 0,
	.blocknum = 0,
};

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
	.idProduct = 0xdf11,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_UPLOAD | USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = USB_CB_SIZE,
	.bcdDFUVersion = 0x011A,
};

static const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_DFU, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,

	/* The ST Microelectronics DfuSe application needs this string.
	 * The format isn't documented... */
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

static const struct usb_interface ifaces[] =
{
	{
	.num_altsetting = 1,
	.altsetting = &iface,
	}
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static char usb_serial_number[25] = { '9', '1', '1', '\0' };

static const char * usb_strings[] =
{
	"STMicroelectronics",
	"STM32  BOOTLOADER",
	usb_serial_number,
	/* This string is used by ST Microelectronics' DfuSe utility. */
	"@Internal Flash  /0x08000000/8*002Ka,56*002Kg",
};

static uint8_t usbdfu_getstatus(usbd_device *usbd_dev, uint32_t *bwPollTimeout)
{
	(void)usbd_dev;

	switch (usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 100;
		return DFU_STATUS_OK;
	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete. */
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;
	default:
		return DFU_STATUS_OK;
	}
}

bool flag_start_app = false;
bool dfu_wants_start_app(void)
{
	bool ret = flag_start_app;
	flag_start_app = false;
	return ret;
}

static void usbdfu_getstatus_complete(usbd_device *usbd_dev, struct usb_setup_data *req)
{
	(void)req;
	(void)usbd_dev;

	// Protect the flash by only writing to the valid flash area
	const uint32_t start_addr = ADDR_APP;
	const uint32_t end_addr = ADDR_APP_END + 1;

	switch (usbdfu_state) {
	case STATE_DFU_DNBUSY:
		flash_unlock();
		if (prog.blocknum == 0) {
			switch (prog.buf[0]) {
			case CMD_ERASE:
				{
					// Clear this page here.
					uint32_t baseaddr = *(uint32_t *)(prog.buf + 1);
					if (baseaddr >= start_addr && (baseaddr + USB_CB_SIZE) <= end_addr)
						flash_erase_page(baseaddr);
				
				}
				break;
			case CMD_SETADDR:
				
				{
					// Assuming little endian here.
					uint32_t *dat = (uint32_t *)(prog.buf + 1);
					prog.addr = *dat;
				}
				break;
			}
		} else {

			//Address_Pointer + ((wBlockNum - 2)*wTransferSize)
			uint32_t baseaddr = prog.addr + ((prog.blocknum - 2) * dfu_function.wTransferSize);
			if (baseaddr >= start_addr && baseaddr + prog.len <= end_addr) {

				int i;
				for (i = 0; i < prog.len; i += 2) {
					uint16_t *dat = (uint16_t *)(prog.buf + i);
					flash_program_half_word(baseaddr + i, *dat);
		
				}
			}
		}
		flash_lock();

		/* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC. */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;
	case STATE_DFU_MANIFEST:
		flag_start_app = true;
		/* USB device must detach, we just reset... */
		//scb_reset_system();
		usbdfu_state = STATE_DFU_IDLE;
		return; /* Will never return. */
	default:
		return;
	}
}

static volatile bool dfu_is_configured = false;

static enum usbd_request_return_codes usbdfu_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	const uint8_t type = USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE;
	const uint8_t mask = USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT;

	if ((req->bmRequestType & mask) != type)
		return USBD_REQ_NOTSUPP; /* Only accept class request. */

	switch (req->bRequest) {
	case DFU_DNLOAD:
		if ((len == NULL) || (*len == 0)) {
			// wLength = 0 means leave DFU
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			*complete = usbdfu_getstatus_complete;
		} else {
			/* Copy download data for use on GET_STATUS. */
			prog.blocknum = req->wValue;
			// Beware overflows!
			prog.len = *len;
			if (prog.len > sizeof(prog.buf))
				prog.len = sizeof(prog.buf);
			memcpy(prog.buf, *buf, prog.len);
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
		}
		return USBD_REQ_HANDLED;
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE. */
		if (usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_ABORT:
		/* Abort returns to dfuIDLE state. */
		usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_DETACH:
		usbdfu_state = STATE_DFU_MANIFEST_SYNC;
		*complete = usbdfu_getstatus_complete;
		return USBD_REQ_HANDLED;
	case DFU_UPLOAD:
		// Send data back to host by reading the image.
		usbdfu_state = STATE_DFU_UPLOAD_IDLE;
		if (!req->wValue) {
			// Send back supported commands.
			(*buf)[0] = 0x00;
			(*buf)[1] = CMD_SETADDR;
			(*buf)[2] = CMD_ERASE;
			*len = 3;
			return USBD_REQ_HANDLED;
		} else {
			//Address_Pointer + ((wBlockNum - 2)*wTransferSize)
			uint32_t baseaddr = prog.addr + ((req->wValue - 2) * USB_CB_SIZE);
			const uint32_t start_addr = ADDR_BL;
			const uint32_t end_addr = ADDR_APP_END + 1;
			if (baseaddr >= start_addr && baseaddr + USB_CB_SIZE <= end_addr) {
				memcpy((*buf), (void*)baseaddr, USB_CB_SIZE);
				*len = USB_CB_SIZE;
			} else {
				usbdfu_state = STATE_DFU_ERROR;
				*len = 0;
			}
		}
		return USBD_REQ_HANDLED;
	case DFU_GETSTATUS: {
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */
		(*buf)[0] = usbdfu_getstatus(usbd_dev, &bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = usbdfu_state;
		(*buf)[5] = 0; /* iString not used here */
		*len = 6;
		*complete = usbdfu_getstatus_complete;
		return USBD_REQ_HANDLED;
		}
	case DFU_GETSTATE:
		/* Return state with no state transision. */
		*buf[0] = usbdfu_state;
		*len = 1;
		return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NOTSUPP;
}

static void usb_dfu_iface_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbdfu_control_request);
}

static void usb_dfu_alt_set_config(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;
	(void)wIndex;
	(void)wValue;
}

static void usb_dfu_preinit(void)
{
	desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));
}

static void usb_dfu_process(usbd_device * usbd_dev)
{
	(void)usbd_dev;
}

static void usb_cdcacm_reset(void)
{
	dfu_is_configured = false;
}

static void usb_cdcacm_disable(void)
{
	dfu_is_configured = false;
}

struct usb_cb_t dfu_cb =
{
	.desc = &desc,
	.strings = usb_strings,
	.num_strings = 4,
	.config = &config,
	.preinit = usb_dfu_preinit,
	.init = NULL,
	.iface_set_config = usb_dfu_iface_set_config,
	.alt_set_config = usb_dfu_alt_set_config,
	.process = usb_dfu_process,
	.reset = usb_cdcacm_reset,
	.disable = usb_cdcacm_disable,
};

