#ifndef USB_H
#define USB_H

#include <stdint.h>
#include <libopencm3/usb/usbd.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define USB_CB_SIZE 512

//struct usbd_device;
typedef struct usb_cb_t
{ 
	const struct usb_device_descriptor * desc;
	const char * const * strings;
	uint8_t num_strings;
	const struct usb_config_descriptor * config;
	void (*preinit)(void);
	void (*init)(usbd_device *);
	void (*iface_set_config)(usbd_device *, uint16_t);
	void (*alt_set_config)(usbd_device *, uint16_t, uint16_t);
	void (*process)(usbd_device *);
	void (*sof)(void);
	void (*reset)(void);
	void (*disable)(void);
	void (*eopf)(usbd_device *);
} usb_cb_t;

void usb_setup(usb_cb_t *);
void usb_disable(void);
void usb_process(void);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif

