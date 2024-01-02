#ifndef USB_DFU_H
#define USB_DFU_H

#include <stdint.h>
#include <stdbool.h>
#include <libopencm3/usb/usbd.h>

#include "usb.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern usb_cb_t dfu_cb;
bool dfu_in_progress(void);
bool dfu_wants_start_app(void);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif

