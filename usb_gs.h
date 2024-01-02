#ifndef USB_GS_H
#define USB_GS_H

#include <stdint.h>
#include <libopencm3/usb/usbd.h>

#include "usb.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern usb_cb_t gs_cb;

struct can_message_t;
void usb_gs_msg_push(uint8_t ch, const struct can_message_t *msg);
bool usb_gs_msg_pop(uint8_t *ch, struct can_message_t *msg);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif

