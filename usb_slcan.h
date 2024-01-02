#ifndef USB_SLCAN_H
#define USB_SLCAN_H

#include "usb.h"
#include "ring.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct ring_t * get_tx_ring0(void);
struct ring_t * get_rx_ring0(void);
struct ring_t * get_tx_ring1(void);
struct ring_t * get_rx_ring1(void);

extern usb_cb_t slcan_cb;

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif

