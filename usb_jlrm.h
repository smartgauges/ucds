#ifndef USB_JLRM_H
#define USB_JLRM_H

#include "usb.h"
#include "ring.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct ring_t * get_tx_ring(void);
struct ring_t * get_rx_ring(void);

extern usb_cb_t jlrm_cb;

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif

