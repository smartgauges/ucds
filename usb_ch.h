#ifndef USB_CH_H
#define USB_CH_H

#include "usb.h"
#include "ring.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct ring_t * get_tx_ring(void);
struct ring_t * get_rx_ring(void);
void set_dtr(void);
void clr_dtr(void);

extern usb_cb_t ch_cb;

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif

