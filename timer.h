#ifndef TIMER_H
#define TIMER_H

#include <inttypes.h>

void timer_setup(void);
void timer_disable(void);

uint32_t timer_get(void);
void udelay(uint32_t us);
void mdelay(uint32_t ms);
uint8_t timeout_is_expired(uint32_t start_us, uint32_t timeout_us);

#endif

