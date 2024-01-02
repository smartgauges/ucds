#ifndef TICK_H
#define TICK_H

#define TICK_HZ 1000
#define SEC_TO_TICK(SEC) (TICK_HZ * SEC)
#define MSEC_TO_TICK(MSEC) ((MSEC * TICK_HZ)/1000)

typedef struct
{
	volatile uint8_t flag_tick;
	volatile uint8_t flag_100ms;
	volatile uint8_t flag_250ms;
	volatile uint8_t flag_1000ms;
	volatile uint32_t msec;
	volatile uint32_t sec;
} tick_t;
extern volatile tick_t tick;

void systick_setup(void);
void systick_disable(void);

#endif

