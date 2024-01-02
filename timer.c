#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#include "timer.h"

void timer_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM5);

	timer_disable_counter(TIM5);
	timer_disable_irq(TIM5, 0x7f);

	timer_direction_up(TIM5);
	timer_continuous_mode(TIM5);
	timer_disable_preload(TIM5);
	/* APB1 -> 1MHz, overflow 4 000 sec */
	timer_set_prescaler(TIM5, 2/*TIMPRE*/ * (rcc_apb1_frequency / 1000000) - 1);
	timer_set_period(TIM5, 0xffffffff);

	timer_enable_update_event(TIM5);
	timer_generate_event(TIM5, TIM_EGR_UG);
	timer_set_counter(TIM5, 0);
	timer_enable_counter(TIM5);
}

uint32_t timer_get(void)
{
	return timer_get_counter(TIM5);
}

static uint32_t timer_delta(uint32_t start)
{
	return timer_get_counter(TIM5) - start;
}

uint8_t timeout_is_expired(uint32_t start_us, uint32_t timeout_us)
{
	uint32_t us = timer_delta(start_us);

	if (us >= timeout_us)
		return 1;

	return 0;
}

void udelay(uint32_t us)
{
	uint32_t start = timer_get();

	while(1) {

		uint32_t t = timer_delta(start);

		if (t >= us)
			break;
	}
}

void mdelay(uint32_t ms)
{
	for (uint32_t i = 0; i < ms; i++)
		udelay(1000);
}

void timer_disable(void)
{
	timer_disable_counter(TIM5);
	rcc_periph_clock_disable(RCC_TIM5);
}

