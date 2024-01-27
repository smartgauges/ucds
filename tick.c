#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/cortex.h>

#include "tick.h"
#include "led.h"

void systick_disable(void)
{
	systick_interrupt_disable();
	systick_counter_disable();
}

void systick_setup(void)
{
	systick_disable();

	/* (ahb_frequency / 8) counts per second */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/* clear counter so it starts right away */
	STK_CVR = 0;

	systick_set_reload((rcc_ahb_frequency / 8) / TICK_HZ);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

volatile tick_t tick = { 0, 0, 0, 0, 0, 0 };
volatile uint32_t usecs = 0;

void sys_tick_handler(void)
{
	static uint16_t div_1000ms = 0;
	static uint16_t div_250ms = 0;
	static uint16_t div_100ms = 0;

	tick.flag_tick = 1;
	tick.msec++;
	usecs += 1000;

	if (++div_1000ms >= SEC_TO_TICK(1)) {

		div_1000ms = 0;
		tick.flag_1000ms = 1;
		tick.sec++;
		tick.msec = 0;
	}

	if (++div_250ms >= MSEC_TO_TICK(250)) {

		div_250ms = 0;
		tick.flag_250ms = 1;
	}

	if (++div_100ms >= MSEC_TO_TICK(50)) {

		div_100ms = 0;
		tick.flag_100ms = 1;
	}

	led_tick();
}

uint32_t systick_get_usecs(void)
{
	cm_disable_interrupts();
	uint32_t v = systick_get_value();
	uint32_t value = usecs;
	cm_enable_interrupts();

	value += (1000 * v) / systick_get_reload();

	return value;
}
