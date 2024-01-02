#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/iwdg.h>

#include "timer.h"
#include "led.h"
#include "clock.h"
#include "tick.h"
#include "adc.h"
#include "usb.h"
#include "usb_dfu.h"
#include "bl.h"

void start_app(void)
{
	/* Boot the application if it's valid. */
	if ((*(volatile uint32_t *)ADDR_APP & 0x2FFE0000) == 0x20000000) {

		cm_disable_interrupts();

		/* Set vector table base address. */
		SCB_VTOR = ADDR_APP & 0xFFFF;
		/* Initialise master stack pointer. */
		asm volatile("msr msp, %0"::"g" (*(volatile uint32_t *)ADDR_APP));
		/* Jump to application. */
		(*(void (**)())(ADDR_APP + 4))();
	}
}

int main(void)
{
	cm_disable_interrupts();

	clock_setup();
	timer_setup();
	led_setup();
	systick_setup();
	adc_setup();

	uint16_t adc = adc_read();
	if (adc == 0xfff)
		adc = 0xffe;

	uint32_t voltage = (adc * 3300) / 0xfff;
	voltage = voltage * (10000 + 2200) / 2200;

	if (voltage > 9000)
		start_app();

	usb_setup(&dfu_cb);

	cm_enable_interrupts();

	iwdg_set_period_ms(1000);
	iwdg_start();

	while(1)
	{
		iwdg_reset();

		if (dfu_wants_start_app())
			start_app();

		if (tick.flag_tick) {

			tick.flag_tick = 0;

			if (tick.flag_1000ms) {

				tick.flag_1000ms = 0;
				led4_blink();

				if (tick.sec == 5 && !dfu_in_progress())
					start_app();
			}
		}
	}
}
