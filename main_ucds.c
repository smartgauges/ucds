#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/cortex.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "ring.h"
#include "led.h"
#include "tick.h"
#include "clock.h"
#include "usb.h"
#include "usb_ucds.h"
#include "timer.h"

int main(void)
{
	cm_disable_interrupts();

	clock_setup();
	timer_setup();
	led_setup();
	systick_setup();

	iwdg_set_period_ms(1000);

	usb_setup(&ucds_cb);

	cm_enable_interrupts();

	while(1)
	{
		iwdg_reset();

		if (tick.flag_tick) {

			tick.flag_tick = 0;

			if (tick.flag_250ms) {

				tick.flag_250ms = 0;

				led4_blink();
			}

			if (tick.flag_1000ms) {

				tick.flag_1000ms = 0;
			}
		}
	}

	return 0;
}

