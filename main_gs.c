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

#include "led.h"
#include "tick.h"
#include "clock.h"
#include "usb.h"
#include "usb_gs.h"
#include "timer.h"
#include "can.h"

static void gs_usb_read(void)
{
	can_message_t msg;
	uint8_t ch = 2;

	if (usb_gs_msg_pop(&ch, &msg)) {

		if (ch == 0)
			can_msg_push(can_get_hscan(), &msg);
		if (ch == 1)
			can_msg_push(can_get_mscan(), &msg);
	}
}

static void gs_can_read(struct can_t * can, uint8_t ch)
{
	can_message_t msg;
	uint8_t r = can_msg_pop(can, &msg);
	if (r)
		usb_gs_msg_push(ch, &msg);

	//ack tx
	r = can_echo_pop(can, &msg);
	if (r)
		usb_gs_msg_push(ch, &msg);

	can_tx_process(can);
}

int main(void)
{
	cm_disable_interrupts();

	clock_setup();
	timer_setup();
	led_setup();
	systick_setup();

	iwdg_set_period_ms(1000);

	usb_setup(&gs_cb);

	can_setup();

	cm_enable_interrupts();

	uint32_t hscan_rx_cnts = 0;
	uint32_t hscan_tx_cnts = 0;
	uint32_t mscan_rx_cnts = 0;
	uint32_t mscan_tx_cnts = 0;

	while(1)
	{
		iwdg_reset();

		gs_usb_read();
		gs_can_read(can_get_hscan(), 0);
		gs_can_read(can_get_mscan(), 1);

		if (tick.flag_tick) {

			tick.flag_tick = 0;

			if (tick.flag_100ms) {

				bool activity = false;

				uint32_t cnts = can_get_rx_cnts(can_get_hscan());
				if (cnts != hscan_rx_cnts)
					activity = true;
				hscan_rx_cnts = cnts;

				cnts = can_get_tx_cnts(can_get_hscan());
				if (cnts != hscan_tx_cnts)
					activity = true;
				hscan_tx_cnts = cnts;

				cnts = can_get_rx_cnts(can_get_mscan());
				if (cnts != mscan_rx_cnts)
					activity = true;
				mscan_rx_cnts = cnts;

				cnts = can_get_tx_cnts(can_get_mscan());
				if (cnts != mscan_tx_cnts)
					activity = true;
				mscan_tx_cnts = cnts;

				if (activity)
					led4_blink();

				tick.flag_100ms = 0;
			}

			if (tick.flag_250ms) {

				tick.flag_250ms = 0;

				led4_blink();
			}
		}
	}

	return 0;
}

