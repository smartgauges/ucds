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
#include "can.h"
#include "usb.h"
#include "usb_jlrm.h"
#include "timer.h"
#include "adc.h"

struct jlrm_t;
typedef struct jlrm_t
{
	uint8_t terminal_open:1;
	uint8_t transmit_enabled:1;
	uint8_t baudrate_configured:1;
	uint8_t baud;
	uint8_t fifo_rx_full;
	uint8_t fifo_tx_full;
	struct ring_t * ring_com_rx;
	struct ring_t * ring_com_tx;
	struct can_t * can;
	char cmd[64];
	uint8_t cmd_idx;
} jlrm_t;

static void jlrm_com_read(struct jlrm_t * jlrm)
{
	uint8_t ch = 0;
	cm_disable_interrupts();
	uint8_t r = ring_read(jlrm->ring_com_rx, &ch);
	cm_enable_interrupts();

	if (r) {

		jlrm->cmd[jlrm->cmd_idx] = ch;
		if (ch == '\r') {
			//jlrm_execute_command(jlrm, jlrm->cmd);
			jlrm->cmd_idx = 0;
		}
		else if (++jlrm->cmd_idx >= sizeof(jlrm->cmd))
			jlrm->cmd_idx = 0;
	}
}

static void jlrm_can_read(struct jlrm_t * jlrm)
{
	can_message_t msg;
	//cm_disable_interrupts();
	uint8_t r = can_msg_pop(jlrm->can, &msg);
	//cm_enable_interrupts();
	//if (r)
	//	jlrm_handle_can_message(jlrm, &msg);

	//ack tx
	r = can_echo_pop(jlrm->can, &msg);
	if (r) {
		//if (msg.IDE)
		//	jlrm_putstr(jlrm, "Z\r");
		//else
		//	jlrm_putstr(jlrm, "z\r");
	}

	can_tx_process(jlrm->can);
}

int main(void)
{
	cm_disable_interrupts();

	clock_setup();
	timer_setup();
	led_setup();
	systick_setup();

	iwdg_set_period_ms(1000);

	usb_setup(&jlrm_cb);

	can_setup();

	jlrm_t jlrm0 = {
		.terminal_open = 0,
		.transmit_enabled = 0,
		.baudrate_configured = 0,
		.baud = 0,
		.ring_com_rx = get_rx_ring(),
		.ring_com_tx = get_tx_ring(),
		.can = can_get_hscan(),
		.cmd_idx = 0
	};

	cm_enable_interrupts();

	while(1)
	{
		iwdg_reset();

		jlrm_com_read(&jlrm0);

		jlrm_can_read(&jlrm0);

		if (tick.flag_tick) {

			tick.flag_tick = 0;

			if (tick.flag_100ms) {

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

