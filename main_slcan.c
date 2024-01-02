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
#include "usb_slcan.h"
#include "timer.h"
#include "adc.h"

typedef enum
{
	can_mode_close,
	can_mode_normal,
	can_mode_listen,
	can_mode_loopback,
} slcan_can_mode_e;

struct slcan_t;
typedef struct slcan_t
{
	uint8_t terminal_open:1;
	uint8_t transmit_enabled:1;
	uint8_t baudrate_configured:1;
	uint8_t ts:1;
	uint8_t baud;
	uint8_t fifo_rx_full;
	uint8_t fifo_tx_full;
	struct ring_t * ring_com_rx;
	struct ring_t * ring_com_tx;
	struct can_t * can;
	char cmd[64];
	uint8_t cmd_idx;
	uint32_t rx_pause;
} slcan_t;

static bool slcan_init(struct slcan_t *slcan, slcan_can_mode_e mode)
{
	bool ret = true;
	uint8_t baud = slcan->baud;

	switch (mode) {
		case can_mode_close:
			can_stop(slcan->can);
			break;

		case can_mode_normal:
			ret = can_start(slcan->can, (e_speed_t)(baud));
			break;

		case can_mode_listen:
		case can_mode_loopback:
			break;
	}

	slcan->fifo_rx_full = 0;
	slcan->fifo_tx_full = 0;

	return ret;
}

static uint8_t slcan_get_status(struct slcan_t * slcan)
{
	uint8_t s = 0;
	s |= slcan->fifo_rx_full ? 0x01 : 0;
	s |= slcan->fifo_tx_full ? 0x02 : 0;

	return s;
}

static void slcan_putstr(struct slcan_t * slcan, const char * s)
{
	size_t l = strlen(s);
	for (size_t i = 0; i < l; i++) {

		cm_disable_interrupts();
		ring_write(slcan->ring_com_tx, (uint8_t *)&s[i]);
		cm_enable_interrupts();
	}
}

static uint8_t read_hex_number_8(const char * str, uint8_t len)
{
	uint8_t d = 0;
	while(len--){
		d <<= 4;
		uint8_t v = 0;
		if((*str >= '0') && (*str <= '9')) v = *str - '0';
		else if((*str >= 'A') && (*str <= 'F')) v = *str - 'A' + 10;
		else if((*str >= 'a') && (*str <= 'f')) v = *str - 'a' + 10;

		d |= v;
		str++;
	}
	return d;
}

static uint32_t read_hex_number(const char * str, uint8_t len)
{
	uint32_t d = 0;
	while(len--){
		d <<= 4;
		uint8_t v = 0;
		if((*str >= '0') && (*str <= '9')) v = *str - '0';
		else if((*str >= 'A') && (*str <= 'F')) v = *str - 'A' + 10;
		else if((*str >= 'a') && (*str <= 'f')) v = *str - 'a' + 10;

		d |= v;
		str++;
	}
	return d;
}

static bool send_can_message_from_string(slcan_t * slcan, const char * str)
{
	char cmd = *str++; // command char
	bool IDE = cmd == 'T' || cmd == 'R'; // upercase means EID
	bool RTR = cmd == 'r' || cmd == 'R'; // the upper or lowercase r means RTR

	uint32_t id;
	uint8_t dlc;

	if(!IDE){ //standard identifier
		id = read_hex_number(str, 3);
		str += 3;
		if(id > 0x7ff) return false;
	}else{
		id = read_hex_number(str, 8);
		str += 8;
		if(id > 0x1fffffff) return false;
	}

	dlc = read_hex_number_8(str, 1);
	str += 1;
	if(dlc > 8)
		return false;

	can_message_t cmsg;
	cmsg.RTR = RTR;
	cmsg.IDE = IDE;
	cmsg.dlc = dlc;
	cmsg.id = id;

	if (!RTR) {
		for (uint8_t i = 0; i < dlc; i++) {
			cmsg.data[i] = read_hex_number_8(str, 2);
			str += 2;
		}
	}
	can_msg_push(slcan->can, &cmsg);

	return true;
}

// puts one ascii hex digit (0-9, A-F) to a string for val in range
// 0-15, returns a pointer to the next position in the string
static char * put_hex_digit(char * str, uint8_t val){
	char c;
	val &= 0x0f;
	if(val < 10){
		c = val + '0';
	}else{
		c = val + 'A' - 10;
	}
	*str++ = c;
	return str;
}

// puts a byte as ascii hex to a string and returns a pointer
// to the nex position in the string.
static char * put_hex_byte(char * str, uint8_t val)
{
	str = put_hex_digit(str, val >> 4);
	str = put_hex_digit(str, val);
	return str;
}

static void execute_status_command(slcan_t * slcan)
{
	uint8_t s = slcan_get_status(slcan);

	char buf[6];
	char * str = buf;

	*str++ = 'F';
	str = put_hex_byte(str, s);
	*str++ = '\r';
	*str++ = 0;

	slcan_putstr(slcan, buf);
}

static void slcan_execute_command(slcan_t * slcan, const char * str, slcan_t * loopback)
{
	(void)loopback;

	switch(str[0]) {

		case 't': // transmit standard ID messages
		case 'r':
			if (slcan->transmit_enabled && send_can_message_from_string(slcan, str)) {
				//slcan_putstr(slcan, "z\r");
				//slcan_putstr(loopback, str);
			}
			else {
				slcan_putstr(slcan, "\a"); // bell
			}
			break;

		case 'T': // transmit extended ID messages
		case 'R':
			if (slcan->transmit_enabled && send_can_message_from_string(slcan, str)) {
				//slcan_putstr(slcan, "Z\r");
			}
			else {
				slcan_putstr(slcan, "\a"); // bell
			}
			break;

		case 'O': //open terminal
			if (slcan->baudrate_configured && !slcan->terminal_open && slcan_init(slcan, can_mode_normal)) {
				slcan->terminal_open = 1;
				slcan->transmit_enabled = 1;
				slcan_putstr(slcan, "\r");
			}
			else {
				slcan_putstr(slcan, "\a"); // bell
			}
			break;

		case 'L': // open terminal listen only
			if (slcan->baudrate_configured && !slcan->terminal_open && slcan_init(slcan, can_mode_listen)) {
				slcan->terminal_open = 1;
				slcan->transmit_enabled = 0;
				slcan_putstr(slcan, "\r");
			}
			else {
				slcan_putstr(slcan, "\a"); // bell
			}
			break;

		case 'l': // (lower case L) open terminal in loopback mode (non standard feature)
			if (slcan->baudrate_configured && !slcan->terminal_open && slcan_init(slcan, can_mode_loopback)) {
				slcan->terminal_open = 1;
				slcan->transmit_enabled = 1;
				slcan_putstr(slcan, "\r");
			}
			else {
				slcan_putstr(slcan, "\a"); // bell
			}
			break;

		case 'C': //close terminal
			if (slcan->terminal_open) {
				slcan->terminal_open = 0;
				slcan->transmit_enabled = 0;
				slcan->baud = 0;
				slcan->ts = 0;
				slcan->baudrate_configured = 0;
				//slcan_init(slcan, can_mode_close);
				slcan_putstr(slcan, "\r");
			}
			else {
				slcan_putstr(slcan, "\a"); // bell
			}
			break;

		case 'F': // request status
			execute_status_command(slcan);
			break;

		case 'S': //select bitrate e.g. "S4" for 125kbit
			if (slcan->terminal_open || str[1] < '0' || str[1] > '8') {
				// any other character is also error
				// or terminal is open, we cannot set a baud rate
				slcan_putstr(slcan, "\a"); // bell
			}
			else {
				slcan->baudrate_configured = 1;
				slcan->baud = str[1] - '0';
				slcan_putstr(slcan, "\r");
			}
			break;

		case 'U': //setup uart baudrate
			slcan_putstr(slcan, "\r");
			break;

		case 'P': //single poll
			slcan_putstr(slcan, "\a"); // bell
			break;

		case 'A': //auto poll
			slcan_putstr(slcan, "\a"); // bell
			break;

		case 'X': //set auto poll/send
			slcan_putstr(slcan, "\r");
			break;

		case 'W': //filter
			slcan_putstr(slcan, "\a"); // bell
			break;

		case 'Z': //timestamp
			if (slcan->terminal_open || (str[1] != '0' && str[1] != '1')) {
				slcan_putstr(slcan, "\a"); // bell
			}
			else {
				slcan->ts = str[1] - '0';
				slcan_putstr(slcan, "\r");
			}
			break;

		case 'v'://version
		case 'V'://version
			slcan_putstr(slcan, "V1220\r");
			break;

		case 'N'://serial number
			slcan_putstr(slcan, "N1234\r");
			break;
	}
}

static void slcan_com_read(struct slcan_t * slcan, struct slcan_t * loopback)
{
	uint8_t ch = 0;
	cm_disable_interrupts();
	uint8_t r = ring_read(slcan->ring_com_rx, &ch);
	cm_enable_interrupts();

	if (r) {

		if (slcan->rx_pause > 5)
			slcan->cmd_idx = 0;

		slcan->rx_pause = 0;

		slcan->cmd[slcan->cmd_idx] = ch;
		if (ch == '\r') {
			slcan_execute_command(slcan, slcan->cmd, loopback);
			slcan->cmd[slcan->cmd_idx + 1] = '\0';
			slcan->cmd_idx = 0;
		}
		else if (++slcan->cmd_idx >= sizeof(slcan->cmd))
			slcan->cmd_idx = 0;
	}
}

static void slcan_handle_can_message(slcan_t * slcan, const can_message_t *cmsg)
{
	//       cmd  id  dlc data   \r  0
	char buf[ 1  + 8 + 1 + 8*2 + 2*2 + 1 + 1];
	char * str = buf;

	if (slcan->terminal_open){

		if (!cmsg->IDE) { // standard identifier
			*str++ = cmsg->RTR ? 'r':'t';
			str = put_hex_digit(str, cmsg->id >> 8);
			str = put_hex_byte(str, cmsg->id & 0xff);
		}
		else{ // extended identifier
			*str++ = cmsg->RTR ? 'R':'T';
			str = put_hex_byte(str, (cmsg->id >> 24) & 0xff);
			str = put_hex_byte(str, (cmsg->id >> 16) & 0xff);
			str = put_hex_byte(str, (cmsg->id >>  8) & 0xff);
			str = put_hex_byte(str, (cmsg->id >>  0) & 0xff);
		}

		str = put_hex_digit(str, cmsg->dlc);

		if (!cmsg->RTR){ // no data on RTR
			for(uint8_t i = 0; i < cmsg->dlc; i++){
				str = put_hex_byte(str, cmsg->data[i]);
			}
		}

		if (slcan->ts) {
			uint16_t ts = tick.msec;
			str = put_hex_byte(str, (ts >> 8) & 0xff);
			str = put_hex_byte(str, (ts >> 0) & 0xff);
		}

		*str++ = '\r';
		*str++ = 0;

		slcan_putstr(slcan, buf);
	}
}

static void slcan_can_read(struct slcan_t * slcan)
{
	can_message_t msg;
	uint8_t r = can_msg_pop(slcan->can, &msg);
	if (r)
		slcan_handle_can_message(slcan, &msg);

	//ack tx
	r = can_echo_pop(slcan->can, &msg);
	if (r) {
		if (msg.IDE)
			slcan_putstr(slcan, "Z\r");
		else
			slcan_putstr(slcan, "z\r");
	}

	can_tx_process(slcan->can);
}

#if 0
static void slcan_print_stat(struct slcan_t * slcan0, struct slcan_t * slcan1)
{
	char buf[] = "r0:0x0000 t0:0x0000 r1:0x0000 t1:0x0000\r\n";

	uint16_t rx0 = can_get_rx_cnts(slcan0->can);
	uint16_t tx0 = can_get_tx_cnts(slcan0->can);
	uint16_t rx1 = can_get_rx_cnts(slcan1->can);
	uint16_t tx1 = can_get_tx_cnts(slcan1->can);

	put_hex_byte(buf + 5, (rx0 >>  8) & 0xff);
	put_hex_byte(buf + 7, (rx0 >>  0) & 0xff);

	put_hex_byte(buf + 15, (tx0 >>  8) & 0xff);
	put_hex_byte(buf + 17, (tx0 >>  0) & 0xff);

	put_hex_byte(buf + 25, (rx1 >>  8) & 0xff);
	put_hex_byte(buf + 27, (rx1 >>  0) & 0xff);

	put_hex_byte(buf + 35, (tx1 >>  8) & 0xff);
	put_hex_byte(buf + 37, (tx1 >>  0) & 0xff);

	slcan_putstr(slcan1, buf);
}
#endif

int main(void)
{
	cm_disable_interrupts();

	clock_setup();
	systick_setup();
	led_setup();
	systick_setup();

	iwdg_set_period_ms(1000);

	usb_setup(&slcan_cb);

	can_setup();

	slcan_t slcan0 = {
		.terminal_open = 0,
		.transmit_enabled = 0,
		.baudrate_configured = 0,
		.baud = 0,
		.ring_com_rx = get_rx_ring0(),
		.ring_com_tx = get_tx_ring0(),
		.can = can_get_hscan(),
		.cmd_idx = 0,
		.rx_pause = 0,
	};

	slcan_t slcan1 = {
		.terminal_open = 0,
		.transmit_enabled = 0,
		.baudrate_configured = 0,
		.baud = 0,
		.ring_com_rx = get_rx_ring1(),
		.ring_com_tx = get_tx_ring1(),
		.can = can_get_mscan(),
		.cmd_idx = 0,
		.rx_pause = 0,
	};

	cm_enable_interrupts();

	while(1)
	{
		iwdg_reset();

		slcan_com_read(&slcan0, &slcan1);
		slcan_com_read(&slcan1, &slcan0);

		slcan_can_read(&slcan0);
		slcan_can_read(&slcan1);

		if (tick.flag_tick) {

			tick.flag_tick = 0;

			if (tick.flag_100ms) {

				tick.flag_100ms = 0;
			}
			
			if (tick.flag_250ms) {

				tick.flag_250ms = 0;
				led4_blink();
			}

			if (tick.flag_1000ms) {

				slcan0.rx_pause++;
				slcan1.rx_pause++;

				//slcan_print_stat(&slcan0, &slcan1);

				tick.flag_1000ms = 0;
			}
		}
	}

	return 0;
}

