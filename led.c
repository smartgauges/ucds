#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "gpio.h"
#include "tick.h"
#include "led.h"

struct gpio_t led2 = GPIO_INIT(B, 10);
struct gpio_t led3 = GPIO_INIT(B, 11);
struct gpio_t led4 = GPIO_INIT(B, 2);

static uint16_t led4_blink_div = 0;
void led4_on(void)
{
	gpio_clear(led4.port, led4.pin);
}

void led4_off(void)
{
	gpio_set(led4.port, led4.pin);
	led4_blink_div = 0;
}

void led4_blink(void)
{
	gpio_clear(led4.port, led4.pin);
	led4_blink_div = 1;
}

static uint16_t led3_blink_div = 0;
void led3_on(void)
{
	gpio_clear(led3.port, led3.pin);
}

void led3_off(void)
{
	gpio_set(led3.port, led3.pin);
	led3_blink_div = 0;
}

void led3_blink(void)
{
	gpio_clear(led3.port, led3.pin);
	led3_blink_div = 1;
}

static uint16_t led2_blink_div = 0;
void led2_on(void)
{
	gpio_clear(led2.port, led2.pin);
}

void led2_off(void)
{
	gpio_set(led2.port, led2.pin);
	led2_blink_div = 0;
}

void led2_blink(void)
{
	gpio_clear(led2.port, led2.pin);
	led2_blink_div = 1;
}

void led_setup(void)
{
	rcc_periph_clock_enable(led2.rcc);

	gpio_set_mode(led2.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, led2.pin);
	gpio_set_mode(led3.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, led3.pin);
	gpio_set_mode(led4.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, led4.pin);

	/* Switch off LED. */
	led2_off();
	led3_off();
	led4_off();
}

static uint8_t led4_is_blink(void)
{
	return led4_blink_div;
}

static uint8_t led3_is_blink(void)
{
	return led3_blink_div;
}

static uint8_t led2_is_blink(void)
{
	return led2_blink_div;
}

#define TIME_ON 40
void led_tick(void)
{
	if (led4_is_blink()) {

		if (led4_blink_div++ >= MSEC_TO_TICK(TIME_ON)) {

			led4_off();
		}
	}

	if (led3_is_blink()) {

		if (led3_blink_div++ >= MSEC_TO_TICK(TIME_ON)) {

			led3_off();
		}
	}

	if (led2_is_blink()) {

		if (led2_blink_div++ >= MSEC_TO_TICK(TIME_ON)) {

			led2_off();
		}
	}
}

