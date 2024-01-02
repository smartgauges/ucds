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

#include "led.h"
#include "tick.h"

#define LED_PORT GPIOB
#define LED4_PIN GPIO2
#define LED3_PIN GPIO11
#define LED2_PIN GPIO10

static uint16_t led4_blink_div = 0;
void led4_on(void)
{
	gpio_clear(LED_PORT, LED4_PIN);
}

void led4_off(void)
{
	gpio_set(LED_PORT, LED4_PIN);
	led4_blink_div = 0;
}

void led4_blink(void)
{
	gpio_clear(LED_PORT, LED4_PIN);
	led4_blink_div = 1;
}

static uint16_t led3_blink_div = 0;
void led3_on(void)
{
	gpio_clear(LED_PORT, LED3_PIN);
	led3_blink_div = 1;
}

void led3_off(void)
{
	gpio_set(LED_PORT, LED3_PIN);
	led3_blink_div = 0;
}

void led3_blink(void)
{
	gpio_clear(LED_PORT, LED3_PIN);
	led3_blink_div = 1;
}

static uint16_t led2_blink_div = 0;
void led2_on(void)
{
	gpio_clear(LED_PORT, LED2_PIN);
	led2_blink_div = 1;
}

void led2_off(void)
{
	gpio_set(LED_PORT, LED2_PIN);
	led2_blink_div = 0;
}

void led2_blink(void)
{
	gpio_set(LED_PORT, LED2_PIN);
	led2_blink_div = 0;
}

void led_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, LED4_PIN);
	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, LED3_PIN);
	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, LED2_PIN);

	/* Switch off LED. */
	led4_off();
	led3_off();
	led2_off();
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

