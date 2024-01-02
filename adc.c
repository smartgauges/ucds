#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#include "gpio.h"
#include "adc.h"
#include "timer.h"

struct gpio_t adc = GPIO_INIT(C, 10);

void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(adc.rcc);

	gpio_set_mode(adc.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, adc.pin);

	/* Make sure the ADC doesn't run during config. */
	adc_power_off(ADC1);

	/* We configure everything for one single conversion. */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_disable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	mdelay(1);

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

uint16_t adc_read(void)
{
	uint8_t channel_array[16];
	uint32_t value = 0;

	/* Select the channel 10 */
	channel_array[0] = 10;
	adc_set_regular_sequence(ADC1, 1, channel_array);

	uint32_t num = 8;
	for (uint32_t i = 0; i < num; i++) {
		/*
		 * Start the conversion directly (not trigger mode).
		 */
		adc_start_conversion_direct(ADC1);

		/* Wait for end of conversion. */
		while(!adc_eoc(ADC1))
			;

		value += adc_read_regular(ADC1);
		udelay(100);
	}

	return value/num;
}

