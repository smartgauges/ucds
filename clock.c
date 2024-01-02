#include <libopencm3/stm32/rcc.h>

#include "clock.h"

static const struct rcc_clock_scale rcc_hse8_72mhz =
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

void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse8_72mhz);
}

