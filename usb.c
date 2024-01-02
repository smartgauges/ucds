#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/common.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include "usb_private.h"

#include <string.h>
#include <stdio.h>

#include "usb.h"
#include "timer.h"

static volatile bool usbd_is_enabled = false;
static volatile bool usbd_is_configured = false;

static usbd_device * g_usbd_dev = 0;
static usb_cb_t * usb_cb = 0;

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[USB_CB_SIZE];

static void iface_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	if (usb_cb->iface_set_config)
		usb_cb->iface_set_config(usbd_dev, wValue);

	usbd_is_configured = true;
}

static void altsetting_set_config(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;
	(void)wIndex;
	(void)wValue;

	if (usb_cb->alt_set_config)
		usb_cb->alt_set_config(usbd_dev, wIndex, wValue);
}

static void usb_sof_cb(void)
{
	if (usb_cb->sof)
		usb_cb->sof();
}

static void usb_eopf_cb(void)
{
	if (usb_cb->eopf)
		usb_cb->eopf(g_usbd_dev);
}

static void usb_reset_cb(void)
{
	if (usb_cb->reset)
		usb_cb->reset();

	usbd_is_configured = false;
}

void usb_setup(usb_cb_t * cb)
{
	usb_cb = cb;

	nvic_disable_irq(NVIC_OTG_FS_IRQ);
	//rcc_periph_clock_disable(RCC_OTGFS);
	rcc_periph_reset_pulse(RST_OTGFS);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_OTGFS);

	if (usb_cb->preinit)
		usb_cb->preinit();

	g_usbd_dev = usbd_init(&stm32f107_usb_driver, usb_cb->desc, usb_cb->config, usb_cb->strings, usb_cb->num_strings, usbd_control_buffer, sizeof(usbd_control_buffer));

	if (usb_cb->init)
		usb_cb->init(g_usbd_dev);

	usbd_register_set_config_callback(g_usbd_dev, iface_set_config);
	usbd_register_set_altsetting_callback(g_usbd_dev, altsetting_set_config);
	usbd_register_reset_callback(g_usbd_dev, usb_reset_cb);
	usbd_register_sof_callback(g_usbd_dev, usb_sof_cb);
	usbd_register_eopf_callback(g_usbd_dev, usb_eopf_cb);

	g_usbd_dev->driver->ep_reset(g_usbd_dev);

	usbd_is_enabled = true;

	usbd_disconnect(g_usbd_dev, true);
	mdelay(10);
	usbd_disconnect(g_usbd_dev, false);

	//second prio
	nvic_set_priority(NVIC_OTG_FS_IRQ, 0x10);
	nvic_enable_irq(NVIC_OTG_FS_IRQ);
}

void usb_disable(void)
{
	//nvic_disable_irq(NVIC_OTG_FS_IRQ);

	usbd_is_enabled = false;
	usbd_is_configured = false;

	if (usb_cb->disable)
		usb_cb->disable();

	rcc_periph_clock_enable(RCC_OTGFS);
	if (g_usbd_dev)
		usbd_disconnect(g_usbd_dev, true);
#if 0
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO9);
#endif
	rcc_periph_clock_disable(RCC_OTGFS);

	g_usbd_dev = 0;
}

static bool usb_is_enabled(void)
{
	return usbd_is_enabled;
}

#if 0
static bool usb_is_configured(void)
{
	return usbd_is_configured;
}
#endif

void usb_process(void)
{
	if (!usb_is_enabled())
		return;

	if (!g_usbd_dev)
		return;

	usbd_poll(g_usbd_dev);

	usb_cb->process(g_usbd_dev);
}

void otg_fs_isr(void)
{
	usb_process();
}

