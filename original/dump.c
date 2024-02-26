#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/iwdg.h>

#include <stdint.h>

void readChar( uint8_t const chr );
void writeByte( uint8_t b );
void writeStr( uint8_t const * const str );
void writeChar( uint8_t const chr );
void writeWordLe( uint32_t const word );
void writeWordBe( uint32_t const word );
void readCmd( uint8_t const * const cmd);
uint32_t hexToInt(uint8_t const * const hex);
void readMem(uint32_t const addr, uint32_t const len);
void writeMem(uint32_t const addr, uint32_t const data);

uint8_t const strHelp[] = 	"Help\r\n-----------\r\n"
				"ADDR, VAL, LEN: 32-bit Hex encoded:\r\n e.g., 0A1337FF\r\n"
				"-----------\r\n"
				"R ADDR LEN - Read 32-bit word\r\n"
				"W ADDR VAL - Write 32-bit word\r\n"
				"D          - Dump all flash memory\r\n"
				"S          - Reboot\r\n"
				"E          - Exit\r\n"
				"H          - Show help \r\n"
				"---------------\r\n";

struct gpio_t
{
	uint32_t rcc;
	uint32_t port;
	uint32_t pin;
};
#define GPIO_INIT(PORT,PIN) { RCC_GPIO##PORT, GPIO##PORT, GPIO##PIN }

struct usart_t
{
	uint32_t baddr;
	uint32_t rcc;
	struct gpio_t tx;
	struct gpio_t rx;
	uint16_t baudrate;
};

static struct usart_t usart2 = 
{
	.baddr = USART2,
	.rcc = RCC_USART2,
	.tx = GPIO_INIT(A, 2),
	.rx = GPIO_INIT(A, 3),
	.baudrate = 0,
};

void usart_setup(struct usart_t * usart, uint32_t speed)
{
	usart_disable(usart->baddr);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(usart->rcc);
	rcc_periph_clock_enable(usart->tx.rcc);
	rcc_periph_clock_enable(usart->rx.rcc);

	gpio_set_mode(usart->tx.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, usart->tx.pin);
	gpio_set_mode(usart->rx.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, usart->rx.pin);

	/* Setup UART parameters. */
	usart_set_baudrate(usart->baddr, speed);
	usart_set_databits(usart->baddr, 8);
	usart_set_stopbits(usart->baddr, USART_STOPBITS_1);
	usart_set_parity(usart->baddr, USART_PARITY_NONE);
	usart_set_flow_control(usart->baddr, USART_FLOWCONTROL_NONE);
	usart_set_mode(usart->baddr, USART_MODE_TX_RX);

	/* Finally enable the USART. */
	usart_enable(usart->baddr);
}

const struct rcc_clock_scale rcc_hsi_8mhz =
{
	/* hsi 8Mhz */
	.pll_source = RCC_CFGR_PLLSRC_HSI_CLK_DIV2,
	.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL2,
	.prediv1 = RCC_CFGR2_PREDIV_NODIV,
	.hpre = RCC_CFGR_HPRE_NODIV,
	.ppre1 = RCC_CFGR_PPRE_NODIV,
	.ppre2 = RCC_CFGR_PPRE_NODIV,
	.adcpre = RCC_CFGR_ADCPRE_DIV2,
	.flash_waitstates = 0,
	.ahb_frequency	= 8e6,
	.apb1_frequency = 8e6,
	.apb2_frequency = 8e6,
};

struct gpio_t led2 = GPIO_INIT(B, 10);

void led2_on(void)
{
	gpio_clear(led2.port, led2.pin);
}

void led2_off(void)
{
	gpio_set(led2.port, led2.pin);
}

void led_setup(void)
{
	rcc_periph_clock_enable(led2.rcc);

	gpio_set_mode(led2.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, led2.pin);

	led2_off();
}

int main(void)
{
	cm_disable_interrupts();

	rcc_clock_setup_pll(&rcc_hsi_8mhz);

	struct gpio_t pb2 = GPIO_INIT(B, 2);
	rcc_periph_clock_enable(pb2.rcc);
	gpio_set_mode(pb2.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pb2.pin);
	uint16_t s = gpio_get(pb2.port, pb2.pin);

	led_setup();
	led2_on();

	iwdg_set_period_ms(1000);
	iwdg_start();

	usart_setup(&usart2, 38400);

	uint32_t sn[3] = { 0, 0, 0 };
	desig_get_unique_id(sn);

	writeStr("\r\n##################################\r\n"
		 "# Low-Level Shell v0.1 alpha     #\r\n"
		 "# CPU-ID: 0x");
	uint32_t cpuid = SCB_CPUID;
	writeWordBe(cpuid);
	writeStr(                    "             #\r\n");
	writeStr("# id: 0x");
	writeWordBe(sn[0]);
	writeWordBe(sn[1]);
	writeWordBe(sn[2]);
	writeStr(                 " #\r\n");
	writeStr("##################################\r\n\r\n");
	writeStr("> ");

	while (1)
	{
		if (((USART_SR(usart2.baddr) & USART_SR_TXE) == 0) && (0))
			usart_send(usart2.baddr, 0x33u);

		if ((USART_SR(usart2.baddr) & USART_SR_RXNE))
		{
			uint16_t data = usart_recv(usart2.baddr);
			readChar((uint8_t) data);
		}

		iwdg_reset();
	}
}

/* hex must have length 8 */
uint32_t hexToInt(uint8_t const * const hex)
{
	uint32_t ind = 0u;
	uint32_t res = 0u;

	for (ind = 0; ind < 8; ++ind)
	{
		uint8_t chr = hex[ind];
		uint32_t val = 0u;

		res <<= 4u;

		if ((chr >= '0') && (chr <= '9'))
		{
			val = chr - '0';
		}
		else if ((chr >= 'a') && (chr <= 'f'))
		{
			val = chr - 'a' + 0x0a;
		}
		else if ((chr >= 'A') && (chr <= 'F'))
		{
			val = chr - 'A' + 0x0a;
		}
		else
		{
			val = 0u;
		}
		res |= val;
	}

	return res;
}

void readChar( uint8_t const chr )
{
	#define CMDBUF_LEN (64u)
	static uint8_t cmdbuf[CMDBUF_LEN] = {0u};
	static uint32_t cmdInd = 0u;

	switch (chr)
	{
		case '\n':
		case '\r':
			cmdbuf[cmdInd] = 0u;
			if (cmdInd != 0)
			{
				writeStr("\r\n");
			}
			readCmd(cmdbuf);
			cmdInd = 0u;
			writeStr("\r\n> ");
			{
				uint32_t ind = 0u;
				for (ind = 0; ind<CMDBUF_LEN; ++ind)
				{
					cmdbuf[ind]=0x00u;
				}
			}
		break;

		case 8:
		case 255:
		case 127: /* TODO backspace */
			if (cmdInd > 0u)
				--cmdInd;
			writeChar(chr);
		break;

		default:
			if (cmdInd < (CMDBUF_LEN - 1))
			{
				cmdbuf[cmdInd] = chr;
				++cmdInd;
				writeChar(chr);
			}
		break;
	}
}

void readCmd( uint8_t const * const cmd )
{
	switch (cmd[0])
	{
		case 0:
			return;
			break;

		/* read 32-bit command */
		case 'r':
		case 'R':
			/* r 08000000 00000100 */
			readMem(hexToInt(&cmd[2]), hexToInt(&cmd[11]));
			break;

		/* write 32-bit command */
		case 'w':
		case 'W':
			/* w 20000000 12345678 */
			writeMem(hexToInt(&cmd[2]), hexToInt(&cmd[11]));
			break;

		/* Dump all flash */
		case 'd':
		case 'D':
			writeStr("\r\n\r\n");
			{
				uint32_t const * addr = (uint32_t*) 0x08000000;
				uint32_t br = 8u;
				while (((uintptr_t) addr) < (0x08000000 + 256u * 1024u))
				{
					if (br == 8u)
					{
						writeStr("\r\n[");
						writeWordBe((uint32_t) addr);
						writeStr("]: ");
						br = 0u;
					}

					writeWordBe(*addr);
					writeChar(' ');
					++addr;
					++br;
					iwdg_reset();
				}
			}
			writeStr("\r\n\r\n");
			break;

		/* Help command */
		case 'h':
		case 'H':
			writeStr(strHelp);
			break;

		/* Reboot */
		case 's':
		case 'S':
			writeStr("Rebooting...\r\n\r\n");
			SCB_AIRCR = 0x05FA0004u;
			break;

		/* exit */
		case 'e':
		case 'E':
			writeStr("Bye.\r\n");
			while (1)
			{
				__asm__ volatile("wfi");
			}
			break;


		default:
			writeStr("Unknown command: ");
			writeStr(cmd);
			writeStr("\r\n");
			break;
	}
}
	
const uint8_t txtMap[] = "0123456789ABCDEF";

void writeByte( uint8_t b )
{
	writeChar(txtMap[b >> 4]);
	writeChar(txtMap[b & 0x0F]);
}

void writeStr( uint8_t const * const str )
{
	uint32_t ind = 0u;

	while (str[ind])
	{
		writeChar(str[ind]);
		++ind;
	}
}

void writeChar( uint8_t const chr )
{
	usart_send_blocking(usart2.baddr, chr);
}

void writeWordLe( uint32_t const word )
{
	writeByte((word & 0x000000FF));
	writeByte((word & 0x0000FF00) >> 8);
	writeByte((word & 0x00FF0000) >> 16);
	writeByte((word & 0xFF000000) >> 24);
}

void writeWordBe( uint32_t const word )
{
	writeByte((word & 0xFF000000) >> 24);
	writeByte((word & 0x00FF0000) >> 16);
	writeByte((word & 0x0000FF00) >> 8);
	writeByte((word & 0x000000FF));

}

void alertCrash( uint32_t crashId )
{
	writeStr("!!! EXCEPTION !!!\r\nID: ");
	writeByte(crashId);
	writeStr("\r\nRestart required!\r\n\r\n");
	SCB_AIRCR = 0x05FA0004u;
	while (1);
}

void readMem(uint32_t const addr, uint32_t const len)
{
	uint32_t it = 0u;
	uint32_t addrx = 0u;
	uint32_t lenx = 0u;

	lenx = len;
	if (lenx == 0u)
	{
		lenx = 4u;
	}

	for (it = 0u; it < (lenx / 4u); ++it)
	{
		addrx = addr + it*4u;
		writeStr("Read [");
		writeWordBe(addrx);
		writeStr("]: ");
		writeWordBe(*((uint32_t*)addrx));
		writeStr("\r\n");
	}
}

void writeMem(uint32_t const addr, uint32_t const data)
{
	writeStr("Write [");
	writeWordBe(addr);
	writeStr("]: ");
	writeWordBe(data);
	*((uint32_t*) addr) = data;
	writeStr("\r\n");
}
