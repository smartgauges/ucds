#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/cortex.h>

#include "gpio.h"
#include "ring.h"
#include "led.h"
#include "can.h"

typedef struct speed_t
{
	uint32_t sjw;
	uint32_t ts1;
	uint32_t ts2;
	uint32_t brp;
} speed_t;

#if 1
/* APB1 36 MHz 87.5% sjw=1 */
static speed_t speeds[e_speed_nums] = 
{
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 225 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_15TQ, CAN_BTR_TS2_2TQ, 100 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 45 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_15TQ, CAN_BTR_TS2_2TQ, 20 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 18 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_2TQ, 9 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_15TQ, CAN_BTR_TS2_2TQ, 4 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_12TQ, CAN_BTR_TS2_2TQ, 3 },
	{ CAN_BTR_SJW_1TQ, CAN_BTR_TS1_15TQ, CAN_BTR_TS2_2TQ, 2 },
};
#else
/* APB1 36 MHz 75% sjw=2 */
static speed_t speeds[e_speed_nums] = 
{
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ, 180 },
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ, 90 },
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ, 36 },
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_14TQ, CAN_BTR_TS2_5TQ, 18 },
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_4TQ, 16 },
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_4TQ, 8 },
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_4TQ, 4 },
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_10TQ, CAN_BTR_TS2_4TQ, 3 },
	{ CAN_BTR_SJW_2TQ, CAN_BTR_TS1_13TQ, CAN_BTR_TS2_4TQ, 2 },
};
#endif

#define MSGS_SIZE 200
//can0
static struct ring_t can_tx_ring0;
static uint8_t can_tx_ring_buffer0[sizeof(struct can_message_t)  * MSGS_SIZE];
static struct ring_t can_rx_ring0;
static uint8_t can_rx_ring_buffer0[sizeof(struct can_message_t)  * MSGS_SIZE];
static struct ring_t can_echo_ring0;
static uint8_t can_echo_ring_buffer0[sizeof(struct can_message_t)  * MSGS_SIZE];

//can1
static struct ring_t can_tx_ring1;
static uint8_t can_tx_ring_buffer1[sizeof(struct can_message_t)  * MSGS_SIZE];
static struct ring_t can_rx_ring1;
static uint8_t can_rx_ring_buffer1[sizeof(struct can_message_t)  * MSGS_SIZE];
static struct ring_t can_echo_ring1;
static uint8_t can_echo_ring_buffer1[sizeof(struct can_message_t)  * MSGS_SIZE];

typedef struct can_t
{
	uint32_t rcc;
	uint32_t baddr;
	uint8_t fid;

	uint32_t irq_tx;
	uint32_t irq_rx;

	struct gpio_t tx;
	struct gpio_t rx;
	struct gpio_t stb;

	volatile uint32_t tx_cnts;
	volatile uint32_t rx_cnts;
	ring_t *ring_rx;
	ring_t *ring_tx;
	ring_t *ring_echo;
} can_t;

static struct can_t can1 =
{
	.rcc = RCC_CAN1,
	.baddr = CAN1,
	.irq_rx = NVIC_USB_LP_CAN_RX0_IRQ,
	.irq_tx = NVIC_USB_HP_CAN_TX_IRQ,
	.fid = 0,
	.tx = GPIO_INIT(B, 9),
	.rx = GPIO_INIT(B, 8),
	.stb = GPIO_INIT(B, 3),

	.tx_cnts = 0,
	.rx_cnts = 0,
	.ring_rx = &can_rx_ring0,
	.ring_tx = &can_tx_ring0,
	.ring_echo = &can_echo_ring0,
};

static struct can_t can2 =
{
	.rcc = RCC_CAN2,
	.baddr = CAN2,
	.irq_rx = NVIC_CAN2_RX0_IRQ,
	.irq_tx = NVIC_CAN2_TX_IRQ,
	.fid = 14,
	.tx = GPIO_INIT(B, 13),
	.rx = GPIO_INIT(B, 12),
	.stb = GPIO_INIT(B, 7),

	.tx_cnts = 0,
	.rx_cnts = 0,
	.ring_rx = &can_rx_ring1,
	.ring_tx = &can_tx_ring1,
	.ring_echo = &can_echo_ring1,
};

static struct can_t can3 =
{
	.rcc = RCC_CAN2,
	.baddr = CAN2,
	.irq_rx = NVIC_CAN2_RX0_IRQ,
	.irq_tx = NVIC_CAN2_TX_IRQ,
	.fid = 14,
	.tx = GPIO_INIT(B, 6),
	.rx = GPIO_INIT(B, 5),
	.stb = GPIO_INIT(B, 4),

	.tx_cnts = 0,
	.rx_cnts = 0,
	.ring_rx = &can_rx_ring1,
	.ring_tx = &can_tx_ring1,
	.ring_echo = &can_echo_ring1,
};

struct can_t * can_get_mscan(void)
{
	return &can1;
}

struct can_t * can_get_hscan(void)
{
	return &can2;
}

static struct can_t * can_get_can3(void)
{
	return &can3;
}

bool can_set_bittiming(struct can_t * can, uint32_t brp, uint32_t phase_seg1, uint32_t phase_seg2, uint32_t sjw)
{
	nvic_disable_irq(can->irq_rx);
	nvic_disable_irq(can->irq_tx);

	uint32_t ier = CAN_IER(can->baddr) & (CAN_IER_FMPIE0/* | CAN_IER_TMEIE*/);

	/* Reset CAN. */
	can_reset(can->baddr);

	/* CAN cell init. apb1 36 MHZ */
	int ret = can_init(can->baddr,
		     false,           /* TTCM: Time triggered comm mode? */
		     true,            /* ABOM: Automatic bus-off management? */
		     false,           /* AWUM: Automatic wakeup mode? */
		     false,            /* NART: No automatic retransmission? */
		     false,           /* RFLM: Receive FIFO locked mode? */
		     true,           /* TXFP: Transmit FIFO priority? */
		     sjw,
		     phase_seg1,
		     phase_seg2,
		     brp,
		     false,
		     false
		     );

	if (ret)
		return false;

	/* CAN filter 0 init. */
	can_filter_id_mask_32bit_init(
				can->fid,     /* Filter ID */
				0,     /* CAN ID */
				0,     /* CAN ID mask */
				0,     /* FIFO assignment (here: FIFO0) */
				true); /* Enable the filter. */

	/* CAN2 uses the same filter bank as CAN1 */
	if (can == can_get_mscan()) {

		/* CAN filter 0 init. */
		can_filter_id_mask_32bit_init(
				can_get_hscan()->fid,     /* Filter ID */
				0,     /* CAN ID */
				0,     /* CAN ID mask */
				0,     /* FIFO assignment (here: FIFO0) */
				true); /* Enable the filter. */
	}

	can_enable_irq(can->baddr, ier | CAN_IER_FMPIE0);

	/* NVIC setup. */
	nvic_set_priority(can->irq_rx, 1);
	nvic_set_priority(can->irq_tx, 1);
	nvic_enable_irq(can->irq_tx);
	nvic_enable_irq(can->irq_rx);

	return true;
}

bool can_set_speed(struct can_t * can, e_speed_t speed)
{
	if (speed >= e_speed_nums)
		return false;

	return can_set_bittiming(can, speeds[speed].brp, speeds[speed].ts1, speeds[speed].ts2, speeds[speed].sjw);
}

void can_stop(struct can_t * can)
{
	nvic_disable_irq(can->irq_rx);
	nvic_disable_irq(can->irq_tx);
	ring_reset(can->ring_rx);
	ring_reset(can->ring_tx);
	ring_reset(can->ring_echo);
}

bool can_start(struct can_t * can, e_speed_t speed)
{
	return can_set_speed(can, speed);
}

static void can_enable(struct can_t * can)
{
	/* Enable peripheral clocks. */
	rcc_periph_clock_enable(can->rcc);
	rcc_periph_clock_enable(can->rx.rcc);
	rcc_periph_clock_enable(can->tx.rcc);
	rcc_periph_clock_enable(can->stb.rcc);

	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(can->rx.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, can->rx.pin);
	gpio_set(can->rx.port, can->rx.pin);

	/* Configure CAN pin: TX. */
	gpio_set_mode(can->tx.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, can->tx.pin);

	gpio_clear(can->stb.port, can->stb.pin);
	gpio_set_mode(can->stb.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, can->stb.pin);
}

void can_setup(void)
{
	ring_init(&can_rx_ring0, can_rx_ring_buffer0, sizeof(can_message_t), MSGS_SIZE);
	ring_init(&can_tx_ring0, can_tx_ring_buffer0, sizeof(can_message_t), MSGS_SIZE);
	ring_init(&can_echo_ring0, can_echo_ring_buffer0, sizeof(can_message_t), MSGS_SIZE);

	ring_init(&can_rx_ring1, can_rx_ring_buffer1, sizeof(can_message_t), MSGS_SIZE);
	ring_init(&can_tx_ring1, can_tx_ring_buffer1, sizeof(can_message_t), MSGS_SIZE);
	ring_init(&can_echo_ring1, can_echo_ring_buffer1, sizeof(can_message_t), MSGS_SIZE);

	/* Enable AFIO clock. */
	rcc_periph_clock_enable(RCC_AFIO);
	// turn off SWJ/JTAG
	AFIO_MAPR = AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;
	//REMAP CAN1
	AFIO_MAPR |= AFIO_MAPR_CAN1_REMAP_PORTB;

	//AFIO_MAPR |= AFIO_MAPR_CAN2_REMAP;

	can_enable(can_get_mscan());
	can_enable(can_get_hscan());

	struct can_t *can = can_get_can3();
	gpio_set(can->stb.port, can->stb.pin);
	gpio_set_mode(can->stb.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, can->stb.pin);
}	

void can_disable(struct can_t * can)
{
	nvic_disable_irq(can->irq_rx);
	nvic_disable_irq(can->irq_tx);

	rcc_periph_clock_enable(can->rcc);
	can_reset(can->baddr);

	rcc_periph_clock_enable(can->stb.rcc);
	gpio_set(can->stb.port, can->stb.pin);

	rcc_periph_clock_enable(can->rx.rcc);
	gpio_set_mode(can->rx.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, can->rx.pin);

	rcc_periph_clock_enable(can->tx.rcc);
	gpio_set_mode(can->tx.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, can->tx.pin);
}

uint32_t can_isr_cnt = 0;

static void can_isr(struct can_t * can)
{
	can_isr_cnt++;

	led4_blink();

#if 0
	//tx enabled
	if (CAN_IER(can->baddr) & CAN_IER_TMEIE) {

		can->tx_cnts++;
		bool r = false;
		if (can_available_mailbox(can->baddr)) {

			struct can_message_t msg;
			r = ring_read(can->ring_tx, &msg);
			if (r) {

				bool rtr = msg.RTR;
				bool ext = msg.IDE;
				can_transmit(can->baddr, msg.id, ext, rtr, msg.dlc, msg.data);
			}
		}

		//tx fifo empty
		if (!r)
			can_disable_irq(can->baddr, CAN_IER_TMEIE);
	}
#endif

	//rx enabled and not empty
	if ((CAN_IER(can->baddr) & CAN_IER_FMPIE0) && (CAN_RF0R(can->baddr) & CAN_RF0R_FMP0_MASK)) {

		uint8_t fmi;
		struct can_message_t msg;
		uint32_t id = 0;
		uint16_t timestamp = 0;
		uint8_t dlc;
		bool rtr = 0, ext = 0;

		can->rx_cnts++;

		can_receive(can->baddr, 0, false, &id, &ext, &rtr, &fmi, &dlc, msg.data, &timestamp);
		can_fifo_release(can->baddr, 0);

		msg.id = id;
		msg.echo_id = 0xffffffff;
		msg.dlc = dlc;
		msg.RTR = rtr;
		msg.IDE = ext;

		ring_write(can->ring_rx, &msg);
	}
}

void usb_hp_can_tx_isr(void)
{
	can_isr(can_get_mscan());
}

void usb_lp_can_rx0_isr(void)
{
	can_isr(can_get_mscan());
}

void can2_tx_isr(void)
{
	can_isr(can_get_hscan());
}

void can2_rx0_isr(void)
{
	can_isr(can_get_hscan());
}

void can_tx_process(struct can_t *can)
{
	//send first msg in fifo
	if (/*!(CAN_IER(can->baddr) & CAN_IER_TMEIE) &&*/ can_available_mailbox(can->baddr)) {

		can_message_t msg;
		cm_disable_interrupts();
		uint8_t r = ring_read(can->ring_tx, &msg);
		cm_enable_interrupts();

		if (r) {
#if 0
			if (!can_available_mailbox(can->baddr)) {

				CAN_TSR(can->baddr) |= CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
			}
#endif
			bool rtr = msg.RTR;
			bool ext = msg.IDE;
			can_transmit(can->baddr, msg.id, ext, rtr, msg.dlc, msg.data);
			//can_enable_irq(can->baddr, CAN_IER_TMEIE);
			can->tx_cnts++;
			//tx ack
			ring_write(can->ring_echo, &msg);
		}
	}
}

void can_msg_push(struct can_t *can, can_message_t * msg)
{
	ring_write(can->ring_tx, msg);
	can_tx_process(can);
}

bool can_msg_pop(struct can_t *can, can_message_t * msg)
{
	uint8_t r = ring_read(can->ring_rx, msg);
	return r ? true : false;
}

bool can_echo_pop(struct can_t *can, can_message_t * msg)
{
	uint8_t r = ring_read(can->ring_echo, msg);
	return r ? true : false;
}

uint32_t can_get_rx_cnts(struct can_t *can)
{
	return can->rx_cnts;
}

uint32_t can_get_tx_cnts(struct can_t *can)
{
	return can->tx_cnts;
}

