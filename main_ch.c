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
#include "usb_ch.h"
#include "timer.h"
#include "adc.h"

typedef struct
{
	uint8_t command;
	uint8_t sequence;
	uint8_t flags;
	uint8_t dSize;
} __attribute__((packed)) CommandHeader;

typedef struct
{
	uint8_t command;
	uint8_t sequence;
	uint16_t flags;
	uint16_t dSize;
} __attribute__((packed)) MsgCommandHeader;

enum FLAG_CHANNELS
{
	FLAG_CHANNEL_1 = 0x20,
	FLAG_CHANNEL_2 = 0x40,
	FLAG_CHANNEL_3 = 0x60,
	FLAG_CHANNEL_4 = 0x80,
	FLAG_CHANNEL_5 = 0xA0,
	FLAG_CHANNEL_6 = 0xC0,
	FLAG_CHANNEL_7 = 0xE0
};

enum HWIdentifiers
{
	/* Old identifier for CAN-Hacker on F105 mcu with dual CAN channels and single LIN channel */
	HW_CH30 = 0xFF,
	/* Old identifier for CAN-Hacker in ODB interface with single CAN channel and single LIN channel */
	HW_ODB_OLD = 0x02,
	/* CAN-Hacker 3.2 on F105 mcu with dual CAN channels and single LIN channel */
	HW_CH32 = 0x01,
	/* CAN-Hacker in ODB interface on F105 mcu with single CAN channel and single LIN channel */
	HW_ODB = 0x04,
	/* CAN-Hacker CH-P on F105 mcu with dual CAN channels and single LIN channel */
	HW_CHP = 0x03,
	/* CAN-Hacker 3.3 on F407 mcu with dual CAN channels and single LIN channel */
	HW_CH33 = 0x11,
	/* CAN-Hacker CH-P on F407 mcu with dual CAN channels and single LIN channel */
	HW_CHPM03 = 0x13,
	/* CAN-Hacker in ODB interface on G431 mcu with single CAN channel and single LIN channel */
	HW_ODB_FD = 0x14,
	/* CAN-Hacker CH-P on G473 mcu with dual CAN channels and single LIN channel */
	HW_FDL2 = 0x06
};

typedef struct
{
	CommandHeader header;
	uint8_t hwId;
} __attribute__((packed)) DeviceIdentifier;

typedef struct
{
	CommandHeader header;
	uint8_t devInfo[6];
} __attribute__((packed)) DeviceInfo;

typedef struct
{
	CommandHeader header;
	uint8_t devFw[7];
} __attribute__((packed)) DeviceFw;

typedef struct
{
	CommandHeader header;
	uint8_t serial[8];
} __attribute__((packed)) DeviceSerial;

enum FLAG_DEVICE_MODES
{
	FLAG_DEVICE_MODE_FULL = 0x00,
	FLAG_DEVICE_MODE_CAN = 0x01,
	FLAG_DEVICE_MODE_LIN = 0x02
};

#define COMMAND_SYNC 0xA5
#define COMMAND_DEVICE_HARDWARE 0x05
#define COMMAND_DEVICE_INFO 0x01
#define COMMAND_DEVICE_FIRMWARE 0x02
#define COMMAND_DEVICE_SERIAL 0x03
#define COMMAND_DEVICE_MODE 0x04
#define COMMAND_DEVICE_OPEN 0x08
#define COMMAND_DEVICE_CLOSE 0x09
#define COMMAND_DEVICE_STAT 0x0A

#define COMMAND_ACK 0x80

enum FLAG_CHANNEL_CONFIGS
{
	FLAG_CONFIG_BUS_SPEED = 0x00,
	FLAG_CONFIG_BUS_SPEED_FD = 0x01,
	FLAG_CONFIG_BUS_SPEED_M = 0x02,
	FLAG_CONFIG_BUS_SPEED_FD_M = 0x03,
	FLAG_CONFIG_TERMINATOR = 0x05,
	FLAG_CONFIG_PULL_UP = 0x06,
	FLAG_CONFIG_CRC_MODE = 0x07,
	FLAG_CONFIG_IDLE_DELAY = 0x08,
	FLAG_CONFIG_MODE = 0x09,
	FLAG_CONFIG_CAN_FRAME = 0x0A
};

enum canBitrate
{
	CAN_BITRATE_10K = 0,
	CAN_BITRATE_20K,
	CAN_BITRATE_33_3K,
	CAN_BITRATE_50K,
	CAN_BITRATE_62_5K,
	CAN_BITRATE_83_3K,
	CAN_BITRATE_95K,
	CAN_BITRATE_100K,
	CAN_BITRATE_125K,
	CAN_BITRATE_250K,
	CAN_BITRATE_400K,
	CAN_BITRATE_500K,
	CAN_BITRATE_800K,
	CAN_BITRATE_1000K
};

typedef struct
{
	CommandHeader header;
	uint8_t speed;
} __attribute__((packed)) ChannelConfigureSpeed;

enum CHANNEL_MODES
{
	MODE_NORMAL = 0x00,
	MODE_LISTEN = 0x01,
	MODE_LOOPBACK = 0x02
};

typedef struct
{
	uint16_t Prescaler;
	uint16_t tqSeg1;
	uint16_t tqSeg2;
	uint16_t SyncJW;
} __attribute__((packed)) BusCustomBaudRate;

typedef struct
{
	CommandHeader header;
	BusCustomBaudRate speed;
} __attribute__((packed)) ChannelConfigureCustomSpeed;


#define COMMAND_CHANNEL_CONFIG 0x11
#define COMMAND_CHANNEL_OPEN 0x18
#define COMMAND_CHANNEL_CLOSE 0x19
#define COMMAND_CHANNEL_RESET 0x1F

enum FLAG_MESSAGE_CHANNELS
{
	FLAG_MESSAGE_CHANNEL_1 = 0x2000,
	FLAG_MESSAGE_CHANNEL_2 = 0x4000,
	FLAG_MESSAGE_CHANNEL_3 = 0x6000,
	FLAG_MESSAGE_CHANNEL_4 = 0x8000,
	FLAG_MESSAGE_CHANNEL_5 = 0xA000,
	FLAG_MESSAGE_CHANNEL_6 = 0xC000,
	FLAG_MESSAGE_CHANNEL_7 = 0xE000
};

#define FLAG_MESSAGE_CONFIRM_REQUIRED 0x0001U

// 29-bit message identifier
#define FLAG_MESSAGE_EXTID 0x00000001U
// Remote frame
#define FLAG_MESSAGE_RTR   0x00000002U
// CAN-FD frame
#define FLAG_MESSAGE_FDF   0x00000004U
// CAN-FD bit rate switch
#define FLAG_MESSAGE_BRS   0x00000008U
// CAN-FD Error status indicator
#define FLAG_MESSAGE_ESI   0x00000010U
// LIN Master request
#define FLAG_MESSAGE_MR    0x00000100U
// LIN Slave response
#define FLAG_MESSAGE_SR    0x00000200U
// Classic CRC for LIN
#define FLAG_MESSAGE_CC    0x00001000U
// Enhanced CRC for LIN
#define FLAG_MESSAGE_EC    0x00002000U
// Error frame
#define FLAG_MESSAGE_EF    0x01000000U
// RX
#define FLAG_MESSAGE_RX    0x10000000U
// TX
#define FLAG_MESSAGE_TX    0x20000000U
// Block TX
#define FLAG_MESSAGE_BLOCK_TX    0x30000000U

typedef struct
{
	uint32_t flags;
	uint32_t time;
	uint32_t crc;
	uint32_t msgID;
	uint16_t dlc;
	uint8_t data[8];
} __attribute__((packed)) BusMessage;

typedef struct
{
	MsgCommandHeader header;
	BusMessage message;
} __attribute__((packed)) RcvMessage;

typedef struct
{
	uint32_t flags;
	/*uint32_t time*/;
	uint32_t msgID;
	uint16_t dlc;
	uint8_t data[8];
} __attribute__((packed)) ToBusMessage;

typedef struct
{
	MsgCommandHeader header;
	ToBusMessage message;
} __attribute__((packed)) SendMessage;

#define COMMAND_MESSAGE 0x40

typedef struct
{
	CommandHeader header;
	uint8_t seed[16];
} __attribute__((packed)) DeviceSeed;

typedef struct
{
	CommandHeader header;
	uint8_t key[8];
} __attribute__((packed)) DeviceKey;

#define COMMAND_LICENSE 0x0f

typedef enum
{
	can_mode_close,
	can_mode_normal,
	can_mode_listen,
	can_mode_loopback,
} ch_can_mode_e;

typedef struct channel_t
{
	uint8_t open:1;
	uint8_t transmit:1;
	uint8_t configured:1;
	struct can_t * can;
} channel_t;

typedef struct chdev_t
{
	uint8_t open:1;
	struct channel_t ch0;
	struct channel_t ch1;

	struct ring_t * ring_com_rx;
	struct ring_t * ring_com_tx;

	char cmd[64];
	uint8_t cmd_idx;
} chdev_t;

uint8_t ch_info[] = "CH-3.2";
uint8_t ch_fw[] = "2.1.1.0";
uint8_t ch_serial[] = { 0, 0, 0, 0, 0, 0, 0, 0x2e };

static void ch_write(struct chdev_t * chdev, uint8_t * buf, uint8_t len)
{
	(void)chdev;

	for (size_t i = 0; i < len; i++) {

		cm_disable_interrupts();
		ring_write(chdev->ring_com_tx, &buf[i]);
		cm_enable_interrupts();
	}
}

static void ch_encrypt_block(const uint32_t key[4], uint32_t *a, uint32_t *b)
{
	for (int i = 0; i < 0x30; ) {
		*a += *b + (*b << 6 ^ *b >> 8) + key[i % 4] + i;
		i++;
		*b += *a + (*a << 6 ^ *a >> 8) + key[i % 4] + i;
		i++;
	}
}
static void ch_decrypt_block(const uint32_t key[4], uint32_t *a, uint32_t *b)
{
	for (int i = 0x30; i != 0; ) {
		i--;
		*b -= *a + (*a << 6 ^ *a >> 8) + key[i % 4] + i;
		i--;
		*a -= *b + (*b << 6 ^ *b >> 8) + key[i % 4] + i;
	}
}

static void ch_encrypt(const uint32_t key[4], void *buf, size_t bufLen)
{
	uint32_t * st = (uint32_t*)buf;
	for (size_t i = bufLen / 8; i != 0; i--) {
		ch_encrypt_block(key, st, st + 1);
		st += 2;
	}
}

static void ch_decrypt(const uint32_t key[4], void *buf, size_t bufLen)
{
	uint32_t * st = (uint32_t*)buf;
	for (size_t i = bufLen / 8; i != 0; i--) {
		ch_decrypt_block(key, st, st + 1);
		st += 2;
	}
}

uint8_t rx_sequence = 0;
uint32_t rx_pause = 0;
static void ch_com_rx_process(struct chdev_t * chdev)
{
	uint8_t b = 0;
	cm_disable_interrupts();
	uint8_t r = ring_read(chdev->ring_com_rx, &b);
	cm_enable_interrupts();

	if (r) {

		if (rx_pause > 5)
			chdev->cmd_idx = 0;

		rx_pause = 0;

		chdev->cmd[chdev->cmd_idx] = b;
		chdev->cmd_idx++;
		if (chdev->cmd_idx >= sizeof(CommandHeader)) {

			CommandHeader * h = (CommandHeader *)chdev->cmd;
			switch (h->command) {

				case COMMAND_SYNC:
					if ((h->sequence == 0x0) && (h->flags == COMMAND_SYNC) && (h->dSize == 0x0)) {

						uint8_t ans[] = { 0x5a, 0x0, 0x5a, 0x0 };

						ch_write(chdev, ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_DEVICE_HARDWARE:
					{
						DeviceIdentifier ans;
						ans.header.command = h->command;
						ans.header.sequence = h->sequence;
						ans.header.flags = 0;
						ans.header.dSize = 1;
						ans.hwId = HW_CH32;

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_DEVICE_INFO:
					{
						DeviceInfo ans;
						ans.header.command = h->command;
						ans.header.sequence = h->sequence;
						ans.header.flags = 0;
						ans.header.dSize = sizeof(ans.devInfo);
						memcpy(ans.devInfo, ch_info, sizeof(ans.devInfo));

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_DEVICE_FIRMWARE:
					{
						DeviceFw ans;
						ans.header.command = h->command;
						ans.header.sequence = h->sequence;
						ans.header.flags = 0;
						ans.header.dSize = sizeof(ans.devFw);
						memcpy(ans.devFw, ch_fw, sizeof(ans.devFw));

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_DEVICE_SERIAL:
					{
						DeviceSerial ans;
						ans.header.command = h->command;
						ans.header.sequence = h->sequence;
						ans.header.flags = 0;
						ans.header.dSize = sizeof(ans.serial);
						memcpy(ans.serial, ch_serial, sizeof(ans.serial));

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_DEVICE_MODE:
					{
						CommandHeader ans;
						ans.command = h->command;
						ans.sequence = h->sequence;
						ans.flags = 0;
						ans.dSize = 0;

						uint8_t valid = 1;
						if (h->flags != FLAG_DEVICE_MODE_CAN)
							valid = 0;

						if (valid)
							ans.command |= COMMAND_ACK;
						else
							ans.command = 0xff;

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_DEVICE_OPEN:
					{
						CommandHeader ans;
						ans.command = h->command | COMMAND_ACK;
						ans.sequence = h->sequence;
						ans.flags = 0;
						ans.dSize = 0;

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;

						set_dtr();
						chdev->open = 1;
						chdev->ch0.open = 0;
						chdev->ch1.open = 0;
						rx_sequence = 0;
					}
					break;

				case COMMAND_DEVICE_CLOSE:
					{

						CommandHeader ans;
						ans.command = h->command | COMMAND_ACK;
						ans.sequence = h->sequence;
						ans.flags = 0;
						ans.dSize = 0;

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;

						clr_dtr();
						chdev->open = 0;
					}
					break;

				case COMMAND_CHANNEL_CONFIG:
					if (chdev->cmd_idx >= (sizeof(CommandHeader) + h->dSize)) {

						uint8_t channel = h->flags & 0xf0;
						uint8_t config = h->flags & 0x0f;
						uint8_t valid = 1;

						CommandHeader ans;
						ans.command = h->command;
						ans.sequence = h->sequence;
						ans.flags = 0;
						ans.dSize = 0;

						if (channel != FLAG_CHANNEL_1 && channel != FLAG_CHANNEL_2)
							valid = 0;

						if (!chdev->open)
							valid = 0;

						struct channel_t * ch = (channel == FLAG_CHANNEL_1) ? &chdev->ch0 : &chdev->ch1;
						if (ch->open)
							valid = 0;

						if (valid) {

							if (config == FLAG_CONFIG_MODE) {

								uint8_t mode = chdev->cmd[sizeof(CommandHeader)];
								if (mode > MODE_LOOPBACK)
									valid = 0;

								if (mode == MODE_NORMAL)
									ch->transmit = 1;
							}
							else if (config == FLAG_CONFIG_BUS_SPEED_M) {

								BusCustomBaudRate s;
								memcpy(&s, chdev->cmd + sizeof(CommandHeader), sizeof(BusCustomBaudRate));

								uint32_t sjw = (s.SyncJW - 1) << 24;
								uint32_t ts2 = (s.tqSeg2 - 1) << 20;
								uint32_t ts1 = (s.tqSeg1 - 1) << 16;
								ch->configured = 1;
								//can_start(ch->can, e_speed_500);
								//           pr  ts1  ts2  sjw
								//11082208  0400 0f00 0200 0100
								can_set_bittiming(ch->can, s.Prescaler, ts1, ts2, sjw);
							}
							else
								valid = 0;
						}

						if (valid)
							ans.command |= COMMAND_ACK;
						else
							ans.command = 0xff;

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_CHANNEL_OPEN:
					{
						uint8_t channel = h->flags & 0xf0;
						uint8_t valid = 1;

						CommandHeader ans;
						ans.command = h->command;
						ans.sequence = h->sequence;
						ans.flags = 0;
						ans.dSize = 0;

						if (!chdev->open)
							valid = 0;

						if (channel != FLAG_CHANNEL_1 && channel != FLAG_CHANNEL_2)
							valid = 0;

						struct channel_t * ch = (channel == FLAG_CHANNEL_1) ? &chdev->ch0 : &chdev->ch1;
						if (!ch->configured)
							valid = 0;

						if (valid)
							ch->open = 1;

						if (valid)
							ans.command |= COMMAND_ACK;
						else
							ans.command = 0xff;

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_CHANNEL_CLOSE:
					{
						uint8_t channel = h->flags & 0xf0;
						uint8_t valid = 1;

						CommandHeader ans;
						ans.command = h->command;
						ans.sequence = h->sequence;
						ans.flags = 0;
						ans.dSize = 0;

						if (channel != FLAG_CHANNEL_1 && channel != FLAG_CHANNEL_2)
							valid = 0;

						if (valid) {

							struct channel_t * ch = (channel == FLAG_CHANNEL_1) ? &chdev->ch0 : &chdev->ch1;
							ch->open = 0;
							ch->configured = 0;
							ch->transmit = 0;
						}

						if (valid)
							ans.command |= COMMAND_ACK;
						else
							ans.command = 0xff;

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;

				case COMMAND_MESSAGE:
					{
						MsgCommandHeader * mh = (MsgCommandHeader *)chdev->cmd;
						if (chdev->cmd_idx >= (sizeof(MsgCommandHeader) + mh->dSize)) {

							uint8_t valid = 1;

							if (mh->flags != FLAG_MESSAGE_CHANNEL_1 && mh->flags != FLAG_MESSAGE_CHANNEL_2)
								valid = 0;

							struct channel_t * ch = (mh->flags == FLAG_MESSAGE_CHANNEL_1) ? &chdev->ch0 : &chdev->ch1;
							if (!ch->open)
								valid = 0;

							SendMessage * msg = (SendMessage *)chdev->cmd;
							if (valid) {

								can_message_t cmsg;
								cmsg.RTR = msg->message.flags & FLAG_MESSAGE_RTR;
								cmsg.IDE = msg->message.flags & FLAG_MESSAGE_EXTID;
								cmsg.dlc = msg->message.dlc;
								cmsg.id = msg->message.msgID;

								if (!cmsg.RTR) {
									for (uint8_t i = 0; i < cmsg.dlc; i++)
										cmsg.data[i] = msg->message.data[i];
								}

								can_msg_push(ch->can, &cmsg);
							}

							if (msg->message.flags & FLAG_MESSAGE_CONFIRM_REQUIRED) {

								MsgCommandHeader ans;
								ans.command = mh->command;
								ans.sequence = mh->sequence;
								ans.flags = 0;
								ans.dSize = 0;

								if (valid)
									ans.command |= COMMAND_ACK;
								else
									ans.command = 0xff;

								ch_write(chdev, (uint8_t *)&ans, sizeof(ans));
							}

							chdev->cmd_idx = 0;
						}
					}
					break;

				case COMMAND_LICENSE:
					if (chdev->cmd_idx >= (sizeof(CommandHeader) + h->dSize)) {

						uint32_t secret[4] = { 0xDEADC0DE, 0xDEADBEEF, 0xBAADF00D, 0xBAADBABE };
						DeviceSeed * s = (DeviceSeed *)chdev->cmd;
						uint8_t seed[16];
						memcpy(&seed, s->seed, sizeof(seed));
						ch_decrypt(secret, seed, sizeof(seed));

						uint8_t key[8];
						memcpy(key, ch_serial, sizeof(key));
						ch_encrypt((uint32_t*)seed, key, sizeof(key));

						DeviceKey ans;
						ans.header.command = h->command;
						ans.header.sequence = h->sequence;
						ans.header.flags = 0;
						ans.header.dSize = sizeof(ans.key);
						memcpy(ans.key, key, sizeof(ans.key));
						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;
				default:
					if (chdev->cmd_idx >= (sizeof(CommandHeader) + h->dSize)) {

						CommandHeader ans;
						ans.command = 0xff;
						ans.sequence = h->sequence;
						ans.flags = 0;
						ans.dSize = 0;

						ch_write(chdev, (uint8_t *)&ans, sizeof(ans));

						chdev->cmd_idx = 0;
					}
					break;
			}
		}
		
		if (chdev->cmd_idx >= sizeof(chdev->cmd))
			chdev->cmd_idx = 0;
	}
}

static void ch_handle_can_message(chdev_t * chdev, int flags, const can_message_t *cmsg)
{
	RcvMessage msg;
	msg.header.command = COMMAND_MESSAGE;
	msg.header.sequence = rx_sequence++;
	msg.header.flags = flags;
	msg.header.dSize = sizeof(BusMessage);

	if (!cmsg->IDE)
		msg.message.flags = FLAG_MESSAGE_RX;
	else
		msg.message.flags = FLAG_MESSAGE_RX | FLAG_MESSAGE_EXTID;

	msg.message.time = tick.msec;
	msg.message.crc = 0;
	msg.message.msgID = cmsg->id;
	msg.message.dlc = cmsg->dlc;

	// no data on RTR
	if (!cmsg->RTR)
		for (uint8_t i = 0; i < cmsg->dlc; i++)
			msg.message.data[i] = cmsg->data[i];

	ch_write(chdev, (uint8_t *)&msg, sizeof(msg));
}

static void ch_can_process(struct chdev_t * chdev)
{
	can_message_t msg;
	uint8_t r = can_msg_pop(chdev->ch0.can, &msg);
	if (r && chdev->ch0.open)
		ch_handle_can_message(chdev, FLAG_MESSAGE_CHANNEL_1, &msg);
	can_tx_process(chdev->ch0.can);

	r = can_msg_pop(chdev->ch1.can, &msg);
	if (r && chdev->ch1.open)
		ch_handle_can_message(chdev, FLAG_MESSAGE_CHANNEL_2, &msg);
	can_tx_process(chdev->ch1.can);
}

int main(void)
{
	cm_disable_interrupts();

	clock_setup();
	systick_setup();
	led_setup();
	systick_setup();

	iwdg_set_period_ms(1000);

	usb_setup(&ch_cb);

	can_setup();

	chdev_t chdev = {

		.open = 0,
		.ch0 = { .open = 0, .transmit = 0, .configured = 0, .can = can_get_hscan() },
		.ch1 = { .open = 0, .transmit = 0, .configured = 0, .can = can_get_mscan() },
		.ring_com_rx = get_rx_ring(),
		.ring_com_tx = get_tx_ring(),
		.cmd_idx = 0
	};

	cm_enable_interrupts();

	while(1)
	{
		iwdg_reset();

		ch_com_rx_process(&chdev);

		ch_can_process(&chdev);

		if (tick.flag_tick) {

			tick.flag_tick = 0;

			if (tick.flag_100ms) {

				rx_pause++;
				tick.flag_100ms = 0;
			}
			
			if (tick.flag_250ms) {

				tick.flag_250ms = 0;
				led4_blink();
			}

			if (tick.flag_1000ms) {

				tick.flag_1000ms = 0;
			}
		}
	}

	return 0;
}

