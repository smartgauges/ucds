#ifndef CAN_H
#define CAN_H

#include <inttypes.h>

typedef enum e_speed_t
{
	e_speed_10 = 0,
	e_speed_20,
	e_speed_50,
	e_speed_100,
	e_speed_125,
	e_speed_250,
	e_speed_500,
	e_speed_800,
	e_speed_1000,
	e_speed_nums
} e_speed_t;

typedef struct can_message_t
{
	struct {
		uint8_t dlc:4;
		uint8_t RTR:1;
		uint8_t IDE:1;
	};
	uint32_t echo_id;
	uint32_t id;
	uint8_t data[8];
} can_message_t;

struct can_t;
struct can_t * can_get_mscan(void);
struct can_t * can_get_hscan(void);

void can_setup(void);
void can_disable(struct can_t * can);
void can_stop(struct can_t * can);
bool can_start(struct can_t * can, e_speed_t speed);
bool can_set_speed(struct can_t * can, e_speed_t speed);
bool can_set_bittiming(struct can_t * can, uint32_t brp, uint32_t phase_seg1, uint32_t phase_seg2, uint32_t sjw);

uint8_t can_get_sts(struct can_t * can);
uint32_t can_get_rx_cnts(struct can_t * can);
uint32_t can_get_tx_cnts(struct can_t * can);

void can_tx_process(struct can_t *can);
void can_msg_push(struct can_t *can, struct can_message_t *msg);
bool can_msg_pop(struct can_t *can, struct can_message_t *msg);
bool can_echo_pop(struct can_t *can, struct can_message_t *msg);

#endif

