#include <string.h>
#include "ring.h"

void ring_init(struct ring_t *ring, void *buf, elem_size_t elem_size, ring_size_t nums)
{
	ring->data = buf;
	ring->elem_size = elem_size;
	ring->nums = nums;
	ring->begin = 0;
	ring->end = 0;
}

void ring_reset(struct ring_t *ring)
{
	ring->begin = 0;
	ring->end = 0;
}

bool ring_write(struct ring_t *ring, const void * data)
{
	if (((ring->end + 1) % ring->nums) != ring->begin) {

		uint8_t * p = (uint8_t *)ring->data + ring->end * ring->elem_size;
		memcpy(p, data, ring->elem_size);
		ring->end++;
		ring->end %= ring->nums;
		return true;
	}

	return false;
}

bool ring_read(struct ring_t *ring, void * data)
{
	if (ring->begin != ring->end) {

		uint8_t * p = (uint8_t *)ring->data + ring->begin * ring->elem_size;
		memcpy(data, p, ring->elem_size);
		ring->begin++;
		ring->begin %= ring->nums;
		return true;
	}

	return false;
}

