#ifndef RING_H
#define RING_H

#include <inttypes.h>
#include <stdbool.h>

typedef int32_t ring_size_t;
typedef uint8_t elem_size_t;
typedef struct ring_t
{
	void * data;
	elem_size_t elem_size;
	ring_size_t nums;
	volatile uint32_t begin;
	volatile uint32_t end;
} ring_t;

void ring_init(struct ring_t *ring, void *buf, elem_size_t elem_size, ring_size_t nums);
void ring_reset(struct ring_t *ring);
bool ring_write(struct ring_t *ring, const void *data);
bool ring_read(struct ring_t *ring, void * data);

#endif

