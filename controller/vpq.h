#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>
#include "vrf_types.h"

/*
Vive/Valve packet queue
*/

typedef struct {
	uint8_t	data[32];
} vpq_buffer_t;

typedef struct {
	uint8_t				next;
	uint8_t				length;
	vrf_packet_type_t	packet_type;
	uint8_t				buffer_index;
} vpq_item_t;

vpq_buffer_t*  VPQ_GetBuffer(uint32_t index);
vpq_item_t* VPQ_GetItem(uint32_t index);
uint8_t VPQ_AllocBuffer();
uint8_t VPQ_AllocItem();

void VPQ_ReleaseBuffer(uint8_t index);
void VPQ_ReleaseItem(uint8_t index);
void VPQ_Release(uint8_t index);
void VPQ_ReleaseChain(uint8_t index);
void VPQ_ReleaseAll(uint8_t q);
bool VPQ_Push(uint8_t q, const void* data, uint8_t length, vrf_packet_type_t type);
bool VPQ_PushLarge(uint8_t q, const void* data, uint32_t length);
uint8_t VPQ_Pop(uint8_t q);
bool VPQ_IsEmpty(uint8_t q);
void VPQ_Init();

#endif //!FIFO_H
