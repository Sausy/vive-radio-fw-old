#include <nrf.h>
#include "vpq.h"
#include <string.h>

#include "util.h"

#define VPQ_BUFFER_COUNT	16
#define VPQ_ITEM_COUNT		20

static vpq_buffer_t		vpq_Buffers[VPQ_BUFFER_COUNT];
static vpq_item_t		vpq_Items[VPQ_ITEM_COUNT];
static uint8_t			vpq_FirstEmptyItem;
static uint8_t			vpq_FirstEmptyBuffer;
static uint8_t			vpq_Queue[2];

vpq_buffer_t*  VPQ_GetBuffer(uint32_t index){
	return &vpq_Buffers[index-1];
}

vpq_item_t* VPQ_GetItem(uint32_t index){
	return &vpq_Items[index-1];
}



uint8_t VPQ_AllocBuffer(){
	vpq_buffer_t* buffer;
	uint8_t index;
	CriticalVar();
	CriticalEnter();
	index = 0;
	if (vpq_FirstEmptyItem != 0){
		index = vpq_FirstEmptyBuffer;
		buffer = VPQ_GetBuffer(vpq_FirstEmptyBuffer);
		vpq_FirstEmptyBuffer = buffer->data[0];
	}
	CriticalLeave();
	return index;
}

uint8_t VPQ_AllocItem(){
	vpq_item_t* item;
	uint8_t index;
	CriticalVar();
	CriticalEnter();
	index = 0;
	if (vpq_FirstEmptyItem != 0){
		index = vpq_FirstEmptyItem;
		item = VPQ_GetItem(vpq_FirstEmptyItem);
		vpq_FirstEmptyItem = item->next;
	}
	CriticalLeave();
	return index;
}

void VPQ_ReleaseBuffer(uint8_t index){
	CriticalVar();
	CriticalEnter();
	VPQ_GetBuffer(index)->data[0] = vpq_FirstEmptyBuffer;
	vpq_FirstEmptyBuffer = index;
	CriticalLeave();
}

void VPQ_ReleaseItem(uint8_t index){
	CriticalVar();
	CriticalEnter();
	VPQ_GetItem(index)->next = vpq_FirstEmptyItem;
	vpq_FirstEmptyItem = index;
	CriticalLeave();
}


void VPQ_Release(uint8_t index){
	vpq_item_t* item;
	CriticalVar();
	if (index == 0){
		return;
	}
	CriticalEnter();
	item = VPQ_GetItem(index);
	if (item->buffer_index != 0){
		VPQ_ReleaseBuffer(item->buffer_index);
		item->buffer_index = 0;
	}
	VPQ_ReleaseItem(index);
	CriticalLeave();
}

void VPQ_ReleaseChain(uint8_t index){
	register uint8_t next;
	for (; index != 0; index = next){
		next = VPQ_GetItem(index)->next;
		VPQ_Release(index);
	}
}

void VPQ_ReleaseAll(uint8_t q){
	CriticalVar();
	CriticalEnter();
	VPQ_ReleaseChain(vpq_Queue[q]);
	vpq_Queue[q] = 0;
	vpq_Queue[1 + q] = 0;

	CriticalLeave();
}

bool VPQ_Push(uint8_t q, const void* data, uint8_t length, vrf_packet_type_t type){
	register uint8_t index, buf_index;
	register vpq_item_t* item;
	CriticalVar();
	CriticalEnter();

	index = VPQ_AllocItem();

	if (index == 0 || length == 0 || length >= 0x20){
		CriticalLeave();
		return false;
	}
	item = VPQ_GetItem(index);
	buf_index = VPQ_AllocBuffer();
	item->buffer_index = buf_index;

	if (buf_index == 0){
		CriticalLeave();
		return false;
	}
	memcpy(VPQ_GetBuffer(buf_index)->data, data, length);
	item->packet_type = type;
	item->length = length;
	item->next = 0;

	if (vpq_Queue[1 + q] != 0){
		VPQ_GetItem(vpq_Queue[1 + q])->next = index;
	} else {
		vpq_Queue[q] = index;
	}
	vpq_Queue[1 + q] = index;
	CriticalLeave();
	return true;
}

bool VPQ_PushLarge(uint8_t q, const void* data, uint32_t length){
	register uint8_t tail;
	const uint8_t* pdata;
	CriticalVar();
	CriticalEnter();
	pdata = (const uint8_t*)data;
	
	tail = vpq_Queue[1 + q];
	for (; length > 27; length -= 27, pdata += 27){
		if (VPQ_Push(q, pdata, 27, vrf_pt_continuation) == 0){
			goto failure_exit;
		}
	}
	if (VPQ_Push(q, pdata, (uint8_t)length, vrf_pt_final) != 0){
		CriticalLeave();
		return true;
	}
failure_exit:

	if (tail != 0){
		VPQ_ReleaseChain(VPQ_GetItem(tail)->next);
		VPQ_GetItem(tail)->next = 0;
		vpq_Queue[1 + q] = tail;
	} else {
		VPQ_ReleaseAll(q);
	}
	CriticalLeave();
	return false;
}

uint8_t VPQ_Pop(uint8_t q){
	register uint8_t index, next;
	CriticalVar();
	CriticalEnter();
	index = vpq_Queue[q];
	if (index != 0){
		next = VPQ_GetItem(index)->next;
		vpq_Queue[q] = next;
		if (next == 0){
			vpq_Queue[1 + q] = 0;
		}
	}
	CriticalLeave();
	return index;
}


bool VPQ_IsEmpty(uint8_t q){
	return vpq_Queue[q] == 0;
}


static void VPQ_InitBuffers(){
	register uint32_t i;
	register vpq_buffer_t* buffer;
	vpq_FirstEmptyBuffer = 0;
	for (i = 1; i <= VPQ_BUFFER_COUNT; ++i){
		buffer = VPQ_GetBuffer(i);
		buffer->data[0] = vpq_FirstEmptyItem;
		vpq_FirstEmptyBuffer = i;
	}
}

static void VPQ_InitItems(){
	uint32_t i;
	vpq_item_t* item;
	vpq_FirstEmptyItem = 0;
	for (i = 1; i <= VPQ_ITEM_COUNT; ++i){
		item = VPQ_GetItem(i);
		item->next = vpq_FirstEmptyItem;
		vpq_FirstEmptyItem = i;
	}
}

void VPQ_Init(){
	VPQ_InitBuffers();
	VPQ_InitItems();
}

