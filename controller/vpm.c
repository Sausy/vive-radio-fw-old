#include <nrf.h>
#include "vpm.h"
#include <string.h>

#include "util.h"

#define VPM_PAGE_OFFSET			0x1F800
#define VPM_PAGE_SIZE			0x400
#define VPM_ENTRIES_PER_PAGE	(VPM_PAGE_SIZE/sizeof(vpm_entry_t))

#define VPM_RAM_PAGE

#ifdef VPM_RAM_PAGE
uint8_t	vpm_Page0[1024];
uint8_t	vpm_Page1[1024];
#endif //VPM_RAM_PAGE

static uint8_t	vpm_CurrentPage;
static uint8_t	vpm_IsUnpaired;

typedef union {
	struct {
		uint8_t			tag;
		vrf_address_t	address;
		vrf_ccm_key_t	key;
	} val;
	uint8_t	data[22];
} vpm_entry_t;


void VPM_Erase(uint8_t pageindex){
	if (pageindex == 0){
#ifdef VPM_RAM_PAGE
		memset(vpm_Page0, 0xFF, sizeof(vpm_Page0));
#else //VPM_RAM_PAGE
		nrf_nvmc_page_erase(VPM_PAGE_OFFSET);
#endif //!VPM_RAM_PAGE
	} else {
#ifdef VPM_RAM_PAGE
		memset(vpm_Page1, 0xFF, sizeof(vpm_Page1));
#else //VPM_RAM_PAGE
		nrf_nvmc_page_erase(VPM_PAGE_OFFSET + VPM_PAGE_SIZE);
#endif //!VPM_RAM_PAGE
	}
}

vpm_entry_t* VPM_GetEntry(uint8_t pageindex, uint8_t index){
	register void* page;
	if (pageindex == 0){
#ifdef VPM_RAM_PAGE
		page = vpm_Page0;
#else //VPM_RAM_PAGE
		page = (void*)VPM_PAGE_OFFSET;
#endif //!VPM_RAM_PAGE
	} else {
#ifdef VPM_RAM_PAGE
		page = vpm_Page1;
#else //VPM_RAM_PAGE
		page = (void*)(VPM_PAGE_OFFSET + VPM_PAGE_SIZE);
#endif //!VPM_RAM_PAGE
	}

	return &((vpm_entry_t*)page)[index - 1];
}

uint8_t VPM_IsEntryEmpty(uint8_t pageindex, uint8_t index){
	register vpm_entry_t* entry;
	register uint8_t i;
	entry = VPM_GetEntry(vpm_CurrentPage, index);
	for (i = 0; i < 20; ++i){
		if (entry->data[i] != 0xFF){
			return 0;
		}
	}
	return 1;
}

uint8_t VPM_FindEmptySlot(uint8_t pageindex){
	register uint8_t i;
	for (i = 1; i <= VPM_ENTRIES_PER_PAGE; ++i){
		if (VPM_IsEntryEmpty(pageindex, i)){
			return i;
		}
	}
	return 0;
}


void VPM_EraseAll(){
	if (VPM_FindEmptySlot(0) != 1){
		VPM_Erase(0);
	}
	if (VPM_FindEmptySlot(1) != 1){
		VPM_Erase(1);
	}
}

uint8_t VPM_FindEntry(const vrf_address_t* address){
	register uint8_t i;
	register vpm_entry_t* entry;
	for (i = VPM_ENTRIES_PER_PAGE; i != 0; --i){
		entry = VPM_GetEntry(vpm_CurrentPage, i);
		if (entry->val.tag != 0xF0){
			continue;
		}
		if (memcmp(&entry->val.address, address, 5) == 0){
			return i;
		} else {
			return 0;
		}
	}
	return 0;
}

const vrf_ccm_key_t* VPM_GetKey(const vrf_address_t* address){
	register uint8_t index;
	index = VPM_FindEntry(address);
	if (index != 0){
		return &VPM_GetEntry(vpm_CurrentPage, index)->val.key;
	}
	return NULL;
}

const vrf_address_t* VPM_GetNextPairedAddress(uint8_t* index){
	register vpm_entry_t* entry;
	for (--(*index); *(index) != 0; --(*index)){
		entry = VPM_GetEntry(vpm_CurrentPage, *index);
		if (entry->val.tag == 0xFF){
			continue;
		}
		if (entry->val.tag != 0xF0){
			continue;
		}
		if (VPM_FindEntry(&entry->val.address) == *index){
			return &entry->val.address;
		}
	}
	return NULL;
}

const vrf_address_t* VPM_GetFirstPairedAddress(uint8_t* index){
	*index = VPM_ENTRIES_PER_PAGE;
	return VPM_GetNextPairedAddress(index);
}

void VPM_WriteEntry(uint8_t pageindex, uint8_t index, const vpm_entry_t* entry){
	register vpm_entry_t* entry_dst;
	entry_dst = VPM_GetEntry(pageindex, index);
#ifdef VPM_RAM_PAGE
	memcpy(entry_dst, entry, sizeof(vpm_entry_t));
#else //VPM_RAM_PAGE
	nrf_nvmc_write_bytes((uint32_t)entry_dst, (const uint8_t*)entry, sizeof(vpm_entry_t));
#endif //!VPM_RAM_PAGE
}

void VPM_Register(const vrf_address_t* address, const vrf_ccm_key_t* key){
	register uint8_t index;
	vpm_entry_t entry;

	index = VPM_FindEmptySlot(vpm_CurrentPage);
	vpm_IsUnpaired = 0;
	if (index == 0){
		VPM_Erase(vpm_CurrentPage);
		index = VPM_FindEmptySlot(vpm_CurrentPage);
		if (index == 0){
			return;
		}
	}
	entry.val.tag = 0xF0;
	entry.val.address = *address;
	entry.val.key = *key;
	VPM_WriteEntry(vpm_CurrentPage, index, &entry);
}

void VPM_Intialize(){
	vpm_CurrentPage = 1;
	vpm_IsUnpaired = 0;
	if (VPM_IsEntryEmpty(vpm_CurrentPage, 1) == 0){
		return;
	}
	vpm_CurrentPage = 0;
	if (VPM_IsEntryEmpty(vpm_CurrentPage, 1) == 0){
		return;
	}
	vpm_IsUnpaired = 1;
}
