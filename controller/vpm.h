#ifndef VPM_H
#define VPM_H

#include "vrf_types.h"

/*
Vive/Valve pairing manager
*/

//struct vrf_address_t;
//struct vrf_ccm_key_t;

void VPM_Intialize();
void VPM_Register(const vrf_address_t* address, const vrf_ccm_key_t* key);
const vrf_ccm_key_t* VPM_GetKey(const vrf_address_t* address);
const vrf_address_t* VPM_GetNextPairedAddress(uint8_t* index);
const vrf_address_t* VPM_GetFirstPairedAddress(uint8_t* index);
void VPM_EraseAll();

#endif //!VPM_H
