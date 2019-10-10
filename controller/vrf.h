#ifndef VRF_H
#define VRF_H

#include <stdbool.h>
#include "vrf_types.h"

#define VRF_RETRANSMIT_DELAY	230

void API_OnConnect();
void API_OnDisconnect();
void API_OnReceive(const void* data, uint8_t length,
	bool is_final);


void VRF_Initialize();
void VRF_Update();
void VRF_StartPair(bool clear_pairings);
void VRF_Disconnect();
bool VRF_CanSendUnreliableData();
bool VRF_SetUnreliableData(const void* data, uint8_t length);

void DbgLEDEnable(uint8_t idx);
void DbgLEDDisable(uint8_t idx);
void DbgLEDDisableAll();
void DbgStr(const char* fmt, ...);

#endif //VRF_H

