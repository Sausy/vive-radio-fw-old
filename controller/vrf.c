#include "vpq.h"
#include "vpm.h"
#include "vrf.h"
#include <string.h>
#include "nrf.h"
#include "micro_esb.h"
#include "uesb_error_codes.h"
#include <stdbool.h>
#include "util.h"



static vrf_state_t		vrf_State;


static uint8_t				vrf_ChannelIndex = 0;
static vrf_channel_table_t	vrf_ChannelTable = {0};

//static uint8_t			vrf_TransmissionSucceeded; //!Name
static uint8_t			vrf_DelayCounter; //!Name
static uesb_payload_t	vrf_TXPayload;
static uesb_payload_t	vrf_RXPayload;
static bool				vrf_InterframeSpacingAdded; //!Name
static uint16_t			vrf_NextTimeDelta; //!Name
//static uint8_t			vrf_TXFailCount; //!Name
static bool				vrf_ReadyForTransmission = false;
static bool				vrf_HasTXPayload = false;
static int32_t			vrf_LastRSSI = 0;

static vrf_ccm_plaintext_t	vrf_TXPlaintext;
static vrf_ccm_ciphertext_t	vrf_TXCiphertext;

static vrf_ccm_plaintext_t	vrf_RXPlaintext;
static vrf_ccm_ciphertext_t	vrf_RXCiphertext;

static vrf_ccm_key_t		vrf_RXKey;
static vrf_address_t		vrf_RemoteAddress;

static uint32_t				vrf_LocalSessionID;

static uint8_t				vrf_ConnectNotificationCount = 0;
static uint16_t				vrf_TicksUntilUpdate = 0;
static uint16_t				vrf_TicksUntilPumpRX = 0;
static uint8_t				vrf_TransmitFailCount = 0;
static uint8_t				vrf_ChannelHopCountdown = 0;
static bool					vrf_StartPair = false;
static bool					vrf_StartConnect = false;
static bool					vrf_StartDisconnect = false;
static uint16_t				vrf_NextTimeDelta = 0; //!NAME
static uint8_t				vrf_RetransmissionCounter = 0;
static uint16_t				vrf_TicksUntilOtherEvent = 0;
static uint32_t				vrf_TimerTicks = 0;
//Packet ID of the last ACK received
static uint8_t				vrf_LastACKPacketID;
//Unreliable packets aren't sent when this value
//matches vrf_LastACKPacketID
static uint8_t				vrf_SuspendUnreliablePacketID; //!NAME

static uint8_t				vrf_ChannelHopsUntilRetransmission;

//static uint8_t				vrf_PairRSSILimit;

static uint8_t				vrf_ConnectionAttemptsRemaining;

static uint8_t				vrf_PairedKeyIndex;

static uint8_t				vrf_UnreliableData[32 - 5];
static uint8_t				vrf_UnreliableDataLength;

static uint32_t				vrf_20000090[2]; //!Unknown purpose


											 //TX hops
static bool						vrf_TransmissionHopsStarted;
static uint8_t					vrf_TransmissionHops = 0;
static uint8_t					vrf_MaxTransmissionHops = 0;

static vrf_discovery_state_t	vrf_DiscoveryState;
static uint8_t					vrf_DiscoveryRSSILimit;
static vrf_firmware_info_packet_t	vrf_RemoteFirmwareInfo;

//Vive controller key & pair address
#define VRF_STATIC_KEY_DATA 0x88, 0x20, 0x2C, 0xBA, 0x5A, 0x44, 0x4F, 0x4A, \
	0x71, 0x28, 0xAE, 0xFB, 0xE3, 0x9B, 0xA1, 0x1B

#define VRF_STATIC_ADDRESS		'V', 'O', 'R', 'T', 'Y'
//#define VRF_STATIC_ADDRESS		'V', 'A', 'L', 'V', 'E'
//#define VRF_STATIC_TIMESTAMP	0x579FA7E2
#define VRF_STATIC_TIMESTAMP	0x5673A81E

static const vrf_ccm_key_t		vrf_StaticKey = {{VRF_STATIC_KEY_DATA}};
static const vrf_address_t		vrf_StaticAddress = {{VRF_STATIC_ADDRESS}};

//#define VRF_DEVKIT

static const uint8_t			vrf_PumpRXPayload[1] = {2};

#ifdef VRF_DEVKIT
static const uint8_t			vrf_DevKitPacket[1] = {2};
#endif //VRF_DEVKIT

/*static const uint8_t			vrf_AESPayload[17] = {1,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
};*/

/*
Maybe put specific parts into their own files
*/


/*
System
*/

void SYS_GetDeviceID(vrf_device_id_t *device_id){
	device_id->data[0] = 0x12345678; // NRF_FICR->DEVICEID[0];
	device_id->data[1] = 0x87654321;  //NRF_FICR->DEVICEID[1];
}

/*
Device RNG
*/

static uint32_t	vrf_RNGState;

void VRF_RNGHWGenerate(void* dst, uint32_t length){
	register uint32_t i;
	NRF_RNG->TASKS_START = 1;
	for (i = 0; i < length; ++i){
		NRF_RNG->EVENTS_VALRDY = 0;
		do {
		} while (NRF_RNG->EVENTS_VALRDY == 0);
		((uint8_t*)dst)[i] = (uint8_t)NRF_RNG->VALUE;
	}
	NRF_RNG->TASKS_STOP = 0;
}

void VRF_RNGInitialize(){
	do {
		VRF_RNGHWGenerate(&vrf_RNGState, sizeof(vrf_RNGState));
	} while (vrf_RNGState == 0);
}

uint32_t VRF_RNGGenerate(){
	vrf_RNGState = (vrf_RNGState >> 1) ^ ((vrf_RNGState & 1) ? 0xd0000001u : 0);
	return vrf_RNGState;
}

/*
Channel table generation
*/

#define VRF_CT_FIRST_CHANNEL		2
#define VRF_CT_CHANNELS_PER_STEP	11

static uint32_t	vrf_CTState;

static uint32_t VRF_CTNext(){
	vrf_CTState = (vrf_CTState >> 1) ^ ((vrf_CTState & 1) ? 0xd0000001u : 0);
	return vrf_CTState;
}

static void VRF_CTMix(uint8_t iterations){
	do {
		VRF_CTNext();
	} while (--iterations != 0);
}

void VRF_CTGenerateInternal(const vrf_address_t* address, vrf_channel_table_t* table){
	register uint8_t i, ch, j;
	vrf_CTState =
		(address->address[1] << 24) |
		(address->address[2] << 16) |
		(address->address[3] << 8) |
		(address->address[4]);

	vrf_CTState ^= address->address[0];

	if (vrf_CTState == 0){
		vrf_CTState = 1;
	}
	//Generate table based on address
	for (i = 0; i < 7; ++i){
		table->table[i] = (VRF_CTNext() % VRF_CT_CHANNELS_PER_STEP) +
			(i * VRF_CT_CHANNELS_PER_STEP) +
			VRF_CT_FIRST_CHANNEL;
		VRF_CTMix(3);
	}
	//Rebalance channels
	for (i = 1; i < 7; ++i){
		ch = table->table[i] - table->table[i - 1];
		if (ch < 5){
			table->table[i] += 5 - ch;
		}
	}
	if (table->table[6] > 80){
		table->table[6] = 80;
	}
	//Shuffle channels
	i = 6;
	do {
		j = VRF_CTNext() % (i+1);
		ch = table->table[i];
		table->table[i] = table->table[j];
		table->table[j] = ch;
		VRF_CTMix(3);
	} while (--i > 1);
}


void VRF_CTGenerate(const vrf_address_t* address, vrf_channel_table_t* table){
	vrf_address_t addr;
	addr = *address;
	addr.address[0] &= ~7;
	VRF_CTGenerateInternal(&addr, table);
	/*DbgStr("CT { %.2X, %.2X, %.2X, %.2X, %.2X, %.2X, %.2X }\r\n",
		table->table[0], table->table[1], table->table[2], table->table[3],
		table->table[4], table->table[5], table->table[6]);//*/
}

/*
CCM
*/

static uint8_t	vrf_CCMScratchBuffer[16 + VRF_PAYLOAD_SIZE];

static vrf_ccm_config_t	vrf_ReceiveConfig;
static vrf_ccm_config_t	vrf_TransmitConfig;



void VRF_CCMInitializeConfigs(const vrf_ccm_key_t* key, const vrf_ccm_iv_t* iv){
	if (key == NULL){
		VRF_RNGGenerate(vrf_ReceiveConfig.key.key, sizeof(vrf_ReceiveConfig.key.key));
	} else if (key != &vrf_ReceiveConfig.key){
		vrf_ReceiveConfig.key = *key;
	}
	if (iv == NULL){
		VRF_RNGGenerate(vrf_ReceiveConfig.iv.data, sizeof(vrf_ReceiveConfig.iv.data));
	} else if (iv != &vrf_ReceiveConfig.iv){
		vrf_ReceiveConfig.iv = *iv;
	}
	memset(&vrf_ReceiveConfig.count, 0, sizeof(vrf_packet_count_t));
	//vrf_ReceiveConfig.count = (const vrf_packet_count_t){ 0 };
	memcpy(&vrf_TransmitConfig, &vrf_ReceiveConfig, sizeof(vrf_ccm_config_t));
	vrf_ReceiveConfig.direction = 1;
	vrf_TransmitConfig.direction = 0;
	vrf_SuspendUnreliablePacketID = (vrf_LastACKPacketID = 0) + 6;
}

void VRF_CCMEncrypt(vrf_ccm_config_t* ccmconfig, vrf_ccm_ciphertext_t* ciphertext,
	vrf_ccm_plaintext_t* plaintext){
	register uint8_t header;

	if (plaintext->length > VRF_PAYLOAD_SIZE){
		ciphertext->length = 0;
	}
	header = plaintext->header;

	plaintext->header = header & (
		(VRF_TYPE_MASK << VRF_TYPE_SHIFT) | 
		(VRF_UNKNOWN_MASK << VRF_UNKNOWN_SHIFT));

	NRF_CCM->CNFPTR = (uint32_t)ccmconfig;
	NRF_CCM->ENABLE = 2; //Enable
	NRF_CCM->INPTR = (uint32_t)plaintext;
	NRF_CCM->OUTPTR = (uint32_t)ciphertext;
	NRF_CCM->MODE = 0; //Encrypt
	NRF_CCM->EVENTS_ENDCRYPT = 0;
	NRF_CCM->EVENTS_ERROR = 0;
	NRF_CCM->TASKS_KSGEN = 1;
	do {
	} while (NRF_CCM->EVENTS_ENDCRYPT == 0);
	NRF_CCM->ENABLE = 0;
	ciphertext->header = header;
	plaintext->header = header;
}

bool VRF_CCMDecrypt(vrf_ccm_config_t* ccmconfig, vrf_ccm_plaintext_t* plaintext,
	vrf_ccm_ciphertext_t* ciphertext){
	register uint8_t header;
	if (ciphertext->length > 31){
		plaintext->length = 0;
		DbgStr("Bad length\r\n");
		return false;
	}
	header = ciphertext->header;
	ciphertext->header = header  & (
		(VRF_TYPE_MASK << VRF_TYPE_SHIFT) |
		(VRF_UNKNOWN_MASK << VRF_UNKNOWN_SHIFT));

	NRF_CCM->CNFPTR = (uint32_t)ccmconfig;
	NRF_CCM->ENABLE = 2;
	NRF_CCM->INPTR = (uint32_t)ciphertext;
	NRF_CCM->OUTPTR = (uint32_t)plaintext;
	NRF_CCM->MODE = 1; //Decrypt
	NRF_CCM->EVENTS_ENDCRYPT = 0;
	NRF_CCM->EVENTS_ERROR = 0;
	NRF_CCM->TASKS_KSGEN = 1;
	do {
	} while (NRF_CCM->EVENTS_ENDCRYPT == 0);
	NRF_CCM->ENABLE = 0;
	plaintext->header = header;
	ciphertext->header = header;
	return NRF_CCM->MICSTATUS & 1;
}

void VRF_CCMConfigure(){
	NRF_CCM->POWER = 1;
	NRF_CCM->SHORTS = 1; //Enable KSGEN -> CRYPT shortcut
	NRF_CCM->SCRATCHPTR = (uint32_t)vrf_CCMScratchBuffer;
}

/*
VRF
*/


/*
Packet header:

xIII xRTT
T = packet type
I = Packet ID low bits
R = LSB of received packet count

Packet types:
	0 = keep alive
	1 = control
	2 = continuation
	3 = finish
*/

/*
Forward declarations
*/

uint8_t VRF_GetChannel();
bool VRF_DecryptPacket(vrf_ccm_plaintext_t* plaintext, vrf_ccm_ciphertext_t* ciphertext);
uint8_t VRF_GetPairingChannel(vrf_address_t* address);
void VRF_HandleControlPacket(const uint8_t* data, uint8_t length);
void VRF_OtherEvent();
uint8_t VRF_ChannelHop();
void VRF_Initialize();
bool VRF_IsConnected();
void VRF_PumpRX();
void VRF_HandleACK(uint8_t header);
void VRF_HandleDiscoveryCandidate(vrf_address_t* address, vrf_ccm_iv_t* iv0,
	vrf_ccm_iv_t* iv1, uint8_t rssi);
void VRF_ClearDiscoveryState();
void VRF_SignalOtherEvent();
void VRF_SendDeviceID(vrf_packet_type_t type);
void VRF_DelayedCallback(uint16_t* counter, void(*callback)());
void VRF_SetAddress(uesb_address_type_t type, const vrf_address_t* address,
	bool stop_transmission_hops);
void VRF_TimeToTicks(uint16_t* ticks, uint16_t time);
void VRF_Disconnect();
void VRF_StartPair(bool clear_pairings);
void VRF_OnUpdate();
void VRF_Update();
void VRF_UpdateRXCountLSB();
void VRF_IncrementPacketCount(vrf_packet_count_t* count);
bool VRF_IsPacketCountNonZero(const vrf_packet_count_t* count);
uint16_t VRF_GetPacketAirBits(uint8_t length);
void VRF_SetState(vrf_state_t state);
void VRF_Send(vrf_packet_type_t type, const void* payload, uint8_t length);
void VRF_SetTXPayload(uint8_t header, const void* payload, uint8_t length, 
	uint8_t count_octet0); //Investigate type of 'header'
void VRF_sub_30D4(uint8_t attempts);
bool VRF_TrySend();
void VRF_OnReceive(uint8_t a, uint8_t b);
void VRF_OnTransmitFail(uint8_t a);
void VRF_OnTransmitSuccess(uint8_t a, uint8_t b);
void VRF_EventHandler();

uint8_t VRF_GetChannel(){
	if (vrf_State != vrf_s_pair_begin){
		if (vrf_ChannelIndex > 7){
			vrf_ChannelIndex = 0;
		}
		return vrf_ChannelTable.table[vrf_ChannelIndex];
	} else {
		if (vrf_ChannelIndex < 2 || vrf_ChannelIndex > 80){
			vrf_ChannelIndex = 2;
		}
		return vrf_ChannelIndex;
	}
}

bool VRF_DecryptPacket(vrf_ccm_plaintext_t* plaintext, vrf_ccm_ciphertext_t* ciphertext){
	vrf_packet_count_t count;
	if (ciphertext->length < 5){
		DbgStr("Bad length\r\n");
		return false;
	}
	if (((vrf_ReceiveConfig.count.count[0] & 1) ^ 1) !=
		VRF_EXTRACT_PARITY(ciphertext->header)){
		DbgStr("Bad parity\r\n");
		return false;
	}
	count = vrf_ReceiveConfig.count;
	VRF_IncrementPacketCount(&vrf_ReceiveConfig.count);
	DbgStr("Header = %2.X\r\n", ciphertext->header);
	if (VRF_CCMDecrypt(&vrf_ReceiveConfig, plaintext, ciphertext)){
		VRF_UpdateRXCountLSB();
		return true;
	}
	DbgStr("Bad CRC\r\n");
	vrf_ReceiveConfig.count = count;
	return false;
}

void VRF_HandleControlPacket(const uint8_t* data, uint8_t length){
	if (length < 1){
		return;
	}
	DbgStr("Ctrl packet: %u\r\n", data[0]);
	switch (data[0]){
	case vrf_cp_0:
	case vrf_cp_aes_key:
	case vrf_cp_2:
		break;
	case vrf_cp_max_transmission_hops:{//{{
		if (length >= 2){
			DbgStr("vrf_MaxTransmissionHops = %u\r\n", data[1]);
			vrf_MaxTransmissionHops = data[1];
		}
		break;
	} //}}
	case vrf_cp_firmware_timestamp:{//{{
		if (length >= sizeof(vrf_firmware_info_packet_t)){
			vrf_RemoteFirmwareInfo= *(vrf_firmware_info_packet_t*)data;
		}
		break;
	} //}}
	}
}

void VRF_Unknown(uint8_t n){
	//TODO
}

void VRF_OtherEvent(){ //!Name
	VRF_Unknown(2);
}

uint8_t VRF_ChannelHop(){
	++vrf_ChannelIndex;

	if (++vrf_TransmissionHops >= vrf_MaxTransmissionHops){
		vrf_TransmissionHops = 0;
	}
	return VRF_GetChannel();
}

void VRF_Initialize(){
	vrf_State = vrf_s_disconnected;

	VRF_CCMConfigure();
	memset(&vrf_ReceiveConfig, 0, sizeof(vrf_ccm_config_t));
	VRF_RNGInitialize();
	VPM_Intialize();

	uesb_config_t config = UESB_DEFAULT_CONFIG;

	config.mode = UESB_MODE_PTX;
	config.crc = UESB_CRC_16BIT;
	config.protocol = UESB_PROTOCOL_ESB_DPL;
	config.bitrate = UESB_BITRATE_1MBPS;
	config.tx_output_power = RADIO_TXPOWER_TXPOWER_Pos4dBm;
	config.dynamic_ack_enabled = 1;
	config.retransmit_count = 1;
	config.retransmit_delay = VRF_RETRANSMIT_DELAY;
	config.tx_mode = UESB_TXMODE_MANUAL;
	config.payload_length = VRF_PAYLOAD_SIZE + VRF_HEADER_LENGTH;
	config.event_handler = VRF_EventHandler;
	config.radio_irq_priority = 1;
	uesb_init(&config);

	NRF_TIMER0->MODE = 0;
	NRF_TIMER0->PRESCALER = 4; //1 MHz
	NRF_TIMER0->CC[0] = 1500;
	NRF_TIMER0->BITMODE = 0; //16 bits
	NRF_TIMER0->SHORTS = 1;
	NRF_TIMER0->INTENSET = 1 << 16; //COMPARE[0] interupt
	NVIC_SetPriority(TIMER0_IRQn, 2);
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->TASKS_START = 1;

	vrf_ReadyForTransmission = true;
}

bool VRF_IsConnected(){
	if (vrf_State == vrf_s_connected){
		return true;
	} else {
		return false;
	}
}

void VRF_PumpRX(){
	if (VRF_IsConnected() && VPQ_IsEmpty(0)){
		//stats_KeepAliveCount
		VPQ_Push(0, vrf_PumpRXPayload, sizeof(vrf_PumpRXPayload), vrf_pt_control);
	}
}

void VRF_HandleACK(uint8_t header){
	register uint16_t rx_packet_id, tx_packet_id;
	register uint8_t header_id, local_id;



	header_id = VRF_EXTRACT_ID(header);
	local_id = vrf_LastACKPacketID & 7;

	//ID based on reponse header
	rx_packet_id = (vrf_LastACKPacketID & (~7)) | header_id;
	if (header_id < local_id){
		rx_packet_id += 8;
	}
	tx_packet_id = vrf_TransmitConfig.count.count[0];
	if (tx_packet_id < vrf_LastACKPacketID){
		tx_packet_id += 256;
	}
	if (rx_packet_id > tx_packet_id){
		//DbgStr("Received new ACK early\r\n");
		return;
	} else if (rx_packet_id != vrf_LastACKPacketID){
		VRF_SignalOtherEvent();
		//stats_ACKCount
		if (vrf_HasTXPayload &&
			VRF_EXTRACT_ID(vrf_TXPayload.data[0]) == (rx_packet_id & 7)){
			//DbgStr("Got most recent ACK\r\n");
			vrf_HasTXPayload = false;
		}
		vrf_SuspendUnreliablePacketID = (vrf_LastACKPacketID = (uint8_t)rx_packet_id) + 6;
	}
}

void VRF_ClearDiscoveryState(){
	memset(&vrf_DiscoveryState, 0, sizeof(vrf_discovery_state_t));
	vrf_DiscoveryState.rssi = 255;
}

void VRF_SignalOtherEvent(){ //!Name
	VRF_Unknown(1);
	VRF_TimeToTicks(&vrf_TicksUntilOtherEvent, 16);
}

void VRF_SendDeviceID(vrf_packet_type_t type){
	vrf_device_id_t devid;
	SYS_GetDeviceID(&devid);
	VRF_SetTXPayload(type, &devid, sizeof(vrf_device_id_t), 0);
}

void VRF_DelayedCallback(uint16_t* counter, void(*callback)()){
	if (*counter == 0){
		return;
	} else if (*counter != 1){
		--(*counter);
	} else {
		*counter = 0;
		callback();
	}
}

void VRF_SetAddress(uesb_address_type_t type, const vrf_address_t* address, 
	bool stop_transmission_hops){
	uesb_set_address(type, address->address);
	DbgStr("Address set to %.02X, %.02X, %.02X, %.02X, %.02X\r\n",
		address->address[0], address->address[1], address->address[2],
		address->address[3], address->address[4]);
	if (type == UESB_ADDRESS_PIPE0){
		vrf_RemoteAddress = *address;
	}
	VRF_CTGenerate(address, &vrf_ChannelTable);
	if (stop_transmission_hops){
		vrf_TransmissionHopsStarted = false;
	}
	vrf_TransmitFailCount = 0;
	vrf_DelayCounter = 0;
}

void VRF_TimeToTicks(uint16_t* ticks, uint16_t time){
	*ticks = (uint16_t)((uint32_t)time * 1000 / 1500);
}

void VRF_Disconnect(){
	vrf_StartDisconnect = true;
}

void VRF_StartPair(bool clearpairings){
	CriticalVar();
	if (clearpairings){
		CriticalEnter();
		VPM_EraseAll();
		CriticalLeave();
	}
	vrf_StartPair = true;
}

void VRF_SetStateWithDiscoveryState(vrf_state_t state){

	VRF_CCMInitializeConfigs(&vrf_StaticKey,
		&vrf_DiscoveryState.challenge);

	DbgStr("VRF_SetStateWithDiscoveryState\r\n");

	VRF_SetAddress(UESB_ADDRESS_PIPE0, &vrf_DiscoveryState.address,
		true);

	vrf_ChannelIndex =
		(uint8_t)((vrf_TimerTicks - vrf_DiscoveryState.target_channel) % 7)+1;

	VRF_SetState(state);
}

uint8_t VRF_GetPairingChannel(vrf_address_t* address){
	register uint8_t channel, i;
	vrf_channel_table_t table;

	channel = VRF_GetChannel();

	VRF_CTGenerate(address, &table);

	for (i = 0; i < 7; ++i){
		if (table.table[i] == channel){
			return i;
		}
	}
	return 0;
}

void VRF_HandleDiscoveryCandidate(vrf_address_t* address,
	vrf_ccm_iv_t* iv0, vrf_ccm_iv_t* challenge, uint8_t rssi){
	if (memcmp(&vrf_DiscoveryState.address, address, sizeof(vrf_address_t)) == 0){
		if (rssi < vrf_DiscoveryState.rssi){
			vrf_DiscoveryState.rssi = rssi;
		}
		DbgStr("PC %.02X, %.02X, %.02X, %.02X, %.02X { %.8X, %.8X } { %.8X, %.8X }\r\n",
			address->address[0], address->address[1], address->address[2], address->address[3],
			address->address[4], iv0->data[0], iv0->data[1], challenge->data[0], challenge->data[1]);//*/
		vrf_DiscoveryState.challenge = *challenge;
	} else if (rssi < vrf_DiscoveryState.rssi){
		vrf_DiscoveryState.address = *address;
		vrf_DiscoveryState.iv0 = *iv0;
		vrf_DiscoveryState.challenge = *challenge;
		vrf_DiscoveryState.rssi = rssi;
		vrf_DiscoveryState.target_channel = 
			vrf_TimerTicks - VRF_GetPairingChannel(address);
		DbgStr("PC %.02X, %.02X, %.02X, %.02X, %.02X { %.8X, %.8X } { %.8X, %.8X }\r\n",
			address->address[0], address->address[1], address->address[2], address->address[3],
			address->address[4], iv0->data[0], iv0->data[1], challenge->data[0], challenge->data[1]);//*/
	}
}

void VRF_OnUpdate(){
	register bool is_first_paired_address;
	register const vrf_address_t* address;
	register uint8_t attempts_remaining;
	vrf_address_t address2;

	switch (vrf_State){
	case vrf_s_pair_begin:{ //{{
		/*DbgStr("Update pair state %u - %u\r\n", vrf_DiscoveryState.rssi, 
			vrf_DiscoveryRSSILimit);*/
		if (vrf_DiscoveryState.rssi < vrf_DiscoveryRSSILimit){
			VRF_SetStateWithDiscoveryState(vrf_s_pair_aes_setup);
		} else if (--vrf_ConnectionAttemptsRemaining != 0){
			if (vrf_DiscoveryRSSILimit < vrf_DiscoveryRSSILimit){
				vrf_DiscoveryRSSILimit += 15;
			}
			VRF_TimeToTicks(&vrf_TicksUntilUpdate, 829);
		} else {
			VRF_SetState(vrf_s_disconnected);
		}
		break;
	} //}}
	case vrf_s_connect_begin:{ //{{
		is_first_paired_address = false;
		address = NULL;
		if (--vrf_ConnectionAttemptsRemaining != 0){
			address = VPM_GetNextPairedAddress(&vrf_PairedKeyIndex);
			if (address == NULL){
				address = VPM_GetFirstPairedAddress(&vrf_PairedKeyIndex);
				is_first_paired_address = true;
			}
		}
		if ((is_first_paired_address || vrf_ConnectionAttemptsRemaining == 0) &&
			vrf_DiscoveryState.rssi != 255){
			VRF_SetStateWithDiscoveryState(vrf_s_connect_send_deviceid);
		} else if (address) {
			//DbgStr("VRF_OnUpdate connect begin\r\n");
			VRF_SetAddress(UESB_ADDRESS_PIPE0, address, true);
			VRF_TimeToTicks(&vrf_TicksUntilUpdate, 74);
		} else {
			VRF_SetState(vrf_s_disconnected);
		}
		break;
	} //}}
	case vrf_s_connect_send_deviceid:{ //{{
		VRF_SetState(vrf_s_disconnected);
		break;
	} //}}
	case vrf_s_pair_aes_setup:{ //{{
		attempts_remaining = vrf_ConnectionAttemptsRemaining;
		VRF_SetState(vrf_s_pair_begin);
		vrf_ConnectionAttemptsRemaining = attempts_remaining;
		break;
	} //}}
	case vrf_s_connect_aes_setup:{ //{{
		VRF_SetState(vrf_s_disconnected);
		break;
	} //}}
	case vrf_s_pair_verify_crypto:{ //{{
		VRF_SetState(vrf_s_pair_begin);
		break;
	} //}}
	case vrf_s_pair_send_key:{ //{{
		attempts_remaining = vrf_ConnectionAttemptsRemaining;
		VRF_SetState(vrf_s_pair_begin);
		vrf_ConnectionAttemptsRemaining = attempts_remaining;
		break;
	} //}}
	case vrf_s_connected:{ //{{
		if (VRF_IsPacketCountNonZero(&vrf_ReceiveConfig.count)){
			address2 = vrf_RemoteAddress;
			address2.address[0] &= ~7;
			//DbgStr("VRF_OnUpdate connected\r\n");
			VRF_SetAddress(UESB_ADDRESS_PIPE0, &address2, false);
			VRF_SetState(vrf_s_connect_send_deviceid);
			//stats_ConnectionTimeouts
		} else {
			VRF_SetState(vrf_s_disconnected);
			//DbgStr("VRF_OnUpdate disconnecting\r\n");
		}
		break;
	} //}}
	default:
		break;
	}
}

void VRF_Update(){
	register uint8_t qitem;
	register vpq_buffer_t* buffer;
	register vpq_item_t* item;
	if (VRF_IsConnected() == false){
		//DbgStr("Not connected!\r\n");
		return;
	}
	if (vrf_HasTXPayload != false &&
		VRF_EXTRACT_TYPE(vrf_TXPayload.data[0]) != vrf_pt_unreliable){
		return;
	}
	qitem = VPQ_Pop(0);
	if (qitem != 0){
		item = VPQ_GetItem(qitem);
		buffer = VPQ_GetBuffer(item->buffer_index);
		if (buffer){
			//DbgStr("Sending %u with %u bytes\r\n", item->packet_type, item->length);
			VRF_Send(item->packet_type,
				buffer->data, item->length);
		}
		VPQ_Release(qitem);
	} else if (vrf_UnreliableDataLength != 0 &&
		vrf_TransmitConfig.count.count[0] != vrf_SuspendUnreliablePacketID){
		VRF_Send(vrf_pt_unreliable, vrf_UnreliableData,
			vrf_UnreliableDataLength);
	}
}


//!NAME
bool VRF_CanSendUnreliableData(){
	return (vrf_UnreliableDataLength != 0) ||
		(vrf_HasTXPayload &&
			VRF_EXTRACT_TYPE(vrf_TXPayload.data[0]) != vrf_pt_unreliable);
}

bool VRF_SetUnreliableData(const void* data, uint8_t length){
	if (length >= 27){
		return false;
	}
	if (!VRF_CanSendUnreliableData()){
		//stats_NotReadyCount
	}
	memcpy(vrf_UnreliableData, data, length);
	vrf_UnreliableDataLength = length;
	return true;
}

void VRF_UpdateRXCountLSB(){
	CriticalVar();
	CriticalEnter();
	if (vrf_State == vrf_s_connected){
		if ((vrf_ReceiveConfig.count.count[0] & 1) != 0){
			vrf_TXPayload.data[0] |= 4;

		} else {
			vrf_TXPayload.data[0] &= ~4;
		}
	}
	CriticalLeave();
}

void VRF_IncrementPacketCount(vrf_packet_count_t* count){
	if (++count->count[0] != 0){
		return;
	}
	if (++count->count[1] != 0){
		return;
	}
	if (++count->count[2] != 0){
		return;
	}
	if (++count->count[3] != 0){
		return;
	}
	count->count[4] = ((count->count[4] + 1) & 0x7F) | (count->count[4] & 0x80);
}


bool VRF_IsPacketCountNonZero(const vrf_packet_count_t* count){
	register uint8_t i;

	for (i = 0; i < 4; ++i){
		if (count->count[i] != 0){
			return true;
		}
	}
	return (count->count[4] & 0x7F) ?
		true : false;
}

uint16_t VRF_GetPacketAirBits(uint8_t length){
	return (9 + length) * 8;
}

static const char* vrf_StateNames[9] = {
	"disconnected",
	"pair begin",
	"pair aes setup",
	"connect begin",
	"connect send device ID",
	"connect aes setup",
	"pair verify crypto",
	"pair send key",
	"connected"
};//*/

void VRF_SetState(vrf_state_t state){
	register const vrf_address_t* address;
	DbgStr("Switching from %s to %s\r\n",vrf_StateNames[vrf_State], vrf_StateNames[state]);
	uint8_t buf[sizeof(vrf_ccm_config_packet_t)];
	vrf_State = state;
	vrf_TicksUntilUpdate = 0;
	vrf_TicksUntilPumpRX = 0;
	memset(&vrf_TXPayload, 0, sizeof(uesb_payload_t));
	vrf_HasTXPayload = false;
	vrf_NextTimeDelta = 0;
	switch (state){
	case vrf_s_disconnected:{ //{{
		VPQ_ReleaseAll(0);
		vrf_ChannelHopsUntilRetransmission = 1;
		break;
	} //}}
	case vrf_s_pair_begin:{ //{{
		DbgLEDEnable(0);
		vrf_DiscoveryRSSILimit = 61;
		vrf_ConnectionAttemptsRemaining = 12;
		VRF_TimeToTicks(&vrf_TicksUntilUpdate, 829);
		VRF_CCMInitializeConfigs(NULL, NULL);
		VRF_SetAddress(UESB_ADDRESS_PIPE0, &vrf_StaticAddress, true);
		VRF_ClearDiscoveryState();
		VRF_SendDeviceID((uint8_t)vrf_pt_unreliable);
		break;
	} //}}
	case vrf_s_connect_begin:{ //{{
		address = VPM_GetFirstPairedAddress(&vrf_PairedKeyIndex);
		if (address != NULL){
			//DbgStr("VRF_SetState connect begin\r\n");
			VRF_SetAddress(UESB_ADDRESS_PIPE0, address, true);
			vrf_ConnectionAttemptsRemaining = 67;
			VRF_TimeToTicks(&vrf_TicksUntilUpdate, 74);
			VRF_ClearDiscoveryState();
			VRF_SendDeviceID(vrf_pt_final);
		} else {
			VRF_SetState(vrf_s_disconnected);
		}
		break;
	} //}}
	case vrf_s_connect_send_deviceid:{ //{{
		VRF_TimeToTicks(&vrf_TicksUntilUpdate, 1500);
		VRF_SendDeviceID(vrf_pt_unreliable);
		break;
	} //}}
	case vrf_s_pair_aes_setup:
	case vrf_s_connect_aes_setup:{ //{{
		((vrf_ccm_config_packet_t*)buf)->address = vrf_RemoteAddress;
		((vrf_ccm_config_packet_t*)buf)->address.address[0] &= ~7;

		SYS_GetDeviceID(&((vrf_ccm_config_packet_t*)buf)->device_id);
		VRF_RNGHWGenerate(&vrf_LocalSessionID, sizeof(vrf_LocalSessionID));
		((vrf_ccm_config_packet_t*)buf)->session_id = vrf_LocalSessionID;
		//DbgStr("Session ID = %.8X\r\n", vrf_LocalSessionID);

		VRF_Send(
			vrf_State == vrf_s_pair_aes_setup ? 1 : 2,
			buf, sizeof(vrf_ccm_config_packet_t));

		VRF_TimeToTicks(&vrf_TicksUntilUpdate, 1000);
		break;
	} //}}
	case vrf_s_pair_verify_crypto:{ //{{
#ifdef VRF_DEVKIT
		VRF_Send(vrf_pt_control, vrf_DevKitPayload, sizeof(vrf_DevKitPayload));
		VRF_TimeToTicks(&vrf_TicksUntilUpdate, 500);
#endif //VRF_DEVKIT
		break;
	} //}}
	case vrf_s_pair_send_key:{ //{{
		VRF_RNGHWGenerate(&vrf_RXKey, sizeof(vrf_RXKey));
		buf[0] = 1;
		memcpy(buf + 1, &vrf_RXKey, sizeof(vrf_RXKey));
		VRF_Send(vrf_pt_control, buf, sizeof(vrf_ccm_key_t) + sizeof(uint8_t));
		VRF_TimeToTicks(&vrf_TicksUntilUpdate, 1000);		
		break;
	} //}}
	case vrf_s_connected:{ //{{
		VPQ_ReleaseAll(0);
		vrf_20000090[0] = vrf_20000090[1] = 0;
		vrf_ChannelHopsUntilRetransmission = 4;
		VRF_TimeToTicks(&vrf_TicksUntilUpdate, 500);
		vrf_TicksUntilPumpRX = 20;
		vrf_InterframeSpacingAdded = 0;

		((vrf_firmware_info_packet_t*)buf)->type = vrf_cp_firmware_timestamp;
		((vrf_firmware_info_packet_t*)buf)->timestamp = VRF_STATIC_TIMESTAMP;
		VPQ_Push(0, buf, sizeof(vrf_firmware_info_packet_t), vrf_pt_control);
		vrf_ConnectNotificationCount = 3;
		break;
	} //}}
	}
}

void VRF_Send(vrf_packet_type_t type, const void* payload, uint8_t length){
	memcpy(vrf_TXPlaintext.payload, payload, length);
	vrf_TXPlaintext.header = (uint8_t)type;
	vrf_TXPlaintext.length = length;
	if (vrf_State >= vrf_s_pair_send_key){
		VRF_IncrementPacketCount(&vrf_TransmitConfig.count);
	}

	VRF_CCMEncrypt(&vrf_TransmitConfig, &vrf_TXCiphertext, &vrf_TXPlaintext);

	VRF_SetTXPayload(vrf_TXCiphertext.header, vrf_TXCiphertext.payload,
		vrf_TXCiphertext.length, vrf_TransmitConfig.count.count[0]);
}

void VRF_SetTXPayload(uint8_t header, const void* payload, uint8_t length,
	uint8_t count_octet0){
	vrf_HasTXPayload = false;
	vrf_TXPayload.pipe = 0;
	vrf_TXPayload.length = length + 1;
	vrf_TXPayload.data[0] = header |
		VRF_MAKE_ID(count_octet0);
	memcpy(vrf_TXPayload.data + 1, payload, length);
	VRF_UpdateRXCountLSB();
	vrf_HasTXPayload = true;
}

//Interframe spacing?
void VRF_sub_30D4(uint8_t attempts){
	register bool adddelay;
	if (attempts != 0){
		if (vrf_TransmissionHopsStarted){
			if (++vrf_DelayCounter < 6){
				adddelay = false;
			} else {
				adddelay = true;
			}
			if (adddelay != false){
				NRF_TIMER0->CC[0] += VRF_GetPacketAirBits(vrf_TXPayload.length) * 230;
				vrf_InterframeSpacingAdded = true;
			}
		} else {
			vrf_DelayCounter = 0;
		}
	} else {
		vrf_DelayCounter = 0;
		vrf_NextTimeDelta = (vrf_InterframeSpacingAdded != false || vrf_State != vrf_s_connected) ?
			1 : 200;
	}
	if (vrf_TransmissionHopsStarted == false){
		vrf_TransmissionHopsStarted = true;
		vrf_TransmissionHops = 0;
	}
	vrf_TransmitFailCount = 0;
}

bool VRF_TrySend(){
	uesb_flush_tx();
	if (vrf_TXPayload.length != 0 &&
		uesb_write_tx_payload(&vrf_TXPayload) == UESB_SUCCESS){
		return true;
	}
	return false;
}

void VRF_OnReceive(uint8_t a, uint8_t b){
	register vrf_connection_config_t* config;
	register const vrf_ccm_key_t* key;
	register vrf_packet_type_t type;
	vrf_address_t pairaddress;
	vrf_ccm_iv_t iv;

	//DbgStr("VRF_OnReceive()\r\n");
	if (uesb_read_rx_payload(&vrf_RXPayload) != 0){
		return;
	}
	if (vrf_RXPayload.length < 1 || vrf_RXPayload.pipe != 0){
		return;
	}
	vrf_LastRSSI = vrf_RXPayload.rssi;
	vrf_RXCiphertext.header = vrf_RXPayload.data[0];
	vrf_RXCiphertext.length = vrf_RXPayload.length - 1;
	vrf_RXCiphertext.s1 = 0;
	memcpy(vrf_RXCiphertext.payload, vrf_RXPayload.data + 1,
		vrf_RXPayload.length - 1);

	switch (vrf_State){
	case vrf_s_pair_begin:{//{{
		DbgLEDEnable(1);
		//DbgStr("Got potential pairing candidate\r\n");
		if (vrf_RXCiphertext.length >= sizeof(vrf_connection_config_t)){
			config = (vrf_connection_config_t*)vrf_RXCiphertext.payload;
			VRF_HandleDiscoveryCandidate(&config->address,
				&config->iv1, &config->challenge, vrf_RXPayload.rssi);
		}
		break;
	}//}}
	case vrf_s_pair_verify_crypto:{//{{
#ifdef VRF_DEVKIT
		if (VRF_DecryptPacket(&vrf_RXPlaintext, &vrf_RXCiphertext)){
			DbgLEDEnable(3);
			VRF_SetState(vrf_s_pair_send_key);
		} else {
			VRF_SetState(vrf_s_pair_begin);
		}
#endif //!VRF_DEVKIT
		break;
	} //}}
	case vrf_s_pair_send_key:{//{
#ifndef VRF_DEKIT
		if (!VRF_DecryptPacket(&vrf_RXPlaintext, &vrf_RXCiphertext)){
			VRF_SetState(vrf_s_pair_begin);
			break;
		}
#endif //!VRF_DEKIT
		pairaddress = vrf_RemoteAddress;
		pairaddress.address[0] &= ~7;
		VPM_Register(&pairaddress, &vrf_RXKey);
		VRF_CCMInitializeConfigs(&vrf_RXKey, &vrf_ReceiveConfig.iv);
		VRF_SetState(vrf_s_connected);
		DbgLEDEnable(2);
		break;
	} //}}
	case vrf_s_connect_begin:{//{{
		if (vrf_RXCiphertext.length >= sizeof(vrf_connection_config_t)){
			config = (vrf_connection_config_t*)vrf_RXCiphertext.payload;
			VRF_HandleDiscoveryCandidate(&config->address,
				&config->iv1, &config->challenge, vrf_RXPayload.rssi);
		}
		break;
	}//}}
	case vrf_s_pair_aes_setup:
	case vrf_s_connect_send_deviceid:
	case vrf_s_connect_aes_setup:{ //{{
		if (vrf_RXCiphertext.length < sizeof(vrf_connection_config_t)){
			return;
		}
		config = (vrf_connection_config_t*)vrf_RXCiphertext.payload;
		if (vrf_State == vrf_s_pair_aes_setup){
			key = &vrf_StaticKey;
		} else {
			key = VPM_GetKey(&config->address);
		}
		if (key == NULL){
			VRF_SetState(vrf_s_disconnected);
		}

		DbgStr("AC %.02X, %.02X, %.02X, %.02X, %.02X { %.8X, %.8X } { %.8X, %.8X }\r\n",
			config->address.address[0], config->address.address[1], config->address.address[2],
			config->address.address[3], config->address.address[4], config->challenge.data[0],
			config->challenge.data[1], config->iv1.data[0], config->iv1.data[1]);

		if (vrf_State != vrf_s_connect_send_deviceid &&
			memcmp(&config->challenge, &vrf_ReceiveConfig.iv, sizeof(vrf_ccm_iv_t)) == 0){
			//DbgStr("VRF_OnReceive aes setup\r\n");
			VRF_SetAddress(UESB_ADDRESS_PIPE0, &config->address, false);
			iv.data[0] = config->challenge.data[0];
			iv.data[1] = vrf_LocalSessionID;
			VRF_CCMInitializeConfigs(key, &iv);
#ifdef VRF_DEVKIT
			VRF_SetState(vrf_State == vrf_s_pair_aes_setup ?
				vrf_s_pair_verify_crypto : vrf_s_connected);
#else //VRF_DEVKIT
			VRF_SetState(vrf_State == vrf_s_pair_aes_setup ?
				vrf_s_pair_send_key : vrf_s_connected);
#endif //!VRF_DEVKIT
		} else if (vrf_State == vrf_s_connect_aes_setup){
			VRF_SetState(vrf_s_connect_send_deviceid);
		} else {
			VRF_CCMInitializeConfigs(key, &config->challenge);
			VRF_SetState(vrf_State == vrf_s_pair_aes_setup ?
				vrf_s_pair_aes_setup : vrf_s_connect_aes_setup);
		}
		break;
	} //}}
	case vrf_s_connected:{ //{{
		if (vrf_RXCiphertext.length == 0){
			//DbgStr("Connected receive ACK\r\n");
			VRF_HandleACK(vrf_RXCiphertext.header);
		} else if (VRF_DecryptPacket(&vrf_RXPlaintext, &vrf_RXCiphertext)){
			VRF_HandleACK(vrf_RXCiphertext.header);
			type = (vrf_packet_type_t)(vrf_RXPlaintext.header & 3);
			if (type == vrf_pt_control){
				//DbgStr("Connected receive ctrl\r\n");
				VRF_HandleControlPacket(vrf_RXPlaintext.payload,
					vrf_RXPlaintext.length);
			} else if (type == vrf_pt_continuation){
				DbgLEDEnable(3);
				//DbgStr("Connected receive continuation\r\n");
				API_OnReceive(vrf_RXPlaintext.payload, vrf_RXPlaintext.length,
					false);
			} else if (type == vrf_pt_final){
				DbgLEDEnable(3);
				//DbgStr("Connected receive final\r\n");
				API_OnReceive(vrf_RXPlaintext.payload, vrf_RXPlaintext.length,
					true);
			}
		} else {
			DbgLEDDisable(0);
		}
		if (VRF_IsPacketCountNonZero(&vrf_ReceiveConfig.count)){
			if (vrf_ConnectNotificationCount){
				--vrf_ConnectNotificationCount;
				API_OnConnect();
			}
			VRF_TimeToTicks(&vrf_TicksUntilUpdate, 500);
			vrf_TicksUntilPumpRX = 20;
		} else {
			//DbgStr("Zero RX count\r\n");
		}
		break;
	} //}}
	default:
		break;
	}
}

void VRF_OnTransmitFail(uint8_t a){


	if (vrf_State == vrf_s_connected){
		//DbgStr("Transmission failed\r\n");
	}

	if (vrf_TransmitFailCount == 0xFF){
		++vrf_TransmitFailCount;
	}
	//stats_TransmissionFailures
	//stats_ChannelTransmissionFailures

	if (vrf_TransmitFailCount == 7){
		//DbgStr("TFC hop\r\n");
		NVIC_DisableIRQ(TIMER0_IRQn);
		VRF_ChannelHop();
		NVIC_EnableIRQ(TIMER0_IRQn);
	} else if (vrf_TransmitFailCount == 14){
		NVIC_DisableIRQ(TIMER0_IRQn);
		vrf_TransmissionHopsStarted = false;
		vrf_ChannelHopCountdown = 7;
		if (VRF_IsConnected()){
			vrf_ChannelIndex = (uint8_t)(VRF_RNGGenerate() % 7);	
		}
		NVIC_EnableIRQ(TIMER0_IRQn);
	} else if (vrf_TransmitFailCount >= 14 &&
		(vrf_TransmitFailCount % 49) == 0){

		NVIC_DisableIRQ(TIMER0_IRQn);
		vrf_ChannelHopCountdown = 7;
		if (VRF_IsConnected()){
			vrf_ChannelIndex = (uint8_t)(VRF_RNGGenerate() % 7);
		}
		NVIC_EnableIRQ(TIMER0_IRQn);
	}
	vrf_ReadyForTransmission = true;
}

void VRF_OnTransmitSuccess(uint8_t a, uint8_t b){
	uint32_t attempts;
	if (vrf_State == vrf_s_connected){
		//DbgStr("Transmit success!\r\n");
	}
	//stats_TransmissionSuccesses
	attempts = 1;
	uesb_get_tx_attempts(&attempts);

	if (vrf_State != vrf_s_pair_begin){
		VRF_sub_30D4((uint8_t)attempts - 1);
	}
	switch (vrf_State){
	case vrf_s_pair_begin:
	case vrf_s_pair_aes_setup:
	case vrf_s_connect_begin:
	case vrf_s_connect_send_deviceid:
	case vrf_s_connect_aes_setup:
		break;
	default:
		if (VRF_EXTRACT_TYPE(vrf_TXPayload.data[0]) == vrf_pt_unreliable &&
			VRF_EXTRACT_ID(vrf_TXPayload.data[0]) != (vrf_SuspendUnreliablePacketID & 7)){
			vrf_HasTXPayload = false;
		}

	}
	vrf_ReadyForTransmission = true;
}

void VRF_EventHandler(){
	uint32_t interrupts;
	uesb_get_clear_interrupts(&interrupts);
	if (interrupts & UESB_INT_TX_SUCCESS_MSK){
		VRF_OnTransmitSuccess(0, 0);
	}
	if (interrupts & UESB_INT_TX_FAILED_MSK){
		uesb_flush_tx();
		VRF_OnTransmitFail(0);
	}
	if (interrupts & UESB_INT_RX_DR_MSK){
		VRF_OnReceive(0, 0);
	}
}


void TIMER0_IRQHandler(){
	if (NRF_TIMER0->EVENTS_COMPARE[1] != 0){
		NRF_TIMER0->EVENTS_COMPARE[1] = 0;
	}
	if (NRF_TIMER0->EVENTS_COMPARE[0] == 0){
		return;
	}
	NRF_TIMER0->EVENTS_COMPARE[0] = 0;
	NRF_TIMER0->CC[0] = 1500 - vrf_NextTimeDelta;
	vrf_NextTimeDelta = 0;
	if (vrf_TransmissionHopsStarted && vrf_State != vrf_s_pair_begin){
		//DbgStr("THS hop\r\n");
		VRF_ChannelHop();
	} else if (vrf_ChannelHopCountdown != 0){
		--vrf_ChannelHopCountdown;
	} else {
		vrf_ChannelHopCountdown = 7;
		//DbgStr("CHC hop\r\n");
		VRF_ChannelHop();
	}
	if (!vrf_ReadyForTransmission){
		++vrf_ChannelHopCountdown;
		return;
	}
	VRF_DelayedCallback(&vrf_TicksUntilUpdate, VRF_OnUpdate);
	uesb_set_rf_channel(VRF_GetChannel());

	if (vrf_State != vrf_s_disconnected && (
		(vrf_TransmissionHops == 0 && vrf_HasTXPayload != false) ||
		(vrf_TransmissionHopsStarted == false) ||
		vrf_State < vrf_s_connected)){
		VRF_TrySend();
		if (vrf_State == vrf_s_pair_verify_crypto){
			//DbgStr("TrySend on channel %u\r\n", VRF_GetChannel());
		}
		if (uesb_start_tx() == UESB_SUCCESS){
			vrf_ReadyForTransmission = false;
		} else {
			DbgStr("uesb_start_tx failed\r\n");
		}
	} else {
		//stats_NotReadyCount
	}
	VRF_DelayedCallback(&vrf_TicksUntilPumpRX, VRF_PumpRX);
	VRF_DelayedCallback(&vrf_TicksUntilOtherEvent, VRF_OtherEvent);
	if (vrf_StartPair){
		vrf_StartConnect = vrf_StartPair = false;
		VRF_SetState(vrf_s_pair_begin);
	} else if (vrf_StartConnect){
		vrf_StartConnect = false;
		VRF_SetState(vrf_s_connect_begin);
	} else if (vrf_StartDisconnect){
		vrf_StartDisconnect = false;
		VRF_SetState(vrf_s_disconnected);
	}
	++vrf_TimerTicks;
}

#pragma weak API_OnConnect
void API_OnConnect(){
}
#pragma weak API_OnDisconnect
void API_OnDisconnect(){
}
#pragma weak API_OnReceive
void API_OnReceive(const void* data, uint8_t length,
	bool is_final){
}

/*

P0	0x0080
P0.D0	0
P0.D1	1
P0.D2	2
P0.D3	3
P0.D4	4
P0.D5	5


SP	0x0081
DPL	0x0082
DPH	0x0083
DPL1	0x0084
DPH1	0x0085
;A		0x0086
PCON	0x0087
PCON.idle	0
PCON.stop	1
PCON.gf0	2
PCON.gf1	3
PCON.pmw	4
PCON.gf2	5
PCON.gf3	6
PCON.gf4	7

TCON	0x0088
TCON.it0	0
TCON.ie0	1
TCON.it1	2
TCON.ie1	3
TCON.tr0	4
TCON.tf0	5
TCON.tr1	6
TCON.tf1	7
TMOD	0x0089
TL0		0x008A
TL1		0x008B
TH0		0x008C
TH1		0x008D
CKCON	0x008E
;A	0x008F


RFCON	0x0090
;A	0x0091
DPS	0x0092
;A	0x0093
P0DIR	0x0094
P0DIR.D0DIR	0
P0DIR.D1DIR	1
P0DIR.D2DIR	2
P0DIR.D3DIR	3
P0DIR.D4DIR	4
P0DIR.D5DIR	5
P0ALT	0x0095
P0ALT.P0_P3_A	0
P0ALT.P0_P3_B	1
P0ALT.P4_A	4
P0ALT.P4_B	5
P0ALT.P5_A	6
P0ALT.P5_B	7
;A	0x0096
;A	0x0097

S0CON	0x0098
S0BUF	0x0099
;A	0x009A
;A	0x009B
;A	0x009C
;A	0x009D
;A	0x009E
;A	0x009F


USBCON	0x00A0
;A	0x00A1
;A	0x00A2
CLKCTL	0x00A3
PWRDWN	0x00A4
WUCONF	0x00A5
INTEXP	0x00A6
;A	0x00A7

IEN0	0x00A8
IP0	0x00A9
S0RELL	0x00AA
REGXH	0x00AB
REGXL	0x00AC
REGXC	0x00AD
;A	0x00AE
;A	0x00AF


;A	0x00B0
RSTRES	0x00B1
SMDAT	0x00B2
SMCTRL	0x00B3
;A	0x00B4
TICKDV	0x00B5
;A	0x00B6
;A	0x00B7
		
IEN1	0x00B8
IP1	0x00B9
S0RELH	0x00BA
;A	0x00BB
SSCONF	0x00BC
SSDAT	0x00BD
SSSTAT	0x00BE
;A	0x00BF


IRCON	0x00C0
CCEN	0x00C1
CCL1	0x00C2
_44CH1	0x00C3
CCL2	0x00C4
CCH2	0x00C5
CCL3	0x00C6
CCH3	0x00C7
		
T2CON	0x00C8
P0EXP	0x00C9
P0EXP.D0EXP	0
P0EXP.D1EXP	1
P0EXP.D2EXP	2
P0EXP.D3EXP	3
P0EXP.D4EXP	4
P0EXP.D5EXP	5
CRCL	0x00CA
CRCH	0x00CB
TL2	0x00CC
TH2	0x00CD
;A	0x00CE
;A	0x00CF


PSW	0x00D0
;A	0x00D1
;A	0x00D2
;A	0x00D3
;A	0x00D4
;A	0x00D5
;A	0x00D6
;A	0x00D7
		
WDCON	0x00D8
USBSLP	0x00D9
;A	0x00DA
;A	0x00DB
;A	0x00DC
;A	0x00DD
;A	0x00DE
;A	0x00DF



ACC	0x00E0
;A	0x00E1
;A	0x00E2
;A	0x00E3
;A	0x00E4
RFDAT	0x00E5
RFCTL	0x00E6
;A	0x00E7

AESCS	0x00E8
MD0	0x00E9
MD1	0x00EA
MD2	0x00EB
MD3	0x00EC
MD4	0x00ED
MD5	0x00EE
ARCON	0x00EF



B	0x00F0
AESKIN	0x00F1
AESIV	0x00F2
AESD	0x00F3
;A	0x00F4
AESIA1	0x00F5
AESIA2	0x00F6
;A	0x00F7
		
FSR	0x00F8
FPCR	0x00F9
FCR	0x00FA
;A	0x00FB
;A	0x00FC
;A	0x00FD
;A	0x00FE
;A	0x00FF

*/