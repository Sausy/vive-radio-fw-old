#include "vrf.h"

#include <nrf.h>
#include "vpq.h"
#include <boards.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include "nrf_delay.h"
#include "cfg.h"

#include <simple_uart.h>

char TmpBuffer[27 * 3 + 1];

const char Lut[16] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
};

typedef enum {
	fw_timestamp = 0,
	hwid_revision = 1,
	hwid_lot = 2,
	hwid_crc = 3, //CRC32 of some HWID related stuff
	vrc_version = 4,
	radion_version = 5,
} vrc_attribute_t;

const uint8_t s_attribs[] = {
	0x83, 0x05 * 6,
	0x00, 0x00, 0x00, 0x00, 0x00, //Timestamp
	0x01, 0x00, 0x00, 0x00, 0x00, //HWID revision 
	0x02, 0x00, 0x00, 0x00, 0x00, //HWID lot
	0x03, 0x00, 0x00, 0x00, 0x00, //HWID CRC
	0x04, 0x05, 0x00, 0x00, 0x00, //VRC version
	0x05, 0x06, 0x00, 0x00, 0x00, //Radio version
}; //Empty attribute response

uint16_t	cfg_bytes_remain;


typedef struct {
	uint32_t	timestamp;
	uint32_t	boardmodel;
	char		vendor[16];
	char		name[16];
	uint32_t	field_28;
	uint32_t	field_2c;
	uint32_t	field_30;
} vrc_firmware_version_t;

static const vrc_firmware_version_t myfwversion = {
	1477875695,
	129,
	{ "abcd" },
	{ "fghj" },
	1,
	2,
	3,
};

void API_OnReceive(const void* data, uint8_t length,
	bool is_final){
	register uint32_t i, j;
	register const uint8_t* u8data;
	register uint16_t len;
	u8data = (const uint8_t*)data;
	uint8_t buf[2];

	if (length == 1 && u8data[0] == 0x5){
		//VPQ_PushLarge(0, &myfwversion, sizeof(myfwversion));
	} else if (length == 1 && u8data[0] == 0x10){
		cfg_bytes_remain = sizeof(s_Cfg);
		buf[0] = (cfg_bytes_remain >> 8) & 0xFF;
		buf[1] = cfg_bytes_remain & 0xFF;

		VPQ_PushLarge(0, buf, sizeof(uint16_t));
	} else if (length == 1 && u8data[0] == 0x11){
		len = cfg_bytes_remain > 0x3E ? 0x3E : cfg_bytes_remain;
		VPQ_PushLarge(0, s_Cfg + sizeof(s_Cfg) - cfg_bytes_remain, len);
		cfg_bytes_remain -= len;
	} else if (length >= 1 && u8data[0] == 0x83){
		DbgStr("Sending attributes\r\n");
		VPQ_PushLarge(0, s_attribs, sizeof(s_attribs));
	} /*else*/ {
		for (i = 0, j = 0; i < length; ++i){
			TmpBuffer[j++] = Lut[(u8data[i] >> 4) & 0xF];
			TmpBuffer[j++] = Lut[(u8data[i]) & 0xF];
			TmpBuffer[j++] = ' ';
		}
		TmpBuffer[j] = 0;
		DbgStr("Received %u bytes: %s\r\n", length, TmpBuffer);
	}
}

static uint8_t LEDS[4] = LEDS_LIST;


/*void DbgStr(const char* str){
	simple_uart_putstring((const uint8_t*)str);
	simple_uart_putstring((const uint8_t*)"\r\n");
}*/

void DbgLEDEnable(uint8_t idx){
	LEDS_ON(1 << LEDS[idx]);
}

void DbgLEDDisable(uint8_t idx){
	LEDS_OFF(1 << LEDS[idx]);
}

void DbgLEDDisableAll(){
	DbgLEDDisable(0);
	DbgLEDDisable(1);
	DbgLEDDisable(2);
	DbgLEDDisable(3);
}

void DbgLEDInit(){
	LEDS_CONFIGURE(LEDS_MASK);
}


void DbgStr(const char* fmt, ...){
	char lBuf[256];
	va_list va;
	va_start(va, fmt);
	vsprintf(lBuf, fmt, va);
	va_end(va);
	simple_uart_putstring((const uint8_t*)lBuf);//*/
}

static bool test = false;


void API_OnConnect(){
	if (!test){
		test = true;
	}
}

const static uint8_t TestData[2] = {
	64, //Time16
	0x1E << 3, //Type/flags
};

void TestTX(){
	if (VRF_CanSendUnreliableData()){
		VRF_SetUnreliableData(TestData, sizeof(TestData));
	}
}

int main(){
	NRF_POWER->TASKS_CONSTLAT = 1;
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	do {
	} while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, false);
	DbgLEDInit();
	DbgLEDDisableAll();
	DbgStr("Init\r\n");
	VPQ_Init();
	VRF_Initialize();
	VRF_Disconnect();
	nrf_delay_ms(500);
	VRF_StartPair(true);
	for (;;){
		TestTX();
		VRF_Update();
		__WFE();

	}
	return 0;
}