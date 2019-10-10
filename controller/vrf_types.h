#ifndef VRF_TYPES_H
#define VRF_TYPES_H

#include <stdint.h>

#define VRF_PAYLOAD_SIZE		27
#define VRF_HEADER_LENGTH		5

#define VRF_TYPE_SHIFT		0
#define VRF_TYPE_MASK		3

#define VRF_PARITY_SHIFT	2
#define VRF_PARITY_MASK		1

#define VRF_ID_SHIFT		4
#define VRF_ID_MASK			7

#define VRF_UNKNOWN_MASK	1
#define VRF_UNKNOWN_SHIFT	7

#define VRF_EXTRACT_TYPE(hdr) \
	((vrf_packet_type_t)(((hdr) >> VRF_TYPE_SHIFT) & VRF_TYPE_MASK))

#define VRF_EXTRACT_PARITY(hdr) \
	(((hdr) >> VRF_PARITY_SHIFT) & VRF_PARITY_MASK)

#define VRF_EXTRACT_ID(hdr) \
	(((hdr) >> VRF_ID_SHIFT) & VRF_ID_MASK)

#define VRF_MAKE_ID(ID) \
	(((ID) &  VRF_ID_MASK) << VRF_ID_SHIFT)

typedef enum{
	vrf_s_disconnected = 0,
	vrf_s_pair_begin,
	vrf_s_pair_aes_setup,
	vrf_s_connect_begin,
	vrf_s_connect_send_deviceid,
	vrf_s_connect_aes_setup,
	vrf_s_pair_verify_crypto,
	vrf_s_pair_send_key,
	vrf_s_connected,
} vrf_state_t;


typedef enum {
	//Used for stuff like button states
	vrf_pt_unreliable = 0,
	//Used internally
	vrf_pt_control,
	//Used for large transmissions
	vrf_pt_continuation,
	vrf_pt_final,
} vrf_packet_type_t;

#pragma pack(push, 1)

typedef struct {
	uint8_t	address[5];
} vrf_address_t;

typedef struct {
	uint8_t	key[16];
} vrf_ccm_key_t;

typedef struct  {
	uint32_t	data[2];
} vrf_ccm_iv_t;

typedef struct {
	uint8_t	count[5];
} vrf_packet_count_t;

typedef struct {
	vrf_ccm_key_t		key;
	vrf_packet_count_t	count;
	uint8_t				ignored[3];
	uint8_t				direction;
	vrf_ccm_iv_t		iv;
} vrf_ccm_config_t;

typedef struct {
	uint8_t	header;
	uint8_t	length;
	uint8_t	s1;
	uint8_t	payload[VRF_PAYLOAD_SIZE+2]; //+2 for the checksum
} vrf_ccm_ciphertext_t;

typedef struct {
	uint8_t		header;
	uint8_t		length;
	uint8_t		s1;
	uint8_t		payload[VRF_PAYLOAD_SIZE];
} vrf_ccm_plaintext_t;


typedef struct {
	uint8_t	table[7];
} vrf_channel_table_t;

typedef vrf_ccm_iv_t vrf_device_id_t;

typedef struct {
	vrf_address_t	address;
	vrf_device_id_t	device_id;
	uint32_t		session_id;
} vrf_ccm_config_packet_t;

typedef struct {
	vrf_ccm_iv_t	challenge;
	vrf_address_t	address;
	vrf_ccm_iv_t	iv1;
} vrf_connection_config_t;
#pragma pack(pop)

#pragma pack(push, 4)
typedef struct {
	uint8_t		type;
	uint32_t	timestamp;
} vrf_firmware_info_packet_t;
#pragma pack(pop)


typedef struct {
	vrf_address_t	address;
	vrf_ccm_iv_t	iv0;
	vrf_ccm_iv_t	challenge;
	uint8_t			rssi;
	uint32_t		target_channel;
} vrf_discovery_state_t;

typedef enum {
	vrf_cp_0 = 0,
	vrf_cp_aes_key,
	vrf_cp_2,
	vrf_cp_max_transmission_hops,
	vrf_cp_firmware_timestamp,
} vrf_control_packet_t;

#endif //!VRF_TYPES_H

