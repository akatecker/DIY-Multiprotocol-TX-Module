/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// HStar D43 1:43 drift car ("SP Racing Speed" transmitter, PCB YY-HX430-TX), see issue #1194
//  - 1 Mbps GFSK, MSB first, on air: AA 55 CF CF CF CF | P0..P39
//  - P0 bits 0..6 = 1100111, bits 7..316 PN9 whitened (x^9+x^5+1, seed 0x1FF, like TI CC1101)
//  - bits 7..12 zero, then 19 bytes in a convolutional code (7,5): byte 0 plain, bytes 1..18 4x4 transposed per 16 bits
//  - bytes: 0 type, 1..5 ID, 6/7 throttle, 8/9 steering, 10 gyro, 11+14 trim, 13 buttons, 15 next channel,
//    16 XOR of 1..15, 17/18 CRC-16 0x1021 init 0x9A55 over 0..16
//  - linked TX alternates two channels, frame starts every 7.68/7.68/4.60 ms
//  - the car answers 0.52 ms after a frame on +0.75 MHz: type 0x82, the 5 ID bytes, AA 55 55 ...
// ID bytes 1..2 = TX ID from module ID and receiver number, bytes 3..5 = car ID. The car ID is learned at bind:
// the nRF24 of the 4-in-1 receives the car's answer to the search frames on 2441 MHz. Channel pair of one original TX.

#if defined(HSTAR_CYRF6936_INO)

#include "iface_cyrf6936.h"
#include "iface_nrf24l01.h"

#define HSTAR_PACKET_LEN		45		// 55 CF CF CF CF + P0..P39
#define HSTAR_PREAMBLE_LEN	1
#define HSTAR_CH_A			21		// 2421 MHz
#define HSTAR_CH_B			59		// 2459 MHz
#define HSTAR_CH_SEARCH		40		// 2440 MHz: search / bind
#define HSTAR_BIND_COUNT	1500	// ~10 s of search frames after pressing bind
#define HSTAR_CYCLE			150		// frames per ~1 s
#define HSTAR_SEARCH_FRAMES	9		// ~60 ms of search frames per second ...
#define HSTAR_SEARCH_CYCLES	10		// ... during the first ~10 s: a blinking car rebinds by itself
#define HSTAR_STEER_RIGHT_MAX	833
#define HSTAR_STEER_LEFT_MAX	939
#define HSTAR_GAS_FWD_MAX		667
#define HSTAR_GAS_REV_MAX		353
#define HSTAR_EPS_MAX			246		// gyro (EPS knob) 0..100 %
#define HSTAR_TRIM			0x1ED	// steering trim of the original transmitter (10 bit)
#define HSTAR_TYPE			0x80	// source byte 0 of transmitter frames
#define HSTAR_CRC_INIT		0x9A55
#define HSTAR_PRESS_MS		200		// button press like the original transmitter
#define HSTAR_RELEASE_MS		200		// pause between two queued presses
#define HSTAR_HOLD_MS		1000	// switch on for longer = toggle switch: switching it off is another press
#define HSTAR_BTN_ARM_MS		500		// ignore switch changes right after init
#define HSTAR_RX_CH			41		// 2441 MHz: answer to the search frames is on 2440.75 MHz
#define HSTAR_RX_LEN		32		// P0..P31 of the answer after CF CF CF CF

static const uint16_t HSTAR_gap_us[3] = { 7680, 7680, 4600 };

// Light (CH5), speed (CH6): one press when switched on, another one when switched off after more than 1 s
static uint8_t HSTAR_btn_prev;
static uint8_t HSTAR_btn_pending[2];
static bool HSTAR_btn_down[2];
static uint32_t HSTAR_btn_time[2];
static uint32_t HSTAR_btn_arm;
static uint32_t HSTAR_btn_on_since[2];

static uint8_t HSTAR_cycle_no;		// position in the ~1 s cycle (search frames first)
static uint8_t HSTAR_search_cycles;	// cycles with search frames left
static uint8_t HSTAR_pair_no;		// alternates the channel pair
static uint8_t HSTAR_id[5];			// TX ID (bytes 1..2), car ID (bytes 3..5)
static uint8_t HSTAR_learn_id[3];
static uint8_t HSTAR_learn_cnt;
static bool HSTAR_rx_on;			// antenna on the nRF24 since the last search frame

// Search frame (2440 MHz) bytes 3..15 of the original transmitter
static const uint8_t HSTAR_search_data[13] = { 0x55, 0x55, 0x03, 0xCC, 0xCC, 0x55, 0xA7, 0x03, 0x2E, 0x03, 0x55, 0x55, 0x55 };

// -1000..+1000 from a channel, small dead band around the center
static int16_t __attribute__((unused)) HSTAR_channel(uint8_t num)
{
	int32_t val = ((int32_t)Channel_data[num] - (CHANNEL_MAX_100 + CHANNEL_MIN_100) / 2) * 1000 / ((CHANNEL_MAX_100 - CHANNEL_MIN_100) / 2);
	if(val > 1000)
		val = 1000;
	else if(val < -1000)
		val = -1000;
	if(val > -20 && val < 20)
		val = 0;
	return val;
}

static uint8_t __attribute__((unused)) HSTAR_buttons()
{
	uint8_t sw = (CH5_SW ? 0x01 : 0) | (CH6_SW ? 0x02 : 0);
	uint32_t ms = millis();
	uint8_t out = 0;
	if((int32_t)(ms - HSTAR_btn_arm) < 0)
	{	// channels may not be valid yet; a switch already on counts as toggle switch
		HSTAR_btn_prev = sw;
		HSTAR_btn_on_since[0] = HSTAR_btn_on_since[1] = ms - HSTAR_HOLD_MS;
		return 0;
	}
	for(uint8_t i = 0; i < 2; i++)
	{
		uint8_t on = (sw >> i) & 1, was = (HSTAR_btn_prev >> i) & 1;
		if(on && !was)
		{	// pressed or switched on
			HSTAR_btn_on_since[i] = ms;
			if(HSTAR_btn_pending[i] < 4)
				HSTAR_btn_pending[i]++;
		}
		else if(!on && was && (int32_t)(ms - HSTAR_btn_on_since[i]) >= HSTAR_HOLD_MS && HSTAR_btn_pending[i] < 4)
			HSTAR_btn_pending[i]++;	// toggle switch flipped back
		if(HSTAR_btn_down[i])
		{
			if((int32_t)(ms - HSTAR_btn_time[i]) >= 0)
			{
				HSTAR_btn_down[i] = false;
				HSTAR_btn_time[i] = ms + HSTAR_RELEASE_MS;
			}
		}
		else if(HSTAR_btn_pending[i] && (int32_t)(ms - HSTAR_btn_time[i]) >= 0)
		{
			HSTAR_btn_pending[i]--;
			HSTAR_btn_down[i] = true;
			HSTAR_btn_time[i] = ms + HSTAR_PRESS_MS;
		}
		if(HSTAR_btn_down[i])
			out |= i ? 0x04 : 0x08;	// bit 3 = light, bit 2 = speed
	}
	HSTAR_btn_prev = sw;
	return out;
}

// Source bytes 0..15 in d[] -> check bytes, coding, whitening -> packet
static void __attribute__((unused)) HSTAR_encode(uint8_t *d)
{
	d[16] = 0;
	for(uint8_t i = 1; i < 16; i++)
		d[16] ^= d[i];
	uint16_t crc = HSTAR_CRC_INIT;
	for(uint8_t k = 0; k < 17; k++)
	{
		crc ^= (uint16_t)d[k] << 8;
		for(uint8_t i = 0; i < 8; i++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	d[17] = crc >> 8;
	d[18] = crc;

	// Frame P0..P39: fixed bits 0..6, zero bits 7..12, code bits from 13
	uint8_t *frame = &packet[5];
	memset(frame, 0, 40);
	frame[0] = 0xCE;
	uint8_t s1 = 0, s2 = 0, code[16];
	for(uint8_t blk = 0; blk < 19; blk++)
	{
		for(uint8_t b = 0; b < 8; b++)
		{
			uint8_t u = (d[blk] >> (7 - b)) & 1;
			code[2 * b] = u ^ s1 ^ s2;		// g = 7
			code[2 * b + 1] = u ^ s2;		// g = 5
			s2 = s1;
			s1 = u;
		}
		for(uint8_t i = 0; i < 16; i++)
			if(code[blk ? (i & 3) * 4 + (i >> 2) : i])	// byte 0 is not transposed
			{
				uint16_t pos = 13 + blk * 16 + i;
				frame[pos >> 3] |= 0x80 >> (pos & 7);
			}
	}
	// PN9 whitening of bits 7..316
	uint16_t pn9 = 0x1FF;
	for(uint16_t pos = 7; pos < 317; pos++)
	{
		if(pn9 & 1)
			frame[pos >> 3] ^= 0x80 >> (pos & 7);
		pn9 = (pn9 >> 1) | (((pn9 ^ (pn9 >> 5)) & 1) << 8);
	}

	// Sync part before P0, the CYRF adds the first preamble byte
	packet[0] = 0x55;
	packet[1] = packet[2] = packet[3] = packet[4] = 0xCF;

	// CYRF sends LSB first
	for(uint8_t i = 0; i < HSTAR_PACKET_LEN; i++)
		packet[i] = bit_reverse(packet[i]);
}

static void __attribute__((unused)) HSTAR_build_packet(uint8_t rf)
{
	uint8_t d[19];
	d[0] = HSTAR_TYPE;
	memcpy(&d[1], HSTAR_id, 5);
	// Throttle CH1: neutral 0x400, forward = magnitude, reverse = 0x400 + magnitude
	int16_t val = HSTAR_channel(CH1);
	uint16_t field = val > 0 ? (uint16_t)((int32_t)val * HSTAR_GAS_FWD_MAX / 1000) : 0x400 | (uint16_t)((int32_t)-val * HSTAR_GAS_REV_MAX / 1000);
	d[6] = field;
	d[7] = field >> 8;
	// Steering CH2: neutral 0x400, left = magnitude, right = 0x400 + magnitude
	val = HSTAR_channel(CH2);
	field = val < 0 ? (uint16_t)((int32_t)-val * HSTAR_STEER_LEFT_MAX / 1000) : 0x400 | (uint16_t)((int32_t)val * HSTAR_STEER_RIGHT_MAX / 1000);
	d[8] = field;
	d[9] = field >> 8;
	// Gyro CH3
	d[10] = ((int32_t)HSTAR_channel(CH3) + 1000) * HSTAR_EPS_MAX / 2000;
	d[11] = (uint8_t)HSTAR_TRIM;
	d[12] = 0;
	d[13] = HSTAR_buttons();
	d[14] = HSTAR_TRIM >> 8;
	// Channel of the next frame
	d[15] = rf ? HSTAR_CH_A : HSTAR_CH_B;
	HSTAR_encode(d);
}

static void __attribute__((unused)) HSTAR_build_search()
{
	uint8_t d[19];
	d[0] = HSTAR_TYPE;
	d[1] = HSTAR_id[0];
	d[2] = HSTAR_id[1];
	memcpy(&d[3], HSTAR_search_data, 13);
	HSTAR_encode(d);
}

// nRF24: raw 1 Mbps receive of the car's answer, sync CF CF CF CF, no CRC
static void __attribute__((unused)) HSTAR_rx_init()
{
	NRF24L01_Initialize();
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x02);			// 4 bytes address
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t*)"\xCF\xCF\xCF\xCF", 4);
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, HSTAR_RX_LEN);
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, HSTAR_RX_CH);
	NRF24L01_SetTxRxMode(RX_EN);
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));	// RX without CRC
	NRF24L01_FlushRx();
	prev_power = 0xFD;									// shared with the CYRF power setting
}

// Answer P0..P31 (P1..P31 if the sync matched one byte late) -> car ID, stored after 2 identical answers
static void __attribute__((unused)) HSTAR_learn(uint8_t *p)
{
	uint8_t f[HSTAR_RX_LEN];
	if(p[0] == 0xCF && p[1] == 0xF8)
		memcpy(f, p, HSTAR_RX_LEN);
	else if(p[0] == 0xF8 && p[1] == 0x6F)
	{
		f[0] = 0xCF;
		memcpy(&f[1], p, HSTAR_RX_LEN - 1);
	}
	else
		return;
	// De-whiten and decode bytes 0..13 (frame bits 13..236)
	uint8_t d[14], s1 = 0, s2 = 0, air[16];
	uint16_t pn9 = 0x1FF;
	for(uint8_t i = 7; i < 13; i++)
		pn9 = (pn9 >> 1) | (((pn9 ^ (pn9 >> 5)) & 1) << 8);
	for(uint8_t blk = 0; blk < 14; blk++)
	{
		for(uint8_t i = 0; i < 16; i++)
		{
			uint16_t pos = 13 + blk * 16 + i;
			air[i] = ((f[pos >> 3] >> (7 - (pos & 7))) & 1) ^ (pn9 & 1);
			pn9 = (pn9 >> 1) | (((pn9 ^ (pn9 >> 5)) & 1) << 8);
		}
		uint8_t byte = 0;
		for(uint8_t m = 0; m < 16; m += 2)
		{
			uint8_t g7 = air[blk ? (m & 3) * 4 + (m >> 2) : m];
			uint8_t g5 = air[blk ? ((m + 1) & 3) * 4 + ((m + 1) >> 2) : m + 1];
			uint8_t u = g5 ^ s2;
			if(g7 != (u ^ s1 ^ s2))
				return;									// bit error
			s2 = s1;
			s1 = u;
			byte = (byte << 1) | u;
		}
		d[blk] = byte;
	}
	// Answer to our search frame: type 80/82, our TX ID, filler AA 55 55 ...
	if((d[0] & 0xFD) != 0x80 || d[1] != HSTAR_id[0] || d[2] != HSTAR_id[1] || d[6] != 0xAA)
		return;
	for(uint8_t i = 7; i < 14; i++)
		if(d[i] != 0x55)
			return;
	if(HSTAR_learn_cnt && memcmp(HSTAR_learn_id, &d[3], 3) == 0)
	{	// same car ID twice: store it and end the bind
		memcpy(&HSTAR_id[2], &d[3], 3);
		for(uint8_t i = 0; i < 3; i++)
			eeprom_write_byte((EE_ADDR)(HSTAR_EEPROM_OFFSET + RX_num * 3 + i), d[3 + i]);
		bind_counter = 1;
	}
	else
	{
		memcpy(HSTAR_learn_id, &d[3], 3);
		HSTAR_learn_cnt = 1;
	}
}

uint16_t HSTAR_callback()
{
	#ifdef MULTI_SYNC
		telemetry_set_input_sync(6653);		// 19.96 ms for 3 frames
	#endif
	if(HSTAR_rx_on)
	{	// answer of the car to the last search frame?
		if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
		{
			uint8_t p[HSTAR_RX_LEN];
			NRF24L01_ReadPayload(p, HSTAR_RX_LEN);
			NRF24L01_WriteReg(NRF24L01_07_STATUS, _BV(NRF24L01_07_RX_DR));
			HSTAR_learn(p);
		}
		rf_switch(SW_CYRF);
		HSTAR_rx_on = false;
	}
	bool search;
	if(bind_counter)
	{	// bind: search frames only
		search = true;
		if(--bind_counter == 0)
		{
			BIND_DONE;
			NRF24L01_SetTxRxMode(TXRX_OFF);
		}
	}
	else
		search = HSTAR_search_cycles && HSTAR_cycle_no < HSTAR_SEARCH_FRAMES;
	if(++HSTAR_cycle_no >= HSTAR_CYCLE)
	{
		HSTAR_cycle_no = 0;
		if(HSTAR_search_cycles)
			HSTAR_search_cycles--;
	}
	if(search)
	{
		CYRF_ConfigRFChannel(HSTAR_CH_SEARCH);
		HSTAR_build_search();
	}
	else
	{
		uint8_t rf = HSTAR_pair_no++ & 1;	// alternate 2421 / 2459 every frame
		CYRF_ConfigRFChannel(rf ? HSTAR_CH_B : HSTAR_CH_A);
		HSTAR_build_packet(rf);
	}
	CYRF_GFSK1M_SetPower();
	delayMicroseconds(270);					// wait for the RF channel to settle
	CYRF_WriteRegister(CYRF_01_TX_LENGTH, HSTAR_PACKET_LEN);
	CYRF_GFSK1M_SendPayload(packet, HSTAR_PACKET_LEN);
	if(search && bind_counter)
	{	// antenna to the nRF24 as soon as the frame is sent, the answer follows ~0.15 ms later
		for(uint16_t t = 0; t < 1000 && !(CYRF_ReadRegister(CYRF_04_TX_IRQ_STATUS) & 0x02); t++);	// TXC
		rf_switch(SW_NRF);
		HSTAR_rx_on = true;
	}
	uint16_t gap = HSTAR_gap_us[hopping_frequency_no];
	if(++hopping_frequency_no >= 3)
		hopping_frequency_no = 0;
	return gap;
}

void HSTAR_init()
{
	bind_counter = IS_BIND_IN_PROGRESS ? HSTAR_BIND_COUNT : 0;
	CYRF_GFSK1M_Init(HSTAR_PACKET_LEN, HSTAR_PREAMBLE_LEN);
	// TX ID like the originals (byte 1 below 0x20); the low 6 bits come straight from the ID,
	// so all 64 receiver numbers give different IDs
	uint32_t id = ((uint32_t)rx_tx_addr[0] << 24) | ((uint32_t)rx_tx_addr[1] << 16) | ((uint16_t)rx_tx_addr[2] << 8) | rx_tx_addr[3];
	uint32_t h = id >> 6;
	h ^= h >> 13;
	HSTAR_id[0] = h % 0x1F + 1;
	HSTAR_id[1] = (uint8_t)(h << 6) | (id & 0x3F);
	for(uint8_t i = 0; i < 3; i++)						// car ID learned at bind
		HSTAR_id[2 + i] = eeprom_read_byte((EE_ADDR)(HSTAR_EEPROM_OFFSET + RX_num * 3 + i));
	HSTAR_learn_cnt = 0;
	HSTAR_rx_on = false;
	if(bind_counter)
		HSTAR_rx_init();
	hopping_frequency_no = 0;
	HSTAR_cycle_no = 0;
	HSTAR_search_cycles = HSTAR_SEARCH_CYCLES;
	HSTAR_pair_no = 0;
	HSTAR_btn_arm = millis() + HSTAR_BTN_ARM_MS;
	for(uint8_t i = 0; i < 2; i++)
	{
		HSTAR_btn_pending[i] = 0;
		HSTAR_btn_down[i] = false;
		HSTAR_btn_time[i] = HSTAR_btn_arm;
	}
}

#endif
