/*
 * ubx.c
 *
 *  Created on: Jan 29, 2025
 *      Author: ozuba
 */

#include "ubx.h"

uint16_t checksum(uint8_t *buffer, size_t size) {
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	for (int i = 0; i < size; i++) {
		ck_a = ck_a + buffer[i];
		ck_b = ck_b + ck_a;
	}
	return ((uint16_t) ck_b << 8) | (uint16_t) ck_a; //return checksum
}

uint8_t send_ubx(ubx_device_t *device, uint8_t class, uint8_t id,
		uint8_t *payload, uint16_t p_size) {
	uint32_t length = 6 + p_size + 2;
	uint8_t buffer[length]; //VLA with Preambles class, id, length and checksum;
	buffer[0] = 0xb5;
	buffer[1] = 0x62;
	buffer[2] = class;
	buffer[3] = id;
	//Payload
	memcpy(&buffer[4], &p_size, sizeof(uint16_t)); //Copy two bytes for length
	memcpy(&buffer[6], payload, p_size); //Copy the payload
	//Calculate and put checksum
	uint16_t sum = checksum(&buffer[2], sizeof(buffer) - 4);
	memcpy(&buffer[6 + p_size], &sum, sizeof(sum)); //Copy two bytes for length
	//send with the callback function
	return device->write(buffer, length); //Write all
}
uint8_t read_ubx(ubx_device_t *device, uint8_t *dest, size_t p_size) {
	uint32_t length = 6 + p_size + 2;
	uint8_t buffer[length]; //VLA with Preambles class, id, length and checksum;
	if (device->read(buffer, length)) {
		return -1; //return error
	}
	//handle checksum
	if(checksum(&buffer[2], sizeof(buffer) - 4)){
		return -2; //checksum error
	}
	//if all is okay copy the ubx to dest
	memcpy(dest,&buffer[6],p_size);
	return 0;
}

