/*
 * ubx.h
 *
 *  Created on: Jan 29, 2025
 *      Author: ozuba
 */

/*Library workflow is the following
 - Setup ubx_device comms abstraction
 - Send ubx request with class + id + payload(Optional)
 - Receive ubx, payload field is a union of all posible payloads
 */

#ifndef UBLOX_GPS_INC_UBX_H_
#define UBLOX_GPS_INC_UBX_H_
#include "ubx_msgs.h"
#include <stdint.h>
#include <string.h>

//ubx_device struct handles write and read interfaces for HAL independance
typedef struct {
	uint8_t (*write)(uint8_t *src, size_t size); //Takes info and sends to device return success
	uint8_t (*read)(uint8_t *dest, size_t size); //Requests N bytes from device returns success
} ubx_device_t;



uint16_t checksum(uint8_t* buffer,size_t size); //Ubx fletcher checksum algorithm
uint8_t send_ubx(ubx_device_t* device,uint8_t class, uint8_t id, uint8_t* payload,uint16_t p_size);
uint8_t read_ubx(ubx_device_t* device,uint8_t* dest,size_t p_size); //returns payload of given size
#endif /* UBLOX_GPS_INC_UBX_H_ */
