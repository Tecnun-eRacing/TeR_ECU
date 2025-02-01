/*
 * TeR_GPS.h
 *
 *  Created on: Jan 17, 2025
 *      Author: ozuba
 */

#ifndef INC_TER_GPS_H_
#define INC_TER_GPS_H_
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include "ubx.h"

//IO abstraction
uint8_t gps_read(uint8_t *dest, size_t size);
uint8_t gps_write(uint8_t *src, size_t size);

void gps(void *argument);

#endif /* INC_TER_GPS_H_ */
