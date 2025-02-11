/*
 * TeR_IMU.h
 *
 *  Created on: Jan 9, 2025
 *      Author: eracing
 */

#ifndef INC_TER_INERTIAL_H_
#define INC_TER_INERTIAL_H_
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include <string.h>
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "asm330lhh_reg.h"
#include "lis3mdl_reg.h"
#include "imu_filter.h"

#define TASK_PERIOD 10 //100hz

void inertial(void *argument);


void configIMU(void);
void configMAG(void);

static int32_t imu_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t imu_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t mag_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t mag_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);


#endif /* INC_TER_INERTIAL_H_ */
