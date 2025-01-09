/*
 * TeR_IMU.h
 *
 *  Created on: Jan 9, 2025
 *      Author: eracing
 */

#ifndef INC_TER_IMU_H_
#define INC_TER_IMU_H_
#include "stm32f4xx_hal.h"
#include "asm330lhh_reg.h"
#include "cmsis_os2.h"
#include <string.h>
#include "i2c.h"

void imu(void *argument);



#endif /* INC_TER_IMU_H_ */
