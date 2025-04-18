/*
 * TeR_SERVOS.h
 *
 *  Created on: Apr 18, 2025
 *      Author: piero
 */

#ifndef INC_TER_SERVOS_H_
#define INC_TER_SERVOS_H_
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
typedef struct{
	uint8_t angle;
	uint8_t offset;
}flap_t;


void setAngle(uint8_t angle,uint8_t channel);

#endif /* INC_TER_SERVOS_H_ */
