/*
 * TeR_SL.h
 *
 *  Created on: Feb 6, 2026
 *      Author: pieroebs
 */

#ifndef INC_TER_SL_H_
#define INC_TER_SL_H_
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "main.h"
typedef enum{
	SL_DV,
	SL_SCS,
}sl_request_t;
#define MAX_TASKS 2
void set_sl_request(sl_request_t request, uint32_t value);
uint8_t can_close_relay(void);






#endif /* INC_TER_SL_H_ */
