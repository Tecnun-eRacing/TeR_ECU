/*
 * TeR_DV_STATEMACHINE.h
 *
 *  Created on: Jan 31, 2026
 *      Author: pieroebs
 */

#ifndef INC_TER_DV_STATEMACHINE_H_
#define INC_TER_DV_STATEMACHINE_H_
#include "TeR_STATEMACHINE.h"
#include "TeR_SL.h"
void as_allowed_timer_callback(void *argument);
typedef enum{
	AS_OFF = 1, // para que coincida con el DBC (mira la value table si no te cuadra esto)
	AS_READY,
	AS_DRIVING,
	AS_EMERGENCY,
	AS_FINISHED
}dv_state_t;





#endif /* INC_TER_DV_STATEMACHINE_H_ */
