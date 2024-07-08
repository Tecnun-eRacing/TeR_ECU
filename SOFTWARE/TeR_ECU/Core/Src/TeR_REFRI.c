/*
 * TeR_REFRI.c
 *
 *  Created on: Jun 30, 2024
 *      Author: ozuba
 */

#include "TeR_REFRI.h"

persist_t REFRI;

void refriManager() {
	int16_t rTemp = inverter_emcu_state_4_right_pwr_stg_temp_deg_c_decode(TeR.tempsRight.pwr_stg_temp_deg_c);
	int16_t lTemp = inverter_emcu_state_4_left_pwr_stg_temp_deg_c_decode(TeR.tempsLeft.pwr_stg_temp_deg_c);

	if(TeR.status.state >= PRECHARGED){
	if (checkPersistance(&REFRI,TeR.status.refri,1000)) { //Si la refri está encendida decidimos si apagarla
		if(rTemp < OFF_THRESHOLD || lTemp < OFF_THRESHOLD){
			easyCommand(TER_COMMAND_CMD_TOGGLE_REFRI_CHOICE);
		}
	} else { // La refri esta apagada la encedemos?
		if(rTemp > ON_THRESHOLD || lTemp > ON_THRESHOLD){
			easyCommand(TER_COMMAND_CMD_TOGGLE_REFRI_CHOICE);
		}
	}
	}
}
