/*
 * TeR_REFRI.c
 *
 *  Created on: Jun 30, 2024
 *      Author: ozuba
 */

#include "TeR_REFRI.h"

persist_t REFRI;

void refriManager() {

	if(TeR.status.state >= PRECHARGED){
	if (checkPersistance(REFRI,TeR.status.refri,1000)) { //Si la refri está encendida decidimos si apagarla
		if(TeR.tempsRight.pwr_stg_temp_deg_c < OFF_THRESHOLD){
			easyCommand(TER_COMMAND_CMD_TOGGLE_REFRI_CHOICE);
		}
	} else { // La refri esta apagada la encedemos?
		if(TeR.tempsRight.pwr_stg_temp_deg_c > ON_THRESHOLD){
			easyCommand(TER_COMMAND_CMD_TOGGLE_REFRI_CHOICE);
		}
	}
	}
}
