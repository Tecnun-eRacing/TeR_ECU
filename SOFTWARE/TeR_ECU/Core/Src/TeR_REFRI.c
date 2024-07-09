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

		if(rTemp > ON_THRESHOLD || lTemp > ON_THRESHOLD){
			switchCommand(TER_COMMAND_CMD_SWITCH_REFRI_CHOICE, TER_COMMAND_ONOFF_ON_CHOICE);
		}else if((rTemp < OFF_THRESHOLD && lTemp < OFF_THRESHOLD)){
			switchCommand(TER_COMMAND_CMD_SWITCH_REFRI_CHOICE, TER_COMMAND_ONOFF_OFF_CHOICE);

		}
}
