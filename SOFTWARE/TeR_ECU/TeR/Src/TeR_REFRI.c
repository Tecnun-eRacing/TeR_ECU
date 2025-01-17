/*
 * TeR_REFRI.c
 *
 *  Created on: Jun 30, 2024
 *      Author: ozuba
 */

#include "TeR_REFRI.h"

persist_t REFRI;

void refriManager() {
	//Gather inverter IGBT temps in degrees
	int16_t rInvTemp = inverter_emcu_state_4_right_pwr_stg_temp_deg_c_decode(
			TeR.tempsRight.pwr_stg_temp_deg_c);
	int16_t lInvTemp = inverter_emcu_state_4_left_pwr_stg_temp_deg_c_decode(
			TeR.tempsLeft.pwr_stg_temp_deg_c);

	//Gather Motor temps in degrees
	int16_t rMotorTemp =
			inverter_emcu_state_4_right_e_machine_temp_2_deg_c_decode(
					TeR.tempsRight.e_machine_temp_2_deg_c);
	int16_t lMotorTemp =
			inverter_emcu_state_4_left_e_machine_temp_2_deg_c_decode(
					TeR.tempsLeft.e_machine_temp_2_deg_c);
	if (((rInvTemp > ON_THRESHOLD || lInvTemp > ON_THRESHOLD)
			|| (rMotorTemp > ON_THRESHOLD || lMotorTemp > ON_THRESHOLD))
					&& !TeR.status.refri) {
		switchCommand(TER_COMMAND_CMD_SWITCH_REFRI_CHOICE,
		TER_COMMAND_ONOFF_ON_CHOICE);
	} else if ((rInvTemp < OFF_THRESHOLD) && (lInvTemp < OFF_THRESHOLD)
			&& (rMotorTemp < OFF_THRESHOLD) && (lMotorTemp < OFF_THRESHOLD)
			&& TeR.status.refri) {
		switchCommand(TER_COMMAND_CMD_SWITCH_REFRI_CHOICE,
		TER_COMMAND_ONOFF_OFF_CHOICE);
	}
}
