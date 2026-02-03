/*
 * TeR_DV_STATEMACHINE.C
 *
 *  Created on: Jan 31, 2026
 *      Author: pieroebs
 */
#include "TeR_DV_STATEMACHINE.h"
persist_t ready_time; // contar el tiempo que estamos en AS_READY
persist_t res_k2;

extern osTimerId_t dv_allowed_timerHandle;
/*
 * This function is called when the timer reaches its autoreload value
 * Uses: To delay any torque requests from the driverless computer for a period of time
 *
 */
void dv_allowed_timer_callback(void *argument){
	TeR.status.as_allowed = 1;
}

//K2 o K3 se pueden usar como go signal (fsg 2026)
dv_state_t get_dv_state() { // fsg 2026 T 14.8
	TeR.status.asms = 0; // leer de un GPIO o del CAN
//todo hacer que el command de precharge no vaya si asms = 1 o crear un command diferente
//todo necesidad de relé de SL de la ECU para abrir el EBS 0 hacer request de activar EBS
	dv_state_t dv_state = AS_OFF;
	if (TeR.asb_status.asb_ebs_state
			== TER_ASB_STATUS_ASB_EBS_STATE_ACTIVATED_CHOICE) {
		if ((TeR.dv_info.mission_status
				== TER_DV_INFO_MISSION_STATUS_FINISHED_CHOICE)
				&& (TeR.wheelInfo.speed == 0)) {
			if (TeR.res_pdo_tx.e_stop_1 || TeR.res_pdo_tx.e_stop_2) { // sl open at res
				dv_state = AS_EMERGENCY;
			} else { // sl not open at RES
				dv_state = AS_FINISHED;
			}
		} else {
			dv_state = AS_EMERGENCY;
		}
	} else if ((TeR.dv_info.mission
			!= TER_DV_INFO_MISSION_MANUAL_CHOICE)
			&& (TeR.status.asms)
			&& (TeR.asb_status.asb_ebs_state
					== TER_ASB_STATUS_ASB_EBS_STATE_INITIAL_CHECK_PASSED_CHOICE)
			&& (TeR.asb_status.asb_redundancy_state
					== TER_ASB_STATUS_ASB_EBS_STATE_INITIAL_CHECK_PASSED_CHOICE)
			&& (TeR.status.state >= PRECHARGED)) {
		if (TeR.status.r2_d == 1) {
			dv_state = AS_DRIVING;
		} else if (TeR.asb_status.asb_redundancy_state
				== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_ACTIVATED_CHOICE) {
			dv_state = AS_READY;
		} else {
			dv_state = AS_OFF;
		}
	}
	return dv_state;
}

void dv_statemachine() {
	dv_state_t prevState = TeR.dv_system_status.as_status;
	dv_state_t state = get_dv_state();
	uint8_t stateChanged = state != prevState ? 1 : 0;
	if (stateChanged) { // Handles setup conditions for the new state
		switch (state) {
		case AS_OFF:
			break;
		case AS_READY:
			ready_time = 0; // reseteamos ready_time TODO configuracion de modos en statemachine
			TeR.config.driving_mode =
					TER_ECU_CONFIG_DRIVING_MODE_DV_TORQUE_REQUEST_CHOICE;
			TeR.config.regen_mode = TER_ECU_CONFIG_REGEN_MODE_FREE_CHOICE;
			break;

		case AS_DRIVING:

			osTimerStart(dv_allowed_timerHandle, 3000); // delay 3s para que el AS pueda pedir cualquier cosa
			break;
		default:

			break;
		}
		TeR.dv_system_status.as_status = state;
	}
	switch (state) {
	case AS_OFF:
		if (TeR.status.asms == 1) {
			//TeR.asb_redundancy_req.state_req = TER_ASB_REDUNDANCY_REQ_STATE_REQ_ENABLED_CHOICE;
			if ((TeR.asb_status.asb_ebs_state
					== TER_ASB_STATUS_ASB_EBS_STATE_INITIAL_CHECK_PASSED_CHOICE)
					&& (TeR.asb_status.asb_redundancy_state
							== TER_ASB_STATUS_ASB_EBS_STATE_INITIAL_CHECK_PASSED_CHOICE)) {
				if (0) { // boton TS externo only
					easyCommand(TER_COMMAND_CMD_PRECHARGE_DV_CHOICE); // enviamos request de precarga
				}
			}
			break;

			case AS_READY:
			if (!checkPersistance(&ready_time, 0, 5000)) { // han pasado al menos 5 segundos?
				if (checkPersistance(&res_k2,TeR.res_pdo_tx.k2 == 1,500)) { // le han dado al k2 del RES?
					easyCommand(TER_COMMAND_CMD_READY2_DRIVE_DV_CHOICE); // enviamos request de paso a r2d
				}
			}
		}
		break;

	case AS_DRIVING:
		// TODO gestion del steering
		break;

	case AS_EMERGENCY:
		break;

	default:

		break;
	}

}

void ami_task() {
	switch (1) {
	case AS_OFF:
		break;
	case AS_READY:
		break;
	case AS_DRIVING:
		break;
	case AS_EMERGENCY:
		break;
	default:
		break;
	}

}
