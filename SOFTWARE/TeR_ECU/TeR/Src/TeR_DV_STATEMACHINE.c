/*
 * TeR_DV_STATEMACHINE.C
 *
 *  Created on: Jan 31, 2026
 *      Author: pieroebs
 */
#include "TeR_DV_STATEMACHINE.h"
persist_t ready_time; // contar el tiempo que estamos en AS_READY
persist_t res_k2; // contar el tiempo que "res k2" ha estado en 1

extern osTimerId_t as_allowed_timerHandle;
extern osTimerId_t as_emergency_beep_timerHandle;
uint32_t emergency_beep_count; // contador de beeps de emergencia
uint8_t as_emergency_triggered; // si hemos entrado alguna vez en as emergency, para dejarlo latcheado ahí

/*
 * This function is called when the timer reaches its autoreload value
 * Uses: To delay any torque requests from the driverless computer for a period of time
 *
 */
void as_allowed_timer_callback(void *argument) {
	TeR.status.as_allowed = 1;
}
void as_emergency_beep_timer_callback(void *argument) {
	HAL_GPIO_TogglePin(DOUT1_GPIO_Port, DOUT1_Pin);
	emergency_beep_count++;
	if (emergency_beep_count >= 24) {
		emergency_beep_count = 0;
		HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_RESET);
		osTimerStop(as_emergency_beep_timerHandle);
	}
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
	} else if ((TeR.dv_info.mission != TER_DV_INFO_MISSION_MANUAL_CHOICE)
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
		}
	} else {
		dv_state = AS_OFF;
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
			//TODO pedir checkup
			break;
		case AS_READY:
			ready_time = 0; // reseteamos ready_time

			break;

		case AS_DRIVING:
			osTimerStart(as_allowed_timerHandle, 3000); // delay 3s the AS system control allowance for steering and torque actuation
			break;

		case AS_EMERGENCY:
			osTimerStart(as_emergency_beep_timerHandle, 200); // start a beep timer that lasts 10 seconds
			break;
		default:
			break;
		}
		TeR.dv_system_status.as_status = state;
	}

	if (state != AS_DRIVING) {
		TeR.dv_system_status.steering_state =
		TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
		TeR.status.as_allowed = 0;
	}
	switch (state) {
	case AS_OFF: //TODO checkear estado de ASB system alive or not
		// T 14.4
		if (TeR.status.asms) {
			if ((TeR.config.dv_mission_req
					!= TER_ECU_CONFIG_DV_MISSION_REQ_MANUAL_CHOICE)
					&& (ter_bpps_bpps_decode(TeR.bpps.bpps)
							>= TeR.config.r2_d_brake)) { // asms puesto y la mision no es manual (se ha elegido misión)
				//SL close request todo
				if (0 && TeR.status.state == RDY2PRECH) { // TODO lectura boton TS externo + SL cerrada
					easyCommand(TER_COMMAND_CMD_PRECHARGE_DV_CHOICE); // enviamos request de precarga DV
				}
				if (TeR.status.state == PRECHARGED) { // pedimos self check (si falla el ASB se encargará de abrir el SDC)
					//TODO monitorizar que la presión de freno se mantiene siempre positiva
					TeR.asb_ebs_state_req.state_req =
					TER_ASB_EBS_STATE_REQ_STATE_REQ_SELF_CHECK_CHOICE;
					TeR.asb_redundancy_req.state_req =
					TER_ASB_REDUNDANCY_REQ_STATE_REQ_SELF_CHECK_CHOICE;
				}
			} else {
				//todo open sdc
			}

		} else if ((TeR.config.dv_mission_req
				== TER_ECU_CONFIG_DV_MISSION_REQ_MANUAL_CHOICE)
				&& (TeR.asb_status.asb_ebs_state
						== TER_ASB_STATUS_ASB_EBS_STATE_DEACTIVATED_CHOICE)
				&& TeR.asb_status.asb_redundancy_state
						== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_DEACTIVATED_CHOICE) {
			//SL close request TODO
		}
		else{
			//todo open sdc
		}
		break;

	case AS_READY:
		if (!checkPersistance(&ready_time, 0, 5000)) { // han pasado al menos 5 segundos? desde que entramos por primera vez a este punto?
			if (!checkPersistance(&res_k2, TeR.res_pdo_tx.k2, 500)) { // le han dado al k2 del RES durante mas de 500 millis ? (por seguridad y esquizofrenia mia)
				easyCommand(TER_COMMAND_CMD_READY2_DRIVE_DV_CHOICE); // enviamos request de paso a r2d DV
			}
		}

		break;

	case AS_DRIVING:
		// TODO gestion del steering
		if (TeR.status.as_allowed) {
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_AVAILABLE_CHOICE;
			//mover steering
			TeR.steer_actuator_set_position.actuator_position =
					TeR.dv_dynamic_req_1.steer_angle_req;
		} else {
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
		}
		break;

	case AS_EMERGENCY:
		//algoimportate todo
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
