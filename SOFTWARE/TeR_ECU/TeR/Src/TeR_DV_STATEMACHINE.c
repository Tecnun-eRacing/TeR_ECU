/*
 * TeR_DV_STATEMACHINE.C
 *
 *  Created on: Jan 31, 2026
 *      Author: pieroebs
 *
 *      Lógica de todo lo relacionado con la obtención y configuración del AS
 *
 *      -> obtención de estado del AS
 *      -> controlar como se cambia de estado en el AS
 *      -> permite o no el uso de actuadores
 *      -> configuración del modo de conduccion del DV
 *      -> request de safety Line (SDC en fsg)
 *
 *
 */
#include "TeR_DV_STATEMACHINE.h"
void permatask();
void dv_stateLoop();
const static uint32_t task_period = 5;
persist_t ready_time; // contar el tiempo que estamos en AS_READY
persist_t res_k2; // contar el tiempo que "res k2" ha estado en 1
uint8_t DV_SM_SL_RELAY;

extern osTimerId_t as_allowed_timerHandle;
extern osTimerId_t as_emergency_beep_timerHandle;
uint32_t emergency_beep_count; // contador de beeps de emergencia
uint8_t ext_TS;

/*
 * This function is called when the timer reaches its autoreload value
 * Use: To delay any requests from the driverless computer for a period of time after entering AS_DRIVING (required by the rules)
 *
 */
void as_allowed_timer_callback(void *argument) {
	TeR.status.as_allowed = 1;
}

/*
 * This timer executes the beep the buzzer at a frequency that i forgot for arround 8-10 seconds
 * The timer gets called every 200ms, when an internal count value is reaached, the timer gets deactivated
 *
 *
 */
void as_emergency_beep_timer_callback(void *argument) {
	HAL_GPIO_TogglePin(DOUT1_GPIO_Port, DOUT1_Pin);
	emergency_beep_count++;
	if (emergency_beep_count >= 24) {
		emergency_beep_count = 0;
		HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_RESET); // asegurar que la hemos apagado
		osTimerStop(as_emergency_beep_timerHandle);
	}
}

//K2 o K3 se pueden usar como go signal (fsg 2026)
/*
 * This function gets the state of the DV statemachine, it follows the rule T 14.8
 * Nothing crazy, just follow the rules stated in T14.8
 *
 */
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
		} else if ((ter_bpps_bpps_decode(TeR.bpps.bpps) >= TeR.config.r2_d_brake)) {
			dv_state = AS_READY;
		}
	} else {
		dv_state = AS_OFF;
	}
	return dv_state;
}

/*
 *
 * Thread that controls the execution of the DV pipeline
 *
 *
 * */

void dvStateMachine(void *argument) {
	uint32_t currentTick = osKernelGetTickCount();
	for(;;){
		currentTick+=task_period;
		osDelayUntil(currentTick);
		dv_stateLoop();
	}
}
/*
 * Main statemachine of the driverless module
 *
 * -> gets the state
 * -> executes state changes transitions configurations
 * -> executes permanents tasks associated with the current state
 *
 * */
void dv_stateLoop(){
	dv_state_t prevState = TeR.dv_system_status.as_status;
	dv_state_t state = get_dv_state();
	uint8_t stateChanged = state != prevState ? 1 : 0;
	if (stateChanged) { // Handles setup conditions for the new state
		switch (state) {
		case AS_OFF:
			break;
		case AS_READY:
			ready_time = 0; // reseteamos ready_time
			// setup dv driving mode & other relevant configurations
			TeR.config.driving_mode = TER_ECU_CONFIG_DRIVING_MODE_DV_TORQUE_REQUEST_CHOICE;
			TeR.config.regen_mode = TER_ECU_CONFIG_REGEN_MODE_FREE_CHOICE;
			TeR.config.regen_enable = TER_ECU_CONFIG_REGEN_ENABLE_ENABLE_CHOICE;
			TeR.config.trq_limit = 20; //hardcodeado aqui porque tengo miedo
			break;

		case AS_DRIVING:
			osTimerStart(as_allowed_timerHandle, 3000); // retrasar la asignacion de as_allowed a 3 segundos en el futuro
			break;

		case AS_EMERGENCY:
			osTimerStart(as_emergency_beep_timerHandle, 200); // start a periodic beep timer that lasts 10 seconds
			break;

		default:
			break;
		}
		TeR.dv_system_status.as_status = state;
	}

	if (state != AS_DRIVING) {
		TeR.dv_system_status.steering_state =
		TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
		TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE;
		TeR.status.as_allowed = 0;
	}

	switch (state) {
	case AS_OFF: //TODO checkear estado de ASB system alive or not ( si ha pasado el self test vamos)
		// T 14.4 very important
		if (TeR.status.asms) {
			if ((TeR.config.dv_mission_req
					!= TER_ECU_CONFIG_DV_MISSION_REQ_MANUAL_CHOICE)
					&& (ter_bpps_bpps_decode(TeR.bpps.bpps)
							>= TeR.config.r2_d_brake)) { // asms puesto, la mision NO es manual, hay presión de freno
				set_sl_request(SL_DV, 1);
				if (ext_TS && TeR.status.state == RDY2PRECH) { // TODO lectura boton TS externo + SL cerrada
					easyCommand(TER_COMMAND_CMD_PRECHARGE_DV_CHOICE); // enviamos request de precarga DV
				}
				if (TeR.status.state == PRECHARGED) { // pedimos self check (si falla el ASB se encargará de abrir el SDC)
					TeR.asb_ebs_state_req.state_req =
					TER_ASB_EBS_STATE_REQ_STATE_REQ_SELF_CHECK_CHOICE;
					TeR.asb_redundancy_req.state_req =
					TER_ASB_REDUNDANCY_REQ_STATE_REQ_SELF_CHECK_CHOICE;
				}
			} else {
				set_sl_request(SL_DV, 0);
			}

		} else if ((TeR.config.dv_mission_req
				== TER_ECU_CONFIG_DV_MISSION_REQ_MANUAL_CHOICE)
				&& (TeR.asb_status.asb_ebs_state
						== TER_ASB_STATUS_ASB_EBS_STATE_DEACTIVATED_CHOICE)
				&& TeR.asb_status.asb_redundancy_state
						== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_DEACTIVATED_CHOICE) {
			set_sl_request(SL_DV, 1);
		} else {
			set_sl_request(SL_DV, 0);
		}
		break;

	case AS_READY:
		if (!checkPersistance(&ready_time, 0, 5000)) { // han pasado al menos 5 segundos? desde que entramos por primera vez a este punto?
			if (!checkPersistance(&res_k2, TeR.res_pdo_tx.k2, 500)) { // le han dado al k2 del RES durante mas de 500 millis ?
				easyCommand(TER_COMMAND_CMD_READY2_DRIVE_DV_CHOICE); // enviamos request de paso a r2d DV
			}
		}
		break;

	case AS_DRIVING:
		if (TeR.status.as_allowed) {
			//request steering
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_AVAILABLE_CHOICE;
			//requests steering TODO validar entradas, OJO, peligrosisimo
			TeR.steer_actuator_set_position.actuator_position =
					TeR.dv_dynamic_req_1.steer_angle_req;
			//requests freno (dv espero que no bloquees freno a 50kmH, de esto no te puedo salvar)
			TeR.asb_brake_req.brake = TeR.dv_dynamic_req_2.asb_brake_req;

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


void permatask(){

}

