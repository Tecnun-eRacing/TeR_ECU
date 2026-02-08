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
void set_steer_angle(int32_t angle);
void unsafe_set_steer_angle(int32_t angle);
const static uint32_t task_period = 5;
persist_t ready_time; // contar el tiempo que estamos en AS_READY
persist_t res_k2; // contar el tiempo que "res k2" ha estado en 1


extern osTimerId_t as_allowed_timerHandle;
extern osTimerId_t as_emergency_beep_timerHandle;
uint32_t emergency_beep_count; // contador de beeps de emergencia
uint8_t ext_TS;

/*
 * This function is called when the SW timer reaches its autoreload value
 * Use: To delay any requests from the driverless computer for a period of time after entering AS_DRIVING (required by the rules)
 * Important, DO NOT block (osDelay for example) in any timer callback, read the manual
 *
 */
void as_allowed_timer_callback(void *argument) {
	if(TeR.dv_system_status.as_status == AS_DRIVING){ // imaginate el caso en el que entras en driving, se lanza el timer, y antes de 3 segundos hay fallo, esto previene jittering (estupidez pero por si acaso)
	TeR.status.as_allowed = 1;
	TeR.dv_system_status.steering_state =
	TER_DV_SYSTEM_STATUS_STEERING_STATE_AVAILABLE_CHOICE;
	}
}

/*
 * This timer executes the beep the buzzer at a frequency that i forgot for around 8-10 seconds
 * The timer gets called every 200ms, when an internal count value is reached, the timer gets deactivated
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
	//TeR.status.asms = 0; // leer de un GPIO o del CAN, comentado para testing con debugger
//todo hacer que el command de precharge no vaya si asms = 1 o crear un command diferente
//todo necesidad de relé de SL de la ECU para abrir el EBS 0 hacer request de activar EBS
	dv_state_t dv_state = AS_OFF;
	if ((TeR.asb_status.asb_ebs_state
			== TER_ASB_STATUS_ASB_EBS_STATE_ACTIVATED_CHOICE)
			|| (TeR.asb_status.asb_redundancy_state
					== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_ACTIVATED_CHOICE)) {
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
					== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_INITIAL_CHECK_PASSED_CHOICE)
			&& (TeR.status.state >= PRECHARGED)) { // >= precharged ya que a partir de este estado el TS está activo, que es lo que requiere la norma
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
	for (;;) {
		currentTick += task_period;
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
 * -> ensures transitions are performed as required by FSG RULES
 * (5s delay as_ready for as_drivind & 3s delay as driving to accept request from dv)
 * -> ensures driverless computer can perform a stable maneuver (o como se escriba) when entering AS EMERGENCY
 * -> CONTROLS THE SL REQUEST OF THE DV STATE MACHINE
 * -> And many other things, check the code
 *
 * */
void dv_stateLoop() {
	dv_state_t prevState = TeR.dv_system_status.as_status;
	dv_state_t state = get_dv_state();
	uint8_t stateChanged = state != prevState ? 1 : 0;
	if (stateChanged) { // Handles setup conditions for the new state
		switch (state) {
		case AS_OFF:
			TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE;
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
			TeR.status.as_allowed = 0;
			break;
		case AS_READY:
			TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE;
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
			TeR.status.as_allowed = 0;
			ready_time = 0; // reseteamos ready_time

			// setup dv driving mode & other relevant configurations for the driverless computer
			TeR.config.driving_mode =
			TER_ECU_CONFIG_DRIVING_MODE_DV_TORQUE_REQUEST_CHOICE;
			TeR.config.regen_mode = TER_ECU_CONFIG_REGEN_MODE_FREE_CHOICE;
			TeR.config.regen_enable = TER_ECU_CONFIG_REGEN_ENABLE_ENABLE_CHOICE;
			TeR.config.trq_limit = 20; // todo quitar en un futuro
			break;

		case AS_DRIVING:
			osTimerStart(as_allowed_timerHandle, 3000); // retrasar la asignacion de as_allowed=1 a 3 segundos en el futuro (cosas normativa)
			break;

		case AS_EMERGENCY:
			TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE;
			TeR.status.as_allowed = 0; // disable AS to make torque requests
			osTimerStart(as_emergency_beep_timerHandle, 200); // start a periodic beep timer that lasts 10 seconds for AS_EMERGENCY
			break;

		default:
			break;
		}
		TeR.dv_system_status.as_status = state;
	}

	switch (state) {

	case AS_OFF:
		// T 14.4 very important this manages the sl relay of the AS
		if (TeR.status.asms) { // driverless, tomaremos asms como punto de decisión si estamos en DV o MANUAL
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
			//MANUAL (miramos TeR.config.dv_mission_req para poder correr sin que el DV esté enchufado, ya que la mision elegida la maneja el DV y pues si no esta enchufado por lo que sea quiero seguir pudiendo testear)
		} else if ((TeR.config.dv_mission_req
				== TER_ECU_CONFIG_DV_MISSION_REQ_MANUAL_CHOICE)
				&& (TeR.asb_status.asb_ebs_state
						== TER_ASB_STATUS_ASB_EBS_STATE_DEACTIVATED_CHOICE)
				&& (TeR.asb_status.asb_redundancy_state
						== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_DEACTIVATED_CHOICE)
				&& (TeR.asb_status.asb_energy_status
						== TER_ASB_STATUS_ASB_ENERGY_STATUS_UNAVAILABLE_CHOICE)) {
			set_sl_request(SL_DV, 1);
		} else {
			set_sl_request(SL_DV, 0);
		}
		break;

	case AS_READY:
		if (!checkPersistance(&ready_time, 0, 5000)) { // han pasado al menos 5 segundos? (leete como funciona checkpersistance para que entiendas este truco)
			if (!checkPersistance(&res_k2, TeR.res_pdo_tx.k2, 500)) { // le han dado al k2 del RES durante mas de 500 millis ?
				easyCommand(TER_COMMAND_CMD_READY2_DRIVE_DV_CHOICE); // enviamos request de paso a r2d DV
			}
		}
		break;

	case AS_DRIVING:

		if (TeR.status.as_allowed) { // si la flag as allowed esta puesta, podemos hacer requests al DV


			//bypass request de freno dv -> asb board
			TeR.asb_brake_req.brake = TeR.dv_dynamic_req_2.asb_brake_req;

			//bypass request steering dv -> steering motor
			set_steer_angle(TeR.dv_dynamic_req_1.steer_angle_req);
		}
		break;

	case AS_EMERGENCY:
		TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE; // no debería de servir para nada pero por si acaso
		if (TeR.wheelInfo.speed > 5) { // permite al DV hacer una parada controlada en caso de entrar en AS_EMERGENCY (ojo que si el source de velocidad son las ruedas, y el freno bloquea, esto no hace nada)
			set_steer_angle(TeR.dv_dynamic_req_1.steer_angle_req);
		} else { // cuando la velocidad sea inferior al threshold, quedará desactivado el steering
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
		}
		break;

	default:
		break;
	}

}
/*
 * Angle should be º * 10000
 * This is to avoid using many encode and decode functions, it is not needed
 * as TeR.dv_dynamic_req_1.steer_angle_req and TeR.steer_actuator_set_position.actuator_position have
 * the same scale factors, so we can avoid enconding and decoding
 * Steering actions will only be performed if the TeR.dv_system_status.steering_state is available, this is done in order
 * to prevent accidental activation of Steering rack in an usafe state
 *
 *
 * */
void set_steer_angle(int32_t angle) {
	if (TeR.dv_system_status.steering_state ==
	TER_DV_SYSTEM_STATUS_STEERING_STATE_AVAILABLE_CHOICE) { // steering permitido
		angle = clamp(angle, -20 * 10000, 20 * 10000); // multiplicado por los factores del DBC todo max y min angle por can configurables
		TeR.steer_actuator_set_position.actuator_position = angle;
		uint8_t TxData[8] = { 0 };
		ter_steer_actuator_set_position_pack(TxData,
				&TeR.steer_actuator_set_position, sizeof(TxData));
		can_scheduler_insert_non_periodic_msg(TxData, sizeof(TxData),
		TER_STEER_ACTUATOR_SET_POSITION_FRAME_ID, 0); // añadir al scheduler
	}
}

/*
 * Set the steer angle without caring if the car is in the ready state, useful for testing purposes
 * under your own responsability
 *
 * */
void unsafe_set_steer_angle(int32_t angle) {
	angle = clamp(angle, -20 * 10000, 20 * 10000); // multiplicado por los factores del DBC todo max y min angle por can configurables
	TeR.steer_actuator_set_position.actuator_position = angle;
	uint8_t TxData[8] = { 0 };
	ter_steer_actuator_set_position_pack(TxData,
			&TeR.steer_actuator_set_position, sizeof(TxData));
	can_scheduler_insert_non_periodic_msg(TxData, sizeof(TxData),
	TER_STEER_ACTUATOR_SET_POSITION_FRAME_ID, 0);
}

void permatask() {
	if (TeR.dv_system_status.as_status == AS_DRIVING && TeR.status.as_allowed) {

	}
//	TeR.dv_driving_dynamics_1.brake_hydr_actual;
//	TeR.dv_driving_dynamics_1.brake_hydr_target;
//	TeR.dv_driving_dynamics_1.motor_moment_actual = TeR.dv_dynamic_req_1.trq_req;
//	TeR.dv_driving_dynamics_1.motor_moment_target = TeR.dv_dynamic_req_1.trq_req;
	TeR.dv_driving_dynamics_1.speed_actual = TeR.wheelInfo.speed;
	TeR.dv_driving_dynamics_1.speed_target = TeR.wheelInfo.speed;
	TeR.dv_driving_dynamics_1.steering_angle_actual = TeR.steer_actuator_status.position;
	TeR.dv_driving_dynamics_1.steering_angle_target = TeR.dv_dynamic_req_1.steer_angle_req;

//	TeR.dv_driving_dynamics_2.acceleration_lateral;
//	TeR.dv_driving_dynamics_2.acceleration_longitudinal;
//	TeR.dv_driving_dynamics_2.yaw_rate;
//	TeR.dv_system_status.ami_state;
//	TeR.dv_system_status.as_ebs_state;
//	TeR.dv_system_status.asb_redundancy_state;
	TeR.dv_system_status.cones_count_actual = TeR.dv_info.cones_count_actual;
	TeR.dv_system_status.cones_count_all= TeR.dv_info.cones_count_all;
	TeR.dv_system_status.lap_counter = TeR.dv_info.lap_counter;
}

void assi_manager(){
	switch(TeR.dv_system_status.as_status){
	case AS_OFF:
		//apagar assi
		break;
	case AS_READY:
		// assi yellow
		break;
	case AS_DRIVING:
		// assi yellow flashing
		break;
	case AS_EMERGENCY:
		// blue flashing
		break;
	case AS_FINISHED:
		//blue continuous
		break;
	default:
		break;
	}
}

