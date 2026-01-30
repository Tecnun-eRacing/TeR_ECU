/*
 * stateMachine.c
 *
 *  Created on: Feb 1, 2024
 *      Author: Ozuba
 *
 * Este fichero encapsula la maquina de estados del TER:
 * Esta consiste de 4 estados:
 *-------------------------------------------------------------------------------------
 * WAITING_FOR_SL -> 0
 * - Estado de inicio, se comprueba si la safety esta cerrada
 * - leyendo el valor de volaje despues del TSMS
 *
 * RDY2PRECH
 *- La safety esta cerrada, se puede precargar
 *
 * PRECHARGING
 *- Estado transitorio hasta que el BMS termine la precarga
 *
 * PRECHARGED
 *- El coche está cargado, se permite hace R2D
 *
 * R2D
 * - Se puede conducir
 *-------------------------------------------------------------------------------------
 * Tras valorar distintas maneras de implementar la maquina de estados
 * lo más optimo es comprobar el estado cada ciclo en una escalera de privilegio
 * puesto que las condiciones de estados más complejos están contenidos en los superiores,
 * de esta manera si una condición no se cumple se degrada al estado más bajo.
 *
 */
#include "TeR_STATEMACHINE.h"

/*Implementacion FreeRTOS
 *
 * - La idea principal es tener una tarea que se encargue de controlar la maquina de estados, de igual prioridad que la recepción de mensajes (no queremos que se pisen)
 *
 * - La ejecución temporizada se realiza utilizando funciones del Kernel tales como osDelayUntil(), debido a que es la forma mas correcta de realizar
 *		 ejecuciones temporizadas sin desfase temporal.
 *
 * - Cuando pasamos al estado DRIVING debemos tener un delay durante 2 segundos, no necesariamente para el beep (que actualmente esta
 * 		implementado utilizando un One Shoot Software timer para evitar halts en las tareas, ver command) sino porque debemos detener la maquina de estados
 *		para evitar la comanda de par durante el pitido.
 *		Se puede implementar de otra manera, añadiendo un estado temporal en el que estaremos esperando durante 2 segs, por ejemplo, "wait2sState" o algo asi
 *
 */

//Persistance checker
persist_t SL;
//Refri config struct
struct ter_refri_config_t refri;
//buttons
uint8_t rButton;
uint8_t lButton;
uint8_t cButton;
uint8_t regenButton;

// FreeRTOS dependencies
const static uint32_t task_period = 5; // Task frequency

//FreeRTOS Task
void stateMachine(void *argument) {
	init_config(); // Arrancar eeprom y cargar configuraciones del sistema
	for (;;) {
		osDelay(task_period); //osDelay porque no necesitamos ejecución estricta sin desfases en la maquina de estados
		stateLoop(); //ejecutamos la maquina de estados del vehiculo
	}
}

state_t evalState(void) {
	state_t status = WAIT_SL; //Iniciamos en el estado 0
	//Lecturas
	TeR.status.sl = checkPersistance(&SL,
			HAL_GPIO_ReadPin(DIN0_GPIO_Port, DIN0_Pin), 500);// Leemos el estado de la safety
	TeR.status.bspd = HAL_GPIO_ReadPin(DIN1_GPIO_Port, DIN1_Pin);// Leemos el estado del BSPD

	if (TeR.status.sl) { //Si esta ok la safety
		status = RDY2PRECH; //Se puede precargar
		if (TeR.BmsAppState.app_state_app
				== HVBMS_BMS_TX_STATE_3_APP_STATE_APP_HV__PRECHARGE__READY_CHOICE) { // Se está haciendo precarga?
			status = PRECHARGING;
		} else if (TeR.BmsAppState.app_state_app
				== HVBMS_BMS_TX_STATE_3_APP_STATE_APP_HV__READY_CHOICE) { // Esta precargado?
			status = PRECHARGED;
			if (TeR.status.r2_d
					&& ((TeR.appStateRight.app_state_app == 4)
							|| (TeR.appStateLeft.app_state_app == 4))) { //la flag de ready2drive esta activada y alguno de los dos inversores operativos
				status = DRIVING;
			}
		}
	}
	return status;
}

void stateLoop(void) {
	uint8_t prevState = TeR.status.state; //Guarda el estado previo
	uint8_t state = evalState(); //Get Current State, guardamos y seteamos al evaluar el caso para evitar desincronizaciones de estado
	uint8_t stateChanged = state != prevState ? 1 : 0; //for state setup
	permaTask(); //Ejecuta las tareas permanentes
	//-----------------------------------[State Transition Tasks]--------------------------------------------//

	if (stateChanged) { // Handles setup conditions for the new state
		//publish_config(&TeR.config, ALL_CONFIGS); // en cada cambio de estado publicamos configuracion entera del coche
		switch (state) {
		case WAIT_SL:
			TeR.BmsAppReq.app_state_req =
			HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_STANDBY_CHOICE;
			//Anounce through USB CDC
			printf("TeR is Waiting for Safety Line");

			//	Apagamos refri ventis bombas
			ter_refri_config_init(&refri);
			refri.entry = TER_REFRI_CONFIG_ENTRY_INTENSITY_CHOICE;
			refri.intensity = 0;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);

			refri.entry = TER_REFRI_CONFIG_ENTRY_POWER_CHOICE;
			refri.power = TER_REFRI_CONFIG_POWER_OFF_CHOICE;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);

			//Security
			easyCommand(TER_COMMAND_CMD_END_LOG_CHOICE);
			TeR.appReqLeft.app_state_req = 1; //Manda el Inverter a su estado off por si estaba en error
			TeR.appReqRight.app_state_req = 1;
			break;

		case RDY2PRECH:
			publish_config(&TeR.config, ALL_CONFIGS);
			TeR.BmsAppReq.app_state_req =
			HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_STANDBY_CHOICE;
			//Anounce through USB CDC
			printf("TeR is Ready To Precharge");
			//Security
			TeR.appReqLeft.app_state_req = 2; //Manda el Inverter a ready
			TeR.appReqRight.app_state_req = 2;
			break;

		case PRECHARGING:
			//Anounce through USB CDC
			printf("TeR is Precharging");
			break;

		case PRECHARGED:
			publish_config(&TeR.config, ALL_CONFIGS);
			TeR.BmsAppReq.app_state_req =
			HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_HV_READY_CHOICE; //mandamos a ready
			//Anounce through USB CDC
			printf("TeR is Precharged");

//			 activamos cooling potencia LOW de MAIN
			ter_refri_config_init(&refri);
			refri.entry = TER_REFRI_CONFIG_ENTRY_POWER_CHOICE;
			refri.power = TER_REFRI_CONFIG_POWER_ON_CHOICE;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);
//			request de intensidad 20%
			refri.entry = TER_REFRI_CONFIG_ENTRY_INTENSITY_CHOICE;
			refri.intensity = 20;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);
//			modo manual
			refri.entry = TER_REFRI_CONFIG_ENTRY_MODE_CHOICE;
			refri.mode = TER_REFRI_CONFIG_MODE_MANUAL_CHOICE;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);

//			activamos cooling  ACCU
		/*	ter_refri_config_init(&refri);
			refri.entry = TER_REFRI_CONFIG_ENTRY_POWER_ACCU_CHOICE;
			refri.power_accu = TER_REFRI_CONFIG_POWER_ACCU_ON_CHOICE;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);
//			request de intensidad 100%
			refri.entry = TER_REFRI_CONFIG_ENTRY_INTENSITY_ACCU_CHOICE;
			refri.intensity_accu = 100;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);
//			modo manual
			refri.entry = TER_REFRI_CONFIG_ENTRY_MODE_ACCU_CHOICE;
			refri.mode_accu = TER_REFRI_CONFIG_MODE_ACCU_MANUAL_CHOICE;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);*/

			//Manda el inverter a listo
			TeR.appReqLeft.app_state_req = 2;
			TeR.appReqRight.app_state_req = 2;
			break;
		case DRIVING:
			TeR.BmsAppReq.app_state_req =
			HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_HV_READY_CHOICE; // mandamos a ready (en teoria es imposible, pero por si pasamos a driving sin pasar por prech)
			//activamos cooling potencia HIGH
			ter_refri_config_init(&refri);
			refri.entry = TER_REFRI_CONFIG_ENTRY_INTENSITY_CHOICE;
			refri.intensity = 80;
			send_config(TER_REFRI_CONFIG_FRAME_ID, &refri);

			easyCommand(TER_COMMAND_CMD_START_LOG_CHOICE);
			HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_SET);
			osDelay(1000);
			HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_RESET);
			break;
		default:
			//Handle Invalid state
			break;
		}
		TeR.status.state = state;
	}
}

/* -------------------------[PermaTask]---------------------------- */

void permaTask() {
	buttonHandler(); // todo TEMPORAL, tiene que ser la pantalla quien gestione los botones, no la ECU
//BrakeLight
	if (ter_bpps_bpps_decode(TeR.bpps.bpps) >= TeR.config.r2_d_brake) {
		HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_RESET);
	}
// Proccess Wheel Data
	TeR.wheelInfo.rl_rpm = ((TeR.dqErpmLeft.e_machine_speed_erpm) / MOTOR_POLES)
			* RED_RATIO;
	TeR.wheelInfo.rr_rpm = ((TeR.dqErpmRight.e_machine_speed_erpm) / MOTOR_POLES)
			* RED_RATIO;
	TeR.wheelInfo.rl_trq = TeR.trqEstLeft.torque_est_nm / RED_RATIO;
	TeR.wheelInfo.rr_trq = TeR.trqEstRight.torque_est_nm / RED_RATIO;
	TeR.wheelInfo.speed =
			3.6
					* ((TeR.wheelInfo.rl_rpm + TeR.wheelInfo.rr_rpm) * PI
							* WHEEL_RADIUS) / 60; //Linear velocity of vehicle

// Bypass Inverter data
	TeR.invInfo.left_dem = TeR.demLeft.dem; //Dem
	TeR.invInfo.right_dem = TeR.demRight.dem; //Dem

	TeR.invInfo.left_motor_temp =
			(uint8_t) inverter_emcu_state_4_left_e_machine_temp_2_deg_c_decode(
					TeR.tempsLeft.e_machine_temp_1_deg_c);
	TeR.invInfo.right_motor_temp =
			(uint8_t) inverter_emcu_state_4_right_e_machine_temp_2_deg_c_decode(
					TeR.tempsRight.e_machine_temp_2_deg_c);

	TeR.invInfo.left_power_stage_temp =
			(uint8_t) inverter_emcu_state_4_left_pwr_stg_temp_deg_c_decode(
					TeR.tempsLeft.pwr_stg_temp_deg_c);

	TeR.invInfo.right_power_stage_temp =
			(uint8_t) inverter_emcu_state_4_right_pwr_stg_temp_deg_c_decode(
					TeR.tempsRight.pwr_stg_temp_deg_c);

	//Fill in Status Message
	TeR.status.ams = TeR.BmsAppState.dio3_state; //1 OK
	TeR.status.imd = TeR.BmsAppState.dio2_state; // 1 OK
	TeR.status.left_inv = (TeR.appStateLeft.app_state_app != 6); //Distinto de fault state
	TeR.status.right_inv = (TeR.appStateRight.app_state_app != 6); //Distinto de fault state
}

void buttonHandler() {
	//limitation handling
	if (read_btn(&lButton, TeR.buttons.el) && !TeR.buttons.eb) {
		if (TeR.config.trq_limit + 10 <= 180) {
			TeR.config.trq_limit += 10;
		} else {
			TeR.config.trq_limit = 180;
		}
		publish_config(&TeR.config, TER_ECU_CONFIG_ENTRY_TRQ_LIMIT_CHOICE);
	}
	if ((read_btn(&rButton, TeR.buttons.er)) && !TeR.buttons.eb) {
		if (TeR.config.trq_limit - 10 >= 60) {
			TeR.config.trq_limit -= 10;
		} else {
			TeR.config.trq_limit = 60;
		}
		publish_config(&TeR.config, TER_ECU_CONFIG_ENTRY_TRQ_LIMIT_CHOICE);
	}
	if (read_btn(&regenButton, TeR.buttons.b3)) {
		TeR.refri_config.entry = TER_REFRI_CONFIG_ENTRY_POWER_CHOICE;
		TeR.refri_config.power =
				(TeR.refri_config.power == TER_REFRI_CONFIG_POWER_ON_CHOICE) ?
						TER_REFRI_CONFIG_POWER_OFF_CHOICE :
						TER_REFRI_CONFIG_POWER_ON_CHOICE;
		send_config(TER_REFRI_CONFIG_FRAME_ID, &TeR.refri_config);
	}

}

