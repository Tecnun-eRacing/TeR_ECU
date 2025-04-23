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

/*Implementacion FreeRTOS Piero
 *
 * - La idea principal es tener una tarea que se encargue de controlar la maquina de estados, de igual prioridad que la recepción de mensajes (no queremos que se pisen)
 *
 * - La ejecución temporizada se realiza utilizando funciones del Kernel tales como osDelayUntil(), debido a que es la forma mas correcta de realizar
 *		 ejecuciones temporizadas sin desfase temporal en un sistema operativo en tiempo real como puede ser FreeRTOS.
 * 		 Podriamos usar software timers (se ha probado y es lo mismo), su implementacion sin embargo no es la mas practica, ya que debemos
 * 		 registrar un callback que mande señales de desbloqueo a los threads, y que estos a su vez esperen a dichas señales,
 * 		 ademas de que NO garantiza ejecucion temporal precisa, ya que por naturaleza la Daemon Task es de baja prioridad(se puede cambiar) (Reference Manual),
 * 		 por lo que se ha decidido utilizar la funcion recomendada por el reference manual para ejecuciones temporales precisas
 *
 * - Cuando pasamos al estado DRIVING debemos tener un delay durante 2 segundos, no necesariamente para el beep (que actualmente esta
 * 		implementado utilizando un One Shoot Software timer para evitar halts en las tareas, ver command) sino porque debemos detener la maquina de estados
 *		para evitar la comanda de par durante el pitido.
 *
 */

//Persistance checker
persist_t SL;
//Refri config struct
struct ter_refri_config_t refri;

// FreeRTOS dependencies
const static uint32_t task_period = 2; // Task frequency 500 Hz

//FreeRTOS Task
void stateMachine(void *argument) {
	initConfig(); // Arrancar eeprom y cargar configuraciones del sistema
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
				== HVBMS_BMS_TX_STATE_3_APP_STATE_APP_HV__PRECHARGE_CHOICE) { // Se está haciendo precarga?
			status = PRECHARGING;
		} else if (TeR.BmsAppState.app_state_app
				== HVBMS_BMS_TX_STATE_3_APP_STATE_APP_HV__READY_CHOICE) { // Esta precargado?
			status = PRECHARGED;
			if (TeR.status.r2_d
					&& ((TeR.appStateRight.app_state_app == 4)
							|| (TeR.appStateLeft.app_state_app == 4))) { //la flag de ready2drive esta activada y los dos inversores operativos
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
		switch (state) {
		case WAIT_SL:
			//Anounce through USB CDC
			printf("TeR is Waiting for Safety Line");

			//	Apagamos refri
			ter_refri_config_init(&refri);
			refri.entry = TER_REFRI_CONFIG_ENTRY_POWER_CHOICE;
			refri.power = TER_REFRI_CONFIG_POWER_OFF_CHOICE;
			sendConfig(TER_REFRI_CONFIG_FRAME_ID, &refri);
			// reseteamos request de pwm a 0
			refri.entry = TER_REFRI_CONFIG_ENTRY_INTENSITY_CHOICE;
			refri.intensity = 0;
			sendConfig(TER_REFRI_CONFIG_FRAME_ID, &refri);

			//Security
			easyCommand(TER_COMMAND_CMD_END_LOG_CHOICE);
			easyCommand(TER_COMMAND_CMD_RESET_BMS_CHOICE); //reset al bms de osto
			break;

		case RDY2PRECH:
			//Anounce through USB CDC
			printf("TeR is Ready To Precharge");
			//Security
			TeR.appReqLeft.app_state_req = 1; //Manda el Inverter a su estado off por si estaba en error
			TeR.appReqRight.app_state_req = 1;
			break;

		case PRECHARGING:
			//Anounce through USB CDC
			printf("TeR is Precharging");
			break;

		case PRECHARGED:
			//Anounce through USB CDC
			printf("TeR is Precharged");

//			 activamos cooling potencia LOW
			ter_refri_config_init(&refri);
			refri.entry = TER_REFRI_CONFIG_ENTRY_POWER_CHOICE;
			refri.power = TER_REFRI_CONFIG_POWER_ON_CHOICE;
			sendConfig(TER_REFRI_CONFIG_FRAME_ID, &refri);
//			request de intensidad 40%
			refri.entry = TER_REFRI_CONFIG_ENTRY_INTENSITY_CHOICE;
			refri.intensity = 40;
			sendConfig(TER_REFRI_CONFIG_FRAME_ID, &refri);
//			modo manual
			refri.entry = TER_REFRI_CONFIG_ENTRY_MODE_CHOICE;
			refri.mode = TER_REFRI_CONFIG_MODE_MANUAL_CHOICE;
			sendConfig(TER_REFRI_CONFIG_FRAME_ID, &refri);

			//Manda el inverter a listo
			TeR.appReqLeft.app_state_req = 2;
			TeR.appReqRight.app_state_req = 2;
			break;
		case DRIVING:
			//activamos cooling potencia HIGH
			ter_refri_config_init(&refri);
			refri.entry = TER_REFRI_CONFIG_ENTRY_INTENSITY_CHOICE;
			refri.intensity = 100;
			sendConfig(TER_REFRI_CONFIG_FRAME_ID, &refri);

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
//BrakeLight
	if (TeR.bpps.bpps >= TeR.config.r2_d_brake + 1) {
		HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_RESET);
	}
// Proccess Wheel Data
	TeR.wheelInfo.rl_rpm = ((TeR.dqErpmLeft.e_machine_speed_erpm) / MOTOR_POLES)
			* RED_RATIO;
	TeR.wheelInfo.rr_rpm = (TeR.dqErpmRight.e_machine_speed_erpm / MOTOR_POLES)
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
					TeR.tempsLeft.e_machine_temp_2_deg_c);
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
	TeR.status.ams = TeR.BmsAppState.dio1_state; //1 OK
	TeR.status.imd = TeR.BmsAppState.dio2_state; // 1 OK
	TeR.status.left_inv = (TeR.appStateLeft.app_state_app != 6); //Distinto de fault state
	TeR.status.right_inv = (TeR.appStateRight.app_state_app != 6); //Distinto de fault state
}
