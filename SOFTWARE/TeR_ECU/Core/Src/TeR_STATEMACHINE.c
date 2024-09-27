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
 * - Esta tarea comparte un Mutex con la función de decodificación encontrada en el módulo TeR_CAN
 *
 * - La ejecución temporizada se realiza utilizando funciones del Kernel tales como osDelayUntil(), debido a que es la forma mas correcta de realizar
 *		 ejecuciones temporizadas sin desfase temporal en un sistema operativo en tiempo real como puede ser FreeRTOS.
 * 		 Podriamos usar software timers (se ha probado y es lo mismo), su implementacion sin embargo no es la mas practica, ya que debemos
 * 		 registrar un callback que mande señales de desbloqueo a los threads, y que estos a su vez esperen a dichas señales,
 * 		 ademas de que NO garantiza ejecucion temporal precisa, ya que por naturaleza la Daemon Task es de baja prioridad(se puede cambiar) (Reference Manual),
 * 		 por lo que se ha decidido utilizar la funcion recomendada por el reference manual para ejecuciones temporales precisas
 *
 *- El funcionamiento consiste en esperar un delay, esperar al mutex, y una vez hecho esto ejecutar la maquina de estados del coche
 *-
 *- En su momento se planteo separar  la funcion torqueManager de la maquina de estados, en su tarea propia, pero esto puede llevar a problemas de sincronización y
 *- en mi opinión haria el codigo mas dificil de leer, con beneficios casi nulos.
 *
 * - Cuando pasamos al estado DRIVING debemos tener un delay durante 2 segundos, no necesariamente para el beep (que actualmente esta
 * 		implementado utilizando un One Shoot Software timer para evitar halts en las tareas, ver command) sino porque debemos detener la maquina de estados
 *		para evitar la comanda de par durante el pitido.
 *
 * 		Existen otras maneras de resolver este problema, una de ellas es bloqueando la region de codigo utilizando un evento
 * 		que se libera 2 segundos despues utilizando un one shot timer, osthreadflagswait(), y que el timer lo ponga a set en 2 seg, desbloqueando la region de codigo
 * 		(probado y funciona, pero no quiero que esto sea un lio para entender para alguien nuevo)
 * 		Otra manera seria aislar el torqueManager en otra tarea y despertarla 2 segundos despues utilizando un oneShotTimer
 *
 * - Debido a esta decisión de arquitectura, debemos soltar el mutex antes del delay (para que otras tareas puedan seguir ejecutandose)
 * 		y resincronizar el tiempo del kernel con el valor posterior al delay (no es necesario, pero lo hacemos porque es gratis)
 * 		y posteriormente readquirir el mutex para continuar con la ejecución
 *
 *- La permatask no interesa separarla ya que los datos tienen que estar sync con la maquina de estados, por lo que no ganamos nada separando
 *
 *  - SOLO EJECUTAREMOS CUANDO HAYAMOS PODIDO OBTENER EL MUTEX
 *
 */


//Persistance checker
persist_t SL;

// FreeRTOS dependencies
extern osMutexId_t preventRaceHandle; // Mutex compartido con la tarea de recepción de CAN (para tener exclusión mutua sobre la modificacion de la variable TeR)
uint32_t currentTick; // declaramos nuestra variable currentTick como global (para reactualizar su valor al parar la maquina de estados)
// IMPORTANTE: Se utiliza osDelayUntil debido a que es la manera recomendada por FreeRTOS en el reference manual para ejecucion temporal estricta sin desfases

//FreeRTOS Task
void stateMachineTask(void *argument) {
	currentTick = osKernelGetTickCount(); // sincronizamos nuestra variable de tick con el tick actual del Kernel
	uint32_t errorCounter = 0; // contador de errores de la no obtención del mutex (debug purposes)
    osStatus_t mutexStatus; //variable que almacena el estado de la obtencion del mutex
	for (;;) {
		currentTick+=2; //incrementamos nuestro tiempo con respecto al tiempo del kernel
		osDelayUntil(currentTick); // bloqueamos la tarea hasta que lleguemos al valor de tick scheduled para la ejecucion.
		mutexStatus = osMutexAcquire(preventRaceHandle,500); //intentamos adquirir mutex de forma segura hasta tMax, el timeout es para saber si nos quedamos pillados y responder
		if(mutexStatus==osOK){ //SI hemos obtenido acceso al Mutex
		stateMachine(); //ejecutamos la maquina de estados del vehiculo, si y solo si el mutex se adquiere correctamente
		osMutexRelease(preventRaceHandle); // y una vez terminada la ejecucion, liberamos el mutex, si y solo si lo teniamos antes
		}
		else{ // NO hemos obtenido acceso al mutex
			errorCounter++; // haremos un handle bien, loggeamos el error
		}
	}
}

state_t getState(void) {
	state_t status = WAIT_SL; //Iniciamos en el estado 0
	//Lecturas
	TeR.status.sl = checkPersistance(&SL,
			HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin), 500);// Leemos el estado de la safety
	TeR.status.bspd = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);	// Leemos el estado del BSPD

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

void stateMachine(void) {
	uint8_t prevState = TeR.status.state; //Guarda el estado previo
	TeR.status.state = getState(); //Get Current State
	uint8_t stateChanged = TeR.status.state != prevState ? 1 : 0; //for state setup
	permaTask(); //Ejecuta las tareas permanentes
	//-----------------------------------[Setups]--------------------------------------------//

	if (stateChanged) { // Handles setup conditions for the new state
		switch (TeR.status.state) {
		case WAIT_SL:
			//Security
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
			switchCommand(TER_COMMAND_CMD_SWITCH_REFRI_CHOICE,
			TER_COMMAND_ONOFF_OFF_CHOICE);
			easyCommand(TER_COMMAND_CMD_RESET_BMS_CHOICE); //reset al bms de osto
			break;

		case RDY2PRECH:
			//Security
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
			TeR.appReqLeft.app_state_req = 1; //Manda el Inverter a su estado off por si estaba en error
			TeR.appReqRight.app_state_req = 1;
			break;

		case PRECHARGING:
			//Security
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
			break;

		case PRECHARGED:
			TeR.appReqLeft.app_state_req = 2; //Manda el inverter a ready
			TeR.appReqRight.app_state_req = 2;
			//Security
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;

			//Arranca la refri
			switchCommand(TER_COMMAND_CMD_SWITCH_REFRI_CHOICE,
			TER_COMMAND_ONOFF_ON_CHOICE);

			//Configura el driving mode
			struct ter_command_t cmdMsg;
			ter_command_init(&cmdMsg);
			cmdMsg.cmd = TER_COMMAND_CMD_SET_LIMITS_CHOICE;
			cmdMsg.trq_limit = 150;
			cmdMsg.kw_limit = 40;
			cmdMsg.speed_limit = 50;
			command(cmdMsg); //Llama a la interpretación del comando (Se lo pasa por copia)

			ter_command_init(&cmdMsg);
			cmdMsg.cmd = TER_COMMAND_CMD_SET_DYNAMIC_CONFIG_CHOICE;
			cmdMsg.cfg_limiter = TER_DYNAMIC_CONFIG_LIMITER_LIMIT_TORQUE_CHOICE;
			cmdMsg.cfg_mode = TER_DYNAMIC_CONFIG_MODE_LINEAL_CHOICE;
			cmdMsg.cfg_traction_control =
			TER_DYNAMIC_CONFIG_TRACTION_CONTROL_OFF_CHOICE;
			command(cmdMsg); //Llama a la interpretación del comando (Se lo pasa por copia)
			break;
		case DRIVING: // se puede utilizar un one shot software timer para hacer wakeup de una tarea torquemanager dentro de 2 seg, pero eso implicaria tener una maquina de estados desincronizada, prefiero asi
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			osMutexRelease(preventRaceHandle); // liberamos el mutex para que se siga ejecutando la recepcion durante el delay (ya que comparten mutex)
			osDelay(2000); //EV 4.12.1, delay para el sonido y ADEMAS para que el coche NO acelere mientras pite, (la maquina de estados se para aqui 2 segs)
			currentTick=osKernelGetTickCount(); // resincronizamos nuestro tick con el valor actual del tick del kernel (debido al delay, hacemos esto porque queremos parar la maquna de estados, no es necesario)
			osMutexAcquire(preventRaceHandle, osWaitForever); //volvemos a obtenerlo para ejecutar el torque manager
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // apagamos la bocina y el coche ya puede acelerar
			startSCS(); //activamos el sistema de señales críticas del vehículo

			break;
		default:
			//Handle Invalid state
			break;
		}
	}

//-----------------------------------[LOOPS]--------------------------------------------//

	switch (TeR.status.state) {
	case WAIT_SL:
		waitSL();
		break;

	case RDY2PRECH:
		rdy2Prech();
		break;

	case PRECHARGING:
		precharging();
		break;

	case PRECHARGED:
		precharged();
		break;
	case DRIVING:
		driving();
		break;
	default:
		//Handle Invalid state
		break;
	}

}

/* -------------------------[Estados]---------------------------- */

void waitSL(void) {

} // Comprueba que la safety esta cerrada
void rdy2Prech(void) {

} // Espera a recibir el comando de precarga
void precharging(void) {

} //Estado transitorio, monitoriza que todo va bien
void precharged(void) {

} //Espera a que se reciba el comando de r2d
void driving(void) {
	trqManager(); //Ejecuta el pipeline de torque

} //Ejecuta la comanda de par

/* -------------------------[PermaTask]---------------------------- */

void permaTask() {
	// Refri Management
	//refriManager();

//BrakeLight
	if (TeR.bpps.bpps > 4) {
		HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_RESET);
	}
// Proccess Wheel Data
	TeR.wheelInfo.rl_rpm =
			((-TeR.dqErpmLeft.e_machine_speed_erpm) / MOTOR_POLES) * RED_RATIO;
	TeR.wheelInfo.rr_rpm = (TeR.dqErpmRight.e_machine_speed_erpm / MOTOR_POLES)
			* RED_RATIO;
	TeR.wheelInfo.rl_trq = TeR.trqEstLeft.torque_est_nm / RED_RATIO;
	TeR.wheelInfo.rr_trq = TeR.trqEstRight.torque_est_nm / RED_RATIO;
	TeR.wheelInfo.speed = 3.6 * (TeR.wheelInfo.rl_rpm * 2 * PI * WHEEL_RADIUS)
			/ 60; //Linear velocity of vehicle

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
	TeR.status.refri = TeR.lvbms.refri_on; // Relay del estado de refri
}
