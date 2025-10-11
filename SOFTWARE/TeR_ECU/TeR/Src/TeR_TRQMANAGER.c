/*
 * torqueManager.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Ozuba
 *
 */

#include "TeR_TRQMANAGER.h"
#include "tv_mds.h"

/* Esquizofrenia RTOS
 * - La ejecución temporizada se realiza utilizando funciones del Kernel tales como osDelayUntil(), debido a que es la forma mas correcta de realizar
 *		 ejecuciones temporizadas sin desfase temporal en un sistema operativo en tiempo real como puede ser FreeRTOS.
 * 		 Podriamos usar software timers (se ha probado y es lo mismo), su implementacion sin embargo no es la mas practica, ya que debemos
 * 		 registrar un callback que mande señales de desbloqueo a los threads, y que estos a su vez esperen a dichas señales,
 * 		 ademas de que NO garantiza ejecucion temporal precisa, ya que por naturaleza la Daemon Task es de baja prioridad(se puede cambiar) (Reference Manual),
 * 		 por lo que se ha decidido utilizar la funcion recomendada por el reference manual para ejecuciones temporales precisas
 *
 */

const static int task_period = 5; // Task frequency 500hz NO TOQUES ESTO QUE ME JODES TODAS LAS GANANCIAS DEL TORQUE POR FAVOR GRACIAS

extern trqMap_t trqDistribution(trq_t limit);

trqPipeline_t DriveConfig; //Configuración en uso
extern osThreadId_t trqManagerTaskHandle; // thread id of trqManager task

void trqManager(void *argument) { // Corre las etapas del pipeline y solicita la comanda
	uint32_t nextTick = osKernelGetTickCount(); // Initialize reference time

	for (;;) {
		nextTick += task_period; //Genera el timestamp de la siguiente ejecucion
		osDelayUntil(nextTick); // Utilizamos OsDelayUntil porque es la manera recomendada en el reference manual para ejecutar tareas sin desfase temporal
		//Check if we are driving
		if (TeR.status.state == DRIVING) {
			//Execute Pipeline
			trq_t availableTorque = DriveConfig.limiter(); //Limita
			trqMap_t trqDistribution = DriveConfig.drivingMode(availableTorque); //Distribuye
			trqMap_t trqToWheels = DriveConfig.tractionControl(trqDistribution); // Limita si se vé que la rueda no puede dar ese grip (Estrategia dependiente del controlador)

			//Solicitud de la comanda
			TeR.trqReqLeft.torque_nm_req = trqToWheels.rLeft;
			TeR.trqReqRight.torque_nm_req = trqToWheels.rRight;

		} else {
			//Config the driving pipeline if not running
			switch (TeR.config.limiter) {

			case TER_ECU_CONFIG_LIMITER_TORQUE_CHOICE:
				DriveConfig.limiter = &limitTorque;
				break;
			}
			switch (TeR.config.driving_mode) {

			case TER_ECU_CONFIG_DRIVING_MODE_LINEAL_CHOICE:
				DriveConfig.drivingMode = &lineal;
				break;

			case TER_ECU_CONFIG_DRIVING_MODE_TORQUE_VECTORING_CHOICE:
				DriveConfig.drivingMode = &trqVectoring;
				tv_deInitPID(); // si el puntero esta creado, lo libera y lo setea a NULL, si el puntero ya es null, no hace nada. Permite reconfigurar gains al salir de driving
				break;
			}
			switch (TeR.config.traction_control) {

			case TER_ECU_CONFIG_TRACTION_CONTROL_OFF_CHOICE:
				DriveConfig.tractionControl = &tractionControlOFF;
				break;
			}

			//Torque Zero safestate
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
		}
	}
}

//------------------------------------------------[Basic Power Limiters]------------------------------------------------//
// void -> trq_t
//Par Máximo constante
trq_t limitTorque(void) {
	return (trq_t) TeR.config.trq_limit; //Devuelve el valor configurado
}

//------------------------------------------------[Basic Driving Modes]------------------------------------------------//
// trq_t -> trqMap_t
trqMap_t lineal(trq_t limit) { //Entrega lineal de par a las 2 ruedas
	trqMap_t trqMap;
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
	return torqueCheck(trqMap, limit, TeR.config.regen_max_trq);
}

//------------------------------------------------[Basic traction Control]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t tractionControlOFF(trqMap_t in) {
	return in;
}

//MANDATORY USE IN EACH DRIVINGMODE also controls the regen
trqMap_t torqueCheck(trqMap_t in, trq_t limit, trq_t maxNegTrq) {
	if (TeR.config.regen_enable == TER_ECU_CONFIG_REGEN_ENABLE_ENABLE_CHOICE) {
		switch (TeR.config.regen_mode) {
		case TER_ECU_CONFIG_REGEN_MODE_BPPS_CHOICE: {
			int8_t trq = map(TeR.bpps.bpps * TeR.config.regen_trq_slope, 0,
					ter_bpps_bpps_encode(MAX_BPPS_VALUE), 0,
					TeR.config.regen_max_trq); //mapeamos el pedal de freno como una recta de slope configurable y clampeo para diferente valor
			trq = -abs(trq / 2); // negativo porque queremos regenerar
			in.rLeft = trq;
			in.rRight = trq;
			if (in.rLeft > 0 || in.rRight > 0) { // esquizofrenia por si de alguna forma se vuelve positivo, rayo cosmico
				in.rLeft = 0;
				in.rRight = 0;
			}

			break;
		}
		case TER_ECU_CONFIG_REGEN_MODE_APPS_CHOICE: {
			int8_t trq = TeR.config.regen_max_trq;
			trq = -abs(trq / 2);
			in.rLeft = trq;
			in.rRight = trq;
			if (in.rLeft > 0 || in.rRight > 0) { // esquizofrenia por si de alguna forma se vuelve positivo, rayo cosmico
				in.rLeft = 0;
				in.rRight = 0;
			}
			break;
		}
		}
	}

//1) First check if negative torque is being requested (avoids backwards spinning of the wheels)
	if (in.rLeft < 0 || in.rRight < 0) {
		if (TeR.wheelInfo.rl_rpm < TeR.config.regen_thr_rpm) {
			in.rLeft = 0;
		}
		if (TeR.wheelInfo.rr_rpm < TeR.config.regen_thr_rpm) {
			in.rRight = 0;
		}
		if (TeR.wheelInfo.speed < TeR.config.regen_thr_speed) {
			in.rRight = 0;
			in.rLeft = 0;
		}
		return in; //return 0 torque as negative torque is being requested with below security speed
	}

	maxNegTrq = maxNegTrq > limit ? limit : maxNegTrq; //check if negative allowance is in limit and if not clamp it (not necessary)

// 2) check if negative torque is being requested and between is betweeen allowedNegativeTorque
	if (in.rLeft <= -maxNegTrq / 2) { // if torque exceeds allowance
		in.rLeft = -maxNegTrq / 2; //clamp to allowance
	}
	if (in.rRight <= -maxNegTrq / 2) { //if torque exceeds allowance
		in.rRight = -maxNegTrq / 2; //clamp to allowance
	}

//3) check if requested torque exceds limit (negative torque excess is taken into account in step 2)
	if (in.rLeft > limit / 2) {
		in.rLeft = limit / 2;
	}
	if (in.rRight > limit / 2) {
		in.rRight = limit / 2;
	}
	return in;
}

uint8_t regen_allowed(trqMap_t in) { // 1 ok 0 not ok

	if (!(TeR.config.regen_enable == TER_ECU_CONFIG_REGEN_ENABLE_ENABLE_CHOICE)) // regen activada?
		return 0;
	if (!((in.rLeft <= TeR.config.regen_max_positive_trq_thr / 2) // esta el piloto pidiendo un poco de torque positivo?
			&& (in.rRight <= TeR.config.regen_max_positive_trq_thr / 2)))
		return 0;
	if (!(hvbms_bms_tx_state_6_cell_max_v_decode(TeR.BmsCellsVolt.cell_min_v) // celdas en rango de tension?
			< TeR.config.regen_max_cell_volt))
		return 0;
	if (!(hvbms_bms_tx_state_9_cell_temp_max_deg_c_decode( // celdas en rango de temperatura?
			TeR.BmsCellsTemp.cell_temp_max_deg_c)
			< TeR.config.regen_max_cell_temp))
		return 0;
	if (!(hvbms_bms_tx_state_4_curr_2_x10_a_decode(TeR.BmsCurrent.curr_2_x10_a) // accu en rango de corriente ?
			> -TeR.config.regen_max_current))
		return 0;

	// si se han complido todas las condiciones necesarias para regenerar, retornamos 1
	return 1;
}


