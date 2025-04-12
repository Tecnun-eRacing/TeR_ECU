/*
 * torqueManager.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Ozuba
 *
 */

#include "TeR_TRQMANAGER.h"
#include "tv_mds.h"

const static int task_period = 10; // Task frequency 100hz

extern trqMap_t trqDistribution(trq_t limit);

trqPipeline_t DriveConfig; //Configuración en uso
extern osThreadId_t trqManagerTaskHandle; // thread id of trqManager task

void trqManager(void *argument) { // Corre las etapas del pipeline y solicita la comanda
	uint32_t nextTick = osKernelGetTickCount(); // Initialize reference time

	for (;;) {
		nextTick += task_period; //Genera el timestamp de la siguiente ejecucion
		osDelayUntil(nextTick);
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
				break;
			}
			switch (TeR.config.traction_control) {

			case TER_ECU_CONFIG_TRACTION_CONTROL_OFF_CHOICE:
				DriveConfig.tractionControl= &tractionControlOFF;
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
	return trqMap;
}

//------------------------------------------------[Basic traction Control]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t tractionControlOFF(trqMap_t in) {
	return in;
}

//MANDATORY USE IN EACH DRIVINGMODE
trqMap_t torqueCheck(trqMap_t in, trq_t limit, trq_t maxNegTrq) { //wrapper function that enables or disables negative torque up to a certain value.

	//1) First check if wheels are spinning at THR speed and negative torque is being requested (avoids backwards speed on wheel)
	if (TeR.wheelInfo.speed < 0 && (in.rLeft < 0 || in.rRight < 0)) {
		in.rLeft = 0;
		in.rRight = 0;
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

