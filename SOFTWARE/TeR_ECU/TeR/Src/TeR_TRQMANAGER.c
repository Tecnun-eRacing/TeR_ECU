/*
 * torqueManager.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Ozuba
 *
 */

#include "TeR_TRQMANAGER.h"
const static task_period = 10; // Task frequency 100hz

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
	return (trq_t) TeR.dynamicConfig.trq_limit; //Devuelve el valor configurado
}

//Potencia mecanica constante
trq_t limitMechPWR(void) {
	int32_t meanRPM = (TeR.wheelInfo.rl_rpm + TeR.wheelInfo.rl_rpm) / 2; //RPMs medias
	if (meanRPM > 0) { //Estamos moviendonos se estima el torque desarrollable
		return (trq_t) (TeR.dynamicConfig.kw_limit * ELEC2MECH_EFF) / meanRPM; //Devolvemos la potencia desarrollable limitada en potencia
	} else if (meanRPM == 0) { //Estamos quietos luego se devuelve la limitación de torque máximo
		return (trq_t) TeR.dynamicConfig.trq_limit;
	}
	return 0;
}

//Velocidad máxima
trq_t limitSpeed(void) {
//Reduce el torque que puedes dar conforme te acercas al valor de speed (Puede generar problematica oscilatoria)
//Velocidad aumenta -> torque dismunuye -> Velocidad Disminuye-> Torque aumenta
//Usar PID

	return 0;
}

//Limit Electrical Power (Dato de los Inverters Controller)
trq_t limitElecPWR(void) {
	return 0;
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
	if (TeR.wheelInfo.speed < 20 && (in.rLeft < 0 || in.rRight < 0)) {
		in.rLeft = 0;
		in.rRight = 0;
		return in; //return 0 torque as negative torque is being requested with below security speed
	}

	maxNegTrq =
			maxNegTrq > limit ? limit : maxNegTrq; //check if negative allowance is in limit and if not clamp it (not necessary)

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

