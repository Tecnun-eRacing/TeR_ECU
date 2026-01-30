/*
 * torqueManager.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Ozuba
 *      Contributors: Piero
 *
 */

#include "TeR_TRQMANAGER.h"
#include "tv_mds.h"

/* RTOS
 * - La ejecución temporizada con osDelayUntil(), debido a que es la forma mas correcta de realizar
 *		 ejecuciones temporizadas sin desfase
 */

const static int task_period = 5; // Task frequency

extern trqMap_t trqDistribution(trq_t limit);

trqPipeline_t DriveConfig; //Configuración en uso
extern osThreadId_t trqManagerTaskHandle; // thread id of trqManager task

void trqManager(void *argument) { // Corre las etapas del pipeline y solicita la comanda
	uint32_t nextTick = osKernelGetTickCount(); // Initialize reference time

	for (;;) {
		nextTick += task_period; //Genera el timestamp de la siguiente ejecucion
		osDelayUntil(nextTick); // esperamos
		//Check if we are driving
		if (TeR.status.state == DRIVING) {
			//Execute Pipeline
			trq_t availableTorque = DriveConfig.limiter(); //Limita (proveniente de config)

			trqMap_t driveTorque = DriveConfig.drivingMode(availableTorque); //Distribuye torque proveniente de input

			driveTorque = DriveConfig.regenMode(driveTorque); //Distribuye torque teniendo en cuenta modo de regeneración (permite / otorga regeneración)

			trqMap_t trqToSanity = DriveConfig.tractionControl(driveTorque); // Limita si se vé que la rueda no puede dar ese grip (Estrategia dependiente del controlador)

			trqMap_t trqToWheels = DriveConfig.sanityChecks(trqToSanity,
					availableTorque); // basicamente que no se vaya para atras y que no pidas 300Nm, entre otras cosas un test que me permite dormir algo mas tranquilo
			//Solicitud de la comanda
			TeR.trqReqLeft.torque_nm_req = trqToWheels.rLeft;
			TeR.trqReqRight.torque_nm_req = trqToWheels.rRight;

		} else {
			//Config the driving pipeline if not driving
			switch (TeR.config.limiter) {

			case TER_ECU_CONFIG_LIMITER_TORQUE_CHOICE:
				DriveConfig.limiter = &limitTorque;
				break;
			default:
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
			default:
				DriveConfig.drivingMode = &lineal;
				break;

			}
			switch (TeR.config.traction_control) {

			case TER_ECU_CONFIG_TRACTION_CONTROL_OFF_CHOICE:
				DriveConfig.tractionControl = &tractionControlOFF;
				break;
			default:
				DriveConfig.tractionControl = &tractionControlOFF;
				break;
			}

			switch (TeR.config.regen_mode) {
			case TER_ECU_CONFIG_REGEN_MODE_APPS_CHOICE:
				DriveConfig.regenMode = &regenModeAPPS;
				break;
			case TER_ECU_CONFIG_REGEN_MODE_DV_CHOICE:
				DriveConfig.regenMode = &regenModeDV;
				break;
			default:
				DriveConfig.regenMode = &regenModeAPPS;
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
	trq_t limit = TeR.config.trq_limit;
	limit = limit > 180 ? 180 : TeR.config.trq_limit; // hard limit, lo siento piloto, no te voy a dar 200Nm
	limit = limit < 10 ? 10 : TeR.config.trq_limit;
	return limit; //Devuelve el valor configurado
}

//------------------------------------------------[Basic Driving Modes]------------------------------------------------//
// trq_t -> trqMap_t
trqMap_t lineal(trq_t limit) { //Entrega lineal de par a las 2 ruedas,se toman valores del APPS como source
	trqMap_t trqMap = {0};
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5); // 0.5 porque tenemos 2 ruedas
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
	return trqMap;
}
// trq_t -> trqMap_t
trqMap_t remoteTrqRequest(trq_t limit) { // aceptar request remotas provenientes del DV
	trqMap_t trqMap = {0};
	float requested_trq = ter_dv_dynamic_req_trq_req_decode(TeR.dv_dynamic_req.trq_req) / 2; // per wheel, importante el decode por factor de escala del DBC
	requested_trq = mapf(requested_trq, -1.0f, 1.0f, -limit, limit); // OJO, no mapees entre max regen y limit, la vas a liar basto!! (0 no sería 0 trq!!!)
	requested_trq = requested_trq / 2; // 2 wheels
	trqMap.rLeft = (trq_t)requested_trq;
	trqMap.rRight = (trq_t)requested_trq;
	return trqMap;
}

//------------------------------------------------[Basic traction Control]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t tractionControlOFF(trqMap_t in) {
	return in;
}
//------------------------------------------------[Basic Regen]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t regenModeAPPS(trqMap_t in) {
	if (!regen_allowed(in)) {
		in.rLeft = in.rLeft < 0 ? 0 : in.rLeft;
		in.rRight = in.rRight < 0 ? 0 : in.rRight;
	}
	if (!((in.rLeft <= TeR.config.regen_max_positive_trq_thr / 2) // threshold de torque pedido a partir del cual consideramos "lift" del pedal /TODO cambiar a valor de apps
	&& (in.rRight <= TeR.config.regen_max_positive_trq_thr / 2)))
		return in;
	int8_t trq = TeR.config.regen_max_trq;
	trq = -abs(trq / 2);
	in.rLeft = trq;
	in.rRight = trq;
	return in;
}
//------------------------------------------------[DV Regen (allow dv to request negative torque within limits)]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t regenModeDV(trqMap_t in) {
	if (!regen_allowed(in)) {
		in.rLeft = in.rLeft < 0 ? 0 : in.rLeft;
		in.rRight = in.rRight < 0 ? 0 : in.rRight;
	}
	int8_t limit = TeR.config.regen_max_trq / 2; // 2 wheels
	in.rLeft = in.rLeft < limit ? limit : in.rLeft; // clampeamos a límite
	in.rRight = in.rRight < limit ? limit : in.rRight;
	return in; //retornamos pedido
}

/*
 * This function is used as a sanity check in the last stage of the pipeline
 * This function checks if somehow you managed to place a trq request that is impossible to fulfill:
 * 1) You tried to request negative torque with the regen disabled -> you may cause accumulator faults / overvoltages / overtemps / you did not want to regen
 * 2) You tried to request negative torque below a threshold speed -> you may cause the wheels to spin backwards -> insta DQ and could be VERY dangerous
 * 3) You tried to exceed the power limitation of the vehicle (bug in drivingMode or regenMode function)
 *
 * All relevant checks should be performed in your DrivingMode and RegenMode functions, do not rely exclusively on sanity checks
 */
trqMap_t trqCheck(trqMap_t in, trq_t limit) {

// 1) Check if regen is allowed, if not, set negative requests to 0
	if (!regen_allowed(in)) {
		in.rLeft = in.rLeft < 0 ? 0 : in.rLeft;
		in.rRight = in.rRight < 0 ? 0 : in.rRight;
	}

// 2) Check if negative torque is being requested below activation speed, VERY IMPORTANT (avoids backwards spinning of the wheels)
	if (in.rLeft < 0 || in.rRight < 0) {
		if (TeR.wheelInfo.rl_rpm < TeR.config.regen_thr_rpm) { // responsabilidad tuya si pones los limites de forma incorrecta.
			in.rLeft = 0;
		}
		if (TeR.wheelInfo.rr_rpm < TeR.config.regen_thr_rpm) {
			in.rRight = 0;
		}
		if (TeR.wheelInfo.speed < TeR.config.regen_thr_speed) {
			in.rRight = 0;
			in.rLeft = 0;
		}
		return in; //return 0 torque as negative torque is being requested with below security speed/rpms
	}

// 3) Check if somehow torque limitation has been exceded and clamp
	int32_t total = abs(in.rLeft) + abs(in.rRight);
	if ((total > limit) && (total != 0)) { // prevent stupid and impossible case when a division by 0 could occur
		float scale = (float) limit / total;
		in.rLeft = (trq_t) (in.rLeft * scale); // scale and clamp
		in.rRight = (trq_t) (in.rRight * scale);
		return in;
	}
	return in;
}

uint8_t regen_allowed() { // 1 ok 0 not ok
	if (!(TeR.config.regen_enable == TER_ECU_CONFIG_REGEN_ENABLE_ENABLE_CHOICE)) // regen activada?
		return 0;
	if (!(hvbms_bms_tx_state_6_cell_max_v_decode(TeR.BmsCellsVolt.cell_min_v) // celdas en rango de tension? (pone min porque el dbc del bms estaba al revés, cuando lo arreglen lo cambio TODO
	< TeR.config.regen_max_cell_volt))
		return 0;
	if (!(hvbms_bms_tx_state_9_cell_temp_max_deg_c_decode( // celdas en rango de temperatura?
			TeR.BmsCellsTemp.cell_temp_max_deg_c)
			< TeR.config.regen_max_cell_temp))
		return 0;
	if (!(hvbms_bms_tx_state_4_curr_2_x10_a_decode(TeR.BmsCurrent.curr_2_x10_a) // accu en rango de corriente ?
	> -TeR.config.regen_max_current)) // always set below your max accumulator regen current, currently is 80A so this should be 60A or so
		return 0;
	// si se han coumplido todas las condiciones necesarias para regenerar, retornamos 1
	return 1;
}

