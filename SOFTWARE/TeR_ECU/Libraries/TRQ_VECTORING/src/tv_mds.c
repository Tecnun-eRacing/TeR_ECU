/*
 * tv_mds.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Piero, Telmo Martinez de Salinas, Ozuba
 */

#include "tv_mds.h"

// todo: La velocidad hay que sacarla mejor de otro sitio ya que de la ruedas no me mola (torque que se autoafecta a si mismo loop chungo)
pid_t *tvPid; //Estructura del PID
float dTorque = 0;
float iMax = 1000; // valor arbitrario, toca testing, esto va hardcoded porque no se deberia de tocar
float looptime = 2; //ms of looptime (same as TeR_TRQMANAGER task

//--------------------------------------------------------[Model Functions]---------------------------------------------------------------//
float yawRef(float steer, float vx) { //STEER ENTRA EN GRADOS, PERO EN LA FORMULA DE YAWRATE ENTRA EN RADIANES
	//girar izq es positivo, realmente es el angulo de giro mediodel modelo bici
	steer = -steer;
	steer = ((steer < STEER_DEADZONE / DEG2RAD)
			&& (steer > -STEER_DEADZONE / DEG2RAD)) ? 0 : steer; // check if steering angle is within the defined deadzone
	return (steer * vx) / ((L_FRONT + L_REAR) + K_U * (vx * vx)); //unidades rad/seg
}

float mz2DeltaTorque(float alpha) { //Takes PID output (Toca revisar unidades de salida del PID)
	return I_ZZ * (2 * R_WHEEL) / (GEAR_R * T_WIDTH) * alpha;
}

//--------------------------------------------------------[Model Functions]---------------------------------------------------------------//

trqMap_t trqVectoring(trq_t limit) {
	if (!tvPid) { // Init pid if not enabled
		tvPid = initPID(((float) TeR.config.trq_kp / 10000.0f),
				((float) TeR.config.trq_ki / 10000.0f),
				((float) TeR.config.trq_kd / 10000.0f), looptime, iMax);
	};

	//Declares a trqMap
	trqMap_t trqMap;
	dTorque = 0;

	//check if car is not at speed, or pedal is not being pressed, if true Reset PID and LINEAR RESPONSE
	//Not in conditions for Torque Vectoring || ((TeR.steer.angle < STEER_DEADZONE) && (TeR.steer.angle > -STEER_DEADZONE)
	if (((TeR.wheelInfo.speed < ACTSPEED) || (TeR.apps.apps_av < ACTAPPS))) { // if below activation speed or pedal below threshold, or steering not turning, return linear response and clear pid error
		trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
		trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
		tvPid->error = 0; //clear P error
		tvPid->errorI = 0; // clear I error
		tvPid->errorD = 0; // clear D error
		tvPid->prevError = 0;
		dTorque = 0;
		trqMap = torqueCheck(trqMap, limit, 0); //no negative torque allowed
		return trqMap; //return tv output
	}
	//Torque Vectoring Available
	//Torque Vectoring Computation
	float ref = yawRef(TeR.steer.angle * DEG2RAD, TeR.wheelInfo.speed * KMH2MS);
	float imuYawR = IMU.w_z * DEG2RAD; // Imu yawRate a radianes
	float corr = pid(tvPid, ref, imuYawR); //Computa el lazo y devuelve el valor de correccion
	dTorque = mz2DeltaTorque(corr); //es una ganancia sin mas, no aporta al control

	//SAFETY CHECKS
	//check if dTorque is in allowable range IF NOT CLAMP DTORQUE TO MAX VALUE
	dTorque = dTorque > MAX_DELTA_TORQUE ? MAX_DELTA_TORQUE : dTorque;
	dTorque = dTorque < - MAX_DELTA_TORQUE ? - MAX_DELTA_TORQUE : dTorque;
	//Fill trqMap structure with dTorque
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5) + dTorque / 2;
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5) - dTorque / 2;
	trqMap = torqueCheck(trqMap, limit, 0); //negative torque is allowed (limit/4, totalmente random)
	return trqMap; //return tv output
}

uint8_t tv_initPID(float Kp, float Ki, float Kd, float iMax) {
	if (!areGainsInRange(Kp, Ki, Kd)) { // if any gain not in predefined range, return 0 gain (prevents disaster)
		Kp = 0;
		Ki = 0;
		Kd = 0;
	}
	tvPid = initPID(((float) TeR.config.trq_kp / 10000.0f),
			((float) TeR.config.trq_ki / 10000.0f),
			((float) TeR.config.trq_kd / 10000.0f), looptime, iMax);
	return 1;
}
uint8_t tv_deInitPID(void) {
	if (tvPid) { // is tvPid pointing to something not 0?
		deInitPID(&tvPid); // if yes free and set to NULL
	}
	return 1;
}

uint8_t areGainsInRange(float Kp, float Ki, float Kd) {
	if (Kp < 0 || Ki < 0 || Kd < 0) {
		return 0;
	} else if (Kp > KPMAX || Ki > KIMAX || Kd > KDMAX) {
		return 0;
	}
	return 1;
}

