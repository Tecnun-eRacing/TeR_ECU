/*
 * tv_mds.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Piero, Telmo Martinez de Salinas, Ozuba
 */

#include "tv_mds.h"
uint8_t angle = 0;
float dTorque = 0;
// la velocidad hay que sacarla mejor de otro sitio ya que de la rueda no me mola
pid_t *tvPid; //Estructura del PID
float Kp = 0; // temporal, leeremos valores de la estructura TeR
float Ki = 0; // temporal, leeremos valores de la estructura TeR
float Kd = 0; // temporal, leeremos valores de la estructura TeR
float iMax = 1; // limitado a mas o menos 60 grados por segundo (1 rad/s 60 grad seg aprox)

//--------------------------------------------------------[Model Functions]---------------------------------------------------------------//
float yawRef(float steer, float vx) { //STEER EN RADIANES
	//girar izq es positivo, realmente es el angulo de giro mediodel modelo bici
	steer = (steer < STEER_DEADZONE && steer > -STEER_DEADZONE) ? 0 : steer; // check if steering angle is within the defined deadzone
	return (steer * DEG2RAD * vx) / ((L_FRONT + L_REAR) + K_U * (vx * vx)); //unidades rad/seg
}

float mz2DeltaTorque(float alpha) { //Takes PID output (Toca revisar unidades de salida del PID)
	return I_ZZ * (2 * R_WHEEL) / (GEAR_R * T_WIDTH) * alpha;
}

//--------------------------------------------------------[Model Functions]---------------------------------------------------------------//

trqMap_t trqVectoring(trq_t limit) {
	if (!tvPid) {// Init pid if not enabled
		tvPid = initPID(((float)TeR.config.trq_kp/1000.0),((float)TeR.config.trq_ki/1000.0),((float)TeR.config.trq_kd/1000.0),10, 10);
	};
	//Declares a trqMap
	trqMap_t trqMap;
	dTorque = 0;

	//check if car is not at speed, or pedal is not being pressed, if true Reset PID and LINEAR RESPONSE
	//Not in conditions for Torque Vectoring
	if (TeR.wheelInfo.speed < 0 || TeR.apps.apps_av < 0) { // if below activation speed or pedal below threshold, return linear response and clear pid error
		trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
		trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
		tvPid->error = 0; //clear P error
		tvPid->errorI = 0; // clear I error
		tvPid->errorD = 0; // clear D error
		tvPid->prevError = 0;
		trqMap = torqueCheck(trqMap, limit, 0); //no negative torque allowed
		return trqMap; //return tv output
	}
	//Torque Vectoring Available
	//Torque Vectoring Computation
	float ref = yawRef(angle, TeR.wheelInfo.rl_rpm);
	float imuYawR = IMU.w_z * DEG2RAD; // Imu yawRate a
	float corr = pid(tvPid, ref, imuYawR); //Computa el lazo y devuelve el valor de correccion
	dTorque = mz2DeltaTorque(corr); //gets t

	//SAFETY CHECKS
	//check if dTorque is in allowable range IF NOT CLAMP DTORQUE TO MAX VALUE
	dTorque = dTorque > MAX_DELTA_TORQUE ? MAX_DELTA_TORQUE : dTorque;
	dTorque = dTorque < - MAX_DELTA_TORQUE ? - MAX_DELTA_TORQUE : dTorque;
	//Fill trqMap structure with dTorque
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5) + dTorque / 2;
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5) - dTorque / 2;
	trqMap = torqueCheck(trqMap, limit, limit); //no negative torque allowed
	return trqMap; //return tv output
}

uint8_t tv_initPID(float Kp, float Ki, float Kd, float iMax) {
	if (!areGainsInRange(Kp, Ki, Kd)) { // if any gain not in predefined range, return 0 gain (prevents disaster)
		Kp = 0;
		Ki = 0;
		Kd = 0;
	}
	tvPid = initPID(Kp, Ki, Kd, 10, iMax);
	return 1;
}
uint8_t tv_deInitPID(void) {
	deInitPID(tvPid);
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

