/*
 * tv_mds.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Ozuba, Telmo Martinez de Salinas
 */

#include "tv_mds.h"


// la velocidad hay que sacarla mejor de otro sitio ya que de la rueda no me mola
pid_t *tvPid; //Estructura del PID
float Kp = 0; // temporal, leeremos valores de la estructura TeR
float Ki = 0; // temporal, leeremos valores de la estructura TeR
float Kd = 0; // temporal, leeremos valores de la estructura TeR
float loopTime = 0.002f; //corremos statemachine cada 2 millis
float iMax = 1; // limitado a mas o menos 60 grados por segundo (1 rad/s 60 grad seg aprox)
float yawRef(float steer, float vx) { //STEER EN RADIANES
	return (steer * DEG2RAD * vx) / ((L_FRONT + L_REAR) + K_U * (vx * vx)); //unidades rad/seg
}

float mz2DeltaTorque(float alpha) { //Takes PID output (Toca revisar unidades de salida del PID)
	return I_ZZ * (2 * R_WHEEL) / (GEAR_R * T_WIDTH) * alpha;
}

trqMap_t trqDistribution(trq_t limit) {
	trqMap_t trqMap;
	float dTorque = 0;
	//Compute Torque
	float ref = yawRef(TeR.steer.angle, TeR.speed.vx_av); // tenemos el Mutex, podemos leer de forma segura
	float imuYawR = ter_ang_rate_yaw_rate_z_decode(TeR.angRate.yaw_rate_z); // valor real de yawRate
	float corr = pid(tvPid, ref, imuYawR); //Computa el lazo y devuelve el valor de correccion
	dTorque = mz2DeltaTorque(corr);
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5)-dTorque/2;
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5)+dTorque/2;
		if(TeR.speed.vx_av < ACTSPEED || TeR.apps.apps_av < ACTAPPS){ // if below activation speed or pedal below threshold, return linear response and clear pid error
		trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5);
		trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5);
		tvPid->error = 0; //clear P error
		tvPid->errorI = 0; // clear I error
		tvPid->errorD = 0; // clear D error
		tvPid->prevError = 0;}  //clear previous error
	trqMap=torqueCheck(trqMap,0,limit); //no negative torque allowed
	return trqMap; //return tv output
}
uint8_t tv_loadGains(float Kp,float Ki, float Kd,float loopTime,float iMax){
	 tvPid=initPID(Kp,Ki,Kd,loopTime,iMax);
	 return 0;
}
