/*
 * tv_mds.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Ozuba, Telmo Martinez de Salinas
 */

#include "tv_mds.h"

pid_t tvPid; //Estructura del PID

float yawRef(float steer, float vx) {
	return (steer * vx) / ((L_FRONT + L_REAR) + K_U * (vx * vx)); //unidades rad/seg
}

float mz2DeltaTorque(float alpha) { //Takes PID output (Toca revisar unidades de salida del PID)
	return I_ZZ * (2 * R_WHEEL) / (GEAR_R * T_WIDTH) * alpha;
}


trqMap_t trqDistribution(trq_t limit) {
	trqMap_t trqMap;
	if(TeR.wheelInfo.speed>10){ // hopes to save pilot from never ending fidget spinner
	float dTorque = 0;
	//Compute Torque
	float ref = yawRef(TeR.steer.angle, TeR.speed.vx_av); // tenemos el Mutex, podemos leer de forma segura
	float imuYawR = ter_ang_rate_yaw_rate_z_decode(TeR.angRate.yaw_rate_z); // valor real de yawRate
	float corr = pid(&tvPid, ref, imuYawR); //Computa el lazo y devuelve el valor de correccion
	dTorque = mz2DeltaTorque(corr);
	//Compute Torque output
	float gas = TeR.apps.apps_av/ 255.0; //Comanda de 0-1 de gas
	trqMap.rRight = gas * limit / 2 + dTorque / 2;
	trqMap.rLeft = gas * limit / 2 - dTorque / 2;
return trqMap;
	}
	else{
		trqMap.rRight = limit/2;
		trqMap.rLeft = limit/2;
	}

}
