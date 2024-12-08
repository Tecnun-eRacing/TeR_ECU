/*
 * tv_mds.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Ozuba, Telmo Martinez de Salinas
 */

#include "tv_mds.h"
static uint8_t actSpeed = 10; //fidget spinner prevention
// la velocidad hay que sacarla mejor de otro sitio ya que de la rueda no me mola
pid_t tvPid; //Estructura del PID
float yawRef(float steer, float vx) { //TODO creo que el steer hay que pasarlo a radianes
	return (steer * vx) / ((L_FRONT + L_REAR) + K_U * (vx * vx)); //unidades rad/seg
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
	float corr = pid(&tvPid, ref, imuYawR); //Computa el lazo y devuelve el valor de correccion
	dTorque = mz2DeltaTorque(corr);
	//Compute Torque output
	//float gas = TeR.apps.apps_av/ 255.0; //Comanda de 0-1 de gas
	//trqMap.rRight = gas * limit / 2 + dTorque / 2;
	//trqMap.rLeft = gas * limit / 2 - dTorque / 2;
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5)+dTorque/2;
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5)-dTorque/2;
	if(trqMap.rLeft<0 || trqMap.rRight<0){ //safety
		trqMap.rLeft = 0;
		trqMap.rRight = 0;
		return trqMap;}//returns 0 torque
	if(trqMap.rLeft+trqMap.rRight>limit){ // safety
		trqMap.rLeft = 0;
		trqMap.rRight = 0;
		return trqMap;}//returns 0 torque
	if(TeR.wheelInfo.speed > actSpeed){ // activates the torque response only if it has a certain speed
		return trqMap;} //returns tv output
	else { //if not in the speed, lineal torque response,
		trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5);
		trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5);
	}
	return trqMap; //returns lineal response as we are not travelling at speed
}
