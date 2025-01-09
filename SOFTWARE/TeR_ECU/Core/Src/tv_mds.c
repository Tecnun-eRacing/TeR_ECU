/*
 * tv_mds.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Ozuba, Telmo Martinez de Salinas
 */

#include "tv_mds.h"

pid_t tvPid; //Estructura del PID





float yawRef(float steer, float vx) {
	return (steer * vx) / ((L_FRONT + L_REAR) + K_U * (vx * vx));
}

float mz2DeltaTorque(float alpha) { //Takes PID output (Toca revisar unidades de salida del PID)
	return I_ZZ * (2 * R_WHEEL) / (GEAR_R * T_WIDTH) * alpha;
}


float trqDistribution() {
	float dTorque = 0;
	//Compute Torque
	float ref = yawRef(TeR.steer.angle, TeR.speed.vx_av);
	float imuYawR = ter_ang_rate_yaw_rate_z_decode(TeR.angRate.yaw_rate_z);
	float corr = pid(&tvPid, ref, imuYawR); //Computa el lazo y devuelve el valor de correccion
	dTorque = mz2DeltaTorque(corr);

	//Compute Torque output
	float gas = TeR.apps.apps_av / 255.0; //Comanda de 0-1 de gas

	TeR.trqReqRight.torque_nm_req = gas * 180 / 2 + dTorque / 2;
	TeR.trqReqLeft.torque_nm_req = gas * 180 / 2 - dTorque / 2;
return 1;
}

