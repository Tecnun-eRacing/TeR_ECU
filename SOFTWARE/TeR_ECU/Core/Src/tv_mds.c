/*
 * tv_mds.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Ozuba Telmo Martinez de Salinas
 */

#include "tv_mds.h"



float yawRef(float steer,float vx){
return (steer*vx)/((L_FRONT+L_REAR)+K_U*(vx*vx));
}


float mz2DeltaTorque(float alpha){ //Takes PID output (Toca revisar unidades de salida del PID)
	return I_ZZ*(2*R_WHEEL)/(GEAR_R*T_WIDTH)*alpha;
}

float pid(pid_t* pid,float yawRateRef,float imuYawRate){

//PID
pid->error = (yawRateRef - imuYawRate); //Proporcional
pid->errorI += pid->error * pid->T; //Integral
pid-> errorD = (pid->error - pid->prevError)/pid->T; //Derivativo
//Antiwindup check
pid->errorI = (pid->errorI < pid->antiWindup) ? pid->errorI : +pid->antiWindup; //Upper bound check
pid->errorI = (pid->errorI > -pid->antiWindup) ? pid->errorI : -pid->antiWindup; //Lower bound check


return pid->Kp * pid->error + pid->Ki * pid->errorI + pid->Kd *pid->errorD; // P + I + D
}


