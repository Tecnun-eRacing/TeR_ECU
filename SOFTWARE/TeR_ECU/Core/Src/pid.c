/*
 * pid.c
 *
 *  Created on: Mar 30, 2024
 *      Author: ozuba
 *
 * A simple PID library using floating arithmetic for its usage all along the control systems
 * of the car. (Torque Vectoring, Thermal Management...). It is thought to be running in a periodic interrupt
 * but it can be easily modified to perceive time using a timeBase timer..
 *
 *
 * -------------------------------------------------[Seguridad]--------------------------------------------
 * Esto es un controlador, alguien va montado encima, si haces el tonto, es peligroso. Esta librería
 * Implementa medidas de seguridad como la posibilidad de saturar individualmente los terminos
 * Integral Derivativo
 */
#include "pid.h"


pid_t* initPID(float Kp, float Ki, float Kd, float loopTime, float iMax) {
	pid_t *pid; //temporal pointer for init
	pid = (pid_t*) calloc(1, sizeof(pid)); //Allocates one pid instance Zeroes memory to prevent disaster
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->T = loopTime; //Tiempo del lazo
	pid->antiWindup = iMax; // Valor de saturación para el termino integral
	return pid;
}

void deInitPID(pid_t *pid) {
	free(pid); //Frees memory
}

float pid(pid_t *pid, float ref, float feedback) {
//PID
	pid->error = (ref - feedback); //Proporcional
	pid->errorI += pid->error * pid->T; //Integral
	pid->errorD = (pid->error - pid->prevError) / pid->T; //Derivativo
//Antiwindup check
	pid->errorI =
			(pid->errorI < pid->antiWindup) ? pid->errorI : +pid->antiWindup; //Upper bound check
	pid->errorI =
			(pid->errorI > -pid->antiWindup) ? pid->errorI : -pid->antiWindup; //Lower bound check

	return pid->Kp * pid->error + pid->Ki * pid->errorI + pid->Kd * pid->errorD; // P + I + D
}
