/*
 * pid.c
 *
 *  Created on: Mar 30, 2024
 *      Author: ozuba
 *
 * A simple PID library using floating arithmetic for its usage all along the control systems
 * of the car. (Torque Vectoring)
 */







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
