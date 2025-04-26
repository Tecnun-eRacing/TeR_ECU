/*
 * TeR_SERVOS.h
 *
 *  Created on: Apr 1, 2025
 *      Author: heliosestevezorozco
 */

#ifndef INC_TER_SERVOS_H_
#define INC_TER_SERVOS_H_


typedef struct {
	string SERVO_R_S;
	string SERVO_R_I;
	string SERVO_L_S;
	string SERVO_L_I;
}  servo_t;

typedef struct { //variables de configuracion del servo
	//GPIO LETTER
	//GPIO PIN
	//TIMER Hz
	// we should have a max-min value treshold for each servo individualy to avoid colissions with the mechanical buffer and excess current drawning, these values will be checked with a DSO and reduced by 0.1 ms */
} servo_cfg_t;

//function declaration

void servos(void *argument);

/*Solve for Prescaler (PSC) with an Auto-Reload Register (ARR) value of 65535 (16 bits, max resolution)*/


#endif /* INC_TER_SERVOS_H_ */

