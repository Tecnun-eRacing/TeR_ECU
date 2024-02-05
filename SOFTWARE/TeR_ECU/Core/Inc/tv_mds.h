/*
 * tv_mds.h
 *
 *  Created on: Jan 31, 2024
 *      Author: Ozuba Telmo Martinez de Salinas
 */
#include "TeR_CAN.h"
#ifndef INC_TV_MDS_H_
#define INC_TV_MDS_H_

#define DEG2RAD 0.0174533 //Degs to radians

/////////////////////////////////////////[Constantes del Vehiculo]/////////////////////////////////////////////////////////////

#define I_ZZ 122.0 //Momento de inercia en eje Z (kg*M^4) (Modelo Juan Gastaminza)
#define T_WIDTH 1.185 //Track width REAR (m)
#define L_FRONT 0.806 //A Distance(from front axle to CDG) (m)
#define L_REAR 0.744 //B Distance (from CDG to rear axle)  (m)
#define H_CDG 0.27   //height of the CDG (en estatico)  (m)
#define GEAR_R 5     //Indice de Reducccion
#define R_WHEEL 0.2023 //Radio de la rueda (m)

#define K_U 0 //Gradiente de subviraje objetivo (rad)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {

	//State Variables
	float error; //Error proporcional
	float errorD; //Error derivativo
	float errorI;//Error integral

	//Config Varialbes
	float Kp;  // Proportional gain
	float Ki;   // Integral gain
	float Kd;  // Derivative gain

	float prevError;
	float antiWindup; //Valor saturacion termino integral
	float T; //Tiempo del lazo
}pid_t;




float yawRef(float steer, float vx); //Funcion que toma angulo de rueda y velocidad de avance y devuelve referencia de giro yawrate
float pid(pid_t* pid, float yawRateRef, float imuYawRate);
float mz2DeltaTorque(float alpha);

float trqDistribution();


#endif /* INC_TV_MDS_H_ */
