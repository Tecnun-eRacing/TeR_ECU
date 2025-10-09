/*
 * tv_mds.h
 *
 *  Created on: Jan 31, 2024
 *      Author: Piero,Ozuba Telmo Martinez de Salinas
 */


#ifndef INC_TV_MDS_H_
#define INC_TV_MDS_H_

#include "TeR_TRQMANAGER.h"
#include "TeR_INERTIAL.h"
#include "TeR_CAN.h" //For controlling TeR vehicle
#include <math.h>
//#include "pid.h"
#define SAFETY
#define DEG2RAD PI/180.0f //Degs to radians
#define KMH2MS 1/3.6f
/////////////////////////////////////////[Constantes del Vehiculo]/////////////////////////////////////////////////////////////

#define I_ZZ 122.0 //Momento de inercia en eje Z (kg*M^4) (Modelo Juan Gastaminza)
#define T_WIDTH 1.185 //Track width REAR (m)
#define L_FRONT 0.806 //A Distance(from front axle to CDG) (m)
#define L_REAR 0.744 //B Distance (from CDG to rear axle)  (m)
#define H_CDG 0.27   //height of the CDG (en estatico)  (m)
#define GEAR_R 5.0     //Indice de Reducccion
#define R_WHEEL 0.2023 //Radio de la rueda (m)
#define K_U 0.0 //Gradiente de subviraje objetivo (rad)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////[Seguridad]/////////////////////////////////////////////////////////////
#define ACTSPEED 5.0 // speed in kmh for threshold
#define ACTAPPS 10.0 // 2 apps (0-255) uint8_t
#define STEER_DEADZONE 10.0*DEG2RAD
#define KPMAX 1000.0 // arbitrary
#define KIMAX 1000.0
#define KDMAX 100.0
#define MAX_DELTA_TORQUE 20




float yawRef(float steer, float vx); //Funcion que toma angulo de rueda y velocidad de avance y devuelve referencia de giro yawrate
float mz2DeltaTorque(float alpha);
trqMap_t trqVectoring(trq_t limit);
uint8_t tv_initPID(float Kp,float Ki, float Kd,float iMax);//wrapper function
uint8_t tv_deInitPID(void);
uint8_t areGainsInRange(float Kp, float Ki, float Kd);
uint8_t isAngleInDeadzone(float angle_deg,float range);
#endif /* INC_TV_MDS_H_ */
