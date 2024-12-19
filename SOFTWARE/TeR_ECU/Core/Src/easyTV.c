/*
 * easyTV.c
 *
 *  Created on: Dec 7, 2024
 *      Author: Piero
 */
//ESTO ES UN CONCEPTO
// la velocidad hay que sacarla mejor de otro sitio ya que de la rueda no me mola
//polinomial based torque vectoring
//the main idea is that a function provides us with a range of possible
//values for deltaT depending on the speed of the vehicle and the
//steering angle
//ostrich algorithm in junction with "confia" method will be used for tunning
#include "easyTV.h"
static uint8_t actSpeed = 10; // activation speed of tv
static uint8_t a,b,c,d;//we will read this from the eeprom
trqMap_t easyTorque(trq_t limit){
trqMap_t trqMap;
float output = 0;
float deltaTorque = 0;
//output=-a*TeR.wheelInfo.speed+constante esta tambien puede estar interesante
//output=a*exp(-(TeR.speed.vx_av-b)/(c))+d; // use a=1.3 b=-140 c=-79 d=-4.1, esta funcion me mola porque puedes modificar muchos parametros de la curva
output=output < 0 ? 0:output;//if we are in the negative part of function return 0
deltaTorque=(TeR.steer.angle*output)/30; //from the maximum deltatorque that we can have, multiply by a factor -1<f<1
//float gas = TeR.apps.apps_av/255.0;
//trqMap.rLeft=(limit/2)*gas+deltaTorque;
//trqMap.rRight=(limit/2)*gas-deltaTorque;
trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5)+deltaTorque;
trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5)-deltaTorque;
if(trqMap.rLeft<0 || trqMap.rRight<0){ //safety
	trqMap.rLeft = 0;
	trqMap.rRight = 0;
	return trqMap; //returns 0
}
if(trqMap.rLeft+trqMap.rRight>limit){ // safety
	trqMap.rLeft = 0;
	trqMap.rRight = 0;
	return trqMap;} //returns 0
if(TeR.speed.vx_av > actSpeed){ // activates the torque response only if it has a certain speed
	return trqMap;}
else { //if not in the speed, lineal torque response
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5);
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5);
}
return trqMap; //lastly returns linear response
}
