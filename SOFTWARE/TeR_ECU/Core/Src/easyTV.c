/*
 * easyTV.c
 *
 *  Created on: Dec 7, 2024
 *      Author: Piero
 */
//ESTO ES UN CONCEPTO
//polinomial based torque vectoring
//the main idea is that a function provides us with a range of possible
//values for deltaT depending on the speed of the vehicle and the
//steering angle
//ostrich algorithm in junction with "confia" method will be used for tunning
#include <easyTV.h>
trqMap_t easyTorque(trq_t limit){
trqMap_t trqMap;
float output = 0;
float deltaTorque = 0;
uint8_t a,b,c,d;//we will read this from the eeprom
output=a*exp(-(TeR.wheelInfo.speed-b)/(c))+d; // use a=1.3 b=-140 c=-79 d=-4.1, esta funcion me mola porque puedes modificar muchos parametros de la curva
deltaTorque=(TeR.steer.angle*output)/30; //from the maximum deltatorque that we can have, multiply by a factor -1<f<1
float gas = TeR.apps.apps_av/255.0;
trqMap.rLeft=(limit/2)*gas+deltaTorque;
trqMap.rRight=(limit/2)*gas-deltaTorque;
if(trqMap.rLeft<0 || trqMap.rRight<0){
	trqMap.rLeft = 0;
	trqMap.rRight = 0;
	return trqMap;
}
if(trqMap.rLeft+trqMap.rRight>limit){
	trqMap.rLeft = 0;
	trqMap.rRight = 0;
	return trqMap;
}
}
