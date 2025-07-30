/*
 * TeR_UTILS.c
 *
 *  Created on: Jun 20, 2024
 *      Author: Ozuba
 */
#include "TeR_UTILS.h"

//Comprueba que un error sucede durante más de tMax
uint8_t checkPersistance(persist_t *instance, uint8_t ok, uint32_t tMax) {

	if (*instance > 0) { //Estabamos en error
		if (ok) { //No tenemos error
			*instance = 0; //Ponemos el timestamp a 0, ya no hay error
		} else if (osKernelGetTickCount() - *instance >= tMax) { //El error supera maxtime
			return 0; //Damos el error
		}
	} else if (!ok) { // no estabamos en error y ahora si
		*instance = osKernelGetTickCount();
	}

	return 1; //Tenemos Error pero no hemos superado maxTime
}

// Mapea un intervalo
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max) {
//Saturar las salidas si la entrada excede el límite de calibracion
	if (x < in_min)
		return out_min;
	if (x > in_max)
		return out_max;
//Mapear si estamos en rango seguro
	long val = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	return val;
}
uint8_t read_btn(uint8_t *lock, uint8_t read) {
	if (osKernelGetTickCount() < 3000) {
		return 0;
	}
	if (*lock && read) { //Si bloquado y boton == 1, devuelvo 0
		return 0;
	}
	//Si no hay lock o no estaba a 1 actualizo
	//bloqueo si pulsado
	//No hago nada sin no pulsado
	*lock = read;
	return read;
}
