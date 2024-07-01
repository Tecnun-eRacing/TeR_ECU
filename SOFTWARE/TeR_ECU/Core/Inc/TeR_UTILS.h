/*
 * TeR_UTILS.h
 *
 *  Created on: Jun 19, 2024
 *      Author: Ozuba
 */
//This file holds diverse code utils that dont fall under any category
#ifndef INC_TER_UTILS_H_
#define INC_TER_UTILS_H_

#include "stm32f4xx_hal.h"

// Boolean Persistance checker (Una función que comprueba que un error se da durante un cierto tiempo)

typedef uint32_t persist_t; // Almacena el timestamp de el flanco de subida

uint8_t checkPersistance(persist_t *instance, uint8_t ok, uint32_t tMax);

//MAP (Mape)
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max); //Función map muy util
#endif /* INC_TER_UTILS_H_ */
