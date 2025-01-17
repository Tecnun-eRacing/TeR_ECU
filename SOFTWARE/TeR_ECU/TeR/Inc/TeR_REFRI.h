/*
 * TeR_REFRI.h
 *
 *  Created on: Jun 30, 2024
 *      Author: ozuba
 */
//Fichero encargado de la gestión del sistema de refrigeración
//Implementa por ahora un controlador de histeresis
#ifndef INC_TER_REFRI_H_
#define INC_TER_REFRI_H_

#include "TeR_UTILS.h"
#include "TeR_CAN.h"
#include "TeR_COMMAND.h"


#define ON_THRESHOLD 50 //Enciende a 50 grados
#define OFF_THRESHOLD 40 //Apaga a 40 grados

void refriManager();// Ejecuta el lazo de refri


#endif /* INC_TER_REFRI_H_ */
