/*
 * stateMachine.h
 *
 *  Created on: Feb 1, 2024
 *      Author: Ozuba
 *
 * Se ha elegido el aproach de consultar todas las condiciones antes de ejecutar el estado
 */

#ifndef INC_TER_STATEMACHINE_H_
#define INC_TER_STATEMACHINE_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "TeR_CAN.h"
#include "TeR_TRQMANAGER.h"
#include "TeR_REFRI.h"
#include "TeR_CONSTANTS.h"
#include "TeR_UTILS.h"

typedef enum {
	WAIT_SL, RDY2PRECH, PRECHARGING, PRECHARGED, DRIVING
} state_t; //Estados

uint8_t initStateMachine(TIM_HandleTypeDef* htim); //Arranca la maquina con un timer
state_t getState(void); //Determina el estado en función de las lecturas y el can
void stateMachine(TIM_HandleTypeDef* beat); //ejecuta el estado actual

//Estados
void waitSL(void); // Comprueba SL esta bien
void rdy2Prech(void); // Espera a recibir el comando de precarga
void precharging(void); //Estado transitorio, monitoriza que todo va bien
void precharged(void); //Espera a que se reciba el comando de r2d
void driving(void); //Ejecuta la comanda de par

//Permanent Task
void permaTask(); //Se ejecuta en todos los estados (conversiones/brakelight...)

#endif /* INC_TER_STATEMACHINE_H_ */
