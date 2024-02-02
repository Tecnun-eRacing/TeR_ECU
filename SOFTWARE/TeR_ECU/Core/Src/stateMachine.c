/*
 * stateMachine.c
 *
 *  Created on: Feb 1, 2024
 *      Author: Ozuba
 *
 * Este fichero encapsula la maquina de estados del TER:
 * Esta consiste de 4 estados:
 *-------------------------------------------------------------------------------------
 * WAITING_FOR_SL -> 0
 * - Estado de inicio, se comprueba si la safety esta cerrada
 * - leyendo el valor de volaje despues del TSMS
 *
 * RDY2PRECH
 *- La safety esta cerrada, se puede precargar
 *
 * PRECHARGING
 *- Estado transitorio hasta que el BMS termine la precarga
 *
 * PRECHARGED
 *- El coche está cargado, se permite hace R2D
 *
 * R2D
 * - Se puede conducir
 *-------------------------------------------------------------------------------------
 * La maquina de estados se basa en condiciones
 *
 */
#include "stateMachine.h"
#include "ter.h"



void checkState(stateMachine_t* machine){
machine->state = WAITING_SL;// Estado Inicial
if(){//Si esta ok la safety


}


}


void stateMachine(void);

//Estados
void waitingSL(void); // Comprueba
void rdy2Prech(void); // Espera a recibir el comando de precarga
void precharging(void); //Estado transitorio, monitoriza que todo va bien
void precharged(void);//Espera a que se reciba el comando de r2d
void driving(void); //Ejecuta la comanda de par









