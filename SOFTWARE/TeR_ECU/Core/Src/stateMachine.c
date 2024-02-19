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
 * Tras valorar distintas maneras de implementar la maquina de estados
 * lo más optimo es comprobar el estado cada ciclo en una escalera de privilegio
 * puesto que las condiciones de estados más complejos están contenidos en los superiores,
 * de esta manera si una condición no se cumple se degrada al estado más bajo.
 *
 */
#include "stateMachine.h"


state_t getState(void) {
	state_t status = WAITING_SL; //Iniciamos en el estado 0
	//Lecturas
	TeR.status.sl_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);// Leemos el estado de la safety

	if (TeR.status.sl_status) { //Si esta ok la safety
		status = RDY2PRECH; //Se puede precargar
		if (1) { // Se está haciendo precarga?
			status = PRECHARGING;

		} else if (1) { // Esta precargado?
			status = PRECHARGED;
			if (TeR.status.r2d) { //la flag de ready2drive esta activada? (can)
				status = DRIVING;
			}
		}
	}
	return status;
}

void stateMachine(void) {
	TeR.status.state = 4;//getState(); //Actualiza el estado
	switch (TeR.status.state) {
	case WAITING_SL:
		waitingSL();
		break;

	case RDY2PRECH:
		rdy2Prech();
		break;

	case PRECHARGING:
		precharging();
		break;

	case PRECHARGED:
		precharged();
		break;
	case DRIVING:
		driving();
		break;
	default:
		//Handle Invalid state
		break;
	}
	//brake light

}

/* -------------------------[Estados]---------------------------- */

void waitingSL(void){
	TeR.trqReqLeft.torque_req = 0;
	TeR.trqReqRight.torque_req = 0;
	TeR.status.r2d = 0;
} // Comprueba que la safety esta cerrada
void rdy2Prech(void){
	TeR.trqReqLeft.torque_req = 0;
	TeR.trqReqRight.torque_req = 0;
	TeR.status.r2d = 0;
} // Espera a recibir el comando de precarga
void precharging(void){
	TeR.trqReqLeft.torque_req = 0;
	TeR.trqReqRight.torque_req = 0;
	TeR.status.r2d = 0;
} //Estado transitorio, monitoriza que todo va bien
void precharged(void){
	TeR.trqReqLeft.torque_req = 0;
	TeR.trqReqRight.torque_req = 0;
	TeR.status.r2d = 0;
} //Espera a que se reciba el comando de r2d
void driving(void){
	TeR.trqReqLeft.torque_req = TeR.apps.apps_av;
	TeR.trqReqRight.torque_req = TeR.apps.apps_av;
} //Ejecuta la comanda de par

