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
	state_t status = WAIT_SL; //Iniciamos en el estado 0
	//Lecturas
	TeR.status.sl_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);// Leemos el estado de la safety
	TeR.status.bspd_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);// Leemos el estado del BSPD

	if (TeR.status.sl_status) { //Si esta ok la safety
		status = RDY2PRECH; //Se puede precargar
		if (TeR.BmsAppState.app_state_app == 3) { // Se está haciendo precarga?
			status = PRECHARGING;
		} else if (TeR.BmsAppState.app_state_app == 4) { // Esta precargado?
			status = PRECHARGED;
			if (TeR.status.r2d && ((TeR.appStateRight.app_state_app >= 2) || (TeR.appStateLeft.app_state_app >= 2))) { //la flag de ready2drive esta activada? (can)
				status = DRIVING;
			}
		}
	}
	return status;
}

void stateMachine(void) {
	uint8_t state = getState(); //Get Current State
	uint8_t stateChanged = TeR.status.state != state ? 1 : 0; //for state setup
	TeR.status.state = state; //getState(); //Actualiza el estado

	//-----------------------------------[Setups]--------------------------------------------//

	if (stateChanged) { // Handles setup conditions for the new state
		switch (TeR.status.state) {
		case WAIT_SL:
			//Security
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
			TeR.status.r2d = 0;
			break;

		case RDY2PRECH:
			//Security
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
			TeR.status.r2d = 0;
			TeR.appReqLeft.app_state_req = 1;//Manda el Inverter a su estado off por si estaba en error
			TeR.appReqRight.app_state_req = 1;
			break;

		case PRECHARGING:
			//Security
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
			TeR.status.r2d = 0;
			break;

		case PRECHARGED:
			TeR.appReqLeft.app_state_req = 2;//Manda el inverter a ready
			TeR.appReqRight.app_state_req = 2;
			//Security
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
			TeR.status.r2d = 0;
			break;
		case DRIVING:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_Delay(2000);//EV 4.12.1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

			break;
		default:
			//Handle Invalid state
			break;
		}
	}

//-----------------------------------[LOOPS]--------------------------------------------//

	switch (TeR.status.state) {
	case WAIT_SL:
		waitSL();
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

void waitSL(void) {

} // Comprueba que la safety esta cerrada
void rdy2Prech(void) {

} // Espera a recibir el comando de precarga
void precharging(void) {

} //Estado transitorio, monitoriza que todo va bien
void precharged(void) {

} //Espera a que se reciba el comando de r2d
void driving(void) {
	TeR.trqReqLeft.torque_nm_req = map(TeR.apps.apps_av,0,255,0,10);
	TeR.trqReqRight.torque_nm_req = map(TeR.apps.apps_av,0,255,0,10);


} //Ejecuta la comanda de par




