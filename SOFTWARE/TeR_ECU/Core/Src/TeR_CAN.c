/*
 * TeR_CAN.c
 *
 *  Created on: Feb 2, 2024
 *      Author: Ozuba
 *
 * ████████╗███████╗██████╗          ██████╗ █████╗ ███╗   ██╗
 * ╚══██╔══╝██╔════╝██╔══██╗        ██╔════╝██╔══██╗████╗  ██║
 *    ██║   █████╗  ██████╔╝        ██║     ███████║██╔██╗ ██║
 *    ██║   ██╔══╝  ██╔══██╗        ██║     ██╔══██║██║╚██╗██║
 *    ██║   ███████╗██║  ██║███████╗╚██████╗██║  ██║██║ ╚████║
 *    ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═══╝
 */

/*
 *  Este Fichero tiene como Objetivo almacenar las funciones de decodificación
 *  y envío de todos los mensajes de una placa, incluye como librerías aquellas
 *  autogeneradas mediante cantools y ofrece una interfáz de cara al micro con dos
 *  Funciones:
 *  - decodeMSG -> Decodifica las estructuras pertinentes
 *  - sendCAN -> Envía los mensajes pertinentes (Esto no va a depender del estado, ya que los inverters siempre estarán a 0)
 *  - command -> Función que se llama cuando se recibe el mensaje de comando para que cada placa lo interprete como corresponde
 *  A su vez están creados aqui todas las estructuras de memoria del can
 *
 */

#include "TeR_CAN.h"
#include "stateMachine.h"

/* ---------------------------[Estructuras del CAN]-------------------------- */
//Pointer to timer and can peripheral being used
CAN_HandleTypeDef *invCAN;
CAN_HandleTypeDef *mainCAN;

TIM_HandleTypeDef *invTIM;
TIM_HandleTypeDef *mainTIM;

//Datos transmision
CAN_TxHeaderTypeDef mainTxHeader; //Header de transmisión
uint8_t mainTxData[8]; //Header de recepción

CAN_TxHeaderTypeDef invTxHeader; //Header de transmisión
uint8_t invTxData[8]; //Header de recepción


uint32_t invMailbox; //Mailbox para el CAN1
uint32_t mainMailbox; //Mailbox para el CAN2

//Datos recepcion
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];


CAN_RxHeaderTypeDef invRxHeader;
uint8_t invRxData[8];

//Index for can senders
uint8_t invIndex;
uint8_t mainIndex;

/* -------------------------------------------------------------------------- */

struct TeR_t TeR;

/* ---------------------------[Inicialización + Interrupts]-------------------------- */

uint8_t initCAN(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan,
		TIM_HandleTypeDef *hMainTIM, TIM_HandleTypeDef *hInvTIM) {
	//Inicializacion de los perifericos can
	invCAN = invCan;
	mainCAN = mainCan;
	// Attach timers
	mainTIM = hMainTIM;
	invTIM = hInvTIM;
	//Arranque del periferico y la interrupcion
	HAL_CAN_Start(invCAN); //Activamos el can
	HAL_CAN_Start(mainCAN); //Activamos el can
	HAL_CAN_ActivateNotification(invCAN, CAN_IT_RX_FIFO0_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura
	HAL_CAN_ActivateNotification(mainCAN, CAN_IT_RX_FIFO1_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura

	HAL_TIM_Base_Start_IT(invTIM); //Arranca el ciclo
	HAL_TIM_Base_Start_IT(mainTIM); //Arranca el ciclo
	return 1;
}


void canLoop(TIM_HandleTypeDef *srcTIM){
	if (srcTIM == invTIM) { //Si es nuestro timer(Da igual si solo hay 1)
		sendInvCAN();

	} else if (srcTIM == mainTIM) {
		sendMainCAN();
	}
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == invCAN) {
		HAL_CAN_GetRxMessage(invCAN, CAN_RX_FIFO0, &invRxHeader, invRxData); //Recoge el mensaje
	}
	decodeMsg(invRxHeader.StdId, invRxData); //llama a la decodificación
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan == mainCAN) {
		HAL_CAN_GetRxMessage(mainCAN, CAN_RX_FIFO1, &RxHeader, RxData); //Recoge el mensaje
	}
	decodeMsg(RxHeader.StdId, RxData); //llama a la decodificación
}

uint8_t sendInvCAN(void) {
	invTxHeader.IDE = CAN_ID_STD;
	invTxHeader.RTR = CAN_RTR_DATA;
	/* ---------------------------[INVERTER CAN]-------------------------- */

	if (HAL_CAN_GetTxMailboxesFreeLevel(invCAN) > 0) { // Hay un slot para nuestro mensaje
		switch (invIndex++) {

		case 0: //Inverter Derecho
			//SETPOINT_1
			invTxHeader.StdId = INVERTER_EMCU_SETPOINT_1_RIGHT_FRAME_ID;
			invTxHeader.DLC = INVERTER_EMCU_SETPOINT_1_RIGHT_LENGTH;
			inverter_emcu_setpoint_1_right_pack(invTxData, &TeR.appReqRight,
					invTxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &invTxHeader, invTxData, &invMailbox); //Envía el mensaje procesado

			//SETPOINT_2
			invTxHeader.StdId = INVERTER_EMCU_SETPOINT_2_RIGHT_FRAME_ID;
			invTxHeader.DLC = INVERTER_EMCU_SETPOINT_2_RIGHT_LENGTH;
			inverter_emcu_setpoint_2_right_pack(invTxData, &TeR.currentReqRight,
					invTxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &invTxHeader, invTxData, &invMailbox); //Envía el mensaje procesado

			//SETPOINT_3
			invTxHeader.StdId = INVERTER_EMCU_SETPOINT_3_RIGHT_FRAME_ID;
			invTxHeader.DLC = INVERTER_EMCU_SETPOINT_3_RIGHT_LENGTH;
			inverter_emcu_setpoint_3_right_pack(invTxData, &TeR.trqReqRight,
					invTxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &invTxHeader, invTxData, &invMailbox); //Envía el mensaje procesado

			break;

		case 1: //Torque Setpoint L
			//SETPOINT_1
			invTxHeader.StdId = INVERTER_EMCU_SETPOINT_1_LEFT_FRAME_ID;
			invTxHeader.DLC = INVERTER_EMCU_SETPOINT_1_LEFT_LENGTH;
			inverter_emcu_setpoint_1_left_pack(invTxData, &TeR.appReqLeft,
					invTxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &invTxHeader, invTxData, &invMailbox); //Envía el mensaje procesado

			//SETPOINT_2
			invTxHeader.StdId = INVERTER_EMCU_SETPOINT_2_LEFT_FRAME_ID;
			invTxHeader.DLC = INVERTER_EMCU_SETPOINT_2_LEFT_LENGTH;
			inverter_emcu_setpoint_2_left_pack(invTxData, &TeR.currentReqLeft,
					invTxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &invTxHeader, invTxData, &invMailbox); //Envía el mensaje procesado

			//SETPOINT_3
			invTxHeader.StdId = INVERTER_EMCU_SETPOINT_3_LEFT_FRAME_ID;
			invTxHeader.DLC = INVERTER_EMCU_SETPOINT_3_LEFT_LENGTH;
			inverter_emcu_setpoint_3_left_pack(invTxData, &TeR.trqReqLeft,
					invTxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &invTxHeader, invTxData, &invMailbox); //Envía el mensaje procesado



			invIndex = 0; //cualquier otro valor retorna al ultimo mensaje
			break;

		default: //Esto evita tener que contar mensajes
			invIndex = 0; //cualquier otro valor retorna al ultimo mensaje
			break;
		}
	}
	return 1;
}

uint8_t sendMainCAN(void) {
	mainTxHeader.IDE = CAN_ID_STD;
	mainTxHeader.RTR = CAN_RTR_DATA;
	/* ---------------------------[MAIN CAN]-------------------------- */

	if (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN) > 0) { // Hay un slot para nuestro mensaje
		switch (mainIndex++) {

		case 0:
			mainTxHeader.StdId = TER_ECU_STATUS_FRAME_ID;
			mainTxHeader.DLC = TER_ECU_STATUS_LENGTH;
			ter_ecu_status_pack(mainTxData, &TeR.status, mainTxHeader.DLC);
			break;

		default: //Esto evita tener que contar mensajes
			mainIndex = 0; //cualquier otro valor retorna al ultimo mensaje
			return 1; //Evita que se envíe un mensaje doble terminando la funcion
			break;
		}
		HAL_CAN_AddTxMessage(mainCAN, &mainTxHeader, mainTxData, &mainMailbox); //Envía el mensaje procesado
	}
	return 1;
}

//Función de decodificación del CAN, si quieres que la ecu disponga de una señal hay que añadirla aquí.
uint8_t decodeMsg(uint32_t canId, uint8_t *data) {

	switch (canId) {
	//Attend the command
	case TER_CMD_FRAME_ID:
		command(data[0], &data[1]); //Llama a la interpretación del comando
		break;

		//Mesage Decoding
	case TER_APPS_FRAME_ID:
		ter_apps_unpack(&TeR.apps, data, TER_APPS_LENGTH);
		break;

	case TER_STEER_FRAME_ID:
		ter_steer_unpack(&TeR.steer, data, TER_STEER_LENGTH);
		break;

	case TER_FRONT_V_FRAME_ID:
		ter_front_v_unpack(&TeR.speed, data, TER_FRONT_V_LENGTH);
		break;

	case TER_ANG_RATE_FRAME_ID:
		ter_ang_rate_unpack(&TeR.angRate, data, TER_ANG_RATE_LENGTH);
		break;

	case INVERTER_EMCU_STATE_2_RIGHT_FRAME_ID:
		inverter_emcu_state_2_right_unpack(&TeR.appStateRight, data,
		INVERTER_EMCU_STATE_2_RIGHT_LENGTH);
		break;

	case INVERTER_EMCU_STATE_2_LEFT_FRAME_ID:
		inverter_emcu_state_2_left_unpack(&TeR.appStateLeft, data,
		INVERTER_EMCU_STATE_2_LEFT_LENGTH);
		break;

	case HVBMS_BMS_TX_STATE_3_FRAME_ID:
		hvbms_bms_tx_state_3_unpack(&TeR.BmsAppState, data,
		HVBMS_BMS_TX_STATE_3_LENGTH);
		break;

	default:
		return -1;
		break;

	}
	return 1;
}

//Implementa aqui los comandos que se han de ejecutar
uint8_t command(uint8_t cmd, uint8_t *args) {
	switch (cmd) { //Hay que generar un archivon los defines de esto en el repo de DBCS

	case 10: //Precarga
		if (TeR.status.state == RDY2PRECH) { //Envía al bms el mensaje de precarga
			TeR.BmsAppReq.app_state_req = 3;
			mainTxHeader.StdId = HVBMS_BMS_RX_CTRL_1_FRAME_ID; //BMS precharge action
			mainTxHeader.DLC = HVBMS_BMS_RX_CTRL_1_LENGTH;
			hvbms_bms_rx_ctrl_1_pack(mainTxData, &TeR.BmsAppReq, mainTxHeader.DLC);
			HAL_CAN_AddTxMessage(mainCAN, &mainTxHeader, mainTxData, &mainMailbox); //Envía el mensaje procesado

		}
		break;

	case 11: //Ready2Drive
		if (TeR.status.state == PRECHARGED) { //Pone el coche en modo driving y añadir freno

			//Permite el paso al estado drive
			TeR.status.r2d = 1;
			TeR.appReqRight.app_state_req = 4;
			TeR.appReqLeft.app_state_req = 4;
			invTxHeader.StdId = INVERTER_EMCU_SETPOINT_1_LEFT_FRAME_ID;
			invTxHeader.DLC = INVERTER_EMCU_SETPOINT_1_LEFT_LENGTH;
			inverter_emcu_setpoint_1_left_pack(invTxData, &TeR.appReqLeft,
					invTxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &invTxHeader, invTxData, &invMailbox); //Envía el mensaje procesado

		}
		break;
	}
	return 1;
}
