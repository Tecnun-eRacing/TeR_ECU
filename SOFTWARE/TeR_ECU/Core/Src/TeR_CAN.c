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
//Datos transmision
CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
uint8_t TxData[8]; //Header de recepción
uint32_t TxMailbox1; //Mailbox para el CAN1
uint32_t TxMailbox2; //Mailbox para el CAN2

//Datos recepcion
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
/* -------------------------------------------------------------------------- */

struct TeR_t TeR;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { //No hay distinción de bus
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData); //Recoge el mensaje
	decodeMsg(RxHeader.StdId, RxData); //llama a la decodificación
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

	default:
		return -1;
		break;

	}
	return 1;
}

//Función de envío de mensajes
uint8_t sendCan(void) {
	//Standar Config
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	TxHeader.StdId = TER_APPS_FRAME_ID;
	ter_ecu_status_pack(TxData, &TeR.status, TER_ECU_STATUS_LENGTH);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox1);

	return 1;
}

//Implementa aqui los comandos que se han de ejecutar
uint8_t command(uint8_t cmd, uint8_t *args) {
	switch (cmd) { //Hay que generar un archivon los defines de esto en el repo de DBCS

	case 10: //Precarga
		if (TeR.status.state == RDY2PRECH) { //Envía al bms el mensaje de precarga
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.StdId = 1243; //BMS precharge action
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox1);
		}
		break;

	case 11: //Ready2Drive
		if (TeR.status.state == PRECHARGED) { //Pone el coche en modo driving
			//Bocina
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			//Permite el paso al estado drive
			TeR.r2d = 1;
		}
		break;

	}
	return 1;
}
