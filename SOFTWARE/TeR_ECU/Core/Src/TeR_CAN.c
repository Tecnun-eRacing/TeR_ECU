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
CAN_HandleTypeDef *pwrTrainCAN;
CAN_HandleTypeDef *mainCAN;


TIM_HandleTypeDef *tim;


//Datos transmision
CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
uint8_t TxData[8]; //Header de recepción
uint32_t TxMailbox1; //Mailbox para el CAN1
uint32_t TxMailbox2; //Mailbox para el CAN2

//Datos recepcion
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

//Index for can senders
uint8_t msgIndex1 = 0;
uint8_t msgIndex2 = 0;

/* -------------------------------------------------------------------------- */

struct TeR_t TeR;


/* ---------------------------[Inicialización + Interrupts]-------------------------- */

uint8_t initCAN(CAN_HandleTypeDef *hPwrTrain,CAN_HandleTypeDef *hMainCan, TIM_HandleTypeDef *htim) {
	//Inicializacion de los perifericos can
	pwrTrainCAN = hPwrTrain;
	mainCAN = hMainCan;
	tim = htim;
	//Arranque del periferico y la interrupcion
	HAL_CAN_Start(pwrTrainCAN); //Activamos el can
	HAL_CAN_Start(mainCAN); //Activamos el can
	HAL_CAN_ActivateNotification(pwrTrainCAN, CAN_IT_RX_FIFO0_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura
	HAL_CAN_ActivateNotification(mainCAN, CAN_IT_RX_FIFO0_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura

	HAL_TIM_Base_Start_IT(tim);
	return 1;
}


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


uint8_t sendCAN(void) {
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	/* ---------------------------[INVERTER CAN]-------------------------- */

	if (HAL_CAN_GetTxMailboxesFreeLevel(pwrTrainCAN) > 0) { // Hay un slot para nuestro mensaje
		switch (msgIndex1++) {

		case 0:
			TxHeader.StdId = INVERTER_EMCU_SETPOINT_3_FRAME_ID;
			TxHeader.DLC = INVERTER_EMCU_SETPOINT_3_LENGTH;
			ter_ecu_status_pack(TxData, &TeR.status,8);
			break;
		case 1:
			//LV_Temps
			TxHeader.StdId = TER_LV_TEMPS_FRAME_ID;
			TxHeader.DLC = TER_LV_TEMPS_LENGTH;
			ter_lv_temps_pack(TxData, &TeR.temps, sizeof(TxData));
			break;

		default: //Esto evita tener que contar mensajes
			msgIndex1 = 0; //cualquier otro valor retorna al ultimo mensaje
			return 1; //Evita que se envíe un mensaje doble terminando la funcion
			break;
		}
		HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &TxMailbox1); //Envía el mensaje procesado
	}

	/* ---------------------------[MAIN CAN]-------------------------- */

	if (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN) > 0) { // Hay un slot para nuestro mensaje
			switch (msgIndex++) {

			case 0:
				TxHeader.StdId = INVERTER_EMCU_SETPOINT_3_FRAME_ID;
				TxHeader.DLC = INVERTER_EMCU_SETPOINT_3_LENGTH;
				ter_ecu_status_pack(TxData, &TeR.status,8);
				break;

			default: //Esto evita tener que contar mensajes
				msgIndex = 0; //cualquier otro valor retorna al ultimo mensaje
				return 1; //Evita que se envíe un mensaje doble terminando la funcion
				break;
			}
			HAL_CAN_AddTxMessage(pwrTrainCAN, &TxHeader, TxData, &TxMailbox2); //Envía el mensaje procesado
		}

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
			HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &TxMailbox1);
		}
		break;

	case 11: //Ready2Drive
		if (TeR.status.state == PRECHARGED && TeR.bpps.bpps > 30) { //Pone el coche en modo driving
		//Bocina
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			//Permite el paso al estado drive
			TeR.status.r2d = 1;
		}
		break;

	}
	return 1;
}
