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

/* ---------------------------[Estructuras del CAN]-------------------------- */
//Pointer to timer and can peripheral being used
CAN_HandleTypeDef *invCAN;
CAN_HandleTypeDef *mainCAN;

TIM_HandleTypeDef *invTIM;
TIM_HandleTypeDef *mainTIM;

//Index for can senders
uint8_t invIndex;
uint8_t mainIndex;
/* -------------------------------------------------------------------------- */

struct TeR_t TeR;

/* ---------------------------[Inicialización + Interrupts]-------------------------- */

uint8_t initCAN(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan,
		TIM_HandleTypeDef *hInvTIM, TIM_HandleTypeDef *hMainTIM) {
	//Inicializacion de los perifericos can
	invCAN = invCan;
	mainCAN = mainCan;
	// Attach timers
	invTIM = hInvTIM;
	mainTIM = hMainTIM;

	//Arranque del periferico y la interrupcion
	configFilter(invCan, mainCan); //Configura los filtros
	//Registramos los 2 callbacks de recepcion a la función conjunta de decodificación
	HAL_CAN_RegisterCallback(invCAN, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
			decodeMsg);
	HAL_CAN_RegisterCallback(mainCAN, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
			decodeMsg);

	//Arranque del modulo
	HAL_CAN_Start(invCAN); //Activamos el can
	HAL_CAN_Start(mainCAN); //Activamos el can

	//Arrancamos las interrupts
	HAL_CAN_ActivateNotification(invCAN, CAN_IT_RX_FIFO0_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura
	HAL_CAN_ActivateNotification(mainCAN, CAN_IT_RX_FIFO0_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura

	//Registro de los callbacks a nuestras funciones de envío
	HAL_TIM_RegisterCallback(invTIM, HAL_TIM_PERIOD_ELAPSED_CB_ID, sendInvCAN);
	HAL_TIM_RegisterCallback(mainTIM, HAL_TIM_PERIOD_ELAPSED_CB_ID,
			sendMainCAN);

	//Arranca los timers y comienza el envio
	HAL_TIM_Base_Start_IT(invTIM); //Arranca el ciclo
	HAL_TIM_Base_Start_IT(mainTIM); //Arranca el ciclo
	return 1;
}
/*----------------------------------[Configuración de filtros]--------------------------------*/

void configFilter(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan) {
	CAN_FilterTypeDef filter;
	//Inverter Filter (CAN1 MASter)
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 0; // which filter bank to use from the assigned ones
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 14; // Los filtros son compartidos
	HAL_CAN_ConfigFilter(invCan, &filter);

	//Main Filter (Slave)
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 15; // which filter bank to use from the assigned ones
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 14; // Cursor de división de filtros
	HAL_CAN_ConfigFilter(mainCan, &filter);

}

/* ----------------------------------[Envío]---------------------------------------- */

/* ---------------------------[INVERTER CAN]-------------------------- */

void sendInvCAN(TIM_HandleTypeDef *htim) {
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	//Van los 3 mensajes de golpe pq justo nos caben en la fifo a la vez y el inverter los requiere
	if (HAL_CAN_GetTxMailboxesFreeLevel(invCAN) > 0) { // Hay un slot para nuestro mensaje
		switch (invIndex++) {

		/* ---------------------------[DERECHO]-------------------------- */

		case 0: //Inverter Derecho
			//SETPOINT_1
			TxHeader.StdId = INVERTER_EMCU_SETPOINT_1_RIGHT_FRAME_ID;
			TxHeader.DLC = INVERTER_EMCU_SETPOINT_1_RIGHT_LENGTH;
			inverter_emcu_setpoint_1_right_pack(TxData, &TeR.appReqRight,
					TxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

			//SETPOINT_2
			TxHeader.StdId = INVERTER_EMCU_SETPOINT_2_RIGHT_FRAME_ID;
			TxHeader.DLC = INVERTER_EMCU_SETPOINT_2_RIGHT_LENGTH;
			inverter_emcu_setpoint_2_right_pack(TxData, &TeR.currentReqRight,
					TxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

			//SETPOINT_3
			TxHeader.StdId = INVERTER_EMCU_SETPOINT_3_RIGHT_FRAME_ID;
			TxHeader.DLC = INVERTER_EMCU_SETPOINT_3_RIGHT_LENGTH;
			inverter_emcu_setpoint_3_right_pack(TxData, &TeR.trqReqRight,
					TxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

			break;

			/* ---------------------------[IZQUIERDO]-------------------------- */

		case 1: //Torque Setpoint L
			//SETPOINT_1
			TxHeader.StdId = INVERTER_EMCU_SETPOINT_1_LEFT_FRAME_ID;
			TxHeader.DLC = INVERTER_EMCU_SETPOINT_1_LEFT_LENGTH;
			inverter_emcu_setpoint_1_left_pack(TxData, &TeR.appReqLeft,
					TxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

			//SETPOINT_2
			TxHeader.StdId = INVERTER_EMCU_SETPOINT_2_LEFT_FRAME_ID;
			TxHeader.DLC = INVERTER_EMCU_SETPOINT_2_LEFT_LENGTH;
			inverter_emcu_setpoint_2_left_pack(TxData, &TeR.currentReqLeft,
					TxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

			//SETPOINT_3
			TxHeader.StdId = INVERTER_EMCU_SETPOINT_3_LEFT_FRAME_ID;
			TxHeader.DLC = INVERTER_EMCU_SETPOINT_3_LEFT_LENGTH;
			inverter_emcu_setpoint_3_left_pack(TxData, &TeR.trqReqLeft,
					TxHeader.DLC);
			HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

			invIndex = 0; //Evita un ciclo muerto
			break;
			/* ---------------------------[Default]-------------------------- */

		default: //Por si algo wtf pasa
			invIndex = 0; //cualquier otro valor retorna al ultimo mensaje
			break;
		}
	}
}
/* ---------------------------[MAIN CAN]-------------------------- */

void sendMainCAN(TIM_HandleTypeDef *htim) {
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN) > 0) { // Hay un slot para nuestro mensaje
		switch (mainIndex++) {

		case 0:
			TxHeader.StdId = TER_TER_STATUS_FRAME_ID;
			TxHeader.DLC = TER_TER_STATUS_LENGTH;
			ter_ter_status_pack(TxData, &TeR.status, TxHeader.DLC);
			break;
		case 1:
			TxHeader.StdId = TER_WHEEL_INFO_FRAME_ID;
			TxHeader.DLC = TER_WHEEL_INFO_LENGTH;
			ter_wheel_info_pack(TxData, &TeR.wheelInfo, TxHeader.DLC);
			break;

		case 2:
			TxHeader.StdId = TER_DYNAMIC_CONFIG_FRAME_ID;
			TxHeader.DLC = TER_DYNAMIC_CONFIG_LENGTH;
			ter_dynamic_config_pack(TxData, &TeR.dynamicConfig, TxHeader.DLC);
			break;

		case 3:
			TxHeader.StdId = TER_INVERTER_INFO_FRAME_ID;
			TxHeader.DLC = TER_INVERTER_INFO_LENGTH;
			ter_inverter_info_pack(TxData, &TeR.invInfo, TxHeader.DLC);
			break;


		default: //Esto evita tener que contar mensajes
			mainIndex = 0; //cualquier otro valor retorna al ultimo mensaje
			return; //Evita que se envíe un mensaje doble terminando la funcion
			break;
		}
		HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado
	}
}

//Función de decodificación del CAN, recive un mensaje de un bus y lo coloca en la estructura global

void decodeMsg(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data); //Recoge el mensaje
	logSCS(header.StdId); //System Critical signal timestamping
	switch (header.StdId) {
	//Attend the command
	case TER_COMMAND_FRAME_ID: //Sistema de comandos
		struct ter_command_t cmdMsg;
		ter_command_init(&cmdMsg); //Por si se usan variables indebidamente inicializadas
		ter_command_unpack(&cmdMsg, data, TER_COMMAND_LENGTH);
		command(cmdMsg); //Llama a la interpretación del comando (Se lo pasa por copia)
		break;

		/* ---------------------------[TER]-------------------------- */

		//Mesage Decoding
	case TER_APPS_FRAME_ID:
		ter_apps_unpack(&TeR.apps, data, header.DLC);
		break;

	case TER_BPPS_FRAME_ID:
		ter_bpps_unpack(&TeR.bpps, data, header.DLC);
		break;

	case TER_STEER_FRAME_ID:
		ter_steer_unpack(&TeR.steer, data, header.DLC);
		break;

	case TER_FRONT_V_FRAME_ID:
		ter_front_v_unpack(&TeR.speed, data, header.DLC);
		break;

	case TER_ANG_RATE_FRAME_ID:
		ter_ang_rate_unpack(&TeR.angRate, data, header.DLC);
		break;

	case TER_LV_STATUS_FRAME_ID:
		ter_lv_status_unpack(&TeR.lvbms, data, header.DLC);
		break;

		/* ---------------------------[INVERTER]-------------------------- */

	case INVERTER_EMCU_STATE_2_RIGHT_FRAME_ID:
		inverter_emcu_state_2_right_unpack(&TeR.appStateRight, data,
				header.DLC);
		break;

	case INVERTER_EMCU_STATE_2_LEFT_FRAME_ID:
		inverter_emcu_state_2_left_unpack(&TeR.appStateLeft, data, header.DLC);
		break;

	case INVERTER_EMCU_STATE_3_RIGHT_FRAME_ID:
		inverter_emcu_state_3_right_unpack(&TeR.dqErpmRight, data, header.DLC);
		break;

	case INVERTER_EMCU_STATE_3_LEFT_FRAME_ID:
		inverter_emcu_state_3_left_unpack(&TeR.dqErpmLeft, data, header.DLC);
		break;

	case INVERTER_EMCU_STATE_4_RIGHT_FRAME_ID:
		inverter_emcu_state_4_right_unpack(&TeR.tempsRight, data, header.DLC);
		break;

	case INVERTER_EMCU_STATE_4_LEFT_FRAME_ID:
		inverter_emcu_state_4_left_unpack(&TeR.tempsLeft, data, header.DLC);
		break;


	case INVERTER_EMCU_STATE_7_LEFT_FRAME_ID:
		inverter_emcu_state_7_left_unpack(&TeR.demLeft, data, header.DLC);
		break;

	case INVERTER_EMCU_STATE_7_RIGHT_FRAME_ID:
		inverter_emcu_state_7_right_unpack(&TeR.demRight, data, header.DLC);
		break;

	case INVERTER_EMCU_STATE_9_LEFT_FRAME_ID:
		inverter_emcu_state_9_left_unpack(&TeR.trqEstLeft, data, header.DLC);
		break;

	case INVERTER_EMCU_STATE_9_RIGHT_FRAME_ID:
		inverter_emcu_state_9_right_unpack(&TeR.trqEstRight, data, header.DLC);
		break;

		/* ---------------------------[HVBMS]-------------------------- */

	case HVBMS_BMS_TX_STATE_3_FRAME_ID:
		hvbms_bms_tx_state_3_unpack(&TeR.BmsAppState, data, header.DLC);
		break;
		/* ---------------------------[Default]-------------------------- */

	default:
		return;
		break;

	}
}

