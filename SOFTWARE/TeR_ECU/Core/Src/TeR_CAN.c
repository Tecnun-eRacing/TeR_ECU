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

//Index for can senders
uint8_t invIndex;
uint8_t mainIndex;

//FreeRTOS
extern osMessageQueueId_t rxMsgHandle; //handle de la cola de recepcion
extern osMutexId_t preventRaceHandle; //handle del mutex de protección contra escritura
/* -------------------------------------------------------------------------- */
struct TeR_t TeR;
/* ---------------------------[Inicialización + Interrupts]-------------------------- */

uint8_t initCAN(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan) {
	//Inicializacion de los perifericos can
	invCAN = invCan;
	mainCAN = mainCan;
	//Arranque del periferico y la interrupcion
	configFilter(invCan, mainCan); //Configura los filtros
	//Registramos los 2 callbacks de recepcion a la función conjunta de decodificación
	HAL_CAN_RegisterCallback(invCAN, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
			canRxCallback);
	HAL_CAN_RegisterCallback(mainCAN, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
			canRxCallback);

	//Arranque del modulo
	HAL_CAN_Start(invCAN); //Activamos el can
	HAL_CAN_Start(mainCAN); //Activamos el can

	//Arrancamos las interrupts
	HAL_CAN_ActivateNotification(invCAN, CAN_IT_RX_FIFO0_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura
	HAL_CAN_ActivateNotification(mainCAN, CAN_IT_RX_FIFO0_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura
	return 1;
}
/*----------------------------------[Funcion de Callback FIFO0]--------------------------------*/

void canRxCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rxHeader; //Header temporal
	canMsg_t msg; //Bufer temporal
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, msg.data); //Recoge el mensaje
	msg.id = rxHeader.StdId;
	msg.DLC = rxHeader.DLC;
	osMessageQueuePut(rxMsgHandle, &msg, 0U, 0U);
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

void invCanTx(void *argument) {
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	//Van los 3 mensajes de golpe pq justo nos caben en la fifo a la vez y el inverter los requiere
	uint32_t currentTick;
	currentTick = osKernelGetTickCount(); // kernel tick sync
	for (;;) {
		currentTick += 2; // mandar inverter cada 2 milis
		osDelayUntil(currentTick);
		currentTick = osKernelGetTickCount();
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
				inverter_emcu_setpoint_2_right_pack(TxData,
						&TeR.currentReqRight, TxHeader.DLC);
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
}
/* ---------------------------[MAIN CAN]-------------------------- */
void mainCanTx(void *argument) {
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	uint32_t currentTick;
	currentTick = osKernelGetTickCount();
	for (;;) {
		currentTick += 10; //mandar MAIN can cada X tiempo
		osDelayUntil(currentTick); // kernel tick sync
		currentTick = osKernelGetTickCount();
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
				ter_dynamic_config_pack(TxData, &TeR.dynamicConfig,
						TxHeader.DLC);
				break;

			case 3:
				TxHeader.StdId = TER_INVERTER_INFO_FRAME_ID;
				TxHeader.DLC = TER_INVERTER_INFO_LENGTH;
				ter_inverter_info_pack(TxData, &TeR.invInfo, TxHeader.DLC);
				break;

			default: //Esto evita tener que contar mensajes
				mainIndex = 0; //cualquier otro valor retorna al ultimo mensaje
				break;
			}
			HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado
		}
	}
}

//Función de decodificación del CAN, recive un mensaje de un bus y lo coloca en la estructura global
void canRx(void *argument) {
	uint32_t errorCounter = 0; // debemos inicializar ! toma valor random
	canMsg_t msg;
	osStatus_t mutexStatus;
	for (;;) {
		osMessageQueueGet(rxMsgHandle, &msg, 0U, osWaitForever); // la tarea se desbloquea cuando hay algo en cola
		mutexStatus = osMutexAcquire(preventRaceHandle, 200); // esperamos MUTEX, si timeout, continuamos (equilibrio seguridad y real-time)
		logSCS(msg.id); //System Critical signal Timestamp
		switch (msg.id) {
		//Attend the command
		case TER_COMMAND_FRAME_ID: //Sistema de comandos
			struct ter_command_t cmdMsg;
			ter_command_init(&cmdMsg); //Por si se usan variables indebidamente inicializadas
			ter_command_unpack(&cmdMsg, msg.data, TER_COMMAND_LENGTH);
			command(cmdMsg); //Llama a la interpretación del comando (Se lo pasa por copia)
			break;

			/* ---------------------------[TER]-------------------------- */

			//Mesage Decoding
		case TER_APPS_FRAME_ID:
			ter_apps_unpack(&TeR.apps, msg.data, msg.DLC);
			break;

		case TER_BPPS_FRAME_ID:
			ter_bpps_unpack(&TeR.bpps, msg.data, msg.DLC);
			break;

		case TER_STEER_FRAME_ID:
			ter_steer_unpack(&TeR.steer, msg.data, msg.DLC);
			break;

		case TER_FRONT_V_FRAME_ID:
			ter_front_v_unpack(&TeR.speed, msg.data, msg.DLC);
			break;

		case TER_ANG_RATE_FRAME_ID:
			ter_ang_rate_unpack(&TeR.angRate, msg.data, msg.DLC);
			break;

		case TER_LV_STATUS_FRAME_ID:
			ter_lv_status_unpack(&TeR.lvbms, msg.data, msg.DLC);
			break;

			/* ---------------------------[INVERTER]-------------------------- */

		case INVERTER_EMCU_STATE_2_RIGHT_FRAME_ID:
			inverter_emcu_state_2_right_unpack(&TeR.appStateRight, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_2_LEFT_FRAME_ID:
			inverter_emcu_state_2_left_unpack(&TeR.appStateLeft, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_3_RIGHT_FRAME_ID:
			inverter_emcu_state_3_right_unpack(&TeR.dqErpmRight, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_3_LEFT_FRAME_ID:
			inverter_emcu_state_3_left_unpack(&TeR.dqErpmLeft, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_4_RIGHT_FRAME_ID:
			inverter_emcu_state_4_right_unpack(&TeR.tempsRight, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_4_LEFT_FRAME_ID:
			inverter_emcu_state_4_left_unpack(&TeR.tempsLeft, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_7_LEFT_FRAME_ID:
			inverter_emcu_state_7_left_unpack(&TeR.demLeft, msg.data, msg.DLC);
			break;

		case INVERTER_EMCU_STATE_7_RIGHT_FRAME_ID:
			inverter_emcu_state_7_right_unpack(&TeR.demRight, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_9_LEFT_FRAME_ID:
			inverter_emcu_state_9_left_unpack(&TeR.trqEstLeft, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_9_RIGHT_FRAME_ID:
			inverter_emcu_state_9_right_unpack(&TeR.trqEstRight, msg.data,
					msg.DLC);
			break;

			/* ---------------------------[HVBMS]-------------------------- */

		case HVBMS_BMS_TX_STATE_3_FRAME_ID:
			hvbms_bms_tx_state_3_unpack(&TeR.BmsAppState, msg.data, msg.DLC);
			break;
			/* ---------------------------[Default]-------------------------- */

		default:
			break;

		}
		if (mutexStatus == osOK) { //si hemos obtenido el mutex, lo liberamos
			osMutexRelease(preventRaceHandle);
		}
		else{
			errorCounter++;
		}
	}
}

