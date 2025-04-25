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

/*Implementacion FreeRTOS Piero
 *
 * - El envio de CAN de inverters, main CAN y decodificación son tareas diferentes, con prioridades diferentes, siendo la de decodificación superior a las anteriores.
 * - En los envios se utiliza vTaskDelayUntil (en nuestro caso osDelayUntil), y para la recepción desbloqueo basado en colas.
 *
 * - La Decodificación y la maquina de estados comparten un MUTEX para evitar que ambas funciones puedan modificar los valores de TeR y provocar
 * 		corrupciones de memoria, race conditions, etc. Este mutex es adquirido al principio de la ejecución y liberado al final de la ejecución de la función
 *
 * - La ejecución temporizada se realiza utilizando funciones del Kernel tales como osDelayUntil(), debido a que es la forma mas correcta de realizar
 *		 ejecuciones temporizadas sin desfase temporal en un sistema operativo en tiempo real como puede ser FreeRTOS.
 * 		 Podriamos usar software timers, su implementacion sin embargo no es la mas practica, ya que debemos registrar un callback que mande señales de desbloqueo
 *  	a los threads, y que estos a su vez esperen a dichas señales, ademas de que no garantiza ejecucion temporal precisa (Reference Manual)
 *  	Usar osDelay() es una buena alternativa, pero puede sufrir desfases temporales ya que su frecuencia depende en parte del tiempo de ejecucion
 *  	de la funcion (ya que el tiempo empieza a contar cuando dicha funcion es llamada, y el tiempo que tarda una funcion no es fijo)
 *
 * - Se utilizan colas para comunicar la interrupcion de recepcion (y su mensaje) con la decodificación, es la manera mas optima cuando utilizamos un sistema
 * 		operativo en tiempo real (no nos interesa llamar funciones dentro de interrupciones, queremos que se encarge el scheduler de cuando hay que decodificar
 * 		 muy en resumen).
 *
 * - SOLO EJECUTAREMOS CUANDO HAYAMOS PODIDO OBTENER EL MUTEX (decodificacion)
 *
 */

#include "TeR_CAN.h"
volatile uint32_t boot_flag __attribute__((section(".no_init")));
/* ---------------------------[Estructuras del CAN]-------------------------- */
//Pointer to timer and can peripheral being used
CAN_HandleTypeDef *invCAN;
CAN_HandleTypeDef *mainCAN;

//Index for can senders
uint8_t invIndex;
uint8_t mainIndex;

/* -------------------------------------------------------------------------- */
struct TeR_t TeR;

//FreeRTOS Dependencies
extern osMessageQueueId_t rxMsgHandle; //handle de la cola de recepcion
extern osMutexId_t preventRaceHandle; // Mutex compartido con la tarea de la maquina de Estados del coche (para tener exclusión mutua sobre la modificacion de la variable TeR)

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
	osMessageQueuePut(rxMsgHandle, &msg, 0U, 0U); //ponemos el mensaje en una cola, que será atendido cuando sea posible
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
	//Tarea con ejecucion temoporizada FreeRTOS (echar ojo a reference manual)
	uint32_t currentTick = osKernelGetTickCount(); // sincronizamos nuestra variable de tick con el valor actual del tick del kernel
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	//Van los 3 mensajes de golpe pq justo nos caben en la fifo a la vez y el inverter los requiere
	for (;;) {
		currentTick += 1; //añadimos 1 tick a el valor actual del tick del kernel
		osDelayUntil(currentTick); //cuando el kernel consiga llegar a el valor actual de currentTick, el kernel desbloqueará la tarea
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
	//Tarea con ejecucion temoporizada FreeRTOS (echar ojo a reference manual)
	uint32_t currentTick = osKernelGetTickCount(); // sincronizamos nuestra variable de tick con el valor actual del tick del kernel
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	for (;;) {
		currentTick += 10; //añadimos 2 ticks a el valor actual del tick del kernel
		osDelayUntil(currentTick); // Cuando el kernel consiga llegar a el valor actual de currentTick, desbloqueará la tarea
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

			case 4:
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
	canMsg_t msg; // tipo de variable que almacena id, datos y DLC del mensaje recibido en la interrupcion
	osStatus_t mutexStatus; // variable que almacena el estado de la obtencion del Mutex
	for (;;) {
		osMessageQueueGet(rxMsgHandle, &msg, 0U, osWaitForever); // la tarea se desbloquea cuando hay algo en cola
		mutexStatus = osMutexAcquire(preventRaceHandle, 500); // esperamos MUTEX, si hay timeout, nos iremos al handle sin ejecutar decodificacion
		if (mutexStatus == osOK) { // solo ejecutamos la recepcion si y solo si tenemos el mutex
			logSCS(msg.id); //System Critical signal Timestamp, solo cuando podamos ejecutar recepcion
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

			case TER_ECU_CONFIG_FRAME_ID:
				ter_ecu_config_unpack(&TeR.config, msg.data, msg.DLC);
				writeConfig(TeR.config); // guardamos la config en la eeprom
				break;
			case BOOTER_BOOT_TX_FRAME_ID:
				struct booter_boot_tx_t boot;
				booter_boot_tx_init(&boot);
				booter_boot_tx_unpack(&boot, msg.data, msg.DLC);
				if ((boot.boot_cmd == BOOTER_BOOT_TX_BOOT_CMD_BOOT_INIT_CHOICE)
						&& (boot.node_id == BOOTER_BOOT_TX_NODE_ID_ECU_CHOICE)) {
					boot_flag = 1;
					HAL_NVIC_SystemReset();
				}
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
				inverter_emcu_state_7_left_unpack(&TeR.demLeft, msg.data,
						msg.DLC);
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
				hvbms_bms_tx_state_3_unpack(&TeR.BmsAppState, msg.data,
						msg.DLC);
				break;
				/* ---------------------------[Default]-------------------------- */

			default:
				break;

			}
			osMutexRelease(preventRaceHandle); // liberamos el mutex si y solo si lo teniamos anteriormente
		} else {
			printf("Couldnt Adquire the TeR structure Mutex");
		}
	}
}

