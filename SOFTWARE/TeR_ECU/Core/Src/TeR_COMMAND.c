/*
 * TeR_COMMAND.c
 *
 *  Created on: Apr 19, 2024
 *      Author: ozuba
 */
#include "TeR_COMMAND.h"

//Implementa aqui los comandos que se han de ejecutar
uint8_t command(struct ter_command_t command) {
	//Buffers volatiles para el envio de lo que toque
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	//Preinicializamos la respuesta
	struct ter_response_t response;
	response.cmd = command.cmd;
	response.code = TER_RESPONSE_CODE_OK_CHOICE; //Lo pone a ok si nadie dice lo contrario

	/*-----------------------------------------[COMANDOS]---------------------------------------*/
	switch (command.cmd) { //Hay que generar un archivon los defines de esto en el repo de DBCS

	case TER_COMMAND_CMD_PRECHARGE_CHOICE: //Precarga
		if (TeR.status.state == RDY2PRECH) { //Envía al bms el mensaje de precarga
			TeR.BmsAppReq.app_state_req =
			HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_HV__PRECHARGE_CHOICE; //Solicitamos la precarga al BMS
			TxHeader.StdId = HVBMS_BMS_RX_CTRL_1_FRAME_ID; //BMS precharge action
			TxHeader.DLC = HVBMS_BMS_RX_CTRL_1_LENGTH;
			hvbms_bms_rx_ctrl_1_pack(TxData, &TeR.BmsAppReq, TxHeader.DLC);
			HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado
		} else {
			response.code = TER_RESPONSE_CODE_INVALID_STATE_CHOICE;
		}
		break;

	case TER_COMMAND_CMD_DISCHARGE_CHOICE: //Descarga
		TeR.BmsAppReq.app_state_req =
		HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_HV__SHUTDOWN_CHOICE; //Ask for HV_Shutwdow
		TxHeader.StdId = HVBMS_BMS_RX_CTRL_1_FRAME_ID; //BMS app_state_req
		TxHeader.DLC = HVBMS_BMS_RX_CTRL_1_LENGTH;
		hvbms_bms_rx_ctrl_1_pack(TxData, &TeR.BmsAppReq, TxHeader.DLC);
		HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

		break;


	case TER_COMMAND_CMD_RESET_BMS_CHOICE: //Descarga
		TeR.BmsAppReq.app_state_req =
		HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_RESET_CHOICE; //Ask for HV_Reset
		TxHeader.StdId = HVBMS_BMS_RX_CTRL_1_FRAME_ID; //BMS app_state_req
		TxHeader.DLC = HVBMS_BMS_RX_CTRL_1_LENGTH;
		hvbms_bms_rx_ctrl_1_pack(TxData, &TeR.BmsAppReq, TxHeader.DLC);
		HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado
		break;


	case TER_COMMAND_CMD_READY2_DRIVE_CHOICE: //Ready2Drive
		if (TeR.status.state == PRECHARGED && TeR.bpps.bpps > 4) { //Pone el coche en modo driving y añadir freno

		//Permite el paso al estado drive
			TeR.status.r2_d = 1;
			TeR.appReqRight.app_state_req = 4;
			TeR.appReqLeft.app_state_req = 4;
		} else {
			response.code = TER_RESPONSE_CODE_INVALID_STATE_CHOICE;
		}
		break;

	case TER_COMMAND_CMD_BEEP_CHOICE: //MADAFUKIN BEEP
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		break;

	case TER_COMMAND_CMD_SET_DYNAMIC_CONFIG_CHOICE:
		trqPipeline_t conf;
		if (TeR.status.state < DRIVING) {
			//Limiters
			switch (command.cfg_limiter) {
			case TER_DYNAMIC_CONFIG_LIMITER_LIMIT_TORQUE_CHOICE:
				conf.limiter = &limitTorque; //Configura el limiter al torque
				break;

			case TER_DYNAMIC_CONFIG_LIMITER_LIMIT_MECH_PWR_CHOICE:
				conf.limiter = &limitMechPWR; //Configura el limiter al torque
				break;

			default:
				response.code = TER_RESPONSE_CODE_OUT_OF_RANGE_CHOICE;
				break;
			}
			switch (command.cfg_mode) {			//Mode

			case TER_DYNAMIC_CONFIG_MODE_LINEAL_CHOICE:
				conf.drivingMode = &lineal; //Configura el limiter al torque
				break;

			default:
				response.code = TER_RESPONSE_CODE_OUT_OF_RANGE_CHOICE;
				break;

			}
			switch (command.cfg_traction_control) {			//Traction Control

			case TER_DYNAMIC_CONFIG_TRACTION_CONTROL_OFF_CHOICE:
				conf.tractionControl = &tractionControlOFF; //Configura el limiter al torque
				break;

			default:
				response.code = TER_RESPONSE_CODE_OUT_OF_RANGE_CHOICE;
				break;

			}
		} else {
			response.code = TER_RESPONSE_CODE_INVALID_STATE_CHOICE;
		}
		if (response.code == TER_RESPONSE_CODE_OK_CHOICE) { // Estaba en rango
			TeR.dynamicConfig.limiter = command.cfg_limiter;
			TeR.dynamicConfig.mode = command.cfg_mode;
			TeR.dynamicConfig.traction_control = command.cfg_traction_control;
			DriveConfig = conf; //Solo si esta bien se copia al struct de pipeline
		}

		break;

	case TER_COMMAND_CMD_SET_LIMITS_CHOICE:
		if (ter_command_trq_limit_is_in_range(command.trq_limit)) {
			TeR.dynamicConfig.trq_limit = command.trq_limit;
		}
		if (ter_command_kw_limit_is_in_range(command.kw_limit)) {
			TeR.dynamicConfig.kw_limit = command.kw_limit;
		}
		if (ter_command_speed_limit_is_in_range(command.speed_limit)) {
			TeR.dynamicConfig.speed_limit = command.speed_limit;
		}
		break;

	case TER_COMMAND_CMD_SWITCH_SCS_CHOICE:
		if (command.onoff) {
			startSCS();
		} else { //if disabled enable
			stopSCS();
		}
		break;

		/*Sends messages not implemented in this board to the main can if the source is internal*/
	default: //Handles commands not implemented here
		if (!HAL_NVIC_GetActive(CAN2_RX0_IRQn)) { //Checks if command is being attended from an external source (CAN2)
			TxHeader.StdId = TER_COMMAND_FRAME_ID;
			TxHeader.DLC = TER_COMMAND_LENGTH;
			ter_command_pack(TxData, &command, TER_COMMAND_LENGTH);
			while(HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox)!= HAL_OK);
			return 0; //Exit function, no result
		}
		break;

	}
	/*Devuelve un mensaje de respuesta*/
	TxHeader.StdId = TER_RESPONSE_FRAME_ID;
	TxHeader.DLC = TER_RESPONSE_LENGTH;
	ter_response_pack(TxData, &response, TxHeader.DLC);
	HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox); //Envía el resultado de la ejecución
	return 1;
}

uint8_t easyCommand(uint8_t cmd) {
	struct ter_command_t cmdMsg;
	ter_command_init(&cmdMsg);
	cmdMsg.cmd = cmd;
	return command(cmdMsg);
}

uint8_t switchCommand(uint8_t cmd, uint8_t onOff) {
	struct ter_command_t cmdMsg;
	ter_command_init(&cmdMsg);
	cmdMsg.cmd = cmd;
	cmdMsg.onoff = onOff;
	return command(cmdMsg);
}

