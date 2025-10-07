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
	uint32_t size = 8;
	uint32_t id = 0;
	struct ter_response_t response;
	response.cmd = command.cmd;
	response.code = TER_RESPONSE_CODE_OK_CHOICE; //Lo pone a ok si nadie dice lo contrario

	/*-----------------------------------------[COMANDOS]---------------------------------------*/
	switch (command.cmd) { //Hay que generar un archivon los defines de esto en el repo de DBCS

	case TER_COMMAND_CMD_PRECHARGE_CHOICE: //Precarga
		if (TeR.status.state == RDY2PRECH) { //Envía al bms el mensaje de precarga
			TeR.BmsAppReq.app_state_req =
			HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_HV_READY_PRECHARGE_CHOICE; //Solicitamos la precarga al BMS
		} else {
			response.code = TER_RESPONSE_CODE_INVALID_STATE_CHOICE;
		}
		break;

	case TER_COMMAND_CMD_DISCHARGE_CHOICE: //Descarga
		TeR.BmsAppReq.app_state_req =
		HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_HV_SHUTDOWN_CHOICE; //Ask for HV_Shutwdown
		break;

	case TER_COMMAND_CMD_RESET_BMS_CHOICE: //Descarga
		TeR.BmsAppReq.app_state_req =
		HVBMS_BMS_RX_CTRL_1_APP_STATE_REQ_STANDBY_CHOICE; //Ask for HV_Reset
		break;

	case TER_COMMAND_CMD_READY2_DRIVE_CHOICE: //Ready2Drive
		if ((TeR.status.state == PRECHARGED)
				&& (ter_bpps_bpps_decode(TeR.bpps.bpps) >= TeR.config.r2_d_brake)) { //Pone el coche en modo driving y añadir freno

			//Permite el paso al estado drive
			TeR.status.r2_d = 1;
			TeR.appReqRight.app_state_req = 4;
			TeR.appReqLeft.app_state_req = 4;
		} else {
			response.code = TER_RESPONSE_CODE_INVALID_STATE_CHOICE;
		}
		break;

	case TER_COMMAND_CMD_BEEP_CHOICE: //MADAFUKIN BEEP
		HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_SET);
		osDelay(1000);
		HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_RESET);
		break;

		/*Sends messages not implemented in this board to the main can if the source is internal*/
	default: //Handles commands not implemented here
		if (!HAL_NVIC_GetActive(CAN2_RX0_IRQn)) { //Checks if command is being attended from an external source (CAN2)
			id = TER_COMMAND_FRAME_ID;
			size = TER_COMMAND_LENGTH;
			ter_command_pack(TxData, &command, size);
			while (!can_scheduler_insert_non_periodic_msg(TxData, size, id, 10)) {
				osDelay(10);
			};
			return 0; //Exit function, no result
		}
		break;

	}
	/*Devuelve un mensaje de respuesta*/
	id = TER_RESPONSE_FRAME_ID;
	size = TER_RESPONSE_LENGTH;
	ter_response_pack(TxData, &response, size);
	while (!can_scheduler_insert_non_periodic_msg(TxData, size, id, 10)) {
		osDelay(10);
	};

	return 1;
}

uint8_t easyCommand(uint8_t cmd) {
	struct ter_command_t cmdMsg;
	ter_command_init(&cmdMsg);
	cmdMsg.cmd = cmd;
	return command(cmdMsg);
}

//Deprecate, now we use the config struct for switching things
uint8_t switchCommand(uint8_t cmd, uint8_t onOff) {
	struct ter_command_t cmdMsg;
	ter_command_init(&cmdMsg);
	cmdMsg.cmd = cmd;
	cmdMsg.onoff = onOff;
	return command(cmdMsg);
}

