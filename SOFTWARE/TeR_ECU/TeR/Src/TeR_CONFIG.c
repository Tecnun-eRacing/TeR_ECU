/*
 * TeR_CONFIG.c
 *
 *  Created on: Apr 12, 2025
 *      Author: piero
 *
 *      Sistema de configs.
 *      Sortea por frame ids de configs y lo envia.
 *      tienes que castear el puntero void a el tipo de dato que le entre
 *
 */
#include "TeR_CONFIG.h"

uint8_t sendConfig(uint32_t frame_id, void *config) {
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	if (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN)) {
		switch (frame_id) {
		case TER_REFRI_CONFIG_FRAME_ID: //configurar refri
			struct ter_refri_config_t *refri_config = (struct ter_refri_config_t *)config;
			TxHeader.StdId = frame_id;
			TxHeader.DLC = TER_REFRI_CONFIG_LENGTH;
			ter_refri_config_pack(TxData,refri_config,
					TxHeader.DLC);
			break;


		}
		while (HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox)
				!= HAL_OK);


		return 0;
	}
	return 1;
}

uint8_t recieveConfig(uint32_t frame_id, void *config){

	switch(frame_id){
	case TER_ECU_CONFIG_FRAME_ID:
		struct ter_ecu_config_t ecu_config = *(struct ter_ecu_config_t *)&config;

		switch(ecu_config.entry){
		case TER_ECU_CONFIG_ENTRY_DRIVING_MODE_CHOICE:

		case TER_ECU_CONFIG_ENTRY_LIMITER_CHOICE:

		case TER_ECU_CONFIG_ENTRY_TRACTION_CONTROL_CHOICE:

		case TER_ECU_CONFIG_ENTRY_TRQ_LIMIT_CHOICE:

		case TER_ECU_CONFIG_ENTRY_R2_D_BRAKE_VAL_CHOICE:

		case TER_ECU_CONFIG_ENTRY_SCS_ENABLE_CHOICE:

		case TER_ECU_CONFIG_ENTRY_TRQV_KP_CHOICE:

		case TER_ECU_CONFIG_ENTRY_TRQV_KI_CHOICE:

		case TER_ECU_CONFIG_ENTRY_TRQV_KD_CHOICE:

		}

	}

}

