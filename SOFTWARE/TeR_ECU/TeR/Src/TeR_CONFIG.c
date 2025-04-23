/*
 * TeR_CONFIG.c
 *
 *  Created on: Apr 12, 2025
 *      Author: piero
 *
 *      Sistema de configs. en pruebas
 *
 *      sendConfig(), envia configuracion para un nodo específico por can
 *      Sortea por frame ids de configs y lo envia.
 *      tienes que castear el puntero void a la config que quieras, de momento solo refri
 *
 *      initConfig(), lee la eeprom, verifica estado y actualiza vector de configuraciones
 *      writeConfig(), escribe TeR.config en la eeprom
 *
 */
#include "TeR_CONFIG.h"
extern I2C_HandleTypeDef hi2c2;
EE24_HandleTypeDef eeprom; // handle de la eeprom
eeprom_data_t data; // estructura de datos de la eeprom

uint8_t sendConfig(uint32_t frame_id, void *config) {
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
while (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN) == 0){
	osDelay(2);}// ESPERAR A QUE HAYA SITIO mejor usar osThreadYield
	if (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN)) {

		switch (frame_id) {
		case TER_REFRI_CONFIG_FRAME_ID: //configurar refri
			struct ter_refri_config_t refri_config =
					*(struct ter_refri_config_t*) config;
			TxHeader.StdId = TER_REFRI_CONFIG_FRAME_ID;
			TxHeader.DLC = TER_REFRI_CONFIG_LENGTH;
			ter_refri_config_pack(TxData, &refri_config, TxHeader.DLC);
			break;

		}
		while (HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox)
				!= HAL_OK){
			osDelay(2); // ESPERAR A ENVIO CORRECTO mejor usar osThreadYield
		}

		return 0;
	}
	return 1;
}

uint8_t initConfig() { //wrapper functions to not directly interact with library
	EE24_Init(&eeprom, &hi2c2, EE24_ADDRESS_DEFAULT);
	EE24_Read(&eeprom, 0, (uint8_t*) &data, sizeof(data), 250); // load config struct
	if (data.written == 1) { // if eeprom has been written, copy data to car
		TeR.config = data.config;
		return 1;
	} // if eeprom was not written or anything when bad (data.written is defaulted 0), default config should be loaded
	defaultConfig();
	writeConfig(TeR.config);
	return 0;
}

uint8_t writeConfig(struct ter_ecu_config_t config) {
	data.config = config;
	data.written = 1;
	return EE24_Write(&eeprom, 0, (uint8_t*)&data, sizeof(data), 500);
}

void defaultConfig(void) {
	TeR.config.driving_mode =
	TER_ECU_CONFIG_DRIVING_MODE_LINEAL_CHOICE;
	TeR.config.limiter = TER_ECU_CONFIG_LIMITER_TORQUE_CHOICE;
	TeR.config.r2_d_brake = 4;
	TeR.config.scs_enable = 1;
	TeR.config.traction_control =
	TER_ECU_CONFIG_TRACTION_CONTROL_OFF_CHOICE;
	TeR.config.trq_kp = 0;
	TeR.config.trq_ki = 0;
	TeR.config.trq_kd = 0;
	TeR.config.trq_limit = 100;
	return;
}


