/*
 * TeR_CONFIG.c
 *
 *  Created on: Apr 12, 2025
 *      Author: piero
 *
 *      Sistema de configs.
 *
 *      sendConfig(), envia configuracion para un nodo específico por CAN
 *      Sortea por frame ids de configs y lo envia.
 *      tienes que castear el puntero generico a la config que quieras, de momento solo refri
 *
 *      initConfig(), lee la eeprom, verifica estado y actualiza vector de configuraciones, si algo va mal se setea una config default, se llama cada vez que se inicializa la maquina de estados
 *      writeConfig(), escribe TeR.config en la eeprom, se llama cada vez que se recibe el mensaje de TeR Config
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
	while (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN) == 0) {
		osDelay(RETRY_TIMEOUT);
	} // ESPERAR A QUE HAYA SITIO mejor usar osThreadYield
	if (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN)) {

		switch (frame_id) {
		case TER_REFRI_CONFIG_FRAME_ID: //configurar refri
			struct ter_refri_config_t refri_config =
					*(struct ter_refri_config_t*) config;
			TxHeader.StdId = TER_REFRI_CONFIG_FRAME_ID;
			TxHeader.DLC = TER_REFRI_CONFIG_LENGTH;
			ter_refri_config_pack(TxData, &refri_config, TxHeader.DLC);
			break;
		default:
			return 1;
		}
		for (int i = 0; i < 10; i++) {
			if (HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox)
					== HAL_OK) {
				break;
			} else {
				osDelay(RETRY_TIMEOUT);
			}

		}

		return 0;
	}
	return 1;
}

uint8_t initConfig() { //wrapper functions to not directly interact with library
	EE24_Init(&eeprom, &hi2c2, EE24_ADDRESS_DEFAULT);
	EE24_Read(&eeprom, 0, (uint8_t*) &data, sizeof(data), 500); // load config struct
	if (data.written == 1) { // if eeprom has been written, copy data to car
		TeR.config = data.config;
		return 1;
	} // if eeprom was not written or anything when bad (data.written is defaulted 0), default config should be loaded
	defaultConfig(&TeR.config);
	writeConfig(TeR.config);
	return 0;
}

uint8_t writeConfig(struct ter_ecu_config_t config) {

	if (config.entry == TER_ECU_CONFIG_ENTRY_EEPROM_CHOICE) {
		switch (config.eeprom) {
		case TER_ECU_CONFIG_EEPROM_CLEAR_AND_DEFAULT_CHOICE:
			defaultConfig(&config); //reset config to predetermined values
			break;
		case TER_ECU_CONFIG_EEPROM_READ_CHOICE:
			publishConfig(&config, ALL_CONFIGS);
			break;
		}
	}
	TeR.config = config;
	data.config = TeR.config;
	data.written = 1;
	return EE24_Write(&eeprom, 0, (uint8_t*) &data, sizeof(data), 500);
}

void defaultConfig(struct ter_ecu_config_t *config) { //set car internal config struct to default
	config->entry = TER_ECU_CONFIG_ENTRY_SCS_ENABLE_CHOICE; // para evitar bucle de reset de eeprom, seteamos entry a un valor por defecto (por definir en .dbc)
	config->driving_mode =
	TER_ECU_CONFIG_DRIVING_MODE_LINEAL_CHOICE;
	config->limiter = TER_ECU_CONFIG_LIMITER_TORQUE_CHOICE;
	config->r2_d_brake = 5;
	config->scs_enable = TER_ECU_CONFIG_SCS_ENABLE_ENABLE_CHOICE;
	config->traction_control =
	TER_ECU_CONFIG_TRACTION_CONTROL_OFF_CHOICE;
	config->trq_kp = 0;
	config->trq_ki = 0;
	config->trq_kd = 0;
	config->trq_limit = 100;
	config->flap_enable = TER_ECU_CONFIG_FLAP_ENABLE_OFF_CHOICE;
	config->flap_l_offset = -8;
	config->flap_r_offset = 65;
	config->flap_l_reverse = TER_ECU_CONFIG_FLAP_L_REVERSE_NORMAL_CHOICE;
	config->flap_r_reverse = TER_ECU_CONFIG_FLAP_R_REVERSE_REVERSE_CHOICE;
	config->flap_pedal_setpoint = 90;
	config->regen_enable = TER_ECU_CONFIG_REGEN_ENABLE_DISABLE_CHOICE; //defaulted off
	config->regen_max_cell_temp = 45;
	config->regen_max_cell_volt = 3900;
	config->regen_max_trq = 10;
	config->regen_thr_speed = 10;
	config->regen_max_current = 40;
	config->regen_thr_rpm = 5;
	config->regen_trq_slope = 1;
	config->regen_mode = TER_ECU_CONFIG_REGEN_MODE_APPS_CHOICE;
	config->regen_max_positive_trq_thr = 5;
	return;
}

uint8_t publishConfig(struct ter_ecu_config_t *config, uint32_t config_id) {
	if (data.written != 1) { // si no se ha cargado la eeprom, no publicamos la configuracion, evitamos exponer junk a la pantalla en el arranque por ejemplo
		return 1;
	}
//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = TER_ECU_CONFIG_FRAME_ID;
	TxHeader.DLC = TER_ECU_CONFIG_LENGTH;
	struct ter_ecu_config_t ecu_config = *(struct ter_ecu_config_t*) config;
	if ((config_id != ALL_CONFIGS) && (config_id <= NB_ENTRIES)
			&& (config_id >= 0)) { // if user is requesting a specific config, and the config is valid, send specific config
		ecu_config.entry = config_id;
		ter_ecu_config_pack(TxData, &ecu_config, TxHeader.DLC);
		for (int i = 0; i < 10; i++) {
			if (HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox)
					== HAL_OK) {
				break;
			} else {
				osDelay(RETRY_TIMEOUT);
			}

		}

	} else { // if user requested all configs or requested config is not valid, send all configs
		for (uint8_t i = 0; i < NB_ENTRIES; i++) {
			if (i == TER_ECU_CONFIG_ENTRY_EEPROM_CHOICE) { //pa que voy a mandar esto
				continue;
			}
			ecu_config.entry = i;
			ter_ecu_config_pack(TxData, &ecu_config, TxHeader.DLC);
			for (int i = 0; i < 10; i++) {
				if (HAL_CAN_AddTxMessage(mainCAN, &TxHeader, TxData, &mailbox)
						== HAL_OK) {
					break;
				} else {
					osDelay(RETRY_TIMEOUT);
				}

			}
		}
	}
	return 0;
}

