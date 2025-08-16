/*
 * TeR_CONFIG.h
 *
 *  Created on: Apr 12, 2025
 *      Author: piero
 */

#ifndef INC_TER_CONFIG_H_
#define INC_TER_CONFIG_H_
#include "TeR_CAN.h"
#include "ee24.h"
typedef struct{
	struct ter_ecu_config_t config;
	uint8_t written;
}eeprom_data_t;
#define NB_ENTRIES 26 //numero de entradas de configuración que hay definidas
#define RETRY_TIMEOUT 10
#define ALL_CONFIGS 0xFFFFFFFF


uint8_t sendConfig(uint32_t frame_id, void *config);
uint8_t initConfig();
uint8_t writeConfig(struct ter_ecu_config_t config);
void defaultConfig(struct ter_ecu_config_t* config);
uint8_t publishConfig(struct ter_ecu_config_t *config,uint32_t config_id);
#endif /* INC_TER_CONFIG_H_ */
