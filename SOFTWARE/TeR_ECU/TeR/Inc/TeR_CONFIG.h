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
uint8_t sendConfig(uint32_t frame_id, void *config);
uint8_t initConfig();
uint8_t writeConfig(struct ter_ecu_config_t config);
void defaultConfig(void);
#endif /* INC_TER_CONFIG_H_ */
