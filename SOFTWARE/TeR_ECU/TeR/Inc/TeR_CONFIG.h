/*
 * TeR_CONFIG.h
 *
 *  Created on: Apr 12, 2025
 *      Author: piero
 */

#ifndef INC_TER_CONFIG_H_
#define INC_TER_CONFIG_H_
#include "TeR_CAN.h"

uint8_t sendConfig(uint32_t frame_id, void *config);
uint8_t recieveConfig(uint32_t frame_id, void *config);
#endif /* INC_TER_CONFIG_H_ */
