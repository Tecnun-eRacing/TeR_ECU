/*
 * scs.h
 *
 *  Created on: Mar 28, 2024
 *      Author: piero oihan miguel
 */

#ifndef INC_SCS_H_
#define INC_SCS_H_
#include "stm32f4xx_hal.h"
#include "ter.h"
#define N_SCS 4

//Publicas
uint8_t initSCS(TIM_HandleTypeDef *timBase, TIM_HandleTypeDef *timCheck);
uint8_t logSCS(uint32_t id);

uint8_t checkSCS(void);


#endif /* INC_SCS_H_ */
