/*
 * easyTV.h
 *
 *  Created on: Dec 7, 2024
 *      Author: Piero
 */

#ifndef INC_EASYTV_H_
#define INC_EASYTV_H_
#include "TeR_CAN.h" //For controlling TeR vehicle
#include "pid.h"
#include "TeR_TRQTYPES.h"
#include "math.h"
trqMap_t easyTorque(trq_t limit);
#endif /* INC_EASYTV_H_ */
