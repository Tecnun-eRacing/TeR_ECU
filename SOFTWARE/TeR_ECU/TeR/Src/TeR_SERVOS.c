/*
 * TeR_SERVOS.c
 *
 *  Created on: Apr 18, 2025
 *      Author: piero
 */
#include "TeR_SERVOS.h"
#include "TeR_CAN.h"

extern TIM_HandleTypeDef htim3;
flap_t flapL;
flap_t flapR;
void servos(void *argument) {
	flapL.offset = 30; // offset flap L
	flapR.offset = 30; // offset flap R
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //start flap
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //start flap
	for (;;) {
		osDelay(10);
		setAngle(flapL.angle, TIM_CHANNEL_1);
		setAngle(TeR.apps.apps_1*30/255, TIM_CHANNEL_2);
	}

}

void setAngle(int8_t angle, uint8_t channel) {
	uint32_t incFreq = 2 * HAL_RCC_GetPCLK1Freq() / (htim3.Instance->PSC + 1);
	uint16_t low = incFreq * 0.0005;
	uint16_t high = incFreq * 0.0025;
	switch (channel) {
	case TIM_CHANNEL_1: // left servo
		TIM3->CCR1 = (angle + flapL.offset)* (high - low) / 180.0 + low;
		break;

	case TIM_CHANNEL_2: // right servo
		TIM3->CCR2 = ( angle + flapR.offset) * (high - low) / 180.0 + low;
		break;
	}
}

