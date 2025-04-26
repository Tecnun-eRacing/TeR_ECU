/*
 * TeR_SERVOS.c
 *
 *  Created on: Apr 1, 2025
 *      Author: heliosestevezorozco
 */

#include "servo.h"
#include "tim.h"
// Max value 2.500 ms
// Min value 0.5 ms

// FreeRTOS dependencies
const static int task_period = 20; // Task frequency
uint32_t currentTick; // declaramos nuestra variable currentTick como global (para reactualizar su valor al parar la maquina de estados)
// IMPORTANTE: Se utiliza osDelayUntil debido a que es la manera recomendada por FreeRTOS en el reference manual para ejecucion temporal estricta sin desfases

//Variables modulo
uint32_t incFreq; //Frecuencia del timer

void setAngle(uint32_t* channel,uint8_t angle);
//180 rotation counterclockwise

void servos(void *argument) {
	servo(&TIM1->CCR1)
	//Turn on PWM
	uint32_t nextTick = osKernelGetTickCount(); // Initialize reference time
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//Calculamos la frecuencia del timer
	incFreq =  2 * HAL_RCC_GetPCLK1Freq() / (htim3.Instance->PSC + 1); //x2 because timers have a multiplier before

	for (;;) {
		nextTick += task_period; //Genera el timestamp de la siguiente ejecucion
		osDelayUntil(nextTick);
		stateLoop(); //ejecutamos la maquina de estados del vehiculo ??????? no deberia de estar aqui

	}
}


void setAngle(uint8_t angle) {
	uint16_t low = incFreq * 0.0005;
	uint16_t high = incFreq * 0.0025;
	TIM1->CCR1 = angle * (high - low) / 180.0 + low;
}
