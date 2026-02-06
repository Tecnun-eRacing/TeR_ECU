/*
 * TeR_SL.c
 *
 *  Created on: Feb 6, 2026
 *      Author: pieroebs
 *
 *      Control de relé de safety line
 *      Solo se cerrará el rele de SL si todas las tareas que intervienen en el cerrado de la SL
 *      están de acuerdo
 */
#include "TeR_SL.h"
sl_request_t sl_request;
uint32_t sl_relay_request;


/*
 * SafetyLine Task
 * comprueba si todas las tasks involucradas están de acuerdo en cerrar la safety line
 * En ese caso, cierra, caso contrario se abrirá la safetyline
 *
 * */
void safetyLine(void *argument){
	for(;;){
		osDelay(10);
		if(can_close_relay()){
			HAL_GPIO_WritePin(SC_EN_GPIO_Port, SC_EN_Pin, 1); // Cerrar rele de SL
			//todo añadir mensaje de can
		}
		else{
			HAL_GPIO_WritePin(SC_EN_GPIO_Port, SC_EN_Pin, 0); // Abrir rele de SL
		}
	}
}

/*
 * Setear bitflag dependiendo de request y value
 * -> request es un enum y value el valor que quieras 1 cerrar rele, 0 abrir rele
 *
 * */
void set_sl_request(sl_request_t request, uint32_t value){
	if(value){
		sl_relay_request |= (1 << request); // poner a 1, la tarea quiere cerrarlo
	}
	else{
		sl_relay_request &= ~(1 << request); // poner bit a 0, la tarea quiere abrirlo
	}
}
uint8_t can_close_relay(void) {
    return (sl_relay_request == (1 << MAX_TASKS) - 1); // Si todos los bits en 1, retorna 1, caso contrario, retorna 0
}

