/*
 * TeR_SL.c
 *
 *  Created on: Feb 6, 2026
 *      Author: pieroebs
 *
 *      Control de relé de safety line
 *      Solo se cerrará el rele de SL si todas las tareas que intervienen en el cerrado de la SL
 *      están de acuerdo
 *      Es sin mas crear un mapa de bitflags e ir poniendo / quitando las flags, si todas estan puestas
 *      se cierra el rele
 *      si alguna no esta puesta, el rele se abre
 *
 *      No es necesario tener una task separada, se podria integrar, por ejemplo en el módulo de las SCS
 *      Lo que pasa, es que de momento, para ver las cosas un poco mas claras, prefiero separado
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
			TeR.status.sl_relay = 1;
		}
		else{
			HAL_GPIO_WritePin(SC_EN_GPIO_Port, SC_EN_Pin, 0); // Abrir rele de SL
			TeR.status.sl_relay = 0;
		}
	}
}

/*
 * Setear bitflag dependiendo de request y value
 * -> request es un enum y value el valor que quieras 1 cerrar rele, 0 abrir rele
 * Con ese enum se bitshiftea n veces el valor del enum
 * ES IMPORTANTE QUE MAX_TASK COINCIDA CON EL NUMERO DE SL_REQUEST_T QUE TENGAS
 *
 * */
void set_sl_request(sl_request_t request, uint32_t value){
	if(value){
		sl_relay_request |= (1 << request); // poner a 1 el bit correspondiente, la tarea quiere cerrarlo
	}
	else{
		sl_relay_request &= ~(1 << request); // poner bit a 0 el bit correspondiente, la tarea quiere abrirlo
	}
}
uint8_t can_close_relay(void) {
    return (sl_relay_request == (1 << MAX_TASKS) - 1); // Si todos los bits en 1, retorna 1, caso contrario, retorna 0
}

