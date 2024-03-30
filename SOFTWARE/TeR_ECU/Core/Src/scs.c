/*
 * scs.c
 *
 *  Created on: Mar 28, 2024
 *      Author: oihan piero ozuba
 */

#include "scs.h"
/*
 * Hay que añadir a tu gestor de interrupciones favorito el callback de checkeo
 *
 */

//Internal Id and timestamp lists
uint32_t scsIds[] = SCS; //Las señales las añade la macro del preprocesor
const uint8_t nSCS = sizeof(scsIds)/sizeof(uint32_t);
uint32_t timestamps[sizeof(scsIds)/sizeof(uint32_t)];

uint32_t lastFailSCS; //Id de la ultima señal que falló (Debugging purposes)


//Timer pointers used by module
TIM_HandleTypeDef *base;
TIM_HandleTypeDef *check;

//Module control functions
uint8_t initSCS(TIM_HandleTypeDef *timBase, TIM_HandleTypeDef *timCheck) {
//Alamacenamos los timers de uso
	base = timBase;
	check = timCheck;
	HAL_TIM_RegisterCallback(check, HAL_TIM_PERIOD_ELAPSED_CB_ID, checkSCS); //Attach check function to timer
	return 1;
}

uint8_t startSCS(void) { //Activa la comprobación activa de tiempos
	//Resetea a 0 el timer y los timestamps para evitar errores al volver a arrancar
	memset(&timestamps,0,sizeof(timestamps));
	base->Instance->CNT  = 0;
	//Arranca los timers
	HAL_TIM_Base_Start(base); //arranca nuestra base de tiempo (Ojo hará overflow en 52 días jajaj)
	HAL_TIM_Base_Start_IT(check); //Arranca el ciclo de comprobación
	return 1;
}

uint8_t stopSCS(void) { //Desactiva la comprobación activa de tiempos
	HAL_TIM_Base_Stop(base); //para nuestro timer para que no haya sorpresas en el próximo check
	HAL_TIM_Base_Stop_IT(check); //Arranca el ciclo de comprobación
	return 1;
}

//Log and check callback functions
uint8_t logSCS(uint32_t id) {
	uint8_t i = 0;
	while(i < nSCS && scsIds[i] != id) //Ojo el orden importa el segundo no se evalua si el primero falla (Crash al reves, acceso fuera del array)
		 i++; //Comprueba si el ID está en el la lista de señales criticas, se para cuando hay un match o se ha excedido el tamaño del array
	if (i >= nSCS) { //Hemos recorrido el array entero (i no es un indice valido N-1)
		return 0; //No era una SCS
	} else { //Era una SCS
		timestamps[i] = base->Instance->CNT; //Loguea el valor del counter en su slot
		return 1; //Es una SCS
	}
	return -1; //Wtf no debería llegar aqui nunca
}

void checkSCS(TIM_HandleTypeDef *tim) {
	for (uint8_t i = 0; i < nSCS; i++) {
		if (base->Instance->CNT - timestamps[i] > SCS_TIMEOUT) { //Hay una señal perdida
			lastFailSCS  = scsIds[i]; //Guarda la ultima señal problematica, util a modo de debug
			command(TER_COMMAND_CMD_DISCHARGE_CHOICE,(void*) 0); //LLama al comando de descarga
			TeR.apps.apps_av = 0; //Porsiaka
		}
	}
}

