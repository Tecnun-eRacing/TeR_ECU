/*
 * stateMachine.h
 *
 *  Created on: Feb 1, 2024
 *      Author: Ozuba
 *
 * Se ha elegido el aproach de consultar todas las condiciones antes de ejecutar el estado
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

enum State_t {WAITING_SL,RDY2PRECH,PRECHARGING,PRECHARGED,DRIVING}; //Estados


struct stateMachine_t{
uint8_t state; //Estado
void (*currentState[5])(void); //Function pointer array
};

void checkState(stateMachine_t* machine); //Función que comprueba cada ciclo si el estado es valido
void stateMachine(void); //Realiza las comprobaciones de cambio de estado y ejecuta el acual


//Estados
void waitingSL(void); // Comprueba
void rdy2Prech(void); // Espera a recibir el comando de precarga
void precharging(void); //Estado transitorio, monitoriza que todo va bien
void precharged(void);//Espera a que se reciba el comando de r2d
void driving(void); //Ejecuta la comanda de par



#endif /* INC_STATEMACHINE_H_ */
