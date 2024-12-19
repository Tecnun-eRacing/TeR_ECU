/*
 * TeR_TRQTYPES.h
 *
 *  Created on: Dec 8, 2024
 *      Author: piero
 *      por naturaleza se van a formar dependencias ciclicas
 *      ya que los modulos de torque necesitan los tipos
 *      y el torquemanager necesita los modulos
 *      dependencia ciclica de manual, esto lo previene
 */

#ifndef INC_TER_TRQTYPES_H_
#define INC_TER_TRQTYPES_H_
#include <stdint.h>
//Dynamic value types
typedef int32_t trq_t; //Mucho ojo va a tener signo por ahora regen/marcha atrás, tiene sentido (Se implementarán sanity checks)
typedef struct { //Si quieres hacer un 4wd añade 2 miembros más y a correr
	trq_t rLeft; //Rear left wheel
	trq_t rRight; //Rear right wheel
}trqMap_t;
#endif /* INC_TER_TRQTYPES_H_ */
