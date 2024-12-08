/*
 * TeR_TRQTYPES.h
 *
 *  Created on: Dec 8, 2024
 *      Author: piero
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

//ManagerConfigs
typedef struct { // Contiene configuraciones del pipeline
	trq_t (*limiter)(void); //Toma un valor de limitación de potencia en kw y devuelve el torque desarrollable (trqLimit)
	trqMap_t (*drivingMode)(trq_t trqLimit); //Toma un torque limite y lo distribuye según decida el modo en las ruedas
	trqMap_t (*tractionControl)();
} trqPipeline_t;


#endif /* INC_TER_TRQTYPES_H_ */
