/*
 * firFIlter.h
 *
 *  Created on: Feb 25, 2024
 *      Author: ozuba
 */

#ifndef SRC_FIRFILTER_H_
#define SRC_FIRFILTER_H_
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
//Include your malloc/free provider
#include <stdlib.h>
/*
 * Librería para filtro fir, consiste en un buffer circular donde se introduce una muestra y la función devuelve un computo
 * de la muestra saliente que sufre de cierta latencia debida al filtrado. Esta librería esta pensada bajo la premisa de que
 * trabajamos en una maquina sin unidad de coma flotante donde por eficiencia es necesario utilizar aritmetica de enteros.
 */

typedef struct {

	uint16_t N; //Orden del filtro/Numero de elementos de los buffers
	int32_t factor; // Factor de escalado para el full range, escala los coeficientes para que el sumatorio no haga overflow en el caso limite
	float coherence; //Valor de coherencia debería valer 1

	//Operation Vars
	int32_t *coeffs; //memory for coefficents
	//Buffer Circular (Doble indice probablemente redundante ya que productor y consumidor se ejecutan en la misma función)
	int32_t *buffer; //Buffer for signal
	uint16_t head; //Indice de la ultima muestra introducida al buffer a partir de la cual referenciamos al resto
} FIR_t;

FIR_t* initFilter(float *coeffs, uint16_t order, int32_t xMax); //Creates filter, with the specified input min and max, should help denormalize coefficents
int32_t filter(FIR_t *filt, int32_t x); //The wanted thing, easy peasy intuitive function
void deInitFilter(FIR_t *filt);

//Circular buffer functions Es basicamente una cola FIFO donde no hay que hacer desfase
void put(FIR_t *filt, int32_t x); // Puts a sample into the front of the buffer
int32_t get(FIR_t *filt, uint16_t i); // gets sample x[n-index]

//Aux functions
void float2int(float factor, float *src, int32_t *dest, size_t size); //Normalizes an array by factor

//Generación de filtros
FIR_t *movingAvgFilter(uint32_t M, int32_t xMax);

#endif /* SRC_FIRFILTER_H_ */
