/*
 * firFilter.c
 *
 *  Created on: Feb 25, 2024
 *      Author: ozuba
 */
#include "firFilter.h"

/* ---------------------------[Constructor/Destructor]-------------------------- */

FIR_t *initFilter(float *coeffs, uint16_t order, int32_t xMax)
{ // For simplicity we asume simetric bounds so +- signals are contemplated todo(Add function normalize intervals and get full scale)
	FIR_t *filter = (FIR_t *)malloc(sizeof(FIR_t));
	filter->N = order;

	// Normalize coefficents to fit signal (Max of any coeff*signal product should fit inside of a int32_t, so scaling 2E31-1/xMax), this should maximize resolution of the filter, guarantees that product doesnt overflow
	filter->coeffs = (int32_t *)malloc(filter->N * sizeof(int32_t)); // reserva array para los coeficientes
	memset(filter->coeffs, 0, filter->N * sizeof(int32_t));			 // To a 0

	// Selección del factor
	// filter->factor = INT32_MAX / xMax; //Selecciona el factor (Cualquier producto < INT32_MAX)
	filter->factor = (INT32_MAX / filter->N) / xMax;			  // Aunque la señal durase todos los taps en xMax no haría overflow al accumulador
	float2int(filter->factor, coeffs, filter->coeffs, filter->N); // Copia los coeficientes escalados

	// Verificación de la coherencia
	int32_t sum = 0;
	for (uint16_t i = 0; i < filter->N; i++)
	{
		sum += filter->coeffs[i];
	}
	filter->coherence = ((float)sum) / filter->factor;

	// Genera el Buffer circular demuestras y lo pone a 0 para evitar sustos
	filter->buffer = (int32_t *)malloc(filter->N * sizeof(int32_t));
	memset(filter->buffer, 0, filter->N * sizeof(int32_t)); // To a 0
	// Initialize index to 0
	filter->head = 0; // Indice de escritura

	return filter;
}

void deInitFilter(FIR_t *filt)
{ // Frees all used memory by filter
	free(filt->coeffs);
	free(filt->buffer);
	free(filt);
}

/* ---------------------------[Filtrado y auxiliares]-------------------------- */

int32_t filter(FIR_t *filt, int32_t x)
{
	put(filt, x);	 // Añade el sample
	int32_t acc = 0; // Acumulador
	// Filtrado
	for (uint16_t i = 0; i < filt->N; i++)
	{ // y[n] = b0*x[n-0] + b1*x1[n-1] +... + x[n-N] (Filtrado)
		acc += get(filt, i) * filt->coeffs[i];
	}
	return (acc / filt->factor); // Dividimos al final(Cada vez que dividimos cometemos un error de +-1)
}

void put(FIR_t *filt, int32_t x)
{
	if (filt->head + 1 < filt->N)
	{				  // Si el indice al que vamos a escribir no está fuera del limite
		filt->head++; // Increments last sample index
	}
	else
	{
		filt->head = 0; // Index overflowed return to beginning
	}
	filt->buffer[filt->head] = x; // Introduce sample in ring buffer
}

// gets sample x[n-i]
int32_t get(FIR_t *filt, uint16_t i)
{
	if (filt->head + i >= filt->N)
	{ // Nuestro valor esta en la cola izquierda
		return filt->buffer[filt->head + i - filt->N];
	}
	else
	{ // Nuestro valor esta a la derecha de la cabeza
		return filt->buffer[filt->head + i];
	}
}

void float2int(float factor, float *src, int32_t *dest, size_t size)
{
	for (uint16_t i = 0; i < size; i++)
	{
		dest[i] = floor(src[i] * factor); // redondeo a la baja para evitar posibles overflows en casos limite
	}
}

/* ---------------------------[Generación de Filtros]-------------------------- */
// Moving Average
FIR_t *movingAvgFilter(uint32_t M, int32_t xMax)
{
	float avgCoeff[M]; // Tasty VLA
	for (uint32_t i = 0; i < M; i++)
	{
		avgCoeff[i] = 1.0 / (float)M;
	}
	return initFilter(avgCoeff, M, xMax);
}
