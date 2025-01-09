/*
 * TeR_SLCAN.h
 *
 *  Created on: Sep 7, 2024
 *      Author: Pieroebs
 *      Nota: esto es una modificacion a la libreria ya existente https://github.com/DanielAdelodun/stm32-slcan-bridge/tree/master
 */

#ifndef INC_TER_SLCAN_H_
#define INC_TER_SLCAN_H_

//Utilidades

/*
#include "TeR_COMMAND.h"
#include "TeR_CAN.h"
#define SLCAN_MTU 30  // (sizeof("T1111222281122334455667788EA5F\r")+1)

#define SLCAN_STD_ID_LEN 3 // MAX VALUE 7FF
#define SLCAN_EXT_ID_LEN 8 // MAX VALUE 1FFFFFFF

typedef enum {
	SLCAN_OK,
	SLCAN_ERROR
} SLCAN_StatusTypeDef;

SLCAN_StatusTypeDef SLCAN_Parse_Frame(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[], uint8_t *Buffer, uint32_t *Len);
SLCAN_StatusTypeDef SLCAN_Parse_Str(uint8_t *Buffer, uint32_t Len, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);
void ClearSLCAN();

extern uint32_t SLCAN_CurrentFilterID;
extern uint32_t SLCAN_CurrentFilterMask;
extern uint8_t SLCAN_CommandStringRx[SLCAN_MTU];
extern uint8_t SLCAN_CommandStringTx[SLCAN_MTU];



*/


























#endif /* INC_TER_SLCAN_H_ */
