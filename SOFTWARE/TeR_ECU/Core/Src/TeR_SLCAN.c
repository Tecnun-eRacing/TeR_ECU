/*
 * TeR_SLCAN.c
 *
 *  Created on: Sep 7, 2024
 *      Author: Pieroebs
 *      Nota: esto es una adaptacion a la libreria ya existente https://github.com/DanielAdelodun/stm32-slcan-bridge/tree/master
 *
 *
 *
 *      El objetivo de esta libreria es proporcionar un puente mediante Serial Line CAN, el cual nos permite, dicho de una forma sencilla
 *      realizar un bridge entre el periferico CAN del microcontrolador y el puerto USB de nuestro ordenador, y que utilizando herramientas del
 *      kernel de Linux como son Socket-CAN, podamos realizar acciones de codificacion y decodificacion sin necesidad de un dispositivo intermedio
 *      como un VECTOR o USB2CAN.
 *
 *
 */

/*
// INCLUDES
#include "TeR_SLCAN.h"
static uint8_t FilterBuf[20];						// Buffer to store ASCII encoded Filter+Mask (Only updated after 'MM' Requests)
static uint8_t StatusBuf[11];						// Buffer to store ASCII encoded Status Flag
static uint32_t NextFilterID = 0;					// 32bit 'Left Aligned' Filter to be Added to Bank 0 of CAN peripheral
static uint32_t NextFilterMask = 0;					// 32bit 'Left Aligned' Mask to be Added to Bank 0 of CAN peripheral
static uint8_t SetTimestamp = 0;					// Should timestamps be added to SLCAN strings?

uint32_t SLCAN_CurrentFilterID = 0;					// 32bit 'Left Aligned' Filter currently assigned to Bank 0 of CAN peripheral
uint32_t SLCAN_CurrentFilterMask = 0;				// 32bit 'Left Aligned' Filter currently assigned to Bank 0 of CAN peripheral
uint8_t SLCAN_CommandStringRx[SLCAN_MTU];			// Holder for outgoing SLCAN Data (i.e. filled on CAN receive) Used in main
uint8_t SLCAN_CommandStringTx[SLCAN_MTU];			// Holder for incoming SLCAN Data (i.e. filled on CAN transmit) Used in USB


SLCAN_StatusTypeDef SLCAN_Parse_Frame(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[], uint8_t *Buffer, uint32_t *Len){
    uint8_t BufferIndex = 0;
    uint8_t IdLen;
    uint32_t ID;

    uint8_t counter;

    // Clear any data in string buffer
    for (counter = 0; counter < SLCAN_MTU; counter++) {
        Buffer[counter] = '\0';
    }

    // *** Add Frame Type ***
    if (pHeader->RTR == CAN_RTR_DATA) {
        Buffer[BufferIndex] = 't';
    }
    else if (pHeader->RTR == CAN_RTR_REMOTE) {
        Buffer[BufferIndex] = 'r';
    }
    else {
        // How did we get here?
        return SLCAN_ERROR;
    }

    // If Extended Frame, convert frame type character to uppercase and note the length of an Extended ID (8 Hex digits)...
    if (pHeader->IDE == CAN_ID_EXT) {
        Buffer[BufferIndex] -= 32;
        IdLen = SLCAN_EXT_ID_LEN;
        ID = pHeader->ExtId;
    }
    // ...Otherwise, note the length of a Standard ID (3 Hex digits)
    else {
        IdLen = SLCAN_STD_ID_LEN;
        ID = pHeader->StdId;
    }

    BufferIndex++;

    // *** Add Identifier ***
    // Note: ID variable is invalid after this step
    for(counter=IdLen; counter > 0; counter--) {
        // Add ID to Buffer[1:IdLen] inclusive by adding one nybble at a time, 'Right' to 'Left'
        Buffer[counter] = (ID & 0xF);	// Match Rightmost 4 Bits of ID, assign to 'Right' of the Buffer
        ID >>= 4;
        BufferIndex++;
    }

    // *** Add DLC to Buffer ***
    Buffer[BufferIndex++] = pHeader->DLC;

    // *** Add data bytes Buffer, one nybble at a time ***
    for (counter = 0; counter < pHeader->DLC; counter++) {
        Buffer[BufferIndex++] = (aData[counter] >> 4);		// Top 4 Bits
        Buffer[BufferIndex++] = (aData[counter] & 0x0F);	// Bottom 4 Bits
    }

    // *** Add Timestamp if requested ***
    if (SetTimestamp) {
        uint32_t Timestamp;
        uint8_t BufferIndexFreeze = BufferIndex;
    	Timestamp = pHeader->Timestamp;
    	for (counter = 4; counter > 0; counter--) {
    		Buffer[BufferIndexFreeze + counter - 1] = (Timestamp & 0x0F);
    		Timestamp >>= 4;
    		BufferIndex++;
    	}
    }


    // *** Convert to ASCII (2nd character to end) ****
    for (counter = 1; counter < BufferIndex; counter++) {
        if (Buffer[counter] < 0xA) {
            Buffer[counter] += 0x30;	// ASCII Digit
        } else {
            Buffer[counter] += 0x37; 	// ASCII Uppercase Character
        }
    }

    // *** Add Carriage Return (SCLAN EOL) ***
    Buffer[BufferIndex++] = '\r';

    // *** Put length of string in Len ***
    *Len = BufferIndex;

    return SLCAN_OK;
}


SLCAN_StatusTypeDef SLCAN_Parse_Str(uint8_t *Buffer, uint32_t Len, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]) {
    uint8_t BufferIndex = 0;

	uint8_t counter;

    // Convert from ASCII (2nd character to end)
    for (BufferIndex = 1; BufferIndex < Len - 1; BufferIndex++) {
        if (Buffer[BufferIndex] >= 'a'){        					// Lowercase letters (Should only be Uppercase...)
            Buffer[BufferIndex] = Buffer[BufferIndex] - 'a' + 10;
        }
        else if (Buffer[BufferIndex] >= 'A') {						// Uppercase letters
        	Buffer[BufferIndex] = Buffer[BufferIndex] - 'A' + 10;
        }
        else {														// Numbers
            Buffer[BufferIndex] = Buffer[BufferIndex] - '0';
        }
    }

    if (Buffer[0] == 'O' || Buffer[0] == 'o') {							// *** Open Channel *** --> "O[CR]"
    	// Ensure Correct Command Length
    	if(Len != 2) return SLCAN_ERROR;
        return SLCAN_OK; //No me interesa porque interactua con la configuracion de mi periferico, hago bypass
    }

    else if (Buffer[0] == 'C' || Buffer[0] == 'c') {					// *** Close Channel *** --> "C[CR]"
    	// Ensure Correct Command Length
    	if (Len != 2) return SLCAN_ERROR;
        return SLCAN_OK; //No me interesa porque interactua con la configuracion de mi periferico, hago bypass
    }

    if (Buffer[0] == 'Z' || Buffer[0] == 'z') {							// *** Set Timestamp *** --> "Zx[CR]"
    	// Ensure Correct Command Length
    	if (Len != 2 && Len != 3) return SLCAN_ERROR;

    	if (Len == 2){
        	SetTimestamp ^= 1U;
    	}

    	else {
    		SetTimestamp = Buffer[1] ? 1U : 0U;
    	}
        return SLCAN_OK;
    }

    else if (Buffer[0] == 'S' || Buffer[0] == 's') {					// *** Set Speed *** --> "Sx[CR]"
    	// Ensure Correct Command Length
    	if (Len != 3) return SLCAN_ERROR;
    	return SLCAN_OK;  //No me interesa porque interactua con la configuracion de mi periferico, hago bypass
    }

    else if (Buffer[0] == 'L' || Buffer[0] == 'l') {					// *** Set Mode *** --> "Lx[CR]" or "L[CR]"
    	// Ensure Correct Command Length
    	if (Len != 3 && Len != 2) return SLCAN_ERROR;
        return SLCAN_OK;  //No me interesa porque interactua con la configuracion de mi periferico, hago bypass
    }

    else if (Buffer[0] == 'M' &&
    		 Buffer[1] != ('M' - 'A' + 10) &&
			 Buffer[1] != ('m' - 'a' + 10))
    {																	// *** Set Filter *** --> "M00000000[CR] or M000[CR]"
    	// Ensure Correct Command Length								  (AKA "Acceptance 'M'ask")
    	if(Len != 10 && Len != 5) return SLCAN_ERROR;
    	return SLCAN_OK;  //No me interesa porque interactua con la configuracion de mi periferico, hago bypass
    }

    else if (Buffer[0] ==  'm'              &&
    		 Buffer[1] != ('M' - 'A' + 10)  &&
			 Buffer[1] != ('m' - 'a' + 10))
    {																	// *** Set Filter Mask *** --> "m00000000[CR]" or "m000[CR]"
    	// Ensure Correct Command Length
    	if(Len != 10 && Len != 5) return SLCAN_ERROR;
    	return SLCAN_OK; //No me interesa porque interactua con la configuracion de mi periferico, hago bypass
    }

    else if ((Buffer[0] ==  'M'        || Buffer[0] ==  'm'       ) &&
    		 (Buffer[1] == ('M' - 'A' + 10) || Buffer[1] == ('m' - 'a' + 10))
			 )
    {																	// *** Return Filter+Mask (as ASCII String) *** --> "MM[CR]"
    	// Ensure Correct Command Length								// Returns: "AxxxxxxxxKyyyyyyyy\r\0"
    	if(Len != 3) return SLCAN_ERROR;
    	return SLCAN_OK; //No me interesa porque interactua con la configuracion de mi periferico, hago bypass
    }

    else if (Buffer[0] == 'F' || Buffer[0] == 'f') {					// *** Read Status Flag (as ASCII String) *** --> "F\r"
    	// Ensure Correct Command Length								// Returns: "Fxxxxxxxx\r\0"
    	if (Len != 2 && Len != 3) return SLCAN_ERROR;

        uint8_t BufferIndexFreeze;
        uint32_t tmpCode = hcan1.ErrorCode;

    	BufferIndex = 0;
    	StatusBuf[BufferIndex++] = 'F';

    	// Add HEX Digits to Buffer, Right to Left, one Nybble at a time
    	BufferIndexFreeze = BufferIndex;
    	for (counter = 8; counter > 0; counter--) {
    		StatusBuf[BufferIndexFreeze + counter - 1] = (tmpCode & 0x0F);
    		tmpCode >>= 4;
    		BufferIndex++;
    	}

        // Convert to ASCII
        for (counter = BufferIndexFreeze; counter < BufferIndex; counter++) {
            if (StatusBuf[counter] < 0xA) {
                StatusBuf[counter] += 0x30;		// ASCII Digit
            } else {
                StatusBuf[counter] += 0x37; 	// ASCII Uppercase Character
            }
        }

    	StatusBuf[BufferIndex++] = '\r';
    	StatusBuf[BufferIndex++] = '\0';

    	if (CDC_Transmit_FS(StatusBuf, BufferIndex) != USBD_OK) {
    		return SLCAN_ERROR;
    	}
    	return SLCAN_OK;
    }

    else if (Buffer[0] == 't' || Buffer[0] == 'T') {	// *** Transmit *** --> "T1122334481122334455667788ADBE[CR]"
    	canMsg_t msg;
        pHeader->RTR = CAN_RTR_DATA;
    }

    else if (Buffer[0] == 'r' || Buffer[0] == 'R') {						// *** Transmit (RTR) ***
        pHeader->RTR = CAN_RTR_REMOTE;
    }

    else {																	// Command Not Found
        return SLCAN_ERROR;
    }

    // Putting Requested Transmission Data into the supplied header and data buffer.
    // (Should I just call HAL CAN Tx Function from here?)
    // Standard ID
    if (Buffer[0] == 't' || Buffer[0] == 'r') {
        pHeader->IDE = CAN_ID_STD;
    }

    // Extended ID
    else if (Buffer[0] == 'T' || Buffer[0] == 'R') {
        pHeader->IDE = CAN_ID_EXT;
    }

    else {
        // How did we get here?
        return SLCAN_ERROR;
    }

    // Reset ID in ExtID and StdId
    // Assign ID value to appropriate struct member
    BufferIndex = 1;
    pHeader->ExtId = 0;
    pHeader->StdId = 0;
    if (pHeader->IDE == CAN_ID_EXT) {
    	// Extended IDs take up 8 HEX digits
        for (counter = 0; counter < SLCAN_EXT_ID_LEN; counter++) {
        	pHeader->ExtId *= 16;
        	pHeader->ExtId += Buffer[counter + 1];
        	BufferIndex++;
        }
    }
    else {
    	// Standard IDs take up 3 HEX digits
        for (counter = 0; counter < SLCAN_STD_ID_LEN; counter++) {
        	pHeader->StdId *= 16;
        	pHeader->StdId += Buffer[counter + 1];
        	BufferIndex++;
        }
    }


    pHeader->DLC = Buffer[BufferIndex++];
    if (pHeader->DLC < 0 || pHeader->DLC > 8) {
        return SLCAN_ERROR;
    }

    // TODO Add a check for non-ASCII Characters (if (Buffer[BufferIndex] < 0 || Buffer[BufferIndex] > 15) ...)
    // TODO Add a check for if there are enough characters available (if (Len - BufferIndex - 1 (\r) ) < (DLC * 2) - 1) ...

    for (counter = 0; counter < pHeader->DLC; counter++) {
        aData[counter] = (Buffer[BufferIndex] << 4) + (0x0F & Buffer[BufferIndex + 1]);
        BufferIndex += 2;
    }
    return SLCAN_OK;
}


*/



