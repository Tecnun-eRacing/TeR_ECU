/*
 * TeR_GPS.c
 *
 *  Created on: Jan 17, 2025
 *      Author: ozuba
 */

#include "TeR_GPS.h"


uint8_t uart_rx_buffer[128];

void gps(void *argument) {


	for(;;){
	    //GPS test

        if (HAL_UART_Receive(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer),100) == HAL_OK) {
            // Transmit the received data over USB CDC
            CDC_Transmit_FS(uart_rx_buffer, sizeof(uart_rx_buffer));
        }
	}
}
