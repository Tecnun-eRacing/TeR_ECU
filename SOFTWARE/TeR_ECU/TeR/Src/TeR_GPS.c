/*
 * TeR_GPS.c
 *
 *  Created on: Jan 17, 2025
 *      Author: ozuba
 */

#include "TeR_GPS.h"
static const uint8_t configUBX[]={0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x9A,0x79};

static const uint8_t getPVTData[] = { 0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08,
		0x19 };

uint8_t buffer[256];


ubx_nav_pvt_t pvt;


void gps(void *argument) {


	ubx_device_t gps;
	gps.write = &gps_write;
	gps.read = &gps_read;

	//Prepare ubx to disable nmea
	ubx_cfg_prt cfg;
	checksum(&configUBX[2], sizeof(configUBX)-4); //Validate checksum usage
	memset(&cfg, 0, sizeof(cfg)); //set blank
	cfg.portID = 0x01; //Uart 1
	cfg.txReady = 0x00;
	cfg.mode = 0x000008D0; //No idea jajaj
	cfg.baudRate = 38400; //Current baud
	cfg.inProtoMask = 0b0000000000000001;//Activate just ubx
	cfg.outProtoMask = 0b0000000000000001;//Activate just ubx
	//Desactiva nmea
	send_ubx(&gps, 0x06, 0x00, &cfg, sizeof(cfg));
	osDelay(100);

	for (;;) {
		//GPS test
		osDelay(100);
		HAL_UART_Transmit(&huart1, getPVTData, sizeof(getPVTData), 100);
		HAL_UART_Receive_DMA(&huart1, buffer, sizeof(buffer));
		//read_ubx(&gps, &pvt, sizeof(pvt));

	}
}

uint8_t gps_read(uint8_t *dest, size_t size) {
	return HAL_UART_Receive(&huart1, dest, size,100);
}
uint8_t gps_write(uint8_t *src, size_t size) {
	return HAL_UART_Transmit(&huart1, src, size, 100); //deactivate nmea
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	CDC_Transmit_FS(buffer, sizeof(buffer));

	for(int i = 0; i < sizeof(buffer);i++){
		if(buffer[i] == 0xB5 && buffer[i+1] == 0x62 && i < sizeof(buffer)-sizeof(pvt)){
			memcpy(&pvt,&buffer[i+6],sizeof(pvt));
		}
	}
	HAL_UART_Receive_DMA(&huart1, buffer, sizeof(buffer));

}
