/*
 * TeR_CAN.c
 *
 *  Created on: Feb 2, 2024
 *      Author: Ozuba
 *
████████╗███████╗██████╗          ██████╗ █████╗ ███╗   ██╗
╚══██╔══╝██╔════╝██╔══██╗        ██╔════╝██╔══██╗████╗  ██║
   ██║   █████╗  ██████╔╝        ██║     ███████║██╔██╗ ██║
   ██║   ██╔══╝  ██╔══██╗        ██║     ██╔══██║██║╚██╗██║
   ██║   ███████╗██║  ██║███████╗╚██████╗██║  ██║██║ ╚████║
   ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═══╝
*/

/*
 *  Este Fichero tiene como Objetivo almacenar las funciones de decodificación
 *  y envío de todos los mensajes de una placa, incluye como librerías aquellas
 *  autogeneradas mediante cantools y ofrece una interfáz de cara al micro con dos
 *  Funciones:
 *  - decodeMSG -> Decodifica las estructuras pertinentes
 *  - sendCAN -> Envía los mensajes pertinentes (Esto no va a depender del estado, ya que los inverters siempre estarán a 0)
 *	- cmd() -> Función que se llama cuando se recibe el mensaje de comando para que cada placa lo interprete como corresponde
 *  A su vez están creados aqui todas las estructuras de memoria del can
 *
 */

#include "TeR_CAN.h"
/* ----------------------- Estructuras del HAL_CAN ------------------------- */
//Datos transmision
CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
uint8_t TxData[8]; //Header de recepción
uint32_t TxMailbox1; //Mailbox para el CAN1
uint32_t TxMailbox2; //Mailbox para el CAN2

//Datos recepcion
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];


/* --------------------- Estructuras de datos del coche ----------------- */
//TER.dbc
struct ter_apps_t apps; //Sensor de acelerador
struct ter_steer_t steer; //Volante
struct ter_front_v_t speed;
struct ter_ang_rate_t angRate;

//Inverters.dbc
struct inverter_emcu_setpoint_3_t trqReqRight;
struct inverter_emcu_setpoint_3_t trqReqLeft;


/* ---------------------------------------------------------------------- */


//Función de decodificación del CAN, si quieres que la ecu disponga de una señal hay que añadirla aquí.
uint8_t decodeMsg(uint32_t canId, uint8_t *data) {

	switch (canId) {
	//Attend the command
	case TER_CMD_FRAME_ID:
		command(data[0],&data[1]); //Llama a la interpretación del comando
		break;

	//Mesage Decoding
	case TER_APPS_FRAME_ID:
		ter_apps_unpack(&apps, data, TER_APPS_LENGTH);
		break;

	case TER_STEER_FRAME_ID:
		ter_steer_unpack(&steer, data, TER_STEER_LENGTH);
		break;

	case TER_FRONT_V_FRAME_ID:
		ter_front_v_unpack(&speed, data, TER_FRONT_V_LENGTH);
		break;

	case TER_ANG_RATE_FRAME_ID:
		ter_ang_rate_unpack(&angRate, data, TER_ANG_RATE_LENGTH);
		break;

	default:
		return -1;
		break;

	}
	return 1;
}


//Función de envío de mensajes
uint8_t sendCan(void){


return 1;
}
//Implementa aqui los comandos que se han de ejecutar
uint8_t command(uint8_t cmd, uint8_t* args){
	switch(cmd){}



return 1;
}
