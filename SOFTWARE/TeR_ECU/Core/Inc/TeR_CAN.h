/*
 * TeR_CAN.h
 *
 *  Created on: Feb 2, 2024
 *      Author: Ozuba
 *
 * ████████╗███████╗██████╗          ██████╗ █████╗ ███╗   ██╗
 * ╚══██╔══╝██╔════╝██╔══██╗        ██╔════╝██╔══██╗████╗  ██║
 *    ██║   █████╗  ██████╔╝        ██║     ███████║██╔██╗ ██║
 *    ██║   ██╔══╝  ██╔══██╗        ██║     ██╔══██║██║╚██╗██║
 *    ██║   ███████╗██║  ██║███████╗╚██████╗██║  ██║██║ ╚████║
 *    ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═══╝
 */


/*  Este Fichero tiene como Objetivo almacenar las funciones de decodificación
 *  y envío de todos los mensajes de una placa, incluye como librerías aquellas
 *  autogeneradas mediante cantools y ofrece una interfáz de cara al micro con dos
 *  Funciones:
 *  - decodeMSG -> Decodifica las estructuras pertinentes
 *  - sendCAN -> Envía los mensajes pertinentes (Esto no va a depender del estado, ya que los inverters siempre estarán a 0)
 *	- cmd() -> Función que se llama cuando se recibe el mensaje de comando para que cada placa lo interprete como corresponde
 *
 *  A su vez están creados aqui todas las estructuras de memoria del CAN que permiten su uso fuera de el
 */



#ifndef INC_TER_CAN_H_
#define INC_TER_CAN_H_
#include "ter.h"
#include "inverter.h"
#include "stm32f4xx_hal.h"
#include "can.h"
/* --------------------- Estructuras de datos del coche ----------------- */
//TER.dbc
struct TeR_t{
	//Propias
	struct ter_ecu_status_t status;
	//Ocultas
	uint32_t r2d;
	//Externas

	struct ter_apps_t apps; //Sensor de acelerador
	struct ter_steer_t steer; //Volante
	struct ter_front_v_t speed; // FrontAxle Speed
	struct ter_ang_rate_t angRate; //Angular rate from imu

//Inverters.dbc
	struct inverter_emcu_setpoint_3_t trqReqRight;
	struct inverter_emcu_setpoint_3_t trqReqLeft;
};
extern struct TeR_t TeR; //Expone los datos del TeR a otros archivos
/* ---------------------------------------------------------------------- */



uint8_t decodeMsg(uint32_t canId, uint8_t *data); //Decodes message according to DBC
uint8_t sendCAN(void);
uint8_t command(uint8_t cmd, uint8_t *args); //

#endif /* INC_TER_CAN_H_ */
