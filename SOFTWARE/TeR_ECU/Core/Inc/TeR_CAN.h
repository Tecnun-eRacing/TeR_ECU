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
#include "stm32f4xx_hal.h"
//DBCS
#include "ter.h"
#include "inverter.h"
#include "hvbms.h"
//UTILIDADES
#include "TeR_SCS.h" //para el logging de scs
#include "TeR_COMMAND.h"//Para las llamadas de comando
/* --------------------- Estructuras de datos del coche ----------------- */
//TER.dbc
struct TeR_t {
//Propias
	struct ter_ter_status_t status;
	struct ter_dynamic_config_t dynamicConfig;
	struct ter_wheel_info_t wheelInfo;
	struct ter_inverter_info_t invInfo;
	//Externas
	//TER.dbc
	struct ter_apps_t apps; //Sensor de acelerador
	struct ter_bpps_t bpps; //Freno
	struct ter_steer_t steer; //Volante
	struct ter_front_v_t speed; // FrontAxle Speed
	struct ter_ang_rate_t angRate; //Angular rate from imu
	struct ter_lv_status_t lvbms; //Sensor de acelerador

	//Inverters.dbc
	//Enviados
	struct inverter_emcu_setpoint_1_left_t appReqLeft; //Comanda estado inverter
	struct inverter_emcu_setpoint_1_right_t appReqRight; //Comanda esatdo inverter

	struct inverter_emcu_setpoint_2_right_t currentReqRight; //Pedido comanda Corriente
	struct inverter_emcu_setpoint_2_left_t currentReqLeft; //Pedido comanda Corriente

	struct inverter_emcu_setpoint_3_right_t trqReqRight; //Pedido comanda Torque
	struct inverter_emcu_setpoint_3_left_t trqReqLeft; //Pedido comanda Torque

	//Received
	struct inverter_emcu_state_2_right_t appStateRight; //Estado inverter
	struct inverter_emcu_state_2_left_t appStateLeft; //Estado inverter

	struct inverter_emcu_state_3_right_t dqErpmRight; //Corriente D,Q y erpm
	struct inverter_emcu_state_3_left_t dqErpmLeft; //Corriente D,Q y erpm

	struct inverter_emcu_state_4_right_t tempsRight; //Temperaturas inverter
	struct inverter_emcu_state_4_left_t tempsLeft; //Temperaturas inverter


	struct inverter_emcu_state_9_right_t trqEstRight; //estimacion de torque producido
	struct inverter_emcu_state_9_left_t trqEstLeft; ////estimacion de torque producido

	struct inverter_emcu_state_7_right_t demRight; //Dem
	struct inverter_emcu_state_7_left_t demLeft;



	//HVBMS.dbc
	//Enviados
	struct hvbms_bms_rx_ctrl_1_t BmsAppReq; //Comanda estado BMS
	//Recibidos
	struct hvbms_bms_tx_state_3_t BmsAppState; //Estado BMS

};
//Struct General de Trabajo
extern struct TeR_t TeR; //Expone los datos del TeR a otros archivos
//Permite a otros modulos acceder a los CAN
extern CAN_HandleTypeDef *mainCAN;
extern CAN_HandleTypeDef *invCAN;

/* ---------------------------------------------------------------------- */

uint8_t initCAN(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan,
		TIM_HandleTypeDef *hInvTIM, TIM_HandleTypeDef *hMainTIM);
void configFilter(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan); //Configs filters
void decodeMsg(CAN_HandleTypeDef *hcan); //Decodes message according to DBC
void sendInvCAN(TIM_HandleTypeDef *htim); //Función Callback de envío del CAN de inverters
void sendMainCAN(TIM_HandleTypeDef *htim); // //Función Callback de envío del CAN principal

#endif /* INC_TER_CAN_H_ */
