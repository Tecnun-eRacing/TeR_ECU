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
#include "booter.h"
//UTILIDADES
#include "TeR_SCS.h" //para el logging de scs
#include "TeR_COMMAND.h"//Para las llamadas de comando
#include "cmsis_os.h" //funciones del Kernel
#include "TeR_CONFIG.h"
/* --------------------- Estructuras de datos del coche ----------------- */
//TER.dbc

typedef struct  {
	uint32_t id;
	uint8_t DLC;
	uint8_t data[8];
}canMsg_t;



struct TeR_t {
//Propias
	struct ter_ter_status_t status;
	struct ter_wheel_info_t wheelInfo;
	struct ter_inverter_info_t invInfo;
	struct ter_ecu_config_t config; //El mensaje con todas las configuraciones

	//IMU related
	struct ter_ang_rate_t angRate; //Angular rate(speed) from imu gyroscope
	struct ter_accel_t accel; //Acceleration from IMU acceleropmeter
	struct ter_gps_lat_long_t latlong; //Position from GPS
	struct ter_ypr_t ypr; //Yaw pitch and roll from sensor fusion
	struct ter_vel_body_t velbody; //Velbody over NED frame from gps

	//Externas
	//TER.dbc
	struct ter_apps_t apps; //Sensor de acelerador
	struct ter_bpps_t bpps; //Freno
	struct ter_steer_t steer; //Volante
	struct ter_front_v_t speed; // FrontAxle Speed
	struct ter_lv_status_t lvbms; //Estado del BMS de baja
	struct ter_btn_t buttons; // botones del volante

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
	struct hvbms_bms_tx_state_6_t BmsCellsVolt; // tensiones media min y max de celdas
	struct hvbms_bms_tx_state_9_t BmsCellsTemp; // temperaturas media min y max de celdas
	struct hvbms_bms_tx_state_4_t BmsCurrent; // corriente del bms

};

//Struct General de Trabajo
extern struct TeR_t TeR; //Expone los datos del TeR a otros archivos


//Permite a otros modulos acceder a los CAN
extern CAN_HandleTypeDef *mainCAN;
extern CAN_HandleTypeDef *invCAN;

/* ---------------------------------------------------------------------- */

uint8_t initCAN(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan);
void configFilter(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan); //Configs filters
void canRx(void *argument); //Decodes message according to DBC
void mainCanTx(void *argument); //main can sender task
void invCanTx(void *argument); // inv can sender task
void sendInvCAN(); //Función Callback de envío del CAN de inverters
void sendMainCAN(); // //Función Callback de envío del CAN principal
void canRxCallback(CAN_HandleTypeDef *hcan);
#endif /* INC_TER_CAN_H_ */
