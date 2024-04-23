/*
 * TeR_COMMAND.h
 *
 *  Created on: Apr 19, 2024
 *      Author: ozuba
 */
//Modulo que gestiona los comandos del vehiculo, contiene la función command
#include "TeR_CAN.h" //Necesario para la interacción expone los can en uso
#include "TeR_TRQMANAGER.h" //Para configurar el torqueManager
#include "TeR_STATEMACHINE.h" //Para los estados




uint8_t command(struct ter_command_t command); //Función comando
