/*
 * TeR_CONFIG.h
 *
 *  Created on: Feb 12, 2025
 *      Author: Ozuba
 */

#ifndef INC_TER_CONFIG_H_
#define INC_TER_CONFIG_H_
/*
 * TeR_Config es el modulo encargado de la gestion de configs del TeR
 * Provee de un espacio de memoria virtual que puede ser escrito y leido mediante
 * las funciones write()  y read()
 *
 * Deberíamos hacer una libería que genere una interfáz tipo diccionario mediante la propia sintaxis del struct
 *
 */
typedef struct {


}ter_config_t;

//RAM
uint8_t cfg_write(void* src, uint16_t vAdress,size_t size); //Lee de la estructura
uint8_t cfg_read(void* dest,uint16_t vAdress,size_t size); //Escribe en la estructura

//STORE/RESTORE
uint8_t cfg_store(); //Vuelca el struct en la eeprom
uint8_t cfg_restore(); //Carga el estruct desde la eeprom

#endif /* INC_TER_CONFIG_H_ */
