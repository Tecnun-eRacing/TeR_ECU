/*
 * torqueManager.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Ozuba
 *
 *
 *  Este archivo implementa el gestor de comanda para el TeR, la gestión de comanda se basa
 *  en un limitador de potencia global y distintos modos de conducción:
 *  - Lineal
 *  - Torque Vectoring
 *  - Control de Tracción (Acceleration)
 *  - Tesis de Andoni medina
 *
 *  El esquema global del gestor de torque tiene la siguiente forma
 *
 *  Inputs ->Modo
 */


