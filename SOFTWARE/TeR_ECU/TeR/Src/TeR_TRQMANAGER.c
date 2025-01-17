/*
 * torqueManager.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Ozuba
 *
 */
/*
                                             STEER
                                             ────────────────────┐
                                             APPS                │
             KWLimit                         ───────────────┐    │
             ──────────────┐                 IMU            │    │                                                      ┌────────┐
                           │                 ──────────┐    │    │                                              ┌──────►│RIGHT   │
                           ▼                           ▼    ▼    ▼                                              │       │INVERTER│
                  ┌──────────────────┐            ┌──────────────────┐ TorqueR   ┌───────────────────┐ TorqueR  │       └────────┘
                  │                  │            │                  ├──────────►│                   ├──────────┘
Speed(Rpm)        │     Limitador    │  Torque    │     Modos de     │           │      Control      │
─────────────────►│        de        ├───────────►│                  │           │        de         │
                  │     Potencia     │ Available  │    Conducción    │ TorqueL   │     Traccion      │ TorqueL
                  │                  │            │                  ├──────────►│                   ├──────────┐
                  └──────────────────┘            └──────────────────┘           └───────────────────┘          │
                                                                                                                │       ┌────────┐
                                                                                                                └──────►│LEFT    │
                                                                                                                        │INVERTER│
                                                                                                                        └────────┘

 The torque manager is the library in charge of managing the dynamic control of the vehicle, it consists on 3 differenciated stages
 which are modular and interchangable with the idea of creating different driving experiences according to the competition. The principal
 interchangeable block is the driving mode


 - Limitador de Potencia: Se establece una limitación de potencia en kw y se calcula un torque máximo desarrollable suponiendo que la potencia
 	 	 	 	 	 se conserva a lo largo del powertrain Pelectrica = Pmecanica*FactorEficiencia

 - Modo de conducción:
 	 * Lineal: El torque se distribuye equitativamente a las 2 ruedas
 	 * BasicTorque: Distrubución del torque basada en una función del steering (Normalmente un polinomio)
 	 * ControlTorque: Distribución del Torque basada en un scheduled gain PID calibrado mediante un modelo bicicleta del vehículo
 	 * Modo Marcha atras: Self-Explainatory (Pitará en modo obra jajaj) Ilegalisimo en competi
	 * Autonomo(Futuro): Permite el control del TeR mediante la librería de comandos TeR_COMMAND
 - Control de tracción:
 	 * Feedforward
 	 * Feedback etc


Para permitir la modularidad se va a utilizar un ciclo de procesado basado en function pointers lo que permite escribir funciones en otros modulos facilmente
- Limitador de potencia void -> trq_t (La estimación es interna al modulo no entra como argumento(Puede hacer estimaciones basadas en el consumo electrico))
- Modo de conducción torque_t -> trqMap_t
- Control de Tracción torqueMap_t -> trqMap_t
 */
#include "TeR_TRQMANAGER.h"

trqPipeline_t DriveConfig; //Configuración en uso


uint8_t trqManager(void) { // Corre las etapas del pipeline y solicita la comanda
	//Pipeline
	trq_t availableTorque = DriveConfig.limiter(); //Limita
	trqMap_t trqDistribution = DriveConfig.drivingMode(availableTorque); //Distribuye
	trqMap_t trqToWheels = DriveConfig.tractionControl(trqDistribution); // Limita si se vé que la rueda no puede dar ese grip (Estrategia dependiente del controlador)

	//Solicitud de la comanda
	TeR.trqReqLeft.torque_nm_req = trqToWheels.rLeft;
	TeR.trqReqRight.torque_nm_req = trqToWheels.rRight;
	//Checks de seguridad y saturaciones: (Redundantes pero permiten dormir tranquilo)

	if (TeR.trqReqLeft.torque_nm_req < 0 || TeR.trqReqLeft.torque_nm_req < 0) {
		TeR.trqReqLeft.torque_nm_req = 0;
		TeR.trqReqRight.torque_nm_req = 0;
	}

	return 1;
}


//------------------------------------------------[Basic Power Limiters]------------------------------------------------//
// void -> trq_t
//Par Máximo constante
trq_t limitTorque(void) {
	return (trq_t) TeR.dynamicConfig.trq_limit; //Devuelve el valor configurado
}

//Potencia mecanica constante
trq_t limitMechPWR(void) {
	int32_t meanRPM = (TeR.wheelInfo.rl_rpm + TeR.wheelInfo.rl_rpm)
			/ 2; //RPMs medias
	if (meanRPM > 0) { //Estamos moviendonos se estima el torque desarrollable
		return (trq_t) (TeR.dynamicConfig.kw_limit * ELEC2MECH_EFF) / meanRPM; //Devolvemos la potencia desarrollable limitada en potencia
	} else if (meanRPM == 0) { //Estamos quietos luego se devuelve la limitación de torque máximo
		return (trq_t) TeR.dynamicConfig.trq_limit;
	}
	return 0;
}

//Velocidad máxima
trq_t limitSpeed(void) {
//Reduce el torque que puedes dar conforme te acercas al valor de speed (Puede generar problematica oscilatoria)
//Velocidad aumenta -> torque dismunuye -> Velocidad Disminuye-> Torque aumenta
//Usar PID

	return 0;
}

//Limit Electrical Power (Dato de los Inverters Controller)
trq_t limitElecPWR(void) {
	return 0;
}

//------------------------------------------------[Basic Driving Modes]------------------------------------------------//
// trq_t -> trqMap_t
trqMap_t lineal(trq_t limit) { //Entrega lineal de par a las 2 ruedas
	trqMap_t trqMap;
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5);
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit*0.5);
	return trqMap;
}

//------------------------------------------------[Basic traction Control]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t tractionControlOFF(trqMap_t in) {
return in;
}



