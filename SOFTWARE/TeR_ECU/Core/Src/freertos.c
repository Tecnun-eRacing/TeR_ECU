/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TeR_CAN.h"
#include "TeR_STATEMACHINE.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim13;
extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END Variables */
/* Definitions for osRunningTask */
osThreadId_t osRunningTaskHandle;
const osThreadAttr_t osRunningTask_attributes = {
  .name = "osRunningTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxTask */
osThreadId_t canRxTaskHandle;
const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for mainCanTxTask */
osThreadId_t mainCanTxTaskHandle;
const osThreadAttr_t mainCanTxTask_attributes = {
  .name = "mainCanTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for invCanTxTask */
osThreadId_t invCanTxTaskHandle;
const osThreadAttr_t invCanTxTask_attributes = {
  .name = "invCanTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for systemCriticalTask */
osThreadId_t systemCriticalTaskHandle;
const osThreadAttr_t systemCriticalTask_attributes = {
  .name = "systemCriticalTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for inertialTask */
osThreadId_t inertialTaskHandle;
const osThreadAttr_t inertialTask_attributes = {
  .name = "inertialTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledsTask */
osThreadId_t ledsTaskHandle;
const osThreadAttr_t ledsTask_attributes = {
  .name = "ledsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for trqManagerTask */
osThreadId_t trqManagerTaskHandle;
const osThreadAttr_t trqManagerTask_attributes = {
  .name = "trqManagerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for stateMachineTask */
osThreadId_t stateMachineTaskHandle;
const osThreadAttr_t stateMachineTask_attributes = {
  .name = "stateMachineTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for servosTask */
osThreadId_t servosTaskHandle;
const osThreadAttr_t servosTask_attributes = {
  .name = "servosTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for rxMsg */
osMessageQueueId_t rxMsgHandle;
const osMessageQueueAttr_t rxMsg_attributes = {
  .name = "rxMsg"
};
/* Definitions for scsTimer */
osTimerId_t scsTimerHandle;
const osTimerAttr_t scsTimer_attributes = {
  .name = "scsTimer"
};
/* Definitions for preventRace */
osMutexId_t preventRaceHandle;
const osMutexAttr_t preventRace_attributes = {
  .name = "preventRace"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void osRunning(void *argument);
extern void canRx(void *argument);
extern void mainCanTx(void *argument);
extern void invCanTx(void *argument);
extern void systemCritical(void *argument);
extern void inertial(void *argument);
extern void leds(void *argument);
extern void gps(void *argument);
extern void trqManager(void *argument);
extern void stateMachine(void *argument);
extern void servos(void *argument);
extern void scsCallback(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {
	HAL_TIM_Base_Start_IT(&htim13);
}
extern volatile unsigned long ulHighFrequencyTimerTicks;
__weak unsigned long getRunTimeCounterValue(void) {
	return ulHighFrequencyTimerTicks;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
	 task. It is essential that code added to this hook function never attempts
	 to block in any way (for example, call xQueueReceive() with a block time
	 specified, or call vTaskDelay()). If the application makes use of the
	 vTaskDelete() API function (as this demo application does) then it is also
	 important that vApplicationIdleHook() is permitted to return to its calling
	 function, because it is the responsibility of the idle task to clean up
	 memory allocated by the kernel to any task that has since been deleted. */
	HAL_IWDG_Refresh(&hiwdg);
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of preventRace */
  preventRaceHandle = osMutexNew(&preventRace_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of scsTimer */
  scsTimerHandle = osTimerNew(scsCallback, osTimerPeriodic, NULL, &scsTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rxMsg */
  rxMsgHandle = osMessageQueueNew (128, sizeof(canMsg_t), &rxMsg_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of osRunningTask */
  osRunningTaskHandle = osThreadNew(osRunning, NULL, &osRunningTask_attributes);

  /* creation of canRxTask */
  canRxTaskHandle = osThreadNew(canRx, NULL, &canRxTask_attributes);

  /* creation of mainCanTxTask */
  mainCanTxTaskHandle = osThreadNew(mainCanTx, NULL, &mainCanTxTask_attributes);

  /* creation of invCanTxTask */
  invCanTxTaskHandle = osThreadNew(invCanTx, NULL, &invCanTxTask_attributes);

  /* creation of systemCriticalTask */
  systemCriticalTaskHandle = osThreadNew(systemCritical, NULL, &systemCriticalTask_attributes);

  /* creation of inertialTask */
  inertialTaskHandle = osThreadNew(inertial, NULL, &inertialTask_attributes);

  /* creation of ledsTask */
  ledsTaskHandle = osThreadNew(leds, NULL, &ledsTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(gps, NULL, &gpsTask_attributes);

  /* creation of trqManagerTask */
  trqManagerTaskHandle = osThreadNew(trqManager, NULL, &trqManagerTask_attributes);

  /* creation of stateMachineTask */
  stateMachineTaskHandle = osThreadNew(stateMachine, NULL, &stateMachineTask_attributes);

  /* creation of servosTask */
  servosTaskHandle = osThreadNew(servos, NULL, &servosTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_osRunning */
/**
 * @brief  Function implementing the osRunningTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_osRunning */
void osRunning(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN osRunning */
	/* Infinite loop */
	for (;;) {
		osDelay(500);
		HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);// Toggle Alive indication
	}
  /* USER CODE END osRunning */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

