/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "chassis_task.h"
#include "display.h"
#include "usart.h"
#include "Stepper.h"
#include "command.h"
#include "rctask.h"
#include "referee_usart_task.h"
#include "SentryRC_Task.h"
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

/* USER CODE END Variables */
/* Definitions for LCD_Display_Tas */
osThreadId_t LCD_Display_TasHandle;
const osThreadAttr_t LCD_Display_Tas_attributes = {
  .name = "LCD_Display_Tas",
  .stack_size = 5120 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 5120 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Command_Task */
osThreadId_t Command_TaskHandle;
const osThreadAttr_t Command_Task_attributes = {
  .name = "Command_Task",
  .stack_size = 5120 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RC_Task */
osThreadId_t RC_TaskHandle;
const osThreadAttr_t RC_Task_attributes = {
  .name = "RC_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for referee_Task */
osThreadId_t referee_TaskHandle;
const osThreadAttr_t referee_Task_attributes = {
  .name = "referee_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CentryRC_Task */
osThreadId_t CentryRC_TaskHandle;
const osThreadAttr_t CentryRC_Task_attributes = {
  .name = "CentryRC_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LCD_Display_Task_Start(void *argument);
void Chassis_Task_Start(void *argument);
void Command_Task_Start(void *argument);
void RC_Task_Start(void *argument);
void referee_Task_Start(void *argument);
void CentryRC_Task_Start(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LCD_Display_Tas */
  LCD_Display_TasHandle = osThreadNew(LCD_Display_Task_Start, NULL, &LCD_Display_Tas_attributes);

  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(Chassis_Task_Start, NULL, &Chassis_Task_attributes);

  /* creation of Command_Task */
  Command_TaskHandle = osThreadNew(Command_Task_Start, NULL, &Command_Task_attributes);

  /* creation of RC_Task */
  RC_TaskHandle = osThreadNew(RC_Task_Start, NULL, &RC_Task_attributes);

  /* creation of referee_Task */
  referee_TaskHandle = osThreadNew(referee_Task_Start, NULL, &referee_Task_attributes);

  /* creation of CentryRC_Task */
  CentryRC_TaskHandle = osThreadNew(CentryRC_Task_Start, NULL, &CentryRC_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LCD_Display_Task_Start */
/**
  * @brief  Function implementing the LCD_Display_Tas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LCD_Display_Task_Start */
void LCD_Display_Task_Start(void *argument)
{
  /* USER CODE BEGIN LCD_Display_Task_Start */
	Display_Task_Setup();
  /* Infinite loop */
  for(;;)
  {
    Display_Task_Loop();
  }
  /* USER CODE END LCD_Display_Task_Start */
}

/* USER CODE BEGIN Header_Chassis_Task_Start */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task_Start */
void Chassis_Task_Start(void *argument)
{
  /* USER CODE BEGIN Chassis_Task_Start */
	Chassis_Task_Setup();
  /* Infinite loop */
  for(;;)
  {
    Chassis_Task_Loop();
  }
  /* USER CODE END Chassis_Task_Start */
}

/* USER CODE BEGIN Header_Command_Task_Start */
/**
* @brief Function implementing the Command_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Command_Task_Start */
void Command_Task_Start(void *argument)
{
  /* USER CODE BEGIN Command_Task_Start */
	Command_Setup();
  /* Infinite loop */
  for(;;)
  {
    Command_Loop();
  }
  /* USER CODE END Command_Task_Start */
}

/* USER CODE BEGIN Header_RC_Task_Start */
/**
* @brief Function implementing the RC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_Task_Start */
void RC_Task_Start(void *argument)
{
  /* USER CODE BEGIN RC_Task_Start */
	RC_Task_Setup();
  /* Infinite loop */
  for(;;)
  {
		RC_Task_Loop();
  }
  /* USER CODE END RC_Task_Start */
}

/* USER CODE BEGIN Header_referee_Task_Start */
/**
* @brief Function implementing the referee_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_referee_Task_Start */
void referee_Task_Start(void *argument)
{
  /* USER CODE BEGIN referee_Task_Start */
  /* Infinite loop */
  referee_usart_task();
  /* USER CODE END referee_Task_Start */
}

/* USER CODE BEGIN Header_CentryRC_Task_Start */
/**
* @brief Function implementing the CentryRC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CentryRC_Task_Start */
void CentryRC_Task_Start(void *argument)
{
  /* USER CODE BEGIN CentryRC_Task_Start */
	SentryRC_Task_Setup();
  /* Infinite loop */
  for(;;)
  {
    SentryRC_Task_Loop();
  }
  /* USER CODE END CentryRC_Task_Start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
