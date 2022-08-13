/***********************************************************
文件名：SentryRC_Task.h
描述：
		FreeRTOS-哨兵通信任务
From:Qi-Q@Rjgawuie
***********************************************************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

void SentryRC_Task_Setup(void);
void SentryRC_Task_Loop(void);
