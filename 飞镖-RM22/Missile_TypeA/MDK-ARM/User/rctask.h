/***********************************************************
文件名：rctask.h
描述：
		FreeRTOS-遥控器任务，遥控器控制飞镖发射架整体运动映射
From:Qi-Q@Rjgawuie
***********************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

extern double accuracy;

void RC_Task_Setup(void);
void RC_Task_Loop(void);
