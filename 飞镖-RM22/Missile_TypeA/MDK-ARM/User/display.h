/***********************************************************
文件名：display.h
描述：
		FreeRTOS-LCD显示屏显示任务
From:Qi-Q@Rjgawuie
***********************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define PageMax 3



void Display_Task_Setup(void);
void Display_Task_Loop(void);
void Next_Page(void);
void Last_Page(void);
void Page_Jump(int8_t pagenum);

