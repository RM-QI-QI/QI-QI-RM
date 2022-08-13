/***********************************************************
�ļ�����display.h
������
		FreeRTOS-LCD��ʾ����ʾ����
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

