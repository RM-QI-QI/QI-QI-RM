/***********************************************************
�ļ�����rctask.h
������
		FreeRTOS-ң��������ң�������Ʒ��ڷ���������˶�ӳ��
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
