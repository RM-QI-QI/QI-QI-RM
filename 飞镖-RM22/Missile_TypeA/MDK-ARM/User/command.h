/***********************************************************
文件名：command.h
描述：
		FreeRTOS-命令任务处理及脚本执行
From:Qi-Q@Rjgawuie
***********************************************************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "CircleBuf.h"

#define SD_NOT_Exist HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15)

extern Circle_Buf_Typedef uartbuf;

extern int SD_exist;

void Command_Setup(void);
void Command_Loop(void);

int Para_Save(void);
int Para_Load(void);
int Data_Keep(void);
int Data_Load(void);
int Run_recv(void);
uint16_t SD_Insert(void);
void TurnLock(void);
void TurnUnlock(void);
