/***********************************************************
文件名：CircleBuf.h
描述：
		环形缓冲区实现
From:Qi-Q@Rjgawuie
***********************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


typedef struct
{
	uint16_t size;		//环形缓冲区大小
	uint16_t count;  	//缓冲区内有效字符计数
	char *buf;  	 		//缓冲区本体指针
	char *read;				//读指针
	char *write;			//写指针
}Circle_Buf_Typedef;

void CircleBuf_Init(Circle_Buf_Typedef *circlebuf,uint16_t bufsize);
void CircleBuf_DeInit(Circle_Buf_Typedef *circlebuf);
void CircleBuf_Write(Circle_Buf_Typedef *circlebuf,char cbwch);
char CircleBuf_Read(Circle_Buf_Typedef *circlebuf);
