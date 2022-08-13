/***********************************************************
�ļ�����CircleBuf.h
������
		���λ�����ʵ��
From:Qi-Q@Rjgawuie
***********************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


typedef struct
{
	uint16_t size;		//���λ�������С
	uint16_t count;  	//����������Ч�ַ�����
	char *buf;  	 		//����������ָ��
	char *read;				//��ָ��
	char *write;			//дָ��
}Circle_Buf_Typedef;

void CircleBuf_Init(Circle_Buf_Typedef *circlebuf,uint16_t bufsize);
void CircleBuf_DeInit(Circle_Buf_Typedef *circlebuf);
void CircleBuf_Write(Circle_Buf_Typedef *circlebuf,char cbwch);
char CircleBuf_Read(Circle_Buf_Typedef *circlebuf);
