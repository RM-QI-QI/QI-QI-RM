/***********************************************************
�ļ�����BL_Service.h
������
		bootloader ����ʵ��
From:Qi-Q@Rjgawuie
***********************************************************/
#ifndef __BL_SERVICE_H
#define __BL_SERVICE_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "CircleBuf.h"

#define Sector_Size 0x4000  //������С(��С������С)
#define Circle_Buf_Size 512  //���λ�������С

#define USED_UART_HANDLER UART8 //ʹ���ĸ�����
#define USED_UART_IRQ UART8_IRQn  //�����ж�

#define Version 03 //�汾��

#define FL_ADDR_START 0x08004000  //FLASH��д������ַ��Χ
#define FL_ADDR_END 0x081FFFFF

#define APP_ADDRESS 0x08004000  //APP���ڵ�ַ��BL�˳�����ת

typedef  void (*pFunction)(void);
extern Circle_Buf_Typedef cb;

void Data_recv(void);
void TransferControl(void);
void IRQ_Rec(void);

#endif
