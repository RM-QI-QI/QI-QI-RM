/***********************************************************
文件名：BL_Service.h
描述：
		bootloader 功能实现
From:Qi-Q@Rjgawuie
***********************************************************/
#ifndef __BL_SERVICE_H
#define __BL_SERVICE_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "CircleBuf.h"

#define Sector_Size 0x4000  //扇区大小(最小扇区大小)
#define Circle_Buf_Size 512  //环形缓冲区大小

#define USED_UART_HANDLER UART8 //使用哪个串口
#define USED_UART_IRQ UART8_IRQn  //串口中断

#define Version 03 //版本号

#define FL_ADDR_START 0x08004000  //FLASH可写扇区地址范围
#define FL_ADDR_END 0x081FFFFF

#define APP_ADDRESS 0x08004000  //APP所在地址，BL退出后跳转

typedef  void (*pFunction)(void);
extern Circle_Buf_Typedef cb;

void Data_recv(void);
void TransferControl(void);
void IRQ_Rec(void);

#endif
