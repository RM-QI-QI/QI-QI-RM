/*********************************************************************************
FILE NAME:bkp_sram.h

STM32备份域SRAM		配合HAL库使用
									RTC电源下非易失RAM
									
									调用初始化后才能读写数据！！！
				
				From:@rjgawuie
*********************************************************************************/
#ifndef _BKP_SRAM_H
#define _BKP_SRAM_H

#include "main.h"

#define BKPSRAM(x) *(__IO uint8_t *)(BKPSRAM_BASE + x) //指向SRAM的指针

void BKP_SRAM_Init(void);

#endif
