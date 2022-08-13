/*********************************************************************************
FILE NAME:bkp_sram.h

STM32������SRAM		���HAL��ʹ��
									RTC��Դ�·���ʧRAM
									
									���ó�ʼ������ܶ�д���ݣ�����
				
				From:@rjgawuie
*********************************************************************************/
#ifndef _BKP_SRAM_H
#define _BKP_SRAM_H

#include "main.h"

#define BKPSRAM(x) *(__IO uint8_t *)(BKPSRAM_BASE + x) //ָ��SRAM��ָ��

void BKP_SRAM_Init(void);

#endif
