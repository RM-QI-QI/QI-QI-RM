/*********************************************************************************
FILE NAME:PVD.h

STM32掉电检测			配合HAL库使用
									在VCC电压低于阈值时执行中断
									*官方例程
									
									在CubeMX　NVIC中勾选PVD外设中断使能
				
				From:@rjgawuie
*********************************************************************************/
#ifndef _PVD_H
#define _PVD_H

#include "main.h"

void PVD_Config(uint32_t mode);

#endif
