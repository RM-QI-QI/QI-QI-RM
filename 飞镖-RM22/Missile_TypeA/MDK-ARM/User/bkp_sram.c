/*********************************************************************************
FILE NAME:bkp_sram.c

STM32备份域SRAM		配合HAL库使用
									RTC电源下非易失RAM
									
									调用初始化后才能读写数据！！！
				
				From:@rjgawuie
*********************************************************************************/
#include "bkp_sram.h"

/** （使用HAL库）备份SRAM初始化,初始化后方可写入
 * 
 * @param[in]   NULL
 * @retval      Null
**/
void BKP_SRAM_Init(void)
{
	/* 电源接口时钟使能 (Power interface clock enable) */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* DBP 位置 1，使能对备份域的访问 */
	HAL_PWR_EnableBkUpAccess();

	/* 通过将 RCC AHB1 外设时钟使能寄存器 (RCC_AHB1ENR) 中的 BKPSRAMEN 位置 1， 使能备份 SRAM 时钟 */
	__HAL_RCC_BKPSRAM_CLK_ENABLE();

	/* 应用程序必须等待备份调压器就绪标志 (BRR) 置 1，指示在待机模式和 VBAT 模式下会保持写入 RAM 中的数据。 */
	HAL_PWREx_EnableBkUpReg();
}
