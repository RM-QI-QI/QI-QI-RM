/*********************************************************************************
FILE NAME:PVD.c

STM32掉电检测			配合HAL库使用
									在VCC电压低于阈值时执行中断
									*官方例程
									
									在CubeMX　NVIC中勾选PVD外设中断使能！！！
				
				From:@rjgawuie
*********************************************************************************/
#include "PVD.h"
#include "display.h"
#include "command.h"


/** PVD配置函数，在Main函数中调用配置PVD
  * @brief  Configures the PVD resources.
  * @param  mode  PVD电压阈值选择，见下表
  * @retval None
	PVD 级别选择 (PVD level selection)
	这些位由软件写入，用于选择电压检测器检测的电压阈值
		PWR_PVDLEVEL_0：2.0 V
		PWR_PVDLEVEL_1：2.1 V
		PWR_PVDLEVEL_2：2.3 V
		PWR_PVDLEVEL_3：2.5 V
		PWR_PVDLEVEL_4：2.6 V
		PWR_PVDLEVEL_5：2.7 V
		PWR_PVDLEVEL_6：2.8 V
		PWR_PVDLEVEL_7：2.9 V
  */
void PVD_Config(uint32_t mode)
{
    /*##-1- Enable Power Clock #################################################*/
    __HAL_RCC_PWR_CLK_ENABLE();
 
    /*##-2- Configure the NVIC for PVD #########################################*/
    HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(PVD_IRQn);
 
    /* Configure the PVD Level to 3 and generate an interrupt on rising and falling
       edges(PVD detection level set to 2.5V, refer to the electrical characteristics
       of you device datasheet for more details) */
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = mode;
    sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING;
    HAL_PWR_ConfigPVD(&sConfigPVD);
 
    /* Enable the PVD Output */
    HAL_PWR_EnablePVD();
}
/** PVD (Programmable Votage Detector)
  * @brief  PWR PVD interrupt callback
  * @retval None
  */
void HAL_PWR_PVDCallback(void)
{
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PWR_PVDCallback could be implemented in the user file
     */
    while(Data_Keep());
}
