/*********************************************************************************
FILE NAME:PVD.c

STM32������			���HAL��ʹ��
									��VCC��ѹ������ֵʱִ���ж�
									*�ٷ�����
									
									��CubeMX��NVIC�й�ѡPVD�����ж�ʹ�ܣ�����
				
				From:@rjgawuie
*********************************************************************************/
#include "PVD.h"
#include "display.h"
#include "command.h"


/** PVD���ú�������Main�����е�������PVD
  * @brief  Configures the PVD resources.
  * @param  mode  PVD��ѹ��ֵѡ�񣬼��±�
  * @retval None
	PVD ����ѡ�� (PVD level selection)
	��Щλ�����д�룬����ѡ���ѹ��������ĵ�ѹ��ֵ
		PWR_PVDLEVEL_0��2.0 V
		PWR_PVDLEVEL_1��2.1 V
		PWR_PVDLEVEL_2��2.3 V
		PWR_PVDLEVEL_3��2.5 V
		PWR_PVDLEVEL_4��2.6 V
		PWR_PVDLEVEL_5��2.7 V
		PWR_PVDLEVEL_6��2.8 V
		PWR_PVDLEVEL_7��2.9 V
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
