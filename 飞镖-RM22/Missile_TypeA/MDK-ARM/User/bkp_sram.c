/*********************************************************************************
FILE NAME:bkp_sram.c

STM32������SRAM		���HAL��ʹ��
									RTC��Դ�·���ʧRAM
									
									���ó�ʼ������ܶ�д���ݣ�����
				
				From:@rjgawuie
*********************************************************************************/
#include "bkp_sram.h"

/** ��ʹ��HAL�⣩����SRAM��ʼ��,��ʼ���󷽿�д��
 * 
 * @param[in]   NULL
 * @retval      Null
**/
void BKP_SRAM_Init(void)
{
	/* ��Դ�ӿ�ʱ��ʹ�� (Power interface clock enable) */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* DBP λ�� 1��ʹ�ܶԱ�����ķ��� */
	HAL_PWR_EnableBkUpAccess();

	/* ͨ���� RCC AHB1 ����ʱ��ʹ�ܼĴ��� (RCC_AHB1ENR) �е� BKPSRAMEN λ�� 1�� ʹ�ܱ��� SRAM ʱ�� */
	__HAL_RCC_BKPSRAM_CLK_ENABLE();

	/* Ӧ�ó������ȴ����ݵ�ѹ��������־ (BRR) �� 1��ָʾ�ڴ���ģʽ�� VBAT ģʽ�»ᱣ��д�� RAM �е����ݡ� */
	HAL_PWREx_EnableBkUpReg();
}
