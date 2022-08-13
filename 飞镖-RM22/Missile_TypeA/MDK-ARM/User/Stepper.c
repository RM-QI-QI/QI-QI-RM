/***********************************************************
�ļ�����Stepper.c
������
		�����������ģ��
		���HAL��ʹ��
		
		
From:Qi-Q@Rjgawuie
***********************************************************/
#include "Stepper.h"

#include "remote_control.h"

#ifndef __STEPPER_C
#define __STEPPER_C

uint8_t TIMcount = 0;

STEPPER_Typedef stepper1;

/***********************************************************
��������Stepper_Init
���ܣ���ʼ���������
������Stepper		������������ַ
			Subdivision		�������ϸ��ֵ
			ENA_GPIO		ʹ����������GPIO
			ENA_Pin			ʹ������
			DIR_GPIO		������������GPIO
			DIR_Pin			��������
			PUL_GPIO		������������GPIO
			PUL_Pin			��������
***********************************************************/
uint16_t Stepper_Init(STEPPER_Typedef *Stepper,
									float STEP_Angle,
									uint8_t Subdivision,
									GPIO_TypeDef *ENA_GPIO,
									uint16_t ENA_Pin,
									GPIO_TypeDef *DIR_GPIO,
									uint16_t DIR_Pin,
									GPIO_TypeDef *PUL_GPIO,
									uint16_t PUL_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	Stepper->Remain_STEP = 0;
	Stepper->Remain_Angle = 0;
	Stepper->ENA = STEPPER_ENABLE;
	Stepper->STEP_Angle = STEP_Angle;
	Stepper->Subdivision = Subdivision;
	Stepper->ENA_GPIO = ENA_GPIO;
	Stepper->ENA_Pin = ENA_Pin;
	Stepper->DIR_GPIO = DIR_GPIO;
	Stepper->DIR_Pin = DIR_Pin;
	Stepper->PUL_GPIO = PUL_GPIO;
	Stepper->PUL_Pin = PUL_Pin;	
	
	switch((uint32_t)ENA_GPIO)      //ʹ��GPIOʹ��
	{
		case (uint32_t)GPIOA:	__HAL_RCC_GPIOA_CLK_ENABLE();break;
		case (uint32_t)GPIOB:	__HAL_RCC_GPIOB_CLK_ENABLE();break;
		case (uint32_t)GPIOC:	__HAL_RCC_GPIOC_CLK_ENABLE();break;
		case (uint32_t)GPIOD:	__HAL_RCC_GPIOD_CLK_ENABLE();break;
		case (uint32_t)GPIOE:	__HAL_RCC_GPIOE_CLK_ENABLE();break;
		case (uint32_t)GPIOF:	__HAL_RCC_GPIOF_CLK_ENABLE();break;
		case (uint32_t)GPIOH:	__HAL_RCC_GPIOH_CLK_ENABLE();break;
		case (uint32_t)GPIOI:	__HAL_RCC_GPIOI_CLK_ENABLE();break;
		default: return 1;
	}
	switch((uint32_t)DIR_GPIO)      //����GPIOʹ��
	{
		case (uint32_t)GPIOA:	__HAL_RCC_GPIOA_CLK_ENABLE();break;
		case (uint32_t)GPIOB:	__HAL_RCC_GPIOB_CLK_ENABLE();break;
		case (uint32_t)GPIOC:	__HAL_RCC_GPIOC_CLK_ENABLE();break;
		case (uint32_t)GPIOD:	__HAL_RCC_GPIOD_CLK_ENABLE();break;
		case (uint32_t)GPIOE:	__HAL_RCC_GPIOE_CLK_ENABLE();break;
		case (uint32_t)GPIOF:	__HAL_RCC_GPIOF_CLK_ENABLE();break;
		case (uint32_t)GPIOH:	__HAL_RCC_GPIOH_CLK_ENABLE();break;
		case (uint32_t)GPIOI:	__HAL_RCC_GPIOI_CLK_ENABLE();break;
		default: return 1;
	}
	switch((uint32_t)PUL_GPIO)      //����GPIOʹ��
	{
		case (uint32_t)GPIOA:	__HAL_RCC_GPIOA_CLK_ENABLE();break;
		case (uint32_t)GPIOB:	__HAL_RCC_GPIOB_CLK_ENABLE();break;
		case (uint32_t)GPIOC:	__HAL_RCC_GPIOC_CLK_ENABLE();break;
		case (uint32_t)GPIOD:	__HAL_RCC_GPIOD_CLK_ENABLE();break;
		case (uint32_t)GPIOE:	__HAL_RCC_GPIOE_CLK_ENABLE();break;
		case (uint32_t)GPIOF:	__HAL_RCC_GPIOF_CLK_ENABLE();break;
		case (uint32_t)GPIOH:	__HAL_RCC_GPIOH_CLK_ENABLE();break;
		case (uint32_t)GPIOI:	__HAL_RCC_GPIOI_CLK_ENABLE();break;
		default: return 1;
	}
	
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	
	HAL_GPIO_WritePin(Stepper->ENA_GPIO, Stepper->ENA_Pin, STEPPER_ENABLE);
	HAL_GPIO_WritePin(Stepper->DIR_GPIO, Stepper->DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Stepper->PUL_GPIO, Stepper->PUL_Pin, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = Stepper->ENA_Pin;
	HAL_GPIO_Init(Stepper->ENA_GPIO, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = Stepper->DIR_Pin;
	HAL_GPIO_Init(Stepper->DIR_GPIO, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = Stepper->PUL_Pin;
	HAL_GPIO_Init(Stepper->PUL_GPIO, &GPIO_InitStruct);
	
	Stepper->sum_Step = 0;
	
	return 0;
}
/***********************************************************
��������STEP_ENABLE
���ܣ�ʹ�ܵ��
������Stepper		������������ַ
***********************************************************/
void STEP_ENABLE(STEPPER_Typedef *Stepper)
{
	Stepper->ENA = STEPPER_ENABLE;
	HAL_GPIO_WritePin(Stepper->ENA_GPIO,Stepper->ENA_Pin,STEPPER_ENABLE);
}

/***********************************************************
��������STEP_DISABLE
���ܣ�ʹ�ܵ��
������Stepper		������������ַ
***********************************************************/
void STEP_DISABLE(STEPPER_Typedef *Stepper)
{
	Stepper->ENA = STEPPER_DISABLE;
	HAL_GPIO_WritePin(Stepper->ENA_GPIO,Stepper->ENA_Pin,STEPPER_DISABLE);
}

/***********************************************************
��������Move_Step
���ܣ����ִ����Ӧ����
������Stepper		������������ַ
			Step			���������Ҫִ�е���Ӧ����
***********************************************************/
void Move_Step(STEPPER_Typedef *Stepper,int Step)
{
	Stepper->Remain_STEP += Step;
}

/***********************************************************
��������Move_Step
���ܣ����ִ����Ӧ�Ƕ�
������Stepper		������������ַ
			Angle			���������Ҫת���ĽǶ�
***********************************************************/
void Move_Angle(STEPPER_Typedef *Stepper,float Angle)
{
	Stepper->Remain_Angle += Angle;
}

/***********************************************************
��������STEP_SetZero
���ܣ����ʣ�������㣨������㣩
������Stepper		������������ַ
***********************************************************/

void STEP_SetZero(STEPPER_Typedef *Stepper)
{
	Stepper->Remain_Angle = 0;
	Stepper->Remain_STEP = 0;
}

/***********************************************************
��������STEP_Remain
���ܣ�����ʣ�ಽ��
������Stepper		������������ַ
***********************************************************/

int STEP_Remain(STEPPER_Typedef *Stepper)
{
	int step = Stepper->Remain_STEP,angle = Stepper->Remain_Angle;
	float error = Stepper->STEP_Angle/Stepper->Subdivision;
	for(;angle>error;angle-=error)  step++;
	for(;angle<(-error);angle+=error)  step--;
	return step;
}

/***********************************************************
��������Angle_Remain
���ܣ�����ʣ��Ƕ�
������Stepper		������������ַ
***********************************************************/

float Angle_Remain(STEPPER_Typedef *Stepper)
{
	return (Stepper->Remain_Angle + Stepper->Remain_STEP * Stepper->STEP_Angle/Stepper->Subdivision);
}

/***********************************************************
��������STEP_MOVE_PROCESS
���ܣ��ж�Ҫ���������
������Stepper		������������ַ
***********************************************************/
uint16_t STEP_MOVE_PROCESS(STEPPER_Typedef *Stepper)
{
	if(TIMcount%2)HAL_GPIO_WritePin(Stepper->PUL_GPIO,Stepper->PUL_Pin,GPIO_PIN_SET);
	else{
		if(Stepper->ENA == STEPPER_ENABLE)
		{
			if(Stepper->Remain_STEP > 0)
			{
				HAL_GPIO_WritePin(Stepper->DIR_GPIO,Stepper->DIR_Pin,STEPPER_CLOCKWISE);
				HAL_GPIO_WritePin(Stepper->PUL_GPIO,Stepper->PUL_Pin,GPIO_PIN_RESET);
				Stepper->Remain_STEP--;
				Stepper->sum_Step++;
			}
			else if(Stepper->Remain_STEP < 0)
			{
				HAL_GPIO_WritePin(Stepper->DIR_GPIO,Stepper->DIR_Pin,STEPPER_COUNTERCLOCKWISE);
				HAL_GPIO_WritePin(Stepper->PUL_GPIO,Stepper->PUL_Pin,GPIO_PIN_RESET);
				Stepper->Remain_STEP++;
				Stepper->sum_Step--;
			}
			else if(Stepper->Remain_Angle > Stepper->STEP_Angle)
			{
				
				HAL_GPIO_WritePin(Stepper->DIR_GPIO,Stepper->DIR_Pin,STEPPER_CLOCKWISE);
				HAL_GPIO_WritePin(Stepper->PUL_GPIO,Stepper->PUL_Pin,GPIO_PIN_RESET);
				Stepper->Remain_Angle-=(Stepper->STEP_Angle / Stepper->Subdivision);
				Stepper->sum_Step++;
			}
			else if(Stepper->Remain_Angle < -(Stepper->STEP_Angle))
			{
				HAL_GPIO_WritePin(Stepper->DIR_GPIO,Stepper->DIR_Pin,STEPPER_COUNTERCLOCKWISE);
				HAL_GPIO_WritePin(Stepper->PUL_GPIO,Stepper->PUL_Pin,GPIO_PIN_RESET);
				Stepper->Remain_Angle+=(Stepper->STEP_Angle / Stepper->Subdivision);
				Stepper->sum_Step--;
			}
//			else if(rc_ctrl.rc.ch[1]>30)
//			{
//				HAL_GPIO_WritePin(Stepper->DIR_GPIO,Stepper->DIR_Pin,STEPPER_CLOCKWISE);
//				HAL_GPIO_WritePin(Stepper->PUL_GPIO,Stepper->PUL_Pin,GPIO_PIN_RESET);
//				Stepper->sum_Step++;
//			}
//			else if(rc_ctrl.rc.ch[1]<-30)
//			{
//				HAL_GPIO_WritePin(Stepper->DIR_GPIO,Stepper->DIR_Pin,STEPPER_COUNTERCLOCKWISE);
//				HAL_GPIO_WritePin(Stepper->PUL_GPIO,Stepper->PUL_Pin,GPIO_PIN_RESET);
//				Stepper->sum_Step--;
//			}
			return 0;
		}
		else return 1;
	}
	return 0;
}

//�ص�������ʹ��ʱ���ɾ���ʹ�õĶ�ʱ��������
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  
//{
//	if(htim -> Instance == TIM1){
//		STEP_MOVE_PROCESS(&stepper1);
//		TIMcount++;
//	}
//}

#endif
