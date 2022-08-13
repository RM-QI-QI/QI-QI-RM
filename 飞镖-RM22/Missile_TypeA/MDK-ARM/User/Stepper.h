/***********************************************************
�ļ�����Stepper.h
������
		�����������ģ��
		���HAL��ʹ��
From:Qi-Q@Rjgawuie
***********************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"

#define STEPPER_ENABLE GPIO_PIN_SET							//ʹ�����ŵ�ƽ
#define STEPPER_DISABLE GPIO_PIN_RESET							//ʧ�����ŵ�ƽ

#define STEPPER_CLOCKWISE GPIO_PIN_SET						//���������ת
#define STEPPER_COUNTERCLOCKWISE GPIO_PIN_RESET 		//���������ת


typedef struct
{
	float STEP_Angle;               //��������
	uint8_t Subdivision;            //������ϸ��ֵ
	volatile int Remain_STEP;           			//ʣ�ಽ��
	volatile float Remain_Angle;							//ʣ��Ƕ�
	char ENA;                       //���ʹ�ܱ�־
	GPIO_TypeDef *ENA_GPIO;         //ʹ��������Ϣ
	uint16_t ENA_Pin;
	GPIO_TypeDef *DIR_GPIO;					//����������Ϣ
	uint16_t DIR_Pin;
	GPIO_TypeDef *PUL_GPIO;					//����������Ϣ
	uint16_t PUL_Pin;
	volatile int32_t sum_Step;
}STEPPER_Typedef;			//����������ƾ��


extern STEPPER_Typedef stepper1;
extern uint8_t TIMcount;

uint16_t Stepper_Init(STEPPER_Typedef *Stepper,
									float STEP_Angle,
									uint8_t Subdivision,
									GPIO_TypeDef *ENA_GPIO,
									uint16_t ENA_Pin,
									GPIO_TypeDef *DIR_GPIO,
									uint16_t DIR_Pin,
									GPIO_TypeDef *PUL_GPIO,
									uint16_t PUL_Pin);
void STEP_ENABLE(STEPPER_Typedef *Stepper);
void STEP_DISABLE(STEPPER_Typedef *Stepper);
void Move_Step(STEPPER_Typedef *Stepper,int Step);
void STEP_SetZero(STEPPER_Typedef *Stepper);
int STEP_Remain(STEPPER_Typedef *Stepper);
float Angle_Remain(STEPPER_Typedef *Stepper);
void Move_Angle(STEPPER_Typedef *Stepper,float Angle);
uint16_t STEP_MOVE_PROCESS(STEPPER_Typedef *Stepper);

