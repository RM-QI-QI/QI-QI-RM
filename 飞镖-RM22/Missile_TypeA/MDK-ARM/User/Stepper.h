/***********************************************************
文件名：Stepper.h
描述：
		步进电机驱动模块
		配合HAL库使用
From:Qi-Q@Rjgawuie
***********************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"

#define STEPPER_ENABLE GPIO_PIN_SET							//使能引脚电平
#define STEPPER_DISABLE GPIO_PIN_RESET							//失能引脚电平

#define STEPPER_CLOCKWISE GPIO_PIN_SET						//步进电机正转
#define STEPPER_COUNTERCLOCKWISE GPIO_PIN_RESET 		//步进电机反转


typedef struct
{
	float STEP_Angle;               //电机步距角
	uint8_t Subdivision;            //驱动器细分值
	volatile int Remain_STEP;           			//剩余步数
	volatile float Remain_Angle;							//剩余角度
	char ENA;                       //电机使能标志
	GPIO_TypeDef *ENA_GPIO;         //使能引脚信息
	uint16_t ENA_Pin;
	GPIO_TypeDef *DIR_GPIO;					//方向引脚信息
	uint16_t DIR_Pin;
	GPIO_TypeDef *PUL_GPIO;					//脉冲引脚信息
	uint16_t PUL_Pin;
	volatile int32_t sum_Step;
}STEPPER_Typedef;			//步进电机控制句柄


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

