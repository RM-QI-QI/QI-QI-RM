#include "bsp_servo_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim8;

void servo_pwm_set(uint16_t pwm, uint8_t i)
{
    switch(i)
    {
        case 0:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
        }break;
        case 1:
        {
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
        }break;   }
}
