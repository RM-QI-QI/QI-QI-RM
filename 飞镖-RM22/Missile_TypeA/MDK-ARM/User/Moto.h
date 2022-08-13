/**
  ******************************************************************************
  * File Name          : Moto.h
  * Description        : C620,C610_FOC Driver 电调(电机)驱动
  ******************************************************************************
 **/
 
#ifndef __Moto_H
#define __Moto_H

#define CHASSIS_CAN hcan2
#define GIMBAL_CAN hcan1

#include "stm32f4xx_hal.h"

/* CAN send and receive ID */

#define    CAN_MOTO_ALL_ID_LOW  0x200
#define    CAN_MOTO_ALL_ID_HIGH  0x1FF
#define    CAN_MOTO_ID_1  0x201
#define    CAN_MOTO_ID_2  0x202
#define    CAN_MOTO_ID_3  0x203
#define    CAN_MOTO_ID_4  0x204
#define    CAN_MOTO_ID_5  0x205
#define    CAN_MOTO_ID_6  0x206
#define    CAN_MOTO_ID_7  0x207
#define    CAN_MOTO_ID_8  0x208

/**********************************
电机数据结构体
*************************************/
typedef struct
{
    volatile uint16_t angle;//机械角
    volatile int16_t speed; //电机转速
    volatile int16_t current; //实际电流
    volatile uint8_t temperate; //电机温度
    volatile uint8_t empty;  //空
	  volatile uint16_t lastangle;//上一次机械角
	  volatile int32_t sum_angle;//角度积分
} Moto_DataTypedef;

 __weak CAN_HandleTypeDef hcan2;          //弱定义，防止CAN2不用时报错
 extern CAN_HandleTypeDef hcan1,hcan2;    //HAL库CAN结构体声明

extern Moto_DataTypedef Moto_Data1[8];
extern Moto_DataTypedef Moto_Data2[8];

void CAN_CMD_MOTO(CAN_HandleTypeDef *CAN_ID,uint32_t MOTO_ID,int16_t data0, int16_t data1, int16_t data2, int16_t data3);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Filter_Init(CAN_HandleTypeDef *h_can);

#endif
