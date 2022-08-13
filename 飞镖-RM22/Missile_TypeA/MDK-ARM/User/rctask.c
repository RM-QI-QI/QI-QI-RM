/***********************************************************
文件名：rctask.c
描述：
		FreeRTOS-遥控器任务，遥控器控制飞镖发射架整体运动映射
From:Qi-Q@Rjgawuie
***********************************************************/

#include "rctask.h"
#include "remote_control.h"
#include "command.h"
#include "chassis_task.h"
#include "Display.h"
#include "Stepper.h"
#include "stdio.h"

uint8_t last_s_state[2];
int16_t last_ch[5];
uint8_t lunched_num;
double accuracy = 10.0f;

///***********************************************************
//函数名：Yaw_Pitch_Adjust
//功能：Yaw轴，Pitch轴调整循环
//参数：None
//***********************************************************/
//void Yaw_Pitch_Adjust()
//{
//	fp32 accuracy = 10.0;
//	fp32 set_angle = (float)length2angle(stepper1.sum_Step/40.0);
//	
//	while(rc_ctrl.rc.ch[4]>-440)
//		{
//			if(rc_ctrl.rc.s[0]==2&&last_s_state[0]==3)	
//				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);		//激光器开关
//			
//			if(rc_ctrl.rc.ch[0]>440&&last_ch[0]<=440) 	Angle_target[2]+=accuracy;		//右摇杆左右调整Yaw轴
//			else if(rc_ctrl.rc.ch[0]<-440&&last_ch[0]>=-440) 	Angle_target[2]-=accuracy;
//			
////			if(rc_ctrl.rc.ch[1]>440&&last_ch[1]<=440) 	;		//右摇杆上下调整Pitch轴
////			else if(rc_ctrl.rc.ch[1]<-440&&last_ch[1]>=-440) 	;
//			
//			
//			for(uint8_t i = 0;i<2;i++)
//				{
//					last_s_state[i] = rc_ctrl.rc.s[i];
//				}
//				for(uint8_t i = 0;i<5;i++)
//				{
//					last_ch[i] = rc_ctrl.rc.ch[i];
//				}
//		}
//}

/***********************************************************
函数名：RC_Task_Setup
功能：遥控任务初始化部分（只会在开始时运行一次）
参数：None
***********************************************************/
void RC_Task_Setup()
{
	
}

/***********************************************************
函数名：RC_Task_Loop
功能：遥控任务循环部分
参数：None
***********************************************************/
void RC_Task_Loop()
{
	
	if(rc_ctrl.rc.s[1]==1&&last_s_state[1]==3)		//回到调试模式
	{
		for(int i=0;i<6;i++)
		{
			speed[i] = 0;
			current[i] = 0;
		}
	}
	else if(rc_ctrl.rc.s[1]==1)		//场前调试
	{
		if(rc_ctrl.rc.s[0]==1&&last_s_state[0]==3)	
			turn_lock == 0?TurnLock():TurnUnlock();		//换弹盘锁
		else if(rc_ctrl.rc.s[0]==2&&last_s_state[0]==3)	
			HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);		//激光器开关
		
		if(rc_ctrl.rc.ch[2]>440&&last_ch[2]<=440) 	Next_Page();		//左摇杆左右换页
		else if(rc_ctrl.rc.ch[2]<-440&&last_ch[2]>=-440) 	Last_Page();
		
		if(rc_ctrl.rc.ch[3]>440&&last_ch[3]<=440) 	Angle_target[0]+=90;		//左摇杆上下换弹
		else if(rc_ctrl.rc.ch[3]<-440&&last_ch[3]>=-440) 	Angle_target[0]-=90;
		
		if(rc_ctrl.rc.ch[4]>440&&last_ch[4]<=440) 			//拨盘摇杆向下调整Yaw/Pitch轴精度
			accuracy/=10.0f;
		else if(rc_ctrl.rc.ch[4]<-440&&last_ch[4]>=-440) 			//拨盘摇杆向上调整Yaw/Pitch轴精度
			accuracy*=10.0f;
		
		if(rc_ctrl.rc.ch[0]>440&&last_ch[0]<=440) 	Angle_target[2]+=accuracy;		//右摇杆左右调整Yaw轴
		else if(rc_ctrl.rc.ch[0]<-440&&last_ch[0]>=-440) 	Angle_target[2]-=accuracy;
		
		if(rc_ctrl.rc.ch[1]>440&&last_ch[1]<=440)		//右摇杆上下调整Pitch轴
		{
			Pitch_AngleSet(Angle_Pitch+accuracy);
		}
		else if(rc_ctrl.rc.ch[1]<-440&&last_ch[1]>=-440)
		{
			Pitch_AngleSet(Angle_Pitch-accuracy);
		}
	}
	else if(rc_ctrl.rc.s[1]==3&&last_s_state[1]==1)		//开始打前哨站
	{
		if(turn_lock == 0) TurnLock();	//检查换弹盘是否锁定
		lunched_num = 0;
	}
	else if(rc_ctrl.rc.s[1]==3)		//打前哨站
	{
		if(rc_ctrl.rc.s[0]>1)	
		{
			for(int i=0;i<6;i++)
			{
				speed[i] = outpost_speed[i];
				current[i] = outpost_current[i];
			}
		}
		if(rc_ctrl.rc.s[0]==1&&last_s_state[0]==3)
		{
			for(int i=0;i<6;i++)
			{
				speed[i] = 0;
				current[i] = 0;
			}
		}
			
		
		if(rc_ctrl.rc.s[0]==2&&last_s_state[0]==3)
		{
			Angle_target[1]+=170;
			osDelay(500);
			Angle_target[1]-=170;
			osDelay(500);
			
			if(lunched_num%2==0)	Angle_target[0]+=180;
			else Angle_target[0]+=90;
			lunched_num++;
		}
	}
	else if(rc_ctrl.rc.s[1]==2&&last_s_state[1]==3)		//开始打基地
	{
		Angle_target[2]+=144.9f;
	}
	else if(rc_ctrl.rc.s[1]==2)		//打基地
	{
		if(rc_ctrl.rc.s[0]>1)	
		{
			for(int i=0;i<6;i++)
			{
				speed[i] = base_speed[i];
				current[i] = base_current[i];
			}
		}
		if(rc_ctrl.rc.s[0]==1&&last_s_state[0]==3)
		{
			for(int i=0;i<6;i++)
			{
				speed[i] = 0;
				current[i] = 0;
			}
		}
		
		if(rc_ctrl.rc.s[0]==2&&last_s_state[0]==3)
		{
			Angle_target[1]+=170;
			osDelay(500);
			Angle_target[1]-=170;
			osDelay(500);
			
			if(lunched_num%2==0)	Angle_target[0]+=180;
			else Angle_target[0]+=90;
			lunched_num++;
		}
	}
	
	for(uint8_t i = 0;i<2;i++)
	{
		last_s_state[i] = rc_ctrl.rc.s[i];
	}
	for(uint8_t i = 0;i<5;i++)
	{
		last_ch[i] = rc_ctrl.rc.ch[i];
	}
	osDelay(10);
}
