/***********************************************************
文件名：SentryRC_Task.c
描述：
		FreeRTOS-哨兵通信任务
From:Qi-Q@Rjgawuie
***********************************************************/
#include "SentryRC_Task.h"

#include "remote_control.h"
#include "RM_Cilent_UI.h"
#include "referee.h"
#include "remote_control.h"

/***********************************************************
函数名：SentryRC_Task_Setup
功能：哨兵通信任务初始化部分（只会在开始时运行一次）
参数：None
***********************************************************/
void SentryRC_Task_Setup()
{
	
}

/***********************************************************
函数名：SentryRC_Task_Loop
功能：哨兵通信任务循环部分
参数：None
***********************************************************/
void SentryRC_Task_Loop()
{
	uint16_t Sentry_ID;
	uint8_t data;
	
	if(student_interactive_data_t.receiver_ID > 100) //蓝方
		Sentry_ID = UI_Data_RobotID_BSentry;
	else Sentry_ID = UI_Data_RobotID_RSentry; //红方
	
	if(rc_ctrl.rc.ch[2]>440)//4
	{
		data = 4;
	}
	else if(rc_ctrl.rc.ch[2]<-440)//2
	{
		data = 2;
	}
	else if(rc_ctrl.rc.ch[3]>440)//3
	{
		data = 3;
	}
	else if(rc_ctrl.rc.ch[3]<-440)//1
	{
		data = 1;
	}
	else data = 0;
	
	Robot_Transfer(Sentry_ID,0x201,1,&data);
	osDelay(200);
}
