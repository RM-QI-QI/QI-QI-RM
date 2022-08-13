/***********************************************************
�ļ�����rctask.c
������
		FreeRTOS-ң��������ң�������Ʒ��ڷ���������˶�ӳ��
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
//��������Yaw_Pitch_Adjust
//���ܣ�Yaw�ᣬPitch�����ѭ��
//������None
//***********************************************************/
//void Yaw_Pitch_Adjust()
//{
//	fp32 accuracy = 10.0;
//	fp32 set_angle = (float)length2angle(stepper1.sum_Step/40.0);
//	
//	while(rc_ctrl.rc.ch[4]>-440)
//		{
//			if(rc_ctrl.rc.s[0]==2&&last_s_state[0]==3)	
//				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);		//����������
//			
//			if(rc_ctrl.rc.ch[0]>440&&last_ch[0]<=440) 	Angle_target[2]+=accuracy;		//��ҡ�����ҵ���Yaw��
//			else if(rc_ctrl.rc.ch[0]<-440&&last_ch[0]>=-440) 	Angle_target[2]-=accuracy;
//			
////			if(rc_ctrl.rc.ch[1]>440&&last_ch[1]<=440) 	;		//��ҡ�����µ���Pitch��
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
��������RC_Task_Setup
���ܣ�ң�������ʼ�����֣�ֻ���ڿ�ʼʱ����һ�Σ�
������None
***********************************************************/
void RC_Task_Setup()
{
	
}

/***********************************************************
��������RC_Task_Loop
���ܣ�ң������ѭ������
������None
***********************************************************/
void RC_Task_Loop()
{
	
	if(rc_ctrl.rc.s[1]==1&&last_s_state[1]==3)		//�ص�����ģʽ
	{
		for(int i=0;i<6;i++)
		{
			speed[i] = 0;
			current[i] = 0;
		}
	}
	else if(rc_ctrl.rc.s[1]==1)		//��ǰ����
	{
		if(rc_ctrl.rc.s[0]==1&&last_s_state[0]==3)	
			turn_lock == 0?TurnLock():TurnUnlock();		//��������
		else if(rc_ctrl.rc.s[0]==2&&last_s_state[0]==3)	
			HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);		//����������
		
		if(rc_ctrl.rc.ch[2]>440&&last_ch[2]<=440) 	Next_Page();		//��ҡ�����һ�ҳ
		else if(rc_ctrl.rc.ch[2]<-440&&last_ch[2]>=-440) 	Last_Page();
		
		if(rc_ctrl.rc.ch[3]>440&&last_ch[3]<=440) 	Angle_target[0]+=90;		//��ҡ�����»���
		else if(rc_ctrl.rc.ch[3]<-440&&last_ch[3]>=-440) 	Angle_target[0]-=90;
		
		if(rc_ctrl.rc.ch[4]>440&&last_ch[4]<=440) 			//����ҡ�����µ���Yaw/Pitch�ᾫ��
			accuracy/=10.0f;
		else if(rc_ctrl.rc.ch[4]<-440&&last_ch[4]>=-440) 			//����ҡ�����ϵ���Yaw/Pitch�ᾫ��
			accuracy*=10.0f;
		
		if(rc_ctrl.rc.ch[0]>440&&last_ch[0]<=440) 	Angle_target[2]+=accuracy;		//��ҡ�����ҵ���Yaw��
		else if(rc_ctrl.rc.ch[0]<-440&&last_ch[0]>=-440) 	Angle_target[2]-=accuracy;
		
		if(rc_ctrl.rc.ch[1]>440&&last_ch[1]<=440)		//��ҡ�����µ���Pitch��
		{
			Pitch_AngleSet(Angle_Pitch+accuracy);
		}
		else if(rc_ctrl.rc.ch[1]<-440&&last_ch[1]>=-440)
		{
			Pitch_AngleSet(Angle_Pitch-accuracy);
		}
	}
	else if(rc_ctrl.rc.s[1]==3&&last_s_state[1]==1)		//��ʼ��ǰ��վ
	{
		if(turn_lock == 0) TurnLock();	//��黻�����Ƿ�����
		lunched_num = 0;
	}
	else if(rc_ctrl.rc.s[1]==3)		//��ǰ��վ
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
	else if(rc_ctrl.rc.s[1]==2&&last_s_state[1]==3)		//��ʼ�����
	{
		Angle_target[2]+=144.9f;
	}
	else if(rc_ctrl.rc.s[1]==2)		//�����
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
