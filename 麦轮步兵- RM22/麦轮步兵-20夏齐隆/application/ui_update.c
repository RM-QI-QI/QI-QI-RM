#include "ui_update.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "remote_control.h"
#include "chassis_task.h"
#include "referee.h"
#include "Gimbal_Task.h"
#include "shoot_task.h"
extern int UI_flag;
uint16_t ShootSpeed=0;      //弹速，15/18/30

uint16_t lastShootSpeed=1;
extern Gimbal_Control_t gimbal_control;
int R=1;
extern int Ready_Flag;
extern ext_game_robot_state_t robot_state;
extern int PITCH_MODE;
void UI_tasks(void const *pvParameters)
{
	Graph_Data Aim[5];
	String_Data strCAP;
	String_Data strSHOOT;
	String_Data strPITCH;
	String_Data strX;
	int cilent_id;
	char tmp1[30];
	char tmp2[30]="SHOOT:READY";
	char tmp3[30]="PITCH:RELETIVE";
	char tmp4[30]="PITCH:ABSOLUTE"	;
    char tmp5[30]="CBQ";
	char tmp6[30]="SHOOT:NO";
	memset(&strCAP,0,sizeof(strCAP));
	memset(&strSHOOT,0,sizeof(strSHOOT));
	memset(&strPITCH,0,sizeof(strPITCH));
	memset(&strX,0,sizeof(strX));
	for(int kk=0;kk<5;kk++)
	{
		memset(&Aim[kk],0,sizeof(Aim[kk]));
	} 		
	  
     	//清空图形数据

	while(1)
	{
		switch (robot_state.robot_id)
		{
			case UI_Data_RobotID_RHero:
				cilent_id = UI_Data_CilentID_RHero;
				break;
			case UI_Data_RobotID_BHero:
				cilent_id = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_REngineer:
				cilent_id = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_BEngineer:
				cilent_id = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				cilent_id = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard2:
				cilent_id = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				cilent_id = UI_Data_CilentID_RStandard3;
				break;											
			case UI_Data_RobotID_BStandard1:
				cilent_id = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				cilent_id = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				cilent_id = UI_Data_CilentID_BStandard3;
				break;
			default:
				cilent_id = UI_Data_CilentID_BHero;
				break;
		}
		
		if(rc_ctrl.key.v&KEY_PRESSED_OFFSET_G)
		{
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_White,3,960,200,960,560);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Yellow,2,945,245,975,245);
			Line_Draw(&Aim[2],"AL3",UI_Graph_ADD,5,UI_Color_Cyan,2,935,362,985,362);
			Line_Draw(&Aim[3],"AL4",UI_Graph_ADD,5,UI_Color_Orange,2,900,500,1020,500);
			Arc_Draw(&Aim[4],"RL5",UI_Graph_ADD,5,UI_Color_Pink,270,90,2,960,540,300,300);
			Char_Draw(&strCAP,"CAP",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp1),2,1060,100,tmp1);
			Char_Draw(&strSHOOT,"SHO",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp2),2,1600,700,tmp6);
			Char_Draw(&strPITCH,"PIT",UI_Graph_ADD,9,UI_Color_Green,20,strlen(tmp3),2,1600,600,tmp3);
			Char_Draw(&strX,"CBQ",UI_Graph_ADD,9,UI_Color_Pink,20,strlen(tmp5),2,1600,500,tmp5);
			Char_ReFresh(robot_state.robot_id,cilent_id,strCAP);
			Char_ReFresh(robot_state.robot_id,cilent_id,strSHOOT);
			Char_ReFresh(robot_state.robot_id,cilent_id,strPITCH);
			Char_ReFresh(robot_state.robot_id,cilent_id,strX);
			UI_ReFresh(robot_state.robot_id,cilent_id,5,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4]);
		}
		
		sprintf(tmp1,"Cap:%.1f(%.2fV)",((get_cap.capvot-13.5f)/10.5f*100.f),get_cap.capvot);
		if(UI_flag==0)
		{
		Char_Draw(&strCAP,"CAP",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp1),2,1060,100,tmp1);
		Char_ReFresh(robot_state.robot_id,cilent_id,strCAP);
		}
		else
		{
		Char_Draw(&strCAP,"CAP",UI_Graph_Change,9,UI_Color_Orange,20,strlen(tmp1),2,1060,100,tmp1);
		Char_ReFresh(robot_state.robot_id,cilent_id,strCAP);	
		}	
		if(Ready_Flag==0)
		{
		Char_Draw(&strSHOOT,"SHO",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp6)+1,2,1600,700,tmp6);	
		}
		else if (Ready_Flag==1)
		{
		Char_Draw(&strSHOOT,"SHO",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp2)+1,2,1600,700,tmp2);	
		}
		
		
		if(PITCH_MODE==0)
	   {
		   Char_Draw(&strPITCH,"PIT",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp3)+1,2,1600,600,tmp3);
		}
		else //if(gimbal_control.gimbal_pitch_motor.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
		{
			 Char_Draw(&strPITCH,"PIT",UI_Graph_Change,9,UI_Color_Green,20,strlen(tmp4)+1,2,1600,600,tmp4);	
		}
		Char_ReFresh(robot_state.robot_id,cilent_id,strPITCH);
	
		Char_ReFresh(robot_state.robot_id,cilent_id,strSHOOT);
		Char_ReFresh(robot_state.robot_id,cilent_id,strX);	

		UI_ReFresh(robot_state.robot_id,cilent_id,5,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4]);
		
		
		vTaskDelay(500);
	}
}


	



	
