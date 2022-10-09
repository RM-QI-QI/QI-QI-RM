#include "UI_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "remote_control.h"
#include "referee.h"
#include "gimbal_behaviour.h"

extern int8_t R;
extern ext_game_robot_state_t robot_state;
extern gimbal_behaviour_e gimbal_behaviour;
void UI_task(void const *pvParameters)
{
	int cilent_id;
	Graph_Data Aim[7];
	String_Data strCAP;
	String_Data strSHOOT;
	char tmp1[30];
	char tmp2[30];
	memset(&strCAP,0,sizeof(strCAP));
	memset(&strSHOOT,0,sizeof(strSHOOT));
	for(int k=0;k<5;k++)
	{
		memset(&Aim[k],0,sizeof(Aim[k]));
	} 										//清空图形数据
	
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
				cilent_id = UI_Data_RobotID_REngineer;
				break;
			case UI_Data_RobotID_BEngineer:
				cilent_id = UI_Data_RobotID_BEngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				cilent_id = UI_Data_RobotID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				cilent_id = UI_Data_RobotID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				cilent_id = UI_Data_RobotID_RStandard3;
				break;											
			case UI_Data_RobotID_BStandard1:
				cilent_id = UI_Data_RobotID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				cilent_id = UI_Data_RobotID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				cilent_id = UI_Data_RobotID_BStandard3;
				break;
			default:
				cilent_id = UI_Data_CilentID_BHero;
				break;
		}
		
		if(rc_ctrl.key.v&KEY_PRESSED_OFFSET_B&&rc_ctrl.key.v&KEY_PRESSED_OFFSET_CTRL&&rc_ctrl.key.v&KEY_PRESSED_OFFSET_SHIFT)
		{
			//准星下坠
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,2,900,540,1020,540);
			Line_Draw(&Aim[2],"AL3",UI_Graph_ADD,5,UI_Color_Cyan,2,930,496,990,496);
			Line_Draw(&Aim[3],"AL4",UI_Graph_ADD,5,UI_Color_Yellow,2,920,350,1000,350);
			//可通过宽度
			Line_Draw(&Aim[4],"AL5",UI_Graph_ADD,5,UI_Color_Green,2,570,0,757,443);
			Line_Draw(&Aim[5],"AL6",UI_Graph_ADD,5,UI_Color_Green,2,1350,0,1163,443);
			Arc_Draw(&Aim[6],"RL7",UI_Graph_ADD,5,UI_Color_Green,270,90,2,960,540,300,300);
			Char_Draw(&strCAP,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp2),2,860,300,tmp2);
			Char_ReFresh(robot_state.robot_id,cilent_id,strCAP);
			Char_ReFresh(robot_state.robot_id,cilent_id,strSHOOT);
			UI_ReFresh(robot_state.robot_id,cilent_id,7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
		}

		sprintf(tmp1,"Cap:%.1f(%.2fV)",((get_cap.capvot/100.0-13.5)/10.5*100),get_cap.capvot/100.0);
		
		if(get_cap.capvot > 1800.0f)
				Char_Draw(&strCAP,"CAP",UI_Graph_Change,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
		else
				Char_Draw(&strCAP,"CAP",UI_Graph_Change,8,UI_Color_Orange,20,strlen(tmp1),2,860,100,tmp1);

		if(R)
		{
			sprintf(tmp2,"SHOOT:ON");
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_Change,8,UI_Color_Orange,30,strlen(tmp2),2,860,300,tmp2);
			Line_Draw(&Aim[0],"AL1",UI_Graph_Change,5,UI_Color_Orange,3,960,300,960,540);
		}
		else
		{
			sprintf(tmp2,"SHOOT:OFF");
			Char_Draw(&strSHOOT,"SHOOT",UI_Graph_Change,8,UI_Color_Green,30,strlen(tmp2),2,860,300,tmp2);
			Line_Draw(&Aim[0],"AL1",UI_Graph_Change,5,UI_Color_Green,3,960,300,960,540);
		}
		
		Char_ReFresh(robot_state.robot_id,cilent_id,strCAP);
		Char_ReFresh(robot_state.robot_id,cilent_id,strSHOOT);
		UI_ReFresh(robot_state.robot_id,cilent_id,7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
		
		
		vTaskDelay(200);
	}
}


