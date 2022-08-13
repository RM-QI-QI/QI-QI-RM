/***********************************************************
文件名：display.c
描述：
		FreeRTOS-LCD显示屏显示任务
From:Qi-Q@Rjgawuie
***********************************************************/

#include "display.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include "TFT20_ILI9225.h"
#include "Stepper.h"
#include "chassis_task.h"
#include "Moto.h"
#include "usart.h"
#include "stdio.h"
#include "ff.h"
#include "sdio_sdcard.h"
#include "sdio.h"
#include "string.h"
#include "rctask.h"


int lstate = 0,Page=1;
volatile int LCD_Busy=0;

/***********************************************************
函数名：DrawPage_1
功能：绘制LCD页面-1
参数：None
***********************************************************/
void DrawPage_1()
{
	
}

/***********************************************************
函数名：DrawPage1
功能：绘制LCD页面1
参数：None
***********************************************************/
void DrawPage1()
{
	TFT20_DisplayImg(0,0,(u8 *)QiQ);
	TFT20_DrawFont_GBK16(86,0,WHITE,BLACK,"剩余角度");
	TFT20_DrawFont_GBK16(154,0,WHITE,BLACK,"剩余步数");
	TFT20_DrawFont_GBK16(86,41,WHITE,BLACK,"云台角度");
	TFT20_DrawFont_GBK16(154,41,WHITE,BLACK,"绝对位置");
	
	TFT20_DrawLine(80,20,220,20,WHITE);
	TFT20_DrawLine(80,40,220,40,WHITE);
	TFT20_DrawLine(80,60,220,60,WHITE);
	TFT20_DrawLine(80,79,220,79,WHITE);
	TFT20_DrawLine(152,0,152,79,WHITE);
}
/***********************************************************
函数名：DrawPage2
功能：绘制LCD页面2
参数：None
***********************************************************/
void DrawPage2()
{
	TFT20_DrawFont_GBK16(8,0,WHITE,BLACK,"ID");
	TFT20_DrawFont_GBK16(12,20,WHITE,BLACK,"P");
	TFT20_DrawFont_GBK16(12,40,WHITE,BLACK,"I");
	TFT20_DrawFont_GBK16(12,60,WHITE,BLACK,"D");
	TFT20_DrawFont_GBK16(0,80,WHITE,BLACK,"转速");
	TFT20_DrawFont_GBK16(4,100,WHITE,BLACK,"SET");
	TFT20_DrawFont_GBK16(8,120,WHITE,BLACK,"ID");
	TFT20_DrawFont_GBK16(0,140,WHITE,BLACK,"转速");
	TFT20_DrawFont_GBK16(4,160,WHITE,BLACK,"SET");
	
	TFT20_DrawFont_GBK16(37,0,WHITE,BLACK,"M1");
	TFT20_DrawFont_GBK16(83,0,WHITE,BLACK,"M2");
	TFT20_DrawFont_GBK16(129,0,WHITE,BLACK,"M3");
	TFT20_DrawFont_GBK16(175,0,WHITE,BLACK,"M4");
	TFT20_DrawFont_GBK16(37,120,WHITE,BLACK,"M5");
	TFT20_DrawFont_GBK16(83,120,WHITE,BLACK,"M6");
	TFT20_DrawFont_GBK16(129,120,WHITE,BLACK,"M7");
	TFT20_DrawFont_GBK16(175,120,WHITE,BLACK,"M8");
	
	TFT20_DrawLine(34,0,34,176,WHITE);
	TFT20_DrawLine(82,0,82,176,WHITE);
	TFT20_DrawLine(128,0,128,176,WHITE);
	TFT20_DrawLine(174,0,174,176,WHITE);
	TFT20_DrawLine(0,118,220,118,WHITE);
}

/***********************************************************
函数名：DrawPage3
功能：绘制LCD页面3
参数：None
***********************************************************/
void DrawPage3()
{
	TFT20_DrawFont_GBK16(8,0,WHITE,BLACK,"ID");
	TFT20_DrawFont_GBK16(0,20,WHITE,BLACK,"转速");
	TFT20_DrawFont_GBK16(4,40,WHITE,BLACK,"SET");
	TFT20_DrawFont_GBK16(8,60,WHITE,BLACK,"ID");
	TFT20_DrawFont_GBK16(12,80,WHITE,BLACK,"P");
	TFT20_DrawFont_GBK16(12,100,WHITE,BLACK,"I");
	TFT20_DrawFont_GBK16(12,120,WHITE,BLACK,"D");
	TFT20_DrawFont_GBK16(0,140,WHITE,BLACK,"转速");
	TFT20_DrawFont_GBK16(4,160,WHITE,BLACK,"SET");
	
	TFT20_DrawFont_GBK16(37,0,WHITE,BLACK,"M1");
	TFT20_DrawFont_GBK16(83,0,WHITE,BLACK,"M2");
	TFT20_DrawFont_GBK16(129,0,WHITE,BLACK,"M3");
	TFT20_DrawFont_GBK16(175,0,WHITE,BLACK,"M4");
	TFT20_DrawFont_GBK16(37,60,WHITE,BLACK,"M5");
	TFT20_DrawFont_GBK16(83,60,WHITE,BLACK,"M6");
	TFT20_DrawFont_GBK16(129,60,WHITE,BLACK,"M7");
	TFT20_DrawFont_GBK16(175,60,WHITE,BLACK,"M8");
	
	TFT20_DrawLine(34,0,34,176,WHITE);
	TFT20_DrawLine(82,0,82,176,WHITE);
	TFT20_DrawLine(128,0,128,176,WHITE);
	TFT20_DrawLine(174,0,174,176,WHITE);
	TFT20_DrawLine(0,58,220,58,WHITE);
}

/***********************************************************
函数名：Next_Page
功能：显示下一个页面
参数：None
***********************************************************/
void Next_Page()
{
	Page++;
	if(Page>PageMax)Page=1;
	Page_Jump(Page);
}

/***********************************************************
函数名：Last_Page
功能：显示上一个页面
参数：None
***********************************************************/
void Last_Page()
{
	Page--;
	if(Page<1)Page=PageMax;
	Page_Jump(Page);
}

/***********************************************************
函数名：Page_Jump
功能：跳转到指定页面
参数：None
***********************************************************/
void Page_Jump(int8_t pagenum)
{
	while(LCD_Busy);
	LCD_Busy = 1;
	Page = pagenum;
	TFT20_Clear(BLACK);
	
	switch(Page)
	{
		case -1:
			DrawPage_1();
			break;
		case 1:
			DrawPage1();
			break;
		case 2:
			DrawPage2();
			break;
		case 3:
			DrawPage3();
			break;
	}
	LCD_Busy = 0;
}

/***********************************************************
函数名：Display_Task_Setup
功能：显示任务初始化部分（只会在开始时运行一次）
参数：None
***********************************************************/
void Display_Task_Setup()
{
	Page_Jump(1);
}

/***********************************************************
函数名：Display_Task_Loop
功能：显示任务循环部分
参数：None
***********************************************************/
void Display_Task_Loop()
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==1&&lstate == 0) {lstate = 1;Next_Page();}      //换页
	else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0&&lstate == 1) lstate=0;
	
	if(LCD_Busy == 0)
	{
//	if(lastPage!=1&&Page==1)
//	{
//		TFT20_Clear(BLACK);
//		DrawPage1();
//	}
//	else if(lastPage!=2&&Page==2)
//	{
//		TFT20_Clear(BLACK);
//		DrawPage2();
//	}
//	else if(lastPage!=3&&Page==3)
//	{
//		TFT20_Clear(BLACK);
//		DrawPage3();
//	}
//	
//	lastPage=Page;
		LCD_Busy=1;
		if(Page==1)
		{
			TFT20_DrawFormat_Float(82,22,WHITE,BLACK,"%8.2f",stepper1.Remain_Angle);
			TFT20_DrawFormat_Int(154,22,WHITE,BLACK,"%8d",stepper1.Remain_STEP);
			TFT20_DrawFormat_Float(82,62,WHITE,BLACK,"%8.2f",(float)length2angle(stepper1.sum_Step/40.0));
			TFT20_DrawFormat_Float(154,62,WHITE,BLACK,"%8.2f",(float)(stepper1.sum_Step/40.0));
			TFT20_DrawFormat_Float(0,80,WHITE,BLACK,"P:%5.2f",P_I_D[11].Kp);
			TFT20_DrawFormat_Float(64,80,WHITE,BLACK,"I:%5.2f",P_I_D[11].Ki);
			TFT20_DrawFormat_Float(128,80,WHITE,BLACK,"D:%5.2f",P_I_D[11].Kd);
			TFT20_DrawFormat_Float(0,100,WHITE,BLACK,"PIDout:%8.2f",Moto1[11].out);
			TFT20_DrawFormat_Float(1,120,WHITE,BLACK,"Angle:%12.2f",Moto_Data2[2].sum_angle*10.0/8192.0);
			TFT20_DrawFormat_Float(1,140,WHITE,BLACK,"Target:%12.2f",Angle_target[2]);
			TFT20_DrawFormat_Float(1,160,WHITE,BLACK,"Accuracy:%6.2f",accuracy);
		}
		else if(Page==2)
		{
			for(int i=0;i<4;i++)
			{
				TFT20_DrawFormat_Float(38+46*i,20,WHITE,BLACK,"%5.2f",P_I_D[i].Kp);
				TFT20_DrawFormat_Float(38+46*i,40,WHITE,BLACK,"%5.2f",P_I_D[i].Ki);
				TFT20_DrawFormat_Float(38+46*i,60,WHITE,BLACK,"%5.2f",P_I_D[i].Kd);
				TFT20_DrawFormat_Int(38+46*i,80,WHITE,BLACK,"%5d",Moto_Data1[i].speed);
				TFT20_DrawFormat_Int(38+46*i,100,WHITE,BLACK,"%5d",speed[i]);
			}
				TFT20_DrawFormat_Int(38,140,WHITE,BLACK,"%5d",Moto_Data1[4].speed);
				TFT20_DrawFormat_Int(84,140,WHITE,BLACK,"%5d",Moto_Data1[5].speed);
				TFT20_DrawFormat_Int(130,140,WHITE,BLACK,"%5d",Moto_Data2[0].speed);
				TFT20_DrawFormat_Int(176,140,WHITE,BLACK,"%5d",Moto_Data2[1].speed);
				TFT20_DrawFormat_Int(38,160,WHITE,BLACK,"%5d",speed[4]);
				TFT20_DrawFormat_Int(84,160,WHITE,BLACK,"%5d",speed[5]);
				TFT20_DrawFont_GBK16(130,160,WHITE,BLACK,"-----");
				TFT20_DrawFont_GBK16(176,160,WHITE,BLACK,"-----");
		}
		else if(Page==3)
		{
			for(int i=0;i<4;i++)
			{
				TFT20_DrawFormat_Float(38+46*i,80,WHITE,BLACK,"%5.2f",P_I_D[i+4].Kp);
				TFT20_DrawFormat_Float(38+46*i,100,WHITE,BLACK,"%5.2f",P_I_D[i+4].Ki);
				TFT20_DrawFormat_Float(38+46*i,120,WHITE,BLACK,"%5.2f",P_I_D[i+4].Kd);
				TFT20_DrawFormat_Int(38+46*i,20,WHITE,BLACK,"%5d",Moto_Data1[i].speed);
				TFT20_DrawFormat_Int(38+46*i,40,WHITE,BLACK,"%5d",speed[i]);
			}
			
				TFT20_DrawFormat_Int(38,140,WHITE,BLACK,"%5d",Moto_Data1[4].speed);
				TFT20_DrawFormat_Int(84,140,WHITE,BLACK,"%5d",Moto_Data1[5].speed);
				TFT20_DrawFormat_Int(130,140,WHITE,BLACK,"%5d",Moto_Data2[0].speed);
				TFT20_DrawFormat_Int(176,140,WHITE,BLACK,"%5d",Moto_Data2[1].speed);
				TFT20_DrawFormat_Int(38,160,WHITE,BLACK,"%5d",speed[4]);
				TFT20_DrawFormat_Int(84,160,WHITE,BLACK,"%5d",speed[5]);
				TFT20_DrawFont_GBK16(130,160,WHITE,BLACK,"-----");
				TFT20_DrawFont_GBK16(176,160,WHITE,BLACK,"-----");
		}
		else if(Page==-1)
		{
			
		}
		LCD_Busy=0;
	}
	
	osDelay(70);
}

