/***********************************************************
文件名：chassis_task.c
描述：
		FreeRTOS-底盘任务，控制飞镖发射架整体运动
From:Qi-Q@Rjgawuie
***********************************************************/

#include "chassis_task.h"

#include "remote_control.h"
#include "Moto.h"
#include "pid.h"
#include "Stepper.h"
#include "math.h"
#include "usart.h"
#include "stdio.h"
#include "bkp_sram.h"
#include "string.h"
#include "display.h"
#include "tim.h"

extern osTimerId_t TIM_STEPHandle; //软件定时器句柄，控制步进电机用

uint16_t measuredSpeed;  //测出的速度

int RunMode = 0;    //运行模式选择，默认PID模式

double length_chassis = Length_Chassis,
			 length_launch = Length_Launch,
			 length_brace = Length_Brace,
			 pitch_error = Pitch_Error,
			 max_length = MAX_Length,
			 min_length = MIN_Length,
			 screw_pitch = Screw_Pitch;

pid_type_def Moto1[12];      //电机PID数据

volatile fp32 Angle_target[3]={0.0f,0.0f,0.0f};
double Angle_Pitch;		//设定的Pitch轴角度

void change_all_para(void);


PID_Value_Typedef P_I_D[12]= {1.5f,0.0f,0.5f,        //PID参数
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															80.0f,0.0f,20.0f,     //角度环
															80.0f,0.0f,20.0f,
															110.0f,0.0f,20.0f};
fp32 Maxout[12]={16000,16000,16000,16000,16000,16000,16000,16000,16000,5000,5000,8000};
fp32 Maxiout[12]={16000,16000,16000,16000,16000,16000,16000,16000,16000,5000,5000,8000};

fp32 Angle_error[3]={1.65f,1.65f,1.65f};
uint8_t pid_mode[12]={PID_POSITION, PID_POSITION, PID_POSITION,
									PID_POSITION, PID_POSITION, PID_POSITION,
									PID_POSITION, PID_POSITION, PID_POSITION,
									PID_POSITION, PID_POSITION, PID_POSITION};

uint8_t turn_lock = 1;		//换弹装置锁，默认使能(1使能，0失能)
int16_t speed[6]={0,0,0,0,0,0};			//速度目标值，恒功率控制模式最大转速
int16_t current[6]={0,0,0,0,0,0};		//电流目标值

int outpost_speed[6]={0,0,0,0,0,0};			//打前哨站速度目标值，恒功率控制模式最大转速
int outpost_current[6]={0,0,0,0,0,0};		//电流目标值
int base_speed[6]={0,0,0,0,0,0};			//打基地速度目标值，恒功率控制模式最大转速
int base_current[6]={0,0,0,0,0,0};		//电流目标值

__IO Backup_Typedef *BKP_Handler = (__IO Backup_Typedef *)(BKPSRAM_BASE); //指向备份RAM的指针

/***********************************************************
函数名：angle2length
功能：角度到底盘距离的解算
参数：angle 目标角度(角度制)
***********************************************************/
double angle2length(double angle)
{
	double Cmax,Cmin,cosA,Part1,Part2;
	cosA=cos((angle-pitch_error)/180*3.1415926);
//	printf("CosA=%f\n",cosA);
	Part1=length_launch*cosA;
	Part2=sqrt(Part1*Part1-length_launch*length_launch+length_brace*length_brace);
	Cmax=length_chassis-Part1-Part2;
//	printf("Part1=%f\nPart2=%f\nCmax=%f\n",Part1,Part2,Cmax);
	if(Cmax>min_length&&Cmax<max_length) return Cmax;
	Cmin=length_chassis-Part1+Part2;
//	printf("Cmin=%f\n",Cmin);
	if(Cmin>min_length&&Cmin<max_length) return Cmax;
	return -1;
}

/***********************************************************
函数名：length2angle
功能：底盘距离到角度的解算
参数：length 支撑轴目前位置
***********************************************************/
double length2angle(double length)
{
	double C=length_chassis-length;
	return acos((length_launch*length_launch+C*C-length_brace*length_brace)/2/length_launch/C)*57.3+pitch_error;
}

/***********************************************************
函数名：Pitch_AngleSet
功能：设置Pitch轴到指定角度
参数：Angle		要设定的角度
返回值：-1		失败，角度可能超过调整范围
				0			成功
***********************************************************/
int Pitch_AngleSet(double Angle)
{
	double length_set;
	length_set = angle2length(Angle);
	if(length_set < 0) return -1;
	else 
	{
		Angle_Pitch = Angle;
		Move_Step(&stepper1,(int)((angle2length(Angle)-stepper1.Remain_Angle)/screw_pitch*360/stepper1.STEP_Angle*stepper1.Subdivision-stepper1.sum_Step-stepper1.Remain_STEP));
		return 0;
	}
}

/***********************************************************
函数名：Chassis_Task_Setup
功能：底盘任务初始化部分（只会在开始时运行一次）
参数：None
***********************************************************/
void Chassis_Task_Setup()
{
	for(int i=0;i < 12;i++)
	{
		PID_init(&Moto1[i], pid_mode[i], &P_I_D[i], Maxout[i], Maxiout[i]);
	}
	Angle_Pitch = (float)length2angle(stepper1.sum_Step/40.0);
}

/***********************************************************
函数名：Chassis_Task_Loop
功能：底盘任务循环部分
参数：None
***********************************************************/
void Chassis_Task_Loop()
{
	if(RunMode == 0)             //PID控制模式
	{
	//	CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_LOW,                          //遥控器控制
	//																						PID_calc(&Moto1[0],(fp32)Moto_Data1[0].speed, 10*(rc_ctrl.rc.ch[3]+rc_ctrl.rc.ch[2]+rc_ctrl.rc.ch[0])),
	//																						PID_calc(&Moto1[1],(fp32)Moto_Data1[1].speed, -10*(rc_ctrl.rc.ch[3]-rc_ctrl.rc.ch[2]-rc_ctrl.rc.ch[0])),
	//																						PID_calc(&Moto1[2],(fp32)Moto_Data1[2].speed, 10*(rc_ctrl.rc.ch[3]-rc_ctrl.rc.ch[2]+rc_ctrl.rc.ch[0])),
	//																						PID_calc(&Moto1[3],(fp32)Moto_Data1[3].speed, -10*(rc_ctrl.rc.ch[3]+rc_ctrl.rc.ch[2]-rc_ctrl.rc.ch[0])));

		CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_LOW,                          
																							PID_calc(&Moto1[0],(fp32)Moto_Data1[0].speed, speed[0]),
																							PID_calc(&Moto1[1],(fp32)Moto_Data1[1].speed, -speed[1]),
																							PID_calc(&Moto1[2],(fp32)Moto_Data1[2].speed, speed[2]),
																							PID_calc(&Moto1[3],(fp32)Moto_Data1[3].speed, -speed[3]));
		CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_HIGH,                          
																							PID_calc(&Moto1[4],(fp32)Moto_Data1[4].speed, speed[4]),
																							PID_calc(&Moto1[5],(fp32)Moto_Data1[5].speed, -speed[5]),
																							0,
																							0);
	}
	else if(RunMode == 1)                    //恒功率模式
	{
		CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_LOW,                          
																							Moto_Data1[0].speed > speed[0]||Moto_Data1[0].speed < -speed[0]?current[0] == 0?current[0]:PID_calc(&Moto1[0],(fp32)Moto_Data1[0].speed, speed[0]+100):current[0],
																							Moto_Data1[1].speed < -speed[1]||Moto_Data1[1].speed > speed[1]?current[1] == 0?current[1]:PID_calc(&Moto1[1],(fp32)Moto_Data1[1].speed, -speed[1]-100):-current[1],
																							Moto_Data1[2].speed > speed[2]||Moto_Data1[2].speed < -speed[2]?current[2] == 0?current[2]:PID_calc(&Moto1[2],(fp32)Moto_Data1[2].speed, speed[2]+100):current[2],
																							Moto_Data1[3].speed < -speed[3]||Moto_Data1[3].speed > speed[3]?current[3] == 0?current[3]:PID_calc(&Moto1[3],(fp32)Moto_Data1[3].speed, -speed[3]-100):-current[3]);
		CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_HIGH,                          
																							Moto_Data1[4].speed > speed[4]||Moto_Data1[4].speed < -speed[4]?current[4] == 0?current[4]:PID_calc(&Moto1[4],(fp32)Moto_Data1[4].speed, speed[4]+100):current[4],
																							Moto_Data1[5].speed < -speed[5]||Moto_Data1[5].speed > speed[5]?current[5] == 0?current[5]:PID_calc(&Moto1[5],(fp32)Moto_Data1[5].speed, -speed[5]-100):-current[5],
																							0,
																							0);
	}
	
	CAN_CMD_MOTO(&hcan2,CAN_MOTO_ALL_ID_LOW,                          
																						turn_lock ? PID_calc(&Moto1[6],(fp32)Moto_Data2[0].speed, PID_calc(&Moto1[9],(fp32)(Moto_Data2[0].sum_angle*10.0/8192.0),Angle_target[0]+Angle_error[0])):0,
																						PID_calc(&Moto1[7],(fp32)Moto_Data2[1].speed, PID_calc(&Moto1[10],(fp32)(Moto_Data2[1].sum_angle*10.0/8192.0),Angle_target[1]+Angle_error[1])),
//																							PID_calc(&Moto1[11],(fp32)(Moto_Data2[2].sum_angle*10.0/8192.0),Angle_target[2]+Angle_error[2]),
																						PID_calc(&Moto1[8],(fp32)Moto_Data2[2].speed, PID_calc(&Moto1[11],(fp32)(Moto_Data2[2].sum_angle*10.0/8192.0),Angle_target[2]+Angle_error[2])),
																						0);
	 
																						

	STEP_MOVE_PROCESS(&stepper1);
	TIMcount++;
	BKP_Handler->pitch_sum_step = stepper1.sum_Step;
	BKP_Handler->updatemark = 1; //向备份域第一位写入1代表有数据更新
	

	osDelay(1);
	

	
	for(int i=0;i<12;i++)                                 //更改PID参数
	{
		if(P_I_D [i].Kd!=Moto1[i].Kd||P_I_D [i].Ki!=Moto1[i].Ki||P_I_D [i].Kp!=Moto1[i].Kp||Maxout[i]!=Moto1[i].max_out||Maxiout[i]!=Moto1[i].max_iout)
		{
			PID_Change(&Moto1[i], pid_mode[i], &P_I_D[i], Maxout[i], Maxiout[i]);
//			PID_clear(&Moto1[i]);
//			PID_init(&Moto1[i], pid_mode[i], &P_I_D[i], Maxout[i], Maxiout[i]);
		}
	}
}

/***********************************************************
函数名：Measure_EXT1_Process
功能：测速模块线中断1 里侧光电门中断
参数：None
***********************************************************/
void Measure_EXT1_Process()
{
	HAL_TIM_Base_Start_IT(&htim1);
}

/***********************************************************
函数名：Measure_EXT2_Process
功能：测速模块线中断1 外侧光电门中断
参数：None
***********************************************************/
void Measure_EXT2_Process()
{
	measuredSpeed = __HAL_TIM_GET_COUNTER(&htim1);
	TIM1->CNT = 0;
	HAL_TIM_Base_Stop_IT(&htim1);
	printf("%f\n",60.0f/(float)(measuredSpeed/100));
}
