/***********************************************************
�ļ�����chassis_task.c
������
		FreeRTOS-�������񣬿��Ʒ��ڷ���������˶�
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

extern osTimerId_t TIM_STEPHandle; //�����ʱ����������Ʋ��������

uint16_t measuredSpeed;  //������ٶ�

int RunMode = 0;    //����ģʽѡ��Ĭ��PIDģʽ

double length_chassis = Length_Chassis,
			 length_launch = Length_Launch,
			 length_brace = Length_Brace,
			 pitch_error = Pitch_Error,
			 max_length = MAX_Length,
			 min_length = MIN_Length,
			 screw_pitch = Screw_Pitch;

pid_type_def Moto1[12];      //���PID����

volatile fp32 Angle_target[3]={0.0f,0.0f,0.0f};
double Angle_Pitch;		//�趨��Pitch��Ƕ�

void change_all_para(void);


PID_Value_Typedef P_I_D[12]= {1.5f,0.0f,0.5f,        //PID����
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															1.5f,0.0f,0.5f,
															80.0f,0.0f,20.0f,     //�ǶȻ�
															80.0f,0.0f,20.0f,
															110.0f,0.0f,20.0f};
fp32 Maxout[12]={16000,16000,16000,16000,16000,16000,16000,16000,16000,5000,5000,8000};
fp32 Maxiout[12]={16000,16000,16000,16000,16000,16000,16000,16000,16000,5000,5000,8000};

fp32 Angle_error[3]={1.65f,1.65f,1.65f};
uint8_t pid_mode[12]={PID_POSITION, PID_POSITION, PID_POSITION,
									PID_POSITION, PID_POSITION, PID_POSITION,
									PID_POSITION, PID_POSITION, PID_POSITION,
									PID_POSITION, PID_POSITION, PID_POSITION};

uint8_t turn_lock = 1;		//����װ������Ĭ��ʹ��(1ʹ�ܣ�0ʧ��)
int16_t speed[6]={0,0,0,0,0,0};			//�ٶ�Ŀ��ֵ���㹦�ʿ���ģʽ���ת��
int16_t current[6]={0,0,0,0,0,0};		//����Ŀ��ֵ

int outpost_speed[6]={0,0,0,0,0,0};			//��ǰ��վ�ٶ�Ŀ��ֵ���㹦�ʿ���ģʽ���ת��
int outpost_current[6]={0,0,0,0,0,0};		//����Ŀ��ֵ
int base_speed[6]={0,0,0,0,0,0};			//������ٶ�Ŀ��ֵ���㹦�ʿ���ģʽ���ת��
int base_current[6]={0,0,0,0,0,0};		//����Ŀ��ֵ

__IO Backup_Typedef *BKP_Handler = (__IO Backup_Typedef *)(BKPSRAM_BASE); //ָ�򱸷�RAM��ָ��

/***********************************************************
��������angle2length
���ܣ��Ƕȵ����̾���Ľ���
������angle Ŀ��Ƕ�(�Ƕ���)
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
��������length2angle
���ܣ����̾��뵽�ǶȵĽ���
������length ֧����Ŀǰλ��
***********************************************************/
double length2angle(double length)
{
	double C=length_chassis-length;
	return acos((length_launch*length_launch+C*C-length_brace*length_brace)/2/length_launch/C)*57.3+pitch_error;
}

/***********************************************************
��������Pitch_AngleSet
���ܣ�����Pitch�ᵽָ���Ƕ�
������Angle		Ҫ�趨�ĽǶ�
����ֵ��-1		ʧ�ܣ��Ƕȿ��ܳ���������Χ
				0			�ɹ�
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
��������Chassis_Task_Setup
���ܣ����������ʼ�����֣�ֻ���ڿ�ʼʱ����һ�Σ�
������None
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
��������Chassis_Task_Loop
���ܣ���������ѭ������
������None
***********************************************************/
void Chassis_Task_Loop()
{
	if(RunMode == 0)             //PID����ģʽ
	{
	//	CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_LOW,                          //ң��������
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
	else if(RunMode == 1)                    //�㹦��ģʽ
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
	BKP_Handler->updatemark = 1; //�򱸷����һλд��1���������ݸ���
	

	osDelay(1);
	

	
	for(int i=0;i<12;i++)                                 //����PID����
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
��������Measure_EXT1_Process
���ܣ�����ģ�����ж�1 ��������ж�
������None
***********************************************************/
void Measure_EXT1_Process()
{
	HAL_TIM_Base_Start_IT(&htim1);
}

/***********************************************************
��������Measure_EXT2_Process
���ܣ�����ģ�����ж�1 ��������ж�
������None
***********************************************************/
void Measure_EXT2_Process()
{
	measuredSpeed = __HAL_TIM_GET_COUNTER(&htim1);
	TIM1->CNT = 0;
	HAL_TIM_Base_Stop_IT(&htim1);
	printf("%f\n",60.0f/(float)(measuredSpeed/100));
}
