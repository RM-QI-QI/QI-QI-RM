/***********************************************************
文件名：chassis_task.h
描述：
		FreeRTOS-底盘任务，控制飞镖发射架整体运动
From:Qi-Q@Rjgawuie
***********************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "pid.h"


//以下为云台参数的默认值
#define Length_Chassis 660.0   //底盘长度
#define Length_Launch 448.0    //发射平面长度
#define Length_Brace 340.0     //云台支撑杆长度
#define Pitch_Error 3.58f-3.6f    //PITCH轴转角误差

#define Screw_Pitch 5.0f					//丝杆牙距

#define MAX_Length 250.5       //滑轨行程
#define MIN_Length 0.0

typedef struct
{
	uint16_t updatemark;
	uint16_t yaw_lastangle;//yaw上一次机械角
	int32_t yaw_sum_angle;//yaw角度积分
	int32_t pitch_sum_step;
	int RunMode;      //运行模式
	int pitchcorrectmark; //是否进行pitch轴校正
	
}Backup_Typedef;

extern __IO Backup_Typedef *BKP_Handler;  //指向备份RAM的指针

extern int RunMode;

extern double length_chassis,
							length_launch,
							length_brace,
							pitch_error,
							max_length,
							min_length,
							screw_pitch;

extern pid_type_def Moto1[12];
extern volatile fp32 Angle_target[3];
extern double Angle_Pitch;

extern uint8_t pid_mode[12];

extern PID_Value_Typedef P_I_D[12];
extern fp32 Maxout[12];
extern fp32 Maxiout[12];
extern fp32 Angle_error[3];
extern uint8_t turn_lock;	//换弹装置锁，默认使能(1使能，0失能)
extern int16_t speed[6];
extern int16_t current[6];

extern int outpost_speed[6];
extern int outpost_current[6];
extern int base_speed[6];
extern int base_current[6];

double angle2length(double angle);
double length2angle(double length);

int Pitch_AngleSet(double Angle);

void Chassis_Task_Setup(void);
void Chassis_Task_Loop(void);
void UART8_IRQProcess(void);
void Measure_EXT1_Process(void);
void Measure_EXT2_Process(void);

