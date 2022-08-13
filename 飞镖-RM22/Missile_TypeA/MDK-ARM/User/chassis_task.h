/***********************************************************
�ļ�����chassis_task.h
������
		FreeRTOS-�������񣬿��Ʒ��ڷ���������˶�
From:Qi-Q@Rjgawuie
***********************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "pid.h"


//����Ϊ��̨������Ĭ��ֵ
#define Length_Chassis 660.0   //���̳���
#define Length_Launch 448.0    //����ƽ�泤��
#define Length_Brace 340.0     //��̨֧�Ÿ˳���
#define Pitch_Error 3.58f-3.6f    //PITCH��ת�����

#define Screw_Pitch 5.0f					//˿������

#define MAX_Length 250.5       //�����г�
#define MIN_Length 0.0

typedef struct
{
	uint16_t updatemark;
	uint16_t yaw_lastangle;//yaw��һ�λ�е��
	int32_t yaw_sum_angle;//yaw�ǶȻ���
	int32_t pitch_sum_step;
	int RunMode;      //����ģʽ
	int pitchcorrectmark; //�Ƿ����pitch��У��
	
}Backup_Typedef;

extern __IO Backup_Typedef *BKP_Handler;  //ָ�򱸷�RAM��ָ��

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
extern uint8_t turn_lock;	//����װ������Ĭ��ʹ��(1ʹ�ܣ�0ʧ��)
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

