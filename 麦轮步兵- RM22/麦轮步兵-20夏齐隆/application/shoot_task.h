#ifndef SHOOT_H
#define SHOOT_H
#include "main.h"
#include "CAN_Receive.h"
#include "remote_control.h"
#include "user_lib.h"
#include "Gimbal_Task.h"
//������俪��ͨ������
#define Shoot_RC_Channel    1
//��̨ģʽʹ�õĿ���ͨ��
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  0.002

//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000

//��곤���ж�
#define PRESS_LONG_TIME 400
//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E


//�����ٶ�
#define MAX_SPEED 15.0f //-12.0f
#define MID_SPEED 12.0f //-12.0f
#define MIN_SPEED 10.0f //-12.0f
#define Ready_Trigger_Speed 6.0f

//���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

//�����ֵ��PID
#define TRIGGER_ANGLE_PID_KP 900///2450.0f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 100.0f

#define TRIGGER_READY_PID_MAX_OUT 8000.0f
#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

//3508����ٶȻ�PID
#define S3505_MOTOR_SPEED_PID_KP 8700.f
#define S3505_MOTOR_SPEED_PID_KI  0.0f
#define S3505_MOTOR_SPEED_PID_KD  10.f
#define S3505_MOTOR_SPEED_PID_MAX_OUT 11000.0f
#define S3505_MOTOR_SPEED_PID_MAX_IOUT 1000.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Three 1.0466666666666f
#define PI_Ten 0.314f

#define TRIGGER_SPEED 3.0f
#define SWITCH_TRIGGER_ON 0

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
	SHOOT_BULLET_ONE,
	SHOOT_DONE,
	SHOOT_AUTO,
} shoot_mode_e;

typedef struct
{
	const motor_measure_t *shoot_motor_measure;
	fp32 speed;
	fp32 speed_set;
	fp32 angle;
	int8_t ecd_count;
	fp32 set_angle;
  int16_t given_current;
	
	bool_t press_l;
	bool_t press_r;
	bool_t last_press_l;
	bool_t last_press_r;
	uint16_t press_l_time;
	uint16_t press_r_time;
	uint16_t rc_s_time;
	
	uint32_t run_time;
	uint32_t cmd_time;
	int16_t move_flag;
	int16_t move_flag_ONE;
	bool_t key;
	int16_t BulletShootCnt;
	int16_t last_butter_count;
  fp32 shoot_CAN_Set_Current;
    fp32  blocking_angle_set;
	fp32  blocking_angle_current;
	int8_t blocking_ecd_count;
} Shoot_Motor_t;

typedef struct
{
  const motor_measure_t *fric_motor_measure;
	fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	uint16_t rc_key_time;
} fric_Motor_t;

typedef struct
{
	const RC_ctrl_t *shoot_rc;                   //ң����ָ��
  shoot_mode_e fric_mode;               //�������״̬��
	shoot_mode_e last_fric_mode;          //����ϴο���״̬��
	fric_Motor_t motor_fric[2];          //����������
	fp32 fric_CAN_Set_Current[2];
	PidTypeDef motor_speed_pid[4];             //�������ٶ�pid
	
  first_order_filter_type_t fric1_cmd_slow_set_speed;  // �˲�����
  first_order_filter_type_t fric2_cmd_slow_set_speed;  // �˲�����
	fp32 angle[2];
	int16_t ecd_count[2];
  int16_t given_current[2];
  fp32 set_angle[2];
	fp32 speed[2];
	fp32 speed_set[2];	
	fp32 current_set[2];	
	bool_t move_flag;

	fp32 min_speed;
	fp32 max_speed;
	int flag[2];
	int laster_add;
} fric_move_t;


//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_init(void);
extern void shoot_control_loop(void);
void shoot_task(void const *pvParameters);
#endif
