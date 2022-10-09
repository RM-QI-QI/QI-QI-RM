/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


//����ʼ����һ��ʱ��
#define SHOOT_TASK_INIT_TIME 300
//��̨ģʽʹ�õĿ���ͨ��
#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f
//���������Ƽ�� 2ms
#define SHOOT_CONTROL_TIME_MS 2
//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
//�������ʱ��
#define BUTTON_TIME                 30
//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15
//���ʱ��
#define SHOOT_TIME                  100
//��곤���ж�
#define PRESS_LONG_TIME             400
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000040372843107647f //-3.14--3.14
#define MOTOR_ECD_TO_ANG          	0.002313193556470837f //-180--180
#define FULL_COUNT                  19

#define KEY_OFF_JUGUE_TIME          500

#define TRIGGER_DONE_TIME           500

#define READY           						1
#define OFF          								0


//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  1000
#define REVERSE_TIME                1000
#define REVERSE_SPEED_LIMIT         13.0f

#define SHOOT_DISABLE_TIME					200

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//�����ֵ��PID
#define TRIGGER_ANGLE_PID_KP        1800.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        10.0f

#define TRIGGER_READY_PID_MAX_OUT   15000.0f
#define TRIGGER_READY_PID_MAX_IOUT  10000.0f


//Ħ���ֵ���ٶȻ�PID
#define FRIC_S_MOTOR_SPEED_PID_KP 20.0f
#define FRIC_S_MOTOR_SPEED_PID_KI 1.0f
#define FRIC_S_MOTOR_SPEED_PID_KD 0.0f

#define FRIC_S_MOTOR_SPEED_PID_MAX_OUT  30000.0f
#define FRIC_S_MOTOR_SPEED_PID_MAX_IOUT 20000.0f

#define FRIC_LEFT_MOTOR_SPEED_PID_KP 20.0f
#define FRIC_LEFT_MOTOR_SPEED_PID_KI 4.0f
#define FRIC_LEFT_MOTOR_SPEED_PID_KD 0.0f

#define FRIC_LEFT_MOTOR_SPEED_PID_MAX_OUT  16385.0f
#define FRIC_LEFT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define FRIC_RIGHT_MOTOR_SPEED_PID_KP 20.0f
#define FRIC_RIGHT_MOTOR_SPEED_PID_KI 4.0f
#define FRIC_RIGHT_MOTOR_SPEED_PID_KD 0.0f

#define FRIC_RIGHT_MOTOR_SPEED_PID_MAX_OUT  16385.0f
#define FRIC_RIGHT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define SHOOT_HEAT_REMAIN_VALUE     80

#define SHOOT_THIRD_MODE 0


typedef struct
{
    const RC_ctrl_t *shoot_rc;
		const motor_measure_t *trigger_motor_measure;
		const motor_measure_t *fric_b_motor_measure;
		const motor_measure_t *fric_left_motor_measure;
		const motor_measure_t *fric_right_motor_measure;
    pid_type_def trigger_pid;
		pid_type_def bullet_pid;
		pid_type_def fric_left_pid;
		pid_type_def fric_right_pid;
	  fp32 trigger_speed;
    fp32 trigger_speed_set;
		fp32 fric_b_speed;
    fp32 fric_b_speed_set;
		fp32 fric_left_speed;
    fp32 fric_left_speed_set;
		fp32 fric_right_speed;
    fp32 fric_right_speed_set;
    fp32 trigger_angle;
    fp32 trigger_angle_set;
    int16_t trigger_given_current;
    int8_t trigger_ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;				//�жϿ���ʱ��
    uint16_t reverse_time;			//�˵�ʱ��
    uint16_t heat_limit;
    uint16_t heat;							//����
			
		uint16_t shoot_time;				//����ʱ��
		
		bool_t shoot_flag;          //����
		bool_t bullet_flag;					//��������
		bool_t shoot_continu_flag;	//�˵�
		bool_t stuck_flag;					//����
		
} shoot_control_t;

extern void shoot_task(void const *pvParameters);

#endif
