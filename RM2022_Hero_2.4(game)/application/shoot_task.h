/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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


//任务开始空闲一段时间
#define SHOOT_TASK_INIT_TIME 300
//云台模式使用的开关通道
#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f
//射击任务控制间隔 2ms
#define SHOOT_CONTROL_TIME_MS 2
//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
//按键检测时间
#define BUTTON_TIME                 30
//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//射击时间
#define SHOOT_TIME                  100
//鼠标长按判断
#define PRESS_LONG_TIME             400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000040372843107647f //-3.14--3.14
#define MOTOR_ECD_TO_ANG          	0.002313193556470837f //-180--180
#define FULL_COUNT                  19

#define KEY_OFF_JUGUE_TIME          500

#define TRIGGER_DONE_TIME           500

#define READY           						1
#define OFF          								0


//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  1000
#define REVERSE_TIME                1000
#define REVERSE_SPEED_LIMIT         13.0f

#define SHOOT_DISABLE_TIME					200

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        1800.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        10.0f

#define TRIGGER_READY_PID_MAX_OUT   15000.0f
#define TRIGGER_READY_PID_MAX_IOUT  10000.0f


//摩擦轮电机速度环PID
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

    uint16_t block_time;				//判断卡弹时间
    uint16_t reverse_time;			//退弹时间
    uint16_t heat_limit;
    uint16_t heat;							//热量
			
		uint16_t shoot_time;				//发弹时间
		
		bool_t shoot_flag;          //发射
		bool_t bullet_flag;					//二级拨弹
		bool_t shoot_continu_flag;	//退单
		bool_t stuck_flag;					//卡弹
		
} shoot_control_t;

extern void shoot_task(void const *pvParameters);

#endif
