#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "Remote_Control.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "CAN_Receive.h"
#include "user_lib.h"

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

#define CHASSIS_RC_DEADLINE 10

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.2f
#define MOTOR_DISTANCE_TO_CENTER   0.32f //0.32f



//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//选择底盘状态 开关通道号
#define MODE_CHANNEL 0



#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f



//底盘电机最大速度
#define MAX_WHEEL_SPEED 4.5f

#define NORMAL_MAX_CHASSIS_SPEED_X 3.6f
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.6f
#define NORMAL_MID_CHASSIS_SPEED_X 2.4f
#define NORMAL_MID_CHASSIS_SPEED_Y 2.4f
#define NORMAL_DOWN_CHASSIS_SPEED_X 0.45f
#define NORMAL_DOWN_CHASSIS_SPEED_Y 0.45f

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 1600000.0f
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP       2000.0f
#define M3505_MOTOR_SPEED_PID_KI       0.0f
#define M3505_MOTOR_SPEED_PID_KD       20.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP  11     //8.f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 1000.f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  5000  //8.5
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_KF 0.f
#define CHASSIS_FOLLOW_GIMBAL_F_divider 0.0
#define CHASSIS_FOLLOW_GIMBAL_F_out_limit 00.f

//底盘电机功率环PID
#define M3505_MOTOR_POWER_PID_KP 2000.f
#define M3505_MOTOR_POWER_PID_KI 85.f
#define M3505_MOTOR_POWER_PID_KD 0.f
#define M3505_MOTOR_POWER_PID_MAX_OUT 60000.0f
#define M3505_MOTOR_POWER_PID_MAX_IOUT 900.0f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.31415926535897932384626433832795f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
////底盘摇摆按键
//#define SWING_KEY KEY_PRESSED_OFFSET_C

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //底盘跟随云台
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //底盘自主
	CHASSIS_VECTOR_SPIN,
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //底盘不跟随
  CHASSIS_VECTOR_RAW,									//底盘原始控制

} chassis_mode_e;

typedef struct
{
	fp32 speed_control_down;
	fp32 speed_control_down_k;
	fp32 power_now;
	fp32 power_set;
	PidTypeDef power_pid[4];                   //功率环pid
}Power_control_D;


typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	uint16_t rc_key_time;
	fp32 get_power;
} Chassis_Motor_t;

typedef struct
{
   cap_measure_t  *get_cap_measure;	
} CAN_power;

typedef struct
{
	fp32 totalCurrentTemp;
	fp32 current[4];
	fp32 power_current[4];
	fp32 speed[4];
	fp32 POWER_MAX;
	fp32 MAX_current[4];
    fp32  SPEED_MIN;	
} Power_Control;
typedef struct
{
   const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
   const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
   const Gimbal_Motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
   CAN_power can_power;
	Power_Control  power_control;
	const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
    chassis_mode_e chassis_mode;               //底盘控制状态机
	chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
	Chassis_Motor_t motor_chassis[4];          //底盘电机数据
	
	PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
	PidTypeDef chassis_angle_pid;              //底盘跟随角度pid
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;   // 滤波数据
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx_max_speed_up;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed_up;  //前进方向最小速度 单位m/s
  fp32 vy_max_speed_up;  //左右方向最大速度 单位m/s
  fp32 vy_min_speed_up;  //左右方向最小速度 单位m/s
	
  fp32 vx_max_speed_mid;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed_mid;  //前进方向最小速度 单位m/s
  fp32 vy_max_speed_mid;  //左右方向最大速度 单位m/s
  fp32 vy_min_speed_mid;  //左右方向最小速度 单位m/s
	
	fp32 vx_max_speed_down;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed_down;  //前进方向最小速度 单位m/s
  fp32 vy_max_speed_down;  //左右方向最大速度 单位m/s
  fp32 vy_min_speed_down;  //左右方向最小速度 单位m/s
	
	fp32 vx_max_speed;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //前进方向最小速度 单位m/s
  fp32 vy_max_speed;  //左右方向最大速度 单位m/s
  fp32 vy_min_speed;  //左右方向最小速度 单位m/s
	
	fp32 vx;                         //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                         //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
	
	fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
	
	fp32 chassis_relative_angle_set; //设置相对云台控制角度
  fp32 chassis_yaw_set;
	
	Power_control_D Power_control;

  fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
  fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
  fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度
   int mode_flag;
} chassis_move_t;


extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
