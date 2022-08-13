#ifndef GIMBALTASK_H
#define GIMBALTASK_H

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "kalman.h"

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//云台控制周期
#define GIMBAL_CONTROL_TIME 1

extern fp32 contral;
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP  980//
#define YAW_SPEED_PID_KI 0.00f//
#define YAW_SPEED_PID_KD 1.0f// 1-20
#define YAW_SPEED_PID_KF 350
#define YAW_SPEED_PID_F_divider 10.f
#define YAW_SPEED_PID_F_out_limit 10000.0f
#define YAW_SPEED_PID_I_uplimit 0.0f
#define YAW_SPEED_PID_I_downlimit 0.0F
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 1000.0f
//#define YAW_SPEED_PID_MAX_ERR 50.0f
//#define YAW_SPEED_PID_MID_ERR 2.0f
//#define YAW_SPEED_PID_MIN_ERR 0.1f6


//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP  35.f   //
#define YAW_GYRO_ABSOLUTE_PID_KI  0 //0.0014f //0.0000000001f
#define YAW_GYRO_ABSOLUTE_PID_KD  0.1f     //0.4-1
#define YAW_GYRO_ABSOLUTE_PID_KF 0.f
#define YAW_GYRO_ABSOLUTE_PID_F_divider 0.0f
#define YAW_GYRO_ABSOLUTE_PID_F_out_limit 0.0f
//#define YAW_GYRO_ABSOLUTE_PID_I_uplimit 0.0f
//#define YAW_GYRO_ABSOLUTE_PID_I_downlimit 0.0F
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 1000.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.5f

//yaw 角度环 角度由编码器 PID参数以0及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 35.f  //78.6432--62.91456
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f//0.118098f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f//10
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 1500.f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 50.f

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 1000.0f  //2000
#define PITCH_SPEED_PID_KI 0.0f    //20
#define PITCH_SPEED_PID_KD 10.0f
#define PITCH_SPEED_PID_KF 0.0f
#define PITCH_SPEED_PID_F_divider 0.0f
#define PITCH_SPEED_PID_F_out_limit 0.0f
#define PITCH_SPEED_PID_I_uplimit 5000.0f
#define PITCH_SPEED_PID_I_downlimit -5000.0F
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 5000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 80.f  //15.0
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 30000.f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.f

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出、
#define PITCH_ENCODE_RELATIVE_PID_KP 180.0f  //15.0
#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.f
#define PITCH_ENCODE_RELATIVE_PID_KF 0.0f
#define PITCH_ENCODE_RELATIVE_PID_F_divider 0.0f
#define PITCH_ENCODE_RELATIVE_PID_F_out_limit 0.0f
#define PITCH_ENCODE_RELATIVE_PID_I_uplimit 5000.0f
#define PITCH_ENCODE_RELATIVE_PID_I_downlimit -5000.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 1000.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 1.0f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5



//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.5f
#define INIT_YAW_SET 0.0f//2.81792283f
#define INIT_PITCH_SET 0.0f
//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000
//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 6000 //8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5


//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband  10	
//#define RC_Pit_deadband 0.5;

#define AUTO_deadband 1
//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN -0.000005f
#define Pitch_RC_SEN -0.000005f //0.005

//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_Sen   0.000050f   //80
#define Pitch_Mouse_Sen  0.000010f  //35
#define Z_Mouse_Sen       0.00010f  //10

#define Pitch_Mouse_Sen_Gro  0.000023f  //35
//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0

//电机码盘值最大以及中值
#define Half_ecd_range 395  //395  7796
#define ecd_range 8191
#define GIMBAL_ACCEL_X_NUM  170.6f
#define GIMBAL_ACCEL_Y_NUM 133.3f
#define GIMBAL_ACCEL_Y_GYRO_NUM 170.3f
#define GIMBAL_ACCEL_Z_NUM 170.3f
//云台任务控制间隔 0.002s
//#define GIMBAL_CONTROL_TIME 0.002
//电机编码值转化成角度值

#define c  1000
#define k  120
#define EPLISON  20
#define DELAT  200
#define MAXOUT  16000

#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000767084019045f //      2*  PI  /8192

#endif

typedef enum				
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
	GIMBAL_MOTOR_ENCONDE,  //电机编码器角度控制
	GIMBAL_AUTO_E,
} gimbal_motor_mode_e;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;
	  fp32 kf;

	
    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;
	
		fp32 integral_uplimit;			//积分正向抗饱和
	  fp32 integral_downlimit;		//积分负向抗饱和

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	  fp32 Fout;
	
//	  fp32 max_err;
//	  fp32 mid_err;
//		fp32 min_err;
	
	  fp32 F_divider;//前馈分离
		fp32 F_out_limit;//前馈限幅
		
    fp32 out;
		
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    Gimbal_PID_t gimbal_motor_absolute_angle_pid;
    Gimbal_PID_t gimbal_motor_relative_angle_pid;
    PidTypeDef gimbal_motor_gyro_pid;
	smc_type_def gimbal_sac_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

	  fp32 control;
    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
	fp32 absolute_angle_set_tras;
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
	int ZERO_ECD ;
	int ZERO_ECD_flag;
	int LAST_ZERO_ECD;
} Gimbal_Motor_t;

typedef struct
{
	fp32 yaw_offset;
	fp32 pitch_offset;
	fp32 time;
	fp32 yaw_set;
	fp32 pitch_set;
	fp32 shoot_ready;
}Gimbal_Auto;
typedef struct
{        
	  const RC_ctrl_t *gimbal_rc_ctrl;
	  const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    Gimbal_Motor_t gimbal_yaw_motor;
	Gimbal_Motor_t gimbal_roll_motor;
    Gimbal_Motor_t gimbal_pitch_motor;
	  Gimbal_Cali_t gimbal_cali;
	Gimbal_Auto gimbal_Auto;
		first_order_filter_type_t gimbal_cmd_slow_set_vx;   // 滤波数据
		first_order_filter_type_t gimbal_cmd_slow_set_vy;
	    first_order_filter_type_t gimbal_cmd_slow_set_vy_gyro;
	    first_order_filter_type_t gimbal_cmd_slow_set_vz;
	
	    first_order_filter_type_t gimbal_cmd_slow_set_vx_RC;
	    first_order_filter_type_t gimbal_cmd_slow_set_vy_RC;
	    first_order_filter_type_t gimbal_cmd_slow_set_vx_auto;
	    first_order_filter_type_t gimbal_cmd_slow_set_vy_auto;
    fp32 SPIN;
	fp32 AUTO;
} Gimbal_Control_t;


/***********视觉*****/
#define Kalman_YawDataInit      \
    {                           \
        0,                      \
            0,                  \
            0,                  \
            0,                  \
            {850, 0.1, 10, 50}, \
    }

#define Kalman_PitchDataInit   \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            {0, 0.03, 5, 250}, \
    }
	
typedef struct //视觉目标速度测量
{
    int delay_cnt; //计算相邻两帧目标不变持续时间,用来判断速度是否为0
    int freq;
    int last_time;         //上次受到目标角度的时间
    float last_position;   //上个目标角度
    float speed;           //速度
    float last_speed;      //上次速度
    float processed_speed; //速度计算结果
} speed_calc_data_t;

typedef struct
{
    uint32_t WorldTime;      //世界时间
    uint32_t Last_WorldTime; //上一次世界时间
} WorldTime_RxTypedef;

typedef struct
{
    float Speed_Gain; //速度增益
} VisionFourth_Ring_t;

typedef struct
{
    Kalman_Data_t *Kalman_Data;
    VisionFourth_Ring_t *VisionFourth_Ring;
} All_VisionCompensate_t;

/***********视觉*****/


/*******补偿***/
typedef struct
{
	float count;//当前的总速度值
	float rate;//每一次叠加的速度值
	fp32 mincount;
	fp32 maxcount;
}SpeedRamp_t;

typedef struct
{
    
         float YawAngle_Error;   //陀螺仪YAW角度差
            float PitchAngle_Error; //陀螺仪Pitch角度差
       
        uint8_t Angle_Error_Data[8];
   

    int Gyro_z;           //陀螺仪加速度小数点后两位
    

} VisionSend_IMU_t;
typedef struct
{
    fp32 FinalOffset;        //最终赋给云台pid 计算的偏差值。
    fp32 FinalOffset_Last;   //最终赋给云台pid 计算的偏差值。
    fp32 FinalSpeed;         //最终赋给云台pid 计算的偏差值算出的速度。
    bool AbleToFire;         //表示目标已经瞄准，可以直接射击
    float FinalOffset_depth; //最终视觉深度
    VisionSend_IMU_t *VisionSend_IMU;

} VisionExportData_t;

/*******补偿***/
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hand_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
void gimbal_task(void const *pvParameters);
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS);
//射击发射开关通道数据
#define Shoot_RC_Channel    1
//云台模式使用的开关通道
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  0.002

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f
//鼠标长按判断
#define PRESS_LONG_TIME 400
//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E

#define QE_SPEED  0.0004
//拨弹速度
#define MAX_SPEED 15.0f //-12.0f
#define MID_SPEED 12.0f //-12.0f
#define MIN_SPEED 10.0f //-12.0f
#define Ready_Trigger_Speed 6.0f

//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

////拨弹轮电机PID
//#define TRIGGER_ANGLE_PID_KP 900.0f
//#define TRIGGER_ANGLE_PID_KI 0.0f
//#define TRIGGER_ANGLE_PID_KD 0.0f

//#define TRIGGER_READY_PID_MAX_OUT 5000.0f
//#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

//#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
//#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

////3508电机速度环PID
//#define S3505_MOTOR_SPEED_PID_KP 15000.0f
//#define S3505_MOTOR_SPEED_PID_KI 0.0f
//#define S3505_MOTOR_SPEED_PID_KD 0.0f
//#define S3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
//#define S3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Three 1.0466666666666f
#define PI_Ten 0.314f

#define TRIGGER_SPEED 3.0f
#define SWITCH_TRIGGER_ON 0


#endif
