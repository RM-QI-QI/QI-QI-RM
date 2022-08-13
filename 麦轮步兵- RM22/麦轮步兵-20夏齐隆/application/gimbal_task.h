#ifndef GIMBALTASK_H
#define GIMBALTASK_H

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "kalman.h"

//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
//��̨��������
#define GIMBAL_CONTROL_TIME 1

extern fp32 contral;
//yaw �ٶȻ� PID�����Լ� PID���������������
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


//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
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

//yaw �ǶȻ� �Ƕ��ɱ����� PID������0�� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP 35.f  //78.6432--62.91456
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f//0.118098f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f//10
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 1500.f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 50.f

//pitch �ٶȻ� PID�����Լ� PID���������������
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

//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP 80.f  //15.0
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 30000.f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.f

//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID�����������������
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



//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.5f
#define INIT_YAW_SET 0.0f//2.81792283f
#define INIT_PITCH_SET 0.0f
//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000
//��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define GIMBAL_CALI_MOTOR_SET 6000 //8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5


//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_deadband  10	
//#define RC_Pit_deadband 0.5;

#define AUTO_deadband 1
//yaw��pitch�Ƕ���ң�����������
#define Yaw_RC_SEN -0.000005f
#define Pitch_RC_SEN -0.000005f //0.005

//yaw,pitch�ǶȺ��������ı���
#define Yaw_Mouse_Sen   0.000050f   //80
#define Pitch_Mouse_Sen  0.000010f  //35
#define Z_Mouse_Sen       0.00010f  //10

#define Pitch_Mouse_Sen_Gro  0.000023f  //35
//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0

//�������ֵ����Լ���ֵ
#define Half_ecd_range 395  //395  7796
#define ecd_range 8191
#define GIMBAL_ACCEL_X_NUM  170.6f
#define GIMBAL_ACCEL_Y_NUM 133.3f
#define GIMBAL_ACCEL_Y_GYRO_NUM 170.3f
#define GIMBAL_ACCEL_Z_NUM 170.3f
//��̨������Ƽ�� 0.002s
//#define GIMBAL_CONTROL_TIME 0.002
//�������ֵת���ɽǶ�ֵ

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
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
	GIMBAL_MOTOR_ENCONDE,  //����������Ƕȿ���
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
	
		fp32 integral_uplimit;			//�������򿹱���
	  fp32 integral_downlimit;		//���ָ��򿹱���

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	  fp32 Fout;
	
//	  fp32 max_err;
//	  fp32 mid_err;
//		fp32 min_err;
	
	  fp32 F_divider;//ǰ������
		fp32 F_out_limit;//ǰ���޷�
		
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
		first_order_filter_type_t gimbal_cmd_slow_set_vx;   // �˲�����
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


/***********�Ӿ�*****/
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
	
typedef struct //�Ӿ�Ŀ���ٶȲ���
{
    int delay_cnt; //����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
    int freq;
    int last_time;         //�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
    float last_position;   //�ϸ�Ŀ��Ƕ�
    float speed;           //�ٶ�
    float last_speed;      //�ϴ��ٶ�
    float processed_speed; //�ٶȼ�����
} speed_calc_data_t;

typedef struct
{
    uint32_t WorldTime;      //����ʱ��
    uint32_t Last_WorldTime; //��һ������ʱ��
} WorldTime_RxTypedef;

typedef struct
{
    float Speed_Gain; //�ٶ�����
} VisionFourth_Ring_t;

typedef struct
{
    Kalman_Data_t *Kalman_Data;
    VisionFourth_Ring_t *VisionFourth_Ring;
} All_VisionCompensate_t;

/***********�Ӿ�*****/


/*******����***/
typedef struct
{
	float count;//��ǰ�����ٶ�ֵ
	float rate;//ÿһ�ε��ӵ��ٶ�ֵ
	fp32 mincount;
	fp32 maxcount;
}SpeedRamp_t;

typedef struct
{
    
         float YawAngle_Error;   //������YAW�ǶȲ�
            float PitchAngle_Error; //������Pitch�ǶȲ�
       
        uint8_t Angle_Error_Data[8];
   

    int Gyro_z;           //�����Ǽ��ٶ�С�������λ
    

} VisionSend_IMU_t;
typedef struct
{
    fp32 FinalOffset;        //���ո�����̨pid �����ƫ��ֵ��
    fp32 FinalOffset_Last;   //���ո�����̨pid �����ƫ��ֵ��
    fp32 FinalSpeed;         //���ո�����̨pid �����ƫ��ֵ������ٶȡ�
    bool AbleToFire;         //��ʾĿ���Ѿ���׼������ֱ�����
    float FinalOffset_depth; //�����Ӿ����
    VisionSend_IMU_t *VisionSend_IMU;

} VisionExportData_t;

/*******����***/
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hand_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
void gimbal_task(void const *pvParameters);
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS);
//������俪��ͨ������
#define Shoot_RC_Channel    1
//��̨ģʽʹ�õĿ���ͨ��
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  0.002

//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f
//��곤���ж�
#define PRESS_LONG_TIME 400
//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E

#define QE_SPEED  0.0004
//�����ٶ�
#define MAX_SPEED 15.0f //-12.0f
#define MID_SPEED 12.0f //-12.0f
#define MIN_SPEED 10.0f //-12.0f
#define Ready_Trigger_Speed 6.0f

//���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

////�����ֵ��PID
//#define TRIGGER_ANGLE_PID_KP 900.0f
//#define TRIGGER_ANGLE_PID_KI 0.0f
//#define TRIGGER_ANGLE_PID_KD 0.0f

//#define TRIGGER_READY_PID_MAX_OUT 5000.0f
//#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

//#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
//#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

////3508����ٶȻ�PID
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
