#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "Remote_Control.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "CAN_Receive.h"
#include "user_lib.h"

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 2
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

#define CHASSIS_RC_DEADLINE 10

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.2f
#define MOTOR_DISTANCE_TO_CENTER   0.32f //0.32f



//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002

//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//ѡ�����״̬ ����ͨ����
#define MODE_CHANNEL 0



#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f



//���̵������ٶ�
#define MAX_WHEEL_SPEED 4.5f

#define NORMAL_MAX_CHASSIS_SPEED_X 3.6f
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.6f
#define NORMAL_MID_CHASSIS_SPEED_X 2.4f
#define NORMAL_MID_CHASSIS_SPEED_Y 2.4f
#define NORMAL_DOWN_CHASSIS_SPEED_X 0.45f
#define NORMAL_DOWN_CHASSIS_SPEED_Y 0.45f

//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 1600000.0f
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP       2000.0f
#define M3505_MOTOR_SPEED_PID_KI       0.0f
#define M3505_MOTOR_SPEED_PID_KD       20.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP  11     //8.f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 1000.f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  5000  //8.5
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_KF 0.f
#define CHASSIS_FOLLOW_GIMBAL_F_divider 0.0
#define CHASSIS_FOLLOW_GIMBAL_F_out_limit 00.f

//���̵�����ʻ�PID
#define M3505_MOTOR_POWER_PID_KP 2000.f
#define M3505_MOTOR_POWER_PID_KI 85.f
#define M3505_MOTOR_POWER_PID_KD 0.f
#define M3505_MOTOR_POWER_PID_MAX_OUT 60000.0f
#define M3505_MOTOR_POWER_PID_MAX_IOUT 900.0f

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.31415926535897932384626433832795f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f
////����ҡ�ڰ���
//#define SWING_KEY KEY_PRESSED_OFFSET_C

//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //���̸�����̨
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //��������
	CHASSIS_VECTOR_SPIN,
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //���̲�����
  CHASSIS_VECTOR_RAW,									//����ԭʼ����

} chassis_mode_e;

typedef struct
{
	fp32 speed_control_down;
	fp32 speed_control_down_k;
	fp32 power_now;
	fp32 power_set;
	PidTypeDef power_pid[4];                   //���ʻ�pid
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
   const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
   const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
   const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
   CAN_power can_power;
	Power_Control  power_control;
	const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
    chassis_mode_e chassis_mode;               //���̿���״̬��
	chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
	Chassis_Motor_t motor_chassis[4];          //���̵������
	
	PidTypeDef motor_speed_pid[4];             //���̵���ٶ�pid
	PidTypeDef chassis_angle_pid;              //���̸���Ƕ�pid
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;   // �˲�����
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx_max_speed_up;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed_up;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed_up;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed_up;  //���ҷ�����С�ٶ� ��λm/s
	
  fp32 vx_max_speed_mid;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed_mid;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed_mid;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed_mid;  //���ҷ�����С�ٶ� ��λm/s
	
	fp32 vx_max_speed_down;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed_down;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed_down;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed_down;  //���ҷ�����С�ٶ� ��λm/s
	
	fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
	
	fp32 vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
	fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
	fp32 chassis_relative_angle_set; //���������̨���ƽǶ�
  fp32 chassis_yaw_set;
	
	Power_control_D Power_control;

  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
   int mode_flag;
} chassis_move_t;


extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
