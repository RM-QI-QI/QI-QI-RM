#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "main.h"
#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rm_usart.h"
#include "stm32.h"
#include "stm32_private.h"
#include "bsp_servo_pwm.h"
#include "kalman.h"
#include "referee.h"
#define int_abs(x) ((x) > 0 ? (x) : (-x))
#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//The coding value of the motor is regular 0-8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }
#define Motor_Ecd_to_rad 0.00076708402f
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

Gimbal_Control_t gimbal_control;

static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
	
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);
	
static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);
	
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
	
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
	
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
	
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
	
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
	
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);
	
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);

static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);

static void GIMBAL_relative_angle_limit_yaw(Gimbal_Motor_t *gimbal_motor, fp32 add);

//static void gimbal_motor_yaw_auto_control(Gimbal_Motor_t *gimbal_motor);

//static void gimbal_motor_pitch_auto_control(Gimbal_Motor_t *gimbal_motor);

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);

static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);

static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);

static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

//static fp32 Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
void SERIO_Control(void);
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS);
fp32 RAMP_fP(float final, float now, float ramp);
const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern ExtY_stm32 stm32_Y; 
extern ExtY_stm32 stm32_Y_pitch; 
extern ExtY_stm32 stm32_Y_auto; 
extern ExtY_stm32 stm32_Y_pitch_auto; 
int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0;
extern gimbal_behaviour_e gimbal_behaviour;
float MAX_PITCH=5112*Motor_Ecd_to_Rad;  float MIN_PITCH=3842*Motor_Ecd_to_Rad;
extern ext_game_robot_state_t robot_state;

VisionSend_IMU_t VisionSend_IMU;
#define VisionExportDataGroundInit \
    {                              \
        {0},                       \
            {0},                   \
            {0},                   \
            0,                     \
            0,                     \
            &VisionSend_IMU,       \
    }
VisionExportData_t VisionExportData = VisionExportDataGroundInit;
/*************卡尔曼结构体**************/
kalman auto_yaw;
kalman auto_pitch;

VisionFourth_Ring_t VisionFourth_Ring_Yaw = {0};
VisionFourth_Ring_t VisionFourth_Ring_Pitch = {0};

Kalman_Data_t Kalman_YawData = Kalman_YawDataInit;
Kalman_Data_t Kalman_PitchData = Kalman_PitchDataInit;

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;

WorldTime_RxTypedef Vision_WorldTime;
WorldTime_RxTypedef VisionKF_TIME;

speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

All_VisionCompensate_t All_VisionComYaw = {&Kalman_YawData, &VisionFourth_Ring_Yaw};
All_VisionCompensate_t All_VisionComPitch = {&Kalman_PitchData, &VisionFourth_Ring_Pitch};

SpeedRamp_t KalmanRamp;
/*************卡尔曼结构体**************/
extern ext_game_robot_state_t robot_state;
int PWM=1200;
WorldTime_RxTypedef Control_WorldTime;
uint32_t Robot_FPS;
void gimbal_task(void const *pvParameters)
{
	  //Wait for gyro mission to update gyro data
  vTaskDelay(GIMBAL_TASK_INIT_TIME);
	gimbal_control.AUTO=0;
	gimbal_control.gimbal_yaw_motor.ZERO_ECD=gimbal_control.gimbal_yaw_motor.ZERO_ECD_flag=6138;
	gimbal_control.gimbal_pitch_motor.ZERO_ECD=4517;
  GIMBAL_Init(&gimbal_control);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,PWM);
	  while (1)			
   {   SERIO_Control();

       GIMBAL_Set_Mode(&gimbal_control);                    
       GIMBAL_Mode_Change_Control_Transit(&gimbal_control); 
	     GIMBAL_Feedback_Update(&gimbal_control);             
		   GIMBAL_Set_Contorl(&gimbal_control);                 
       GIMBAL_Control_loop(&gimbal_control);                
      
	   
		  Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
	    Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.given_current;
	   
      CAN_cmd_gimbal(Yaw_Can_Set_Current, Pitch_Can_Set_Current,0);
      vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
 }
}

    fp32 Pitch_speed_pid[8] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD, PITCH_SPEED_PID_KF,PITCH_SPEED_PID_I_uplimit,PITCH_SPEED_PID_I_downlimit,PITCH_SPEED_PID_F_divider,PITCH_SPEED_PID_F_out_limit};
    fp32   Yaw_speed_pid[8] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD, YAW_SPEED_PID_KF,YAW_SPEED_PID_I_uplimit,YAW_SPEED_PID_I_downlimit,YAW_SPEED_PID_F_divider,YAW_SPEED_PID_F_out_limit};
int B_flag=0;
void SERIO_Control(void)
{
	if(gimbal_control.gimbal_rc_ctrl->key.v &KEY_PRESSED_OFFSET_B&&B_flag==0)
	{   B_flag=1;
		if(PWM==1200)
		{
		PWM=2500;
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,PWM);
		}
       else if(PWM==2500)
	   {
		   PWM=1200;
       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,PWM);
	   }		   
	}
	if((gimbal_control.gimbal_rc_ctrl->key.v &KEY_PRESSED_OFFSET_B)==0)
	{
	B_flag=0;	
	}
	
}
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
		const static fp32 gimbal_x_order_filter[1] = {GIMBAL_ACCEL_X_NUM};
		const static fp32 gimbal_y_order_filter[1] = {GIMBAL_ACCEL_Y_NUM+7};
		const static fp32 gimbal_y_gyro_order_filter[1] = {GIMBAL_ACCEL_Y_GYRO_NUM};
		const static fp32 gimbal_z_order_filter[1] = {GIMBAL_ACCEL_Z_NUM};
		
		const static fp32 gimbal_x_order_filter_RC[1] = {GIMBAL_ACCEL_X_NUM};
		const static fp32 gimbal_y_order_filter_RC[1] = {GIMBAL_ACCEL_Y_NUM};
		
		const static fp32 gimbal_x_order_filter_auto[1] = {GIMBAL_ACCEL_X_NUM-50};
		const static fp32 gimbal_y_order_filter_auto[1] = {GIMBAL_ACCEL_Y_NUM};
    // Motor data pointer acquisition
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
	
    //Data Pointer acquisition of gyroscope
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_gyro_data_point();
    
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    // Initiating motor mode
    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		
    
	  GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
	  GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
	
	  GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
	  
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT); 
		
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    
	  gimbal_total_pid_clear(gimbal_init);
	
	//Generation of first-order filter instead of oblique wave function  
    first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_vx, GIMBAL_CONTROL_TIME, gimbal_x_order_filter);
    first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_vy, GIMBAL_CONTROL_TIME, gimbal_y_order_filter);
	  first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_vy_gyro, GIMBAL_CONTROL_TIME, gimbal_y_gyro_order_filter);
	  first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_vx_RC, GIMBAL_CONTROL_TIME, gimbal_x_order_filter_RC);
    first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_vy_RC, GIMBAL_CONTROL_TIME, gimbal_y_order_filter_RC);
	  first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_vz, GIMBAL_CONTROL_TIME, gimbal_z_order_filter);
	  first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_vx_auto, GIMBAL_CONTROL_TIME, gimbal_x_order_filter_auto);
    first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_vy_auto, GIMBAL_CONTROL_TIME, gimbal_y_order_filter_auto);
    //SMC_init(&gimbal_init->gimbal_yaw_motor.gimbal_sac_pid,c,k,EPLISON,DELAT,MAXOUT);
	
    GIMBAL_Feedback_Update(gimbal_init);

    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.ZERO_ECD*Motor_Ecd_to_rad;;
    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;

    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.ZERO_ECD*Motor_Ecd_to_rad;
    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;

    stm32_pid_init();
	  stm32_pid_init_pitch();
	  stm32_auto_pid_init();
	  stm32_pid_auto_init_pitch();
	  kalmanCreate(&auto_yaw,400,400);
    kalmanCreate(&auto_pitch,200,400);
	
	kalman_filter_init_t yaw_kalman_filter_para = {
    .P_data = {2, 0, 0, 2},
    .A_data = {1, 0.002 /*0.001*/, 0, 1}, //采样时间间隔
    .H_data = {1, 0, 0, 1},
    .Q_data = {1, 0, 0, 1},
    .R_data = {500, 0, 0, 1000} //500 1000
};                             //初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
    .P_data = {2, 0, 0, 2},
    .A_data = {1, 0.002 /*0.001*/, 0, 1}, //采样时间间隔
    .H_data = {1, 0, 0, 1},
    .Q_data = {1, 0, 0, 1},
    .R_data = {200, 0, 0, 400}}; //初始化pitch的部分kalman参数  200 400

	  kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
    kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	
}



static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }

      
	  gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_ROLL_ADDRESS_OFFSET);
	  gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET );//INS_YAW_ADDRESS_OFFSET
	  
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,0);                                                                                     
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd*Motor_Ecd_to_Rad;
                                                                                          
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET ))//
                                                          - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET ));//INS_GYRO_X_ADDRESS_OFFSET
	
}
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(gimbal_set_mode);
}
int PITCH_MODE=0;
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }
    
    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw_control
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
			  //enconde_control
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
	    
    }

    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro_control
		
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);	 
		
		//gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
	else if(gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_AUTO_E)
	{  
		 stm32_pid_init_pitch();
		 gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
		 gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
		 PITCH_MODE=0;
	}
	if(gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{ 
		stm32_pid_init_pitch_gyro();
		gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
		PITCH_MODE=1;
	}
	
	else if(gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{ 
		stm32_pid_init_pitch();
		gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
		PITCH_MODE=0;
	}
	
}
//static void gimbal_motor_yaw_auto_control(Gimbal_Motor_t *gimbal_motor) //auto
//{
//   stm32_step_auto(gimbal_control.gimbal_Auto.yaw_set,0, 0);
//	 gimbal_motor->current_set=stm32_Y_auto.Out1;
//   gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);	
//}
//static void gimbal_motor_pitch_auto_control(Gimbal_Motor_t *gimbal_motor)  //auto
//{
//	stm32_step_pitch_auto(gimbal_control.gimbal_Auto.pitch_set,0, 0);
//	gimbal_motor->current_set=stm32_Y_pitch_auto.Out1;
//  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
//	
//}
fp32 pitch_error=0;
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //Angle loop, speed loop cascade pid debugging
    if(gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_AUTO_E)
	{
		   
	
	//	float Vision_Speed = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Control_WorldTime.WorldTime, gimbal_motor->relative_angle_set);
	//	All_VisionComPitch.Kalman_Data->Kf_result=kalman_filter_calc(&pitch_kalman_filter,gimbal_motor->relative_angle_set,Vision_Speed);
		//All_VisionComYaw.Kalman_Data->Kf_Delay++;

//////		 if (All_VisionComYaw.Kalman_Data->Kf_result[Kf_Speed] >= 0)
//////            {
//////                kf_angle_temp = All_VisionComYaw.Kalman_Data->Parameter.Predicted_Factor * (All_VisionComYaw.Kalman_Data->Kf_result[Kf_Speed] - All_VisionComYaw.Kalman_Data->Parameter.Predicted_SpeedMin);
//////            }
//////            else if (All_VisionComYaw.Kalman_Data->Kf_result[Kf_Speed] < 0)
//////            {
//////                kf_angle_temp = All_VisionComYaw.Kalman_Data->Parameter.Predicted_Factor * (All_VisionComYaw.Kalman_Data->Kf_result[Kf_Speed] + All_VisionComYaw.Kalman_Data->Parameter.Predicted_SpeedMin);
//////            }

            //kf_angle_temp *= (VisionExportData.FinalOffset_depth / 1000.0f) / (ext_game_robot_state.data.shooter_id1_17mm_speed_limit - 4);
			 /***********补偿速度斜坡***********/
            //VAL_LIMIT(kf_angle_temp, KalmanRamp.mincount, KalmanRamp.maxcount);
            //All_VisionComYaw.Kalman_Data->Vision_Speed = RAMP_fP(kf_angle_temp, All_VisionComYaw.Kalman_Data->Vision_Speed, KalmanRamp.rate);

            /***********补偿速度斜坡 END***********/
		
       gimbal_motor->relative_angle_set=gimbal_motor->relative_angle_set;//All_VisionComPitch.Kalman_Data->Kf_result[Kf_Angle] ;//+ All_VisionComYaw.Kalman_Data->Vision_Speed;
	
	}
	if(gimbal_motor==&gimbal_control.gimbal_yaw_motor)
	{
	  gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, 0);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, 0,gimbal_motor->motor_gyro_set);
	  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);	
	}
	else if(gimbal_motor==&gimbal_control.gimbal_pitch_motor)
	{
		pitch_error=gimbal_motor->relative_angle_set-gimbal_motor->relative_angle;
	  stm32_step_pitch(gimbal_motor->relative_angle_set,gimbal_motor->relative_angle, 0);
	  gimbal_motor->current_set=stm32_Y_pitch.Out1;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
	}
}


static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * err;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}


static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = gimbal_pid_clear->Fout = 0.0f;
}


static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set_yaw;  static fp32 angle_set_pitch;
    if (gimbal_motor == NULL)
    {
        return;
    }
	  if(gimbal_motor==&gimbal_control.gimbal_yaw_motor)
	  {
			angle_set_yaw = gimbal_motor->absolute_angle_set+0.000004f; //陀螺仪问题
			gimbal_motor->absolute_angle_set = rad_format(angle_set_yaw + add);
	  }
	  else
	  {      //pitch最大俯仰角限制
			angle_set_pitch=gimbal_motor->absolute_angle_set;
			if(gimbal_motor->gimbal_motor_measure->ecd>=MAX_PITCH)
			{
				gimbal_motor->absolute_angle_set=rad_format(angle_set_pitch);
			}
			else if(gimbal_motor->gimbal_motor_measure->ecd<=MAX_PITCH)
			{
				gimbal_motor->absolute_angle_set=rad_format(angle_set_pitch);
			}
				else
			{
				gimbal_motor->absolute_angle_set = rad_format(angle_set_pitch + add);
			}
		
	}
}

float Vision_Speed=0;
float kf_angle_temp; //预测角度斜坡暂存量 
extern uint32_t deltaTime;
static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{

    if (gimbal_motor == NULL)
    {
        return;
    }

        
    //Angle loop, speed loop cascade pid debugging
    if(gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_AUTO_E)
	  {
		//Vision_Speed = Target_Speed_Calc(&Vision_Yaw_speed_Struct, VisionKF_TIME.WorldTime, gimbal_motor->absolute_angle_set);
		//All_VisionComYaw.Kalman_Data->Kf_result=kalman_filter_calc(&yaw_kalman_filter,gimbal_motor->absolute_angle_set,Vision_Speed);
		All_VisionComYaw.Kalman_Data->Kf_Delay++;
//            
//		 All_VisionComYaw.Kalman_Data->Parameter.Predicted_Factor=0.5;
//		All_VisionComYaw.Kalman_Data->Parameter.Predicted_SpeedMin=0.5; 
//		KalmanRamp.mincount=0.2; KalmanRamp.maxcount=2.5;
//		
//		 if (All_VisionComYaw.Kalman_Data->Kf_result[Kf_Speed] >= 0)
//            {
//                kf_angle_temp = All_VisionComYaw.Kalman_Data->Parameter.Predicted_Factor * (All_VisionComYaw.Kalman_Data->Kf_result[Kf_Speed] - All_VisionComYaw.Kalman_Data->Parameter.Predicted_SpeedMin);
//            }
//            else if (All_VisionComYaw.Kalman_Data->Kf_result[Kf_Speed] < 0)
//            {
//                kf_angle_temp = All_VisionComYaw.Kalman_Data->Parameter.Predicted_Factor * (All_VisionComYaw.Kalman_Data->Kf_result[Kf_Speed] + All_VisionComYaw.Kalman_Data->Parameter.Predicted_SpeedMin);
//            }
//           
//          //  kf_angle_temp *= (VisionExportData.FinalOffset_depth / 1000.0f) / (robot_state.shooter_id1_17mm_speed_limit - 4);
//			 /***********补偿速度斜坡***********/
//            VAL_LIMIT(kf_angle_temp, KalmanRamp.mincount, KalmanRamp.maxcount);
//            All_VisionComYaw.Kalman_Data->Vision_Speed = RAMP_fP(kf_angle_temp, All_VisionComYaw.Kalman_Data->Vision_Speed, KalmanRamp.rate);

            /***********补偿速度斜坡 END***********/
		
		
     gimbal_motor->absolute_angle_set=rad_format(gimbal_motor->absolute_angle_set+All_VisionComYaw.Kalman_Data->Kf_result[Kf_Angle]	);//+ All_VisionComYaw.Kalman_Data->Vision_Speed;
	}
	
	
	if(gimbal_motor==&gimbal_control.gimbal_yaw_motor)
	{
   stm32_step(gimbal_motor->absolute_angle_set,gimbal_motor->absolute_angle, 0);
	 gimbal_motor->current_set=stm32_Y.Out1;
   gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
	}
	else if(gimbal_motor==&gimbal_control.gimbal_pitch_motor)
	{
//	 gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, 
//		                                                gimbal_motor->absolute_angle, 
//	                                                gimbal_motor->absolute_angle_set, 
//		                                                gimbal_motor->motor_gyro);
//    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, 
//		                                      0, 
//		                                      gimbal_motor->motor_gyro_set);
//	 gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		
	stm32_step_pitch(gimbal_motor->absolute_angle_set,gimbal_motor->absolute_angle, 0);
	gimbal_motor->current_set=stm32_Y_pitch.Out1;
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
	}
		
		
	
	
}


//Mechanical median decoding function of encoder
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{

	int32_t relative_ecd = ecd - offset_ecd;
	
  if (relative_ecd > gimbal_control.gimbal_yaw_motor.ZERO_ECD+4096)
  {
       // relative_ecd -= 8191;
  }
	return relative_ecd*Motor_Ecd_to_Rad;
}


//Setup of GIMBAL control
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }	
	  fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
		
		if(gimbal_behaviour != GIMBAL_AUTO)
		{		
			gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);
		}
	
    else
    {
     gimbal_auto_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);   
    }		
		    
    if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {   
        gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        GIMBAL_relative_angle_limit_yaw(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }
	 else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_AUTO_E)
    { 
        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }
		   
    if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    { 
        gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    { 
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
	  else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
	  else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_AUTO_E)
    {
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
   
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
	else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_AUTO_E && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_AUTO_E)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }
	 else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_AUTO_E && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_AUTO_E)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}


static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		
    gimbal_motor->relative_angle_set += add;

    
	if( gimbal_motor->relative_angle_set==gimbal_control.gimbal_yaw_motor.relative_angle_set)
	{
		if( gimbal_motor->relative_angle_set<0)
		{
			gimbal_motor->relative_angle_set=3.1415926*2+gimbal_motor->relative_angle_set;	
		}
	  else if( gimbal_motor->relative_angle_set >2*3.1415926)
	  {
		  gimbal_motor->relative_angle_set= gimbal_motor->relative_angle_set-2*3.1415926;
	  }
  }
	
  else if( gimbal_motor->relative_angle_set==gimbal_control.gimbal_pitch_motor.relative_angle_set)
	{
		if( gimbal_motor->relative_angle_set<MIN_PITCH)
		{
			gimbal_motor->relative_angle_set=MIN_PITCH;	
		}
	  else if( gimbal_motor->relative_angle_set >MAX_PITCH)
	  {
		  gimbal_motor->relative_angle_set=MAX_PITCH;
	  }
  }
   
}


static void GIMBAL_relative_angle_limit_yaw(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		
    gimbal_motor->relative_angle_set += add;

}

static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
/**
 * @brief 计算视觉速度
 * 
 * @param S 
 * @param time 
 * @param position 
 * @return  
 */
//static fp32 Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
//{
//    S->delay_cnt++;

//    if (time != S->last_time)
//    {
//        S->speed = (position - S->last_position) /deltaTime /*(time - S->last_time) */* 2; //计算速度

//        S->processed_speed = S->speed;

//        S->last_time = time;
//        S->last_position = position;
//        S->last_speed = S->speed;
//        S->delay_cnt = 0;
//    }	

//   if (S->delay_cnt > 300 /*100*/) // delay 200ms speed = 0
//   {
//      S->processed_speed = 0; //时间过长则认为速度不变
//   }

//	 if( S->processed_speed>0.0001f) S->processed_speed=0.0001f;
//	 else if( S->processed_speed<-0.0001f) S->processed_speed=-0.0001f;
//    return S->processed_speed; //计算出的速度
//}

static uint32_t FPS_Calculate(uint16_t deltaTime)
{                                                  //FPS计算函数。
    return (1.0f / (double)(deltaTime)) * 1000.0f; // 别忘了先转换为浮点数，否则会有精度丢失
}

void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS) //获取当前系统时钟节拍并算出FPS
{
    time->WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    *FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime);
    time->Last_WorldTime = time->WorldTime;
}

///**
//  * @brief  斜坡函数,使目标输出值缓慢等于期望值
//  * @param  期望最终输出,当前输出,变化速度(越大越快)
//  * @retval 当前输出
//  * @attention  
//  */
//fp32 RAMP_fP(float final, float now, float ramp)
//{
//	float buffer = 0;

//	buffer = final - now;

//	if (buffer > 0)
//	{
//		if (buffer > ramp)
//		{
//			now += ramp;
//		}
//		else
//		{
//			now += buffer;
//		}
//	}
//	else
//	{
//		if (buffer < -ramp)
//		{
//			now += -ramp;
//		}
//		else
//		{
//			now += buffer;
//		}
//	}

//	return now;
//}
