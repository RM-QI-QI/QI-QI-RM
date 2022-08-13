#include "chassis_task.h"   //
#include "time.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "referee.h"
#include "bsp_laser.h"
#include "pid.h"
#include "Remote_Control.h"
#include "pid.h"
#include "arm_math.h"

#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "CAN_Receive.h"
#include "chassis_behaviour.h"
extern cap_measure_t get_cap;
extern referee_measure_t get_referee;
extern kind_measure_t kind;
extern TIM_HandleTypeDef htim8;
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
#define abs(x) ((x) > 0 ? (x) : (-x))
uint8_t shift_flag=0;
uint8_t ctrl_flag;

chassis_move_t chassis_move;
//Referee system
extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;

extern Gimbal_Control_t gimbal_control;
//static void chassis_level(void);
fp32 Power_Set;
fp32 den_k;
fp32 wz;
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);

static void chassis_init(chassis_move_t *chassis_move_init);

static void chassis_feedback_update(chassis_move_t *chassis_move_update);

static void chassis_set_mode(chassis_move_t *chassis_move_mode);

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

static void chassis_set_contorl(chassis_move_t *chassis_move_control);

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
static void Power_Charge(fp32 power);
void chassic_mode_set(void);
void Angle_Error_Compare(int now_angle,int zero_angle,int last_zero_angle);
int Sway(int zero_angle,int add_angle);
void Judge_swing_mode(void);
void RAND(int min,int max);
fp32 w_set;
fp32 K;
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
fp32 speed_set_x,speed_set_y=0;
fp32 power_charge=0;

void chassis_task(void const *pvParameters)
{   
	chassis_move.power_control.SPEED_MIN=0.25f; //Minimum speed limit_power
	  
    vTaskDelay(CHASSIS_TASK_INIT_TIME);	  //GYRO initializing
   
    chassis_init(&chassis_move);
	int i;
	for(i=0;i<4;i++)
	{
	  chassis_move.motor_chassis[i].give_current=0;	
	}
	
		while(1)
		{	
       
		chassic_mode_set();
		 
		chassis_set_mode(&chassis_move); 
		
		chassis_mode_change_control_transit(&chassis_move);
		
		chassis_feedback_update(&chassis_move);	
		
		chassis_set_contorl(&chassis_move);
		
		chassis_control_loop(&chassis_move);			
		CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current,chassis_move.motor_chassis[1].give_current,
														 chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		
		vTaskDelay(2);
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif			
		}
}

//模式选择及参数设定
void chassic_mode_set(void)
{
	
if(	switch_is_mid(chassis_move.chassis_RC->rc.s[0])) 	// Power first
	{	    
          	if(robot_state.robot_level==0||robot_state.robot_level==1||robot_state.chassis_power_limit==60)  //level_1   
			{    chassis_move.power_control.POWER_MAX=60;
				power_charge=6000;
				Power_Charge(power_charge);
				CAN_CMD_cap(power_charge);
				 w_set=7.5;
				speed_set_x=2.1f;
				speed_set_y=1.82f;
                 K=100.f;
				
			}
			 if(robot_state.robot_level==2||robot_state.chassis_power_limit==80)  //level_2
			{   chassis_move.power_control.POWER_MAX=80;
				power_charge=8000;
				Power_Charge(power_charge);
				CAN_CMD_cap(power_charge);
				 w_set=8.5;
				speed_set_x=2.6f;
				speed_set_y=2.0f;
                 K=200.f;
				
			}
			 if(robot_state.robot_level==3||robot_state.chassis_power_limit==100)  //level_3 
			{   chassis_move.power_control.POWER_MAX=100;
				power_charge=10000;
				Power_Charge(power_charge);
				CAN_CMD_cap(power_charge);
				 w_set=9.5;
				speed_set_x=3.0f;
				speed_set_y=2.3f;
                 K=300.f;
				
			}
	}
	else if(switch_is_up(chassis_move.chassis_RC->rc.s[0]))  //Volume first
	{
				if(robot_state.robot_level==0||robot_state.robot_level==1||robot_state.chassis_power_limit==45)  //level_1
			{    chassis_move.power_control.POWER_MAX=45;
				power_charge=4500;
				Power_Charge(power_charge);
				CAN_CMD_cap(power_charge);
				 w_set=6.3;
				speed_set_x=1.82f;
				speed_set_y=1.5f;
                 K=95.f;
				
			}
			 if(robot_state.robot_level==2||robot_state.chassis_power_limit==50)  //level_2
			{    chassis_move.power_control.POWER_MAX=50;
				power_charge=5000;
				Power_Charge(power_charge);
				CAN_CMD_cap(power_charge);
				 w_set=7.1;
				speed_set_x=1.9f;
				speed_set_y=1.6f;
                 K=130.f;
				
			}
			 if(robot_state.robot_level==3||robot_state.chassis_power_limit==55) //level_3
			{   chassis_move.power_control.POWER_MAX=55;
				power_charge=5500;
				Power_Charge(power_charge);
				CAN_CMD_cap(power_charge);
				 w_set=7.5; // 8.1
				speed_set_x=1.94;
				speed_set_y=1.7f;
                 K=150.f;
			}	
		
	}	
	
	
}

//电容充电功率设定
void Power_Charge(fp32 power)
{
	if(power_heat_data_t.chassis_power_buffer>60)
	{
	power*=2.1f;	
	}
	else if(power_heat_data_t.chassis_power_buffer>30)
	{
	power*=1.3f;	
	}
	else if(power_heat_data_t.chassis_power_buffer>15)
	{
	power*=1.1f;	
	}
	else if (power_heat_data_t.chassis_power_buffer>8)
	{
	power*=1.0f;	
	}
	else
	{
	power*=0.85f;	
	}
	
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
	

	  if (chassis_move_init == NULL)
    {
        return;
    }
	
	  
    const static fp32 motor_speed_pid[8] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD,0,0,0,0,0};
    
    const static fp32 chassis_yaw_pid[8] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD,CHASSIS_FOLLOW_GIMBAL_PID_KF,0,0,CHASSIS_FOLLOW_GIMBAL_F_divider,CHASSIS_FOLLOW_GIMBAL_F_out_limit};		
	  
    const static fp32 motor_power_pid[3] = {M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD};
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
		uint8_t i;
    
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    
    chassis_move_init->chassis_RC = get_remote_control_point();
    
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
	chassis_move_init->can_power.get_cap_measure=get_cap_measure_point();
	  
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);		
    
	for (i = 0; i < 4; i++)
	{
    PID_Init(&chassis_move_init->Power_control.power_pid[i], PID_POSITION, motor_power_pid, M3505_MOTOR_POWER_PID_MAX_OUT, M3505_MOTOR_POWER_PID_MAX_IOUT);	
	}		
		
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    
	chassis_move_init->vx_max_speed = 3.75;
    chassis_move_init->vx_min_speed = -3.75;
    chassis_move_init->vy_max_speed = 3.75;
    chassis_move_init->vy_min_speed = -3.75;
		
    chassis_move_init->vx_max_speed_up = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed_up = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_speed_up = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed_up = -NORMAL_MAX_CHASSIS_SPEED_Y;
		
    chassis_move_init->vx_max_speed_down = NORMAL_DOWN_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed_down = -NORMAL_DOWN_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_speed_down = NORMAL_DOWN_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed_down = -NORMAL_DOWN_CHASSIS_SPEED_Y;

   
    chassis_feedback_update(chassis_move_init);
}
 float RealPower_Total; fp32 actvel[4]={0};//=0.1047197551f;
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }	
    uint8_t i = 0;
		//0.00000394047917046875f
    for (i = 0; i < 4; i++)
    {
        
    chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		
    };

    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
    
	
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
	  chassis_behaviour_mode_set(chassis_move_mode);
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }
    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }
 
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && 
			   chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) && 
			   chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && 
			        chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
   
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && 
			        chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
	if ((chassis_move_transit->last_chassis_mode == CHASSIS_VECTOR_SPIN) && 
			   chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->mode_flag=1;
    }
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}
int x_flag=0; int X_FLAG=0; int zero=0; fp32 power_cet; int power_flag1=0, power_flag2=0;
fp32 kx=1.f,ky=1.f,kw=1.f;
int UI_flag=0;

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
	
	power_cet=(get_cap.capvot-13.5f)/10.5f*100.f;
  int16_t vx_channel_RC, vy_channel_RC;
  fp32 vx_set_channel_RC, vy_set_channel_RC;
	
  rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel_RC, CHASSIS_RC_DEADLINE);
  rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[0], vy_channel_RC, CHASSIS_RC_DEADLINE);
	
	vx_set_channel_RC = -vx_channel_RC * CHASSIS_VX_RC_SEN;
  vy_set_channel_RC = vy_channel_RC * CHASSIS_VY_RC_SEN;	
 
  // First order low pass filter
 
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel_RC);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel_RC);

if(chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_C&&power_cet>4&&power_flag1==0&&power_flag2==0)
{    
	power_flag2=1;     //与超级电容的使用与UI显示相关
	power_flag1=1;
	UI_flag=1;
}
else if((chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_C)==0&&power_flag1==1&&power_flag2==1)
{     
	 power_flag2=0;
	 UI_flag=1;
}
else if((chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_C&&power_flag2==0&&power_flag1==1)||power_cet<4)
{
	  power_flag2=1;
		power_flag1=0;
	  UI_flag=0;
}
else if((chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_C)==0&&power_flag1==0&&power_flag2==1)
{
	  power_flag2=0;
		power_flag1=0;
	  UI_flag=0;
}
	
if(UI_flag==1)
{     
	if(get_cap.capvot>17)   //测试中，电容电压低于17后，电机偶尔欠压
{
     chassis_move.power_control.POWER_MAX+=13; 
	   K+=100;
	   kx=1.05f;  
     ky=1.05f;
		 kw=1.1f;	
}
}
else
{
    kx=1.0f;  
    ky=1.0f;
		kw=1.0f;	
	
}
	if(chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_A)
	{
		*vy_set=-speed_set_y*ky;
			
	}
	else if(chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_D)
	{
	   *vy_set=speed_set_y*ky;	
	}
	else	
	{
		*vy_set=0;
	}
	
	
	
	
	if(chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_W)
	{
		*vx_set =-speed_set_x*kx;
    	
			
	}
	else if(chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_S)
	{
	   *vx_set=speed_set_x*kx;	
		
	}
	else
	{
		*vx_set=0;
	}
	
	
  if(x_flag==0)
  {
	  //与云台指定方向相关（车正前方或正后方）用于一键掉头和小陀螺就近对位
		if(chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_X&&(X_FLAG%2==0))
		{
		X_FLAG++;
		x_flag=1;
		gimbal_control.gimbal_yaw_motor.ZERO_ECD=gimbal_control.gimbal_yaw_motor.ZERO_ECD_flag-4096;
		
		}
		else if(chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_X&&(X_FLAG%2==1))
		{
		X_FLAG++;
		x_flag=1;
		gimbal_control.gimbal_yaw_motor.ZERO_ECD=gimbal_control.gimbal_yaw_motor.ZERO_ECD_flag;
		
		}
	   
  }

  if(gimbal_control.gimbal_yaw_motor.ZERO_ECD>=4096) gimbal_control.gimbal_yaw_motor.LAST_ZERO_ECD=gimbal_control.gimbal_yaw_motor.ZERO_ECD-4096;
  else  gimbal_control.gimbal_yaw_motor.LAST_ZERO_ECD=gimbal_control.gimbal_yaw_motor.ZERO_ECD+4096;
  
 if((chassis_move_rc_to_vector->chassis_RC->key.v &KEY_PRESSED_OFFSET_X)==0)
  {
	x_flag=0;	 
  }
 if(X_FLAG%2==1)
 {
	*vx_set=-1.f*(*vx_set);
	*vy_set=-1.f*(*vy_set); 
 }


  // First order low pass filter
 
   
  if (vy_set_channel_RC == 0)
	{
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }
	if (vx_set_channel_RC == 0)
  {
       chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }


   *vx_set += chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
   *vy_set += chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}fp32 relative_angle=0.0f;

int swing_flag_artificial=0; int swing_flag_auto=0; int swing=0; int relative_angle_compare=0;
int SW=1; 
int Rand=0;   fp32 SW_ING=1.0;
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
		    
  fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
  fp32 relative_angle=0.0f;
	fp32 relative_angle_swing=0.0f;
	//fp32 relative_angle=0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
    int add_angle=8191/8.4;

    //Follow the GIMBAL
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {   
    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		int sway_angle=0;
		
		if(chassis_move_control->chassis_RC->key.v &KEY_PRESSED_OFFSET_SHIFT)
		{
			zero=0;
		}
		if(chassis_move_control->mode_flag==1)
		{
			 
			 //小陀螺停止，就近对位
		Angle_Error_Compare(gimbal_control.gimbal_yaw_motor.relative_angle/Motor_Ecd_to_Rad,gimbal_control.gimbal_yaw_motor.ZERO_ECD,gimbal_control.gimbal_yaw_motor.LAST_ZERO_ECD);
			
		}
	//相对角度设置 ，摇摆运动
		Judge_swing_mode();
		if(swing_flag_auto)
		{  
       add_angle=8191/Rand;
			 sway_angle=Sway(gimbal_control.gimbal_yaw_motor.ZERO_ECD,add_angle);
			 relative_angle_compare=abs((chassis_move_control->chassis_yaw_motor->relative_angle/Motor_Ecd_to_Rad-sway_angle));
			
			if(relative_angle_compare<60)   { RAND(4,17);  /*SW*=-1;*/}

			sway_angle=Sway(gimbal_control.gimbal_yaw_motor.ZERO_ECD,add_angle);
			relative_angle=chassis_move_control->chassis_yaw_motor->relative_angle-(sway_angle*Motor_Ecd_to_Rad);
			
			relative_angle_swing=chassis_move_control->chassis_yaw_motor->relative_angle-(chassis_move_control->chassis_yaw_motor->ZERO_ECD*Motor_Ecd_to_Rad);
		  if(relative_angle_swing>PI) relative_angle_swing=-2*PI+relative_angle_swing;
      sin_yaw = arm_sin_f32(-(relative_angle_swing));
      cos_yaw = arm_cos_f32(-(relative_angle_swing));	
		}
		else if (swing)
		{  if(swing%2==1)
			{
				add_angle*=-1;
			}
			 sway_angle=Sway(gimbal_control.gimbal_yaw_motor.ZERO_ECD,add_angle);
			 relative_angle=chassis_move_control->chassis_yaw_motor->relative_angle-(sway_angle*Motor_Ecd_to_Rad);
			
			 relative_angle_swing=chassis_move_control->chassis_yaw_motor->relative_angle-(chassis_move_control->chassis_yaw_motor->ZERO_ECD*Motor_Ecd_to_Rad);
		   if(relative_angle_swing>PI) relative_angle_swing=-2*PI+relative_angle_swing;
       sin_yaw = arm_sin_f32(-(relative_angle_swing));
       cos_yaw = arm_cos_f32(-(relative_angle_swing));
			
		}
		else
		{
       relative_angle=chassis_move_control->chassis_yaw_motor->relative_angle-(chassis_move_control->chassis_yaw_motor->ZERO_ECD*Motor_Ecd_to_Rad);
		if(relative_angle>PI) relative_angle=-2*PI+relative_angle;
        sin_yaw = arm_sin_f32(-(relative_angle));
        cos_yaw = arm_cos_f32(-(relative_angle));
		}
		
		
		
    chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
    chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        
    chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
       

		chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, relative_angle, chassis_move_control->chassis_relative_angle_set);
		
		if(chassis_move_control->chassis_RC->key.v &KEY_PRESSED_OFFSET_X)
		{  
			zero=1;
			
		}
        if(zero==1)
		{
		chassis_move_control->wz_set=0.0f;	
		if(abs(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd-gimbal_control.gimbal_yaw_motor.ZERO_ECD)<1)	
		{
		zero=0;	
		}
		}
        
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, -4.5f, 4.5f);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, -4.4f , 4.4f);
    }
    //Little Top
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
    {   
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        relative_angle=chassis_move_control->chassis_yaw_motor->relative_angle-(chassis_move_control->chassis_yaw_motor->ZERO_ECD*Motor_Ecd_to_Rad);
		    if(relative_angle>PI) relative_angle=-2*PI+relative_angle;
        sin_yaw = arm_sin_f32((relative_angle));
        cos_yaw = arm_cos_f32((relative_angle));
        chassis_move_control->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
        chassis_move_control->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
        chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);  
		if(chassis_move_control->chassis_RC->key.v &KEY_PRESSED_OFFSET_SHIFT)
		{
			chassis_move_control->wz_set=0;
		}
		else
		{
		chassis_move_control->wz_set =-w_set*kw/**SW_ING*/;
		}
        
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, -4.5f, 4.5f);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, -4.4f , 4.4f);
			  
    }

    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

     
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
		
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	  wheel_speed[0] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = (-vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
    wheel_speed[2] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * (MOTOR_DISTANCE_TO_CENTER) * wz_set;
    wheel_speed[3] = (vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * (MOTOR_DISTANCE_TO_CENTER) * wz_set);//*1.08f
}
fp32 speed_abs_all;

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
  fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
    speed_abs_all=fabs(chassis_move_control_loop->motor_chassis[0].speed_set)+fabs(chassis_move_control_loop->motor_chassis[1].speed_set)
	                                          +fabs(chassis_move_control_loop->motor_chassis[2].speed_set)+fabs(chassis_move_control_loop->motor_chassis[3].speed_set);
 
    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
       
        return;
    }
    
   
      for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }
    
    
 
    for (i = 0; i < 4; i++)
    {
    PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
		chassis_move_control_loop->power_control.speed[i]=chassis_move_control_loop->motor_chassis[i].speed;
		if(abs(chassis_move_control_loop->power_control.speed[i])<chassis_move_control_loop->power_control.SPEED_MIN)
		{
			
			chassis_move_control_loop->power_control.speed[i]=chassis_move_control_loop->power_control.SPEED_MIN;				
		}
		
    }

	
 
   for (i = 0; i < 4; i++)
    {
    chassis_move_control_loop->power_control.current[i]=chassis_move_control_loop->motor_chassis[i].give_current=(fp32)(chassis_move_control_loop->motor_speed_pid[i].out);
		chassis_move_control_loop->power_control.totalCurrentTemp+=abs(chassis_move_control_loop->power_control.current[i]);
		
    }
	

 //功率控制
	for(i=0;i<4;i++)
	{
	   chassis_move_control_loop->power_control.MAX_current[i]=(K*chassis_move_control_loop->power_control.current[i]/chassis_move_control_loop->power_control.totalCurrentTemp)
		*(chassis_move_control_loop->power_control.POWER_MAX)/abs(chassis_move_control_loop->motor_chassis[i].speed);	
	}
	 chassis_move_control_loop->power_control.totalCurrentTemp=0;
	
	for(i=0;i<4;i++)
	{   
		if(abs(chassis_move_control_loop->motor_chassis[i].give_current)>=abs(chassis_move_control_loop->power_control.MAX_current[i]))
		{
			
			chassis_move_control_loop->motor_chassis[i].give_current=chassis_move_control_loop->power_control.MAX_current[i];
		}
	
		else
		{ 
	    chassis_move_control_loop->motor_chassis[i].give_current=(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);  		
		}
	
	}
	chassis_move_control_loop->mode_flag=0;
}

void Angle_Error_Compare(int now_angle,int zero_angle,int last_zero_angle)  //就近对位
{
	fp32 flag_angle[2]={0};
	if(zero_angle>4096)
	{
		flag_angle[0]=abs((now_angle-zero_angle));
		if(flag_angle[0]>4096)  flag_angle[0]=8191-	zero_angle+now_angle;
		
        flag_angle[1]=abs((now_angle-last_zero_angle));
        if(flag_angle[1]>4096)  flag_angle[1]=8191-	now_angle+last_zero_angle;
		
        if(	flag_angle[0]>flag_angle[1])
		{
		 zero_angle-=4096;	X_FLAG++;
		 gimbal_control.gimbal_yaw_motor.ZERO_ECD=zero_angle;
		}
	}
	else if(zero_angle<=4096)
	{ 
		flag_angle[0]=abs((now_angle-zero_angle));
		if(flag_angle[0]>4096)  flag_angle[0]=8191-	now_angle+zero_angle;
		
        flag_angle[1]=abs((now_angle-last_zero_angle));
        if(flag_angle[1]>4096)  flag_angle[1]=8191-	last_zero_angle+now_angle;
		
        if(	flag_angle[0]>flag_angle[1])
		{
			
		zero_angle+=4096;	X_FLAG++;
		gimbal_control.gimbal_yaw_motor.ZERO_ECD=zero_angle;
		}
	}	
}
int swing_key_time=0; int BK_flag=0;
void Judge_swing_mode(void)
{
	if((chassis_move.chassis_RC->key.v&KEY_PRESSED_OFFSET_R))
	{
		swing_flag_artificial=0;
		swing_flag_auto=0;	
		swing_key_time=0;
		swing=0;
		return;
	}
	if(chassis_move.chassis_RC->key.v&KEY_PRESSED_OFFSET_CTRL&&BK_flag==0)
	{   
		swing_flag_artificial=1; //人工
		swing_key_time++;
		swing_flag_auto=0;
		if(swing_key_time>400)
		{
		swing_flag_auto=1;
		RAND(4,17); //随机(-12,12)角度			
		swing_flag_artificial=0;
		swing_key_time=0;
		swing=0;
	  swing_key_time	=0;
    BK_flag=1;			
	    }   
	}
	if( BK_flag==1&&(chassis_move.chassis_RC->key.v&KEY_PRESSED_OFFSET_CTRL)==0)
	{
		BK_flag=0;
	}
	if(swing_flag_artificial==1&&(chassis_move.chassis_RC->key.v&KEY_PRESSED_OFFSET_CTRL)==0)
	{
    swing++; swing_key_time	=0;	
		swing_flag_artificial=0;
	}		
	
	
}
int Sway(int zero_angle,int add_angle)  //摇摆角度
{
	int true_angle=0;
	true_angle=zero_angle-add_angle;
	if(true_angle<0) true_angle=8191+true_angle;
	else if(true_angle>8191) true_angle=true_angle-8191;
	return true_angle;
}

void RAND(int min,int max)
{    fp32 mid;
	   mid=(min+max-1)/2;
 	   Rand=min+rand()%max;   
	   if(Rand<mid) Rand*=-1;
	   else Rand-=mid;	
}fp32 t=3.1415926/2;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == htim8.Instance)
		{  t+=1;
		//SW_ING=(50+rand()%51)/100.f;
			SW_ING=0.2+abs((0.2+sin(t)));
		}
}
