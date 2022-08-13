#include "shoot_task.h"
#include "CAN_receive.h"
#include "pid.h"
#include "bsp_laser.h"
#include"stm32.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "stm32f4xx.h"
#include "ui_update.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ADRC.h"
#include "Gimbal_Task.h"
#define abs(x) ((x) > 0 ? (x) : (-x))
#define shoot_laser_on()    laser_on()      
#define shoot_laser_off()   laser_off()     
extern ExtY_stm32 stm32_Y;
extern ExtU_stm32 stm32_U;
extern fp32 error;
extern B_ADRC ADRC_B;
fp32 limit_V;
fp32 Max_SPEED;
extern Gimbal_Control_t gimbal_control;
extern cap_measure_t get_cap;
extern referee_measure_t get_referee;
extern kind_measure_t kind;
extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;

fric_move_t fric_move;
static void Shoot_Feedback_Update(void);

Shoot_Motor_t trigger_motor;          
static PidTypeDef trigger_motor_pid;         
//static void shoot_fric1_on(fric_move_t *fric1_on);
//static void shoot_fric2_on(fric_move_t *fric2_on);
static void shoot_fric_off(fric_move_t *fric1_off);
shoot_mode_e shoot_mode = SHOOT_STOP; 
shoot_mode_e last_fric_mode= SHOOT_STOP; 
static void fric_control_loop(fric_move_t *fric_move_control_loop);
static void Shoot_Set_Mode(void);
static void shoot_bullet_control(void);
static void shoot_level(void);
void Choose_Shoot_Mode(void);
fp32 fric;

void shoot_init(void)
{  
	fric_move.laster_add=0;
	trigger_motor.move_flag =1;
  trigger_motor.move_flag_ONE=1;
	stm32_shoot_pid_init();
	static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    
  fric_move.shoot_rc = get_remote_control_point();
    
  trigger_motor.shoot_motor_measure = get_trigger_motor_measure_point();
	trigger_motor.blocking_angle_set=0;
    
  PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		

		
  const static fp32 motor_speed_pid[3] = {S3505_MOTOR_SPEED_PID_KP, S3505_MOTOR_SPEED_PID_KI, S3505_MOTOR_SPEED_PID_KD};
	const static fp32 fric_1_order_filter[1] = {0.1666666667f};
  const static fp32 fric_2_order_filter[1] = {0.1666666667f};
		
	fric_move.motor_fric[0].fric_motor_measure = get_shoot_motor_measure_point(0);
	PID_Init(&fric_move.motor_speed_pid[0], PID_POSITION, motor_speed_pid, S3505_MOTOR_SPEED_PID_MAX_OUT, S3505_MOTOR_SPEED_PID_MAX_IOUT);
	fric_move.motor_fric[1].fric_motor_measure = get_shoot_motor_measure_point(1);
	PID_Init(&fric_move.motor_speed_pid[1], PID_POSITION, motor_speed_pid, S3505_MOTOR_SPEED_PID_MAX_OUT, S3505_MOTOR_SPEED_PID_MAX_IOUT);
  fric_move.motor_speed_pid[0].mode_again=KI_SEPRATE;
  fric_move.motor_speed_pid[1].mode_again=KI_SEPRATE;

  first_order_filter_init(&fric_move.fric1_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_1_order_filter);
  first_order_filter_init(&fric_move.fric2_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_2_order_filter);
    
  fric_move.max_speed =  4.75f;
  fric_move.min_speed = -4.75f;
		
    
  Shoot_Feedback_Update();
	trigger_motor.set_angle=trigger_motor.angle;
}
 float speed=3.5; int jam_flag=0;
void shoot_task(void const *pvParameters)
{  
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
	  shoot_init();
	
	 // shoot_laser_on();
	  while (1)
	  {
		
	  shoot_level();
      	  
    Shoot_Set_Mode();        
    Shoot_Feedback_Update(); 
	  shoot_control_loop();
	  CAN_cmd_shoot( fric_move.fric_CAN_Set_Current[0],fric_move.fric_CAN_Set_Current[1],trigger_motor.given_current,0);	
    vTaskDelay(1);
//#if INCLUDE_uxTaskGetStackHighWaterMark
//        shoot_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif		
		}
}

extern int laster_flag;
extern uint16_t ShootSpeed;  fp32 KH=0;
void shoot_level(void)
{
				
	  
if(switch_is_down(fric_move.shoot_rc->rc.s[1])) //Burst first
	 { 
		ShootSpeed=15;
		KH=1.5;
		if(robot_state.robot_level==0||robot_state.robot_level==1)  
		{
		 fric=1.764f;
     trigger_motor.speed_set=4.7;				
		}
		else if(robot_state.robot_level==2)
		{
			fric=1.764f; 
      trigger_motor.speed_set=7.5;
		}
		else if(robot_state.robot_level==3)
		{
		  fric=1.764f; 
		  trigger_motor.speed_set=9.4;
		}			
	 }
else  if(switch_is_mid(fric_move.shoot_rc->rc.s[1])) //Cooling first
	  {  
			if(robot_state.robot_level==0||robot_state.robot_level==1||robot_state.shooter_id1_17mm_speed_limit==15)  
			{ 
				KH=0.32;
				ShootSpeed=15;
			 fric=1.764f;
			 trigger_motor.speed_set=5;				
			}
			else if(robot_state.robot_level==2||robot_state.shooter_id1_17mm_speed_limit==18)
			{ 
				KH=0.32;
				ShootSpeed=18;
				fric=1.943f; 
				trigger_motor.speed_set=7.8;
			}
			else if(robot_state.robot_level==3||robot_state.shooter_id1_17mm_speed_limit==18)
			{
			KH=0.32;
			ShootSpeed=18;
			fric=1.943f; 
			trigger_motor.speed_set=10.3;
			}			  
	  }
else if(switch_is_up(fric_move.shoot_rc->rc.s[1]))  //Speed first
	  {  
			if(robot_state.robot_level==0||robot_state.robot_level==1||robot_state.shooter_id1_17mm_speed_limit==18)  
			{
				KH=1.0;
				ShootSpeed=18;
				fric=1.943f;
				trigger_motor.speed_set=4;				
			}
			else if(robot_state.robot_level==2|robot_state.shooter_id1_17mm_speed_limit==30)
			{ 
				KH=1.0;
				ShootSpeed=30;
				fric=2.9f; 
				trigger_motor.speed_set=6.3;
			}
			else if(robot_state.robot_level==3||robot_state.shooter_id1_17mm_speed_limit==30)
			{
				KH=1.0;
				ShootSpeed=30;
				fric=2.9f; 
				trigger_motor.speed_set=8.3;
			}	 
	  }	
	
}
int shoot_auto_flag=0;  int G_flag=0; ;

fp32 speed_; fp32 speed_t;
int flag=0; int time_l=0; int flag1=0;
int Ready_Flag=0;
static void Shoot_Feedback_Update(void)
{   if(shoot_mode !=SHOOT_STOP&&(fric_move.shoot_rc->rc.ch[4]>50||fric_move.shoot_rc->mouse.press_l))
	//if(switch_is_mid(fric_move.shoot_rc->rc.s[0]))
   {
	   //fric_move.shoot_rc->mouse.press_l
	  shoot_mode = SHOOT_BULLET;
	  if(last_fric_mode!=SHOOT_BULLET)
	   {
		   last_fric_mode=SHOOT_BULLET; 
         //trigger_motor.move_flag = 0;		   
	   }
    time_l++;       	   
   }
   else
     {
	     time_l=0,flag1=0;   flag=0; 
     }
   
   if(time_l>150)
   {
	  flag=0; 
	  flag1=0;
   }
   else if(shoot_mode == SHOOT_BULLET)
   {
	 flag=1;  
   }
   if(flag==1&&flag1==0)
   {
	  trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
    flag1=1;  
   }
  
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
		uint8_t i;
    //Speed filtering of spool wheel motor
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //13/2000 
   //Second-order low-pass filter
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;
     
    //Motor number reset, because the output shaft rotation one circle, motor shaft rotation 36 circles, motor shaft data into the output shaft data, used to control the output shaft angle
   
		if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
		{
			trigger_motor.ecd_count--;
		}
		else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
		{
			trigger_motor.ecd_count++;
		}

		if (trigger_motor.ecd_count == FULL_COUNT)
		{
			trigger_motor.ecd_count = -(FULL_COUNT - 1);
		}
		else if (trigger_motor.ecd_count == -FULL_COUNT)
		{
			trigger_motor.ecd_count = FULL_COUNT - 1;
		}

		trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;


    for (i = 0; i < 2; i++)
    {
        
      fric_move.motor_fric[i].speed = 0.000415809748903494517209f * fric_move.motor_fric[i].fric_motor_measure->speed_rpm;
      fric_move.motor_fric[i].accel = fric_move.motor_speed_pid[i].Dbuf[0] * 500.0f;
    }
	  speed_t=-1*fric_move.motor_fric[0].fric_motor_measure->speed_rpm;
	  speed_=fric_move.motor_fric[1].fric_motor_measure->speed_rpm;
}


static void shoot_ready(void)
{
	if(fric_move.shoot_rc->mouse.press_r)
	{
		Ready_Flag=1;
	}
	else 
	{
		Ready_Flag=0;
	}
	
} 
int F_flag=0;  int laster_time=0;
static void Shoot_Set_Mode(void)
{
	
              
//   if (switch_is_down(fric_move.shoot_rc->rc.s[Shoot_RC_Channel]))
//    {    
//         shoot_mode = SHOOT_STOP;
//		last_fric_mode=SHOOT_STOP;
//		shoot_laser_off(); 
//    }
//    else if (switch_is_mid(fric_move.shoot_rc->rc.s[Shoot_RC_Channel]))
//    {
//        shoot_mode = SHOOT_READY;
//		shoot_laser_on();
//    } 
	shoot_ready();
	if(switch_is_down(fric_move.shoot_rc->rc.s[0])||Ready_Flag==1)
	{
		 shoot_mode = SHOOT_READY;
		//shoot_laser_on();
		
	}
	else
	{
		shoot_mode = SHOOT_STOP;
		last_fric_mode=SHOOT_STOP;
	}
	if(fric_move.shoot_rc->key.v&KEY_PRESSED_OFFSET_V&&laster_flag==0&&F_flag==0)
	{
		laster_time++;
		if(laster_time>300)
		{
			shoot_laser_on();
			F_flag=1;
			laster_time=0;
			fric_move.laster_add++;	
		}
	}
	else if((fric_move.shoot_rc->key.v&KEY_PRESSED_OFFSET_V)==0)
	{
			laster_time=0;
			F_flag=0;
	}
	else if(F_flag==0&&fric_move.shoot_rc->key.v&KEY_PRESSED_OFFSET_V&&laster_flag==1)
	{
		  laster_time++;
		if(laster_time>300)
		{
			shoot_laser_off();
			F_flag=1;
			laster_time=0;
		}
	}
   
   
}
fp32 angle=0; 
int16_t shoot_Current[2];
int trigger_flag=0,trigger_flag1=0; int add_t=0;
int UI_JAW=0; //UIÏÔÊ¾¿¨µ¯
void shoot_control_loop(void)
{

    if (shoot_mode == SHOOT_BULLET/*|| trigger_motor.move_flag == 1 */)
    {                                
       trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
       trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
			if (trigger_flag1==0)
			{
				shoot_bullet_control();
			}
//		shoot_fric1_on(&fric_move);
//		shoot_fric2_on(&fric_move);
		  fric_control_loop(&fric_move);
		  if(trigger_motor.speed>2)
		  {	
				trigger_flag=1;	
				jam_flag=0;
				UI_JAW=0;
		  }
      if (trigger_motor.speed<0.1f&&trigger_flag==1)
		  {
				jam_flag++;
				if(jam_flag>20)
				{
					trigger_flag1=1;
					trigger_flag=0;
					jam_flag=0;
				}
				UI_JAW=0;
		}
    }
		
	else if (shoot_mode == SHOOT_READY)
	{                  //1.92f  13.6m/s     2.24f 17.3m/s   3.22f; /29.2
		trigger_motor.speed_set = 0;
//		shoot_fric1_on(&fric_move);
//		shoot_fric2_on(&fric_move);
		fric_control_loop(&fric_move);
		trigger_motor.move_flag =1;
		trigger_motor.move_flag_ONE =1;
	}
		
	else if(shoot_mode == SHOOT_STOP)	
	{   
    shoot_fric_off(&fric_move);
    trigger_motor.speed_set = 0;
		fric=0.0f;
		fric_control_loop(&fric_move);
    }
	
		if(trigger_flag1==0)
		{		
		 PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
		}
		else if(trigger_flag1==1)
		{
		 PID_Calc(&trigger_motor_pid, trigger_motor.speed, -4.0);	
		 add_t++;
		 if(add_t>190)
			{
			 add_t=0;	
			 trigger_flag1=0;	
			}
		 trigger_motor.set_angle=trigger_motor.angle;
		}
		trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
   
   
}



static void shoot_bullet_control(void)
{  
	
     
//one-fourth Pi Angle at a time
	
	if((robot_state.shooter_id1_17mm_cooling_limit-power_heat_data_t.shooter_id1_17mm_cooling_heat>KH*robot_state.shooter_id1_17mm_cooling_rate)||fric_move.shoot_rc->rc.ch[4]>50) 
	{
		 if ( shoot_mode == SHOOT_BULLET&&trigger_motor.move_flag ==1&&flag==0)
		 {
			trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
			trigger_motor.cmd_time = xTaskGetTickCount();
			trigger_motor.move_flag =0;
		 }
	}
	else
	{
	   trigger_motor.speed_set = 0;	
	}
	
//not up to speed
	 if (rad_format(trigger_motor.set_angle - trigger_motor.angle)>0.05f)
    {
           
    }
    else
    {  
		if( shoot_mode == SHOOT_BULLET)
		 {
				trigger_motor.move_flag =1;
		 }
    }


}
//static void shoot_fric1_on(fric_move_t *fric1_on)
//	{
//		fric1_on->speed_set[0]=-fric;

//	}
//	
//static void shoot_fric2_on(fric_move_t *fric2_on)
//	{
//		fric2_on->speed_set[1]=fric;
//	}
	
static void shoot_fric_off(fric_move_t *fric_off)
{
    fric_off->speed_set[0]=0.0f;
    fric_off->speed_set[1]=0.0f;
}

static void fric_control_loop(fric_move_t *fric_move_control_loop)
{
    uint8_t i = 0;
//		first_order_filter_cali(&fric_move_control_loop->fric1_cmd_slow_set_speed, fric_move_control_loop->speed_set[0]);
//	  first_order_filter_cali(&fric_move_control_loop->fric2_cmd_slow_set_speed, fric_move_control_loop->speed_set[1]);
			
	  fric_move_control_loop->speed_set[0] =fric; //fp32_constrain(fric_move_control_loop->speed_set[0], fric_move_control_loop->min_speed, fric_move_control_loop->max_speed);
    fric_move_control_loop->speed_set[1] =-fric; //fp32_constrain(fric_move_control_loop->speed_set[1], fric_move_control_loop->min_speed, fric_move_control_loop->max_speed);								
    //To limit its maximum speed
	  for (i = 0; i < 2; i++)
    {
			 fric_move_control_loop->motor_speed_pid[i].max_out = TRIGGER_BULLET_PID_MAX_OUT;
			 fric_move_control_loop->motor_speed_pid[i].max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
	  }
    
    for (i = 0; i < 2; i++)
    {
		fric_move_control_loop->motor_fric[i].speed_set = fric_move.speed_set[i];
    PID_Calc(&fric_move_control_loop->motor_speed_pid[i], fric_move_control_loop->motor_fric[i].speed, fric_move_control_loop->motor_fric[i].speed_set);

    }
	  if(switch_is_up(fric_move.shoot_rc->rc.s[1])&&robot_state.robot_level>1) 
	  {  
			for (i = 0; i < 2; i++)
			{
				fric_move_control_loop->motor_fric[i].speed_set = fric_move.speed_set[i];
				PID_Calc(&fric_move_control_loop->motor_speed_pid[i], fric_move_control_loop->motor_fric[i].speed, fric_move_control_loop->motor_fric[i].speed_set);
			}
			
			for (i = 0; i < 2; i++)
			{
				fric_move_control_loop->motor_fric[i].give_current = (int16_t)(fric_move_control_loop->motor_speed_pid[i].out);
			}
			fric_move.fric_CAN_Set_Current[0]=fric_move.motor_fric[0].give_current;
			fric_move.fric_CAN_Set_Current[1]=fric_move.motor_fric[1].give_current;
        
	  }
	else
	{
		
    stm32_step_shoot_0(fric_move_control_loop->speed_set[0],fric_move_control_loop->motor_fric[0].speed);
	  stm32_step_shoot_1(fric_move_control_loop->speed_set[1],fric_move_control_loop->motor_fric[1].speed);
         
    fric_move.fric_CAN_Set_Current[0]=stm32_Y.out_shoot;
	  fric_move.fric_CAN_Set_Current[1]=stm32_Y.out_shoot1;
	}
}
