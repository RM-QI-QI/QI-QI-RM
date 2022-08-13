#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "user_lib.h"
#include "rm_usart.h"
#include "kalman.h"
#include "shoot_task.h"
#define int_abs(x) ((x) > 0 ? (x) : (-x))
uint8_t SPIN = 0;
extern kalman_filter_t yaw_kalman_filter;
extern kalman_filter_t pitch_kalman_filter;

void Mean_Filtering_Y(fp32 Y);
void Mean_Filtering_P(fp32 P);
void VISION_DATA_USE(fp32 Y,fp32 P);
void MOUSE_OF_AUTO(Gimbal_Control_t *gimbal_auto_control_set);
void AUTO_PID(void);
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
  */
  void Fiter(fp32 pitch);
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
extern vision_rxfifo_t vision_rxfifo;
/**
  * @brief          云台校准的通过判断角速度来判断云台是否到达极限位置
  * @author         RM
  * @param[in]      对应轴的角速度，单位rad/s
  * @param[in]      计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param[in]      记录的角度 rad
  * @param[in]      反馈的角度 rad
  * @param[in]      记录的编码值 raw
  * @param[in]      反馈的编码值 raw
  * @param[in]      校准的步骤 完成一次 加一
  * @retval         返回空
  */
#define GIMBAL_CALI_GYRO_JUDGE(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

extern fric_move_t fric_move;	
//云台行为状态机
gimbal_behaviour_e gimbal_behaviour=GIMBAL_RELATIVE_ANGLE;
/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
//static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
//云台行为状态机设置
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set);
//云台编码值控制

//无力模式控制
void gimbal_auto_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_auto_control_set);
//static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);		
//static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
//{
//    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
//    {
//        return;
//    }

//    *yaw = 0.0f;
//    *pitch = 0.0f;
//}
int V_FALG=0; int mouse_r_flag=0;
void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
	  if (gimbal_mode_set == NULL)
    {
        return;
    }
    //云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);	

    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
	  else if (gimbal_behaviour == GIMBAL_AUTO)
	  {
		    gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_AUTO_E;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_AUTO_E;
		
	  }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {  
		if(mouse_r_flag==1)
		{
			gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		}
		
		if(fric_move.laster_add!=0&&gimbal_mode_set->gimbal_rc_ctrl->key.v &KEY_PRESSED_OFFSET_V&&gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO&&V_FALG==0)
		{
		V_FALG=1;
		gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		}
		
		else if(fric_move.laster_add!=0&&gimbal_mode_set->gimbal_rc_ctrl->key.v &KEY_PRESSED_OFFSET_V&&gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE&&V_FALG==0)
		{
		V_FALG=1;
		gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;	
		}
		
    gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
	if((gimbal_mode_set->gimbal_rc_ctrl->key.v &KEY_PRESSED_OFFSET_V)==0)	
	{
		V_FALG=0;
	}
}

static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set)
{
	  if (gimbal_mode_set == NULL)
    {
        return;
    }

		//开关控制 云台状态
//    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
//    {
//            gimbal_behaviour = GIMBAL_ZERO_FORCE;
//			  
//    }
	
     if (gimbal_mode_set->gimbal_rc_ctrl->key.v &KEY_PRESSED_OFFSET_R||(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[4]!=0))
    {
           gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
			  
    }
	 else if (gimbal_mode_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_SHIFT)
    {
           gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
			  
    }
   else if (gimbal_mode_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_Z)
    {
           gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
			  
    }
//  else if(gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r)
//	{
//		gimbal_behaviour = GIMBAL_AUTO;
//		mouse_r_flag=1;
//	}
//  else if((mouse_r_flag==1)&&(gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r==0))
//  {
//	  gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
//  }
}

/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */

int yaw_flag=0;  
fp32 Pitch_Set[8]={0};
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }	
	static fp32 rc_add_yaw, rc_add_pit;  static fp32 rc_add_yaw_RC, rc_add_pit_RC;
  static int16_t yaw_channel = 0; static int16_t pitch_channel = 0;
	static int16_t yaw_channel_RC = 0; static int16_t pitch_channel_RC = 0;
	fp32 rc_add_z=0;
  fp32 yaw_turn=0; fp32 yaw_turn1=0; fp32 pitch_turn=0;
  if(yaw_flag==0)	
  {
	  if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_X)
	  {
			yaw_turn=3.1415926f;
			yaw_flag=1;		
	  }
	
   }
   if((gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_X)==0)
   {
	 yaw_flag=0;	  
   }

if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_Q)
{
	yaw_turn1=0.0025f;
	
}
else if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_E)
{
	yaw_turn1=-0.0025f;
	
}
   //将遥控器的数据处理死区 
		rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel_RC, RC_deadband);
		rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel_RC, RC_deadband);

    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->mouse.x , yaw_channel, RC_deadband-6	);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->mouse.y, pitch_channel, RC_deadband-5	);
	
    rc_add_yaw_RC = yaw_channel_RC* Yaw_RC_SEN;
    rc_add_pit_RC = -(pitch_channel_RC * Pitch_RC_SEN);
     
    rc_add_yaw = -yaw_channel* Yaw_Mouse_Sen;
	
   if( gimbal_control_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	 {
		rc_add_pit=  pitch_channel*Pitch_Mouse_Sen_Gro;
		first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vy_gyro, rc_add_pit);	
	 }
	else
	{
		rc_add_pit=  -pitch_channel*Pitch_Mouse_Sen;
		Fiter(rc_add_pit);
    rc_add_pit=(rc_add_pit*8+Pitch_Set[1]-Pitch_Set[7])/8;
		first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vy, rc_add_pit);	
	}
    //rc_add_z=-gimbal_control_set->gimbal_rc_ctrl->mouse.z*Z_Mouse_Sen;
	
	

   first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vx_RC, rc_add_yaw_RC);
   first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vy_RC, rc_add_pit_RC);

   
    //一阶低通滤波代替斜波作为输入
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vx, rc_add_yaw);	
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vz, rc_add_z);	
	

 
   
   
//	
//    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
//    {
//        gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);

//    }
//    
//     //将控制增加量赋值

  if(gimbal_control_set->gimbal_cmd_slow_set_vx.out>3.f)gimbal_control_set->gimbal_cmd_slow_set_vx.out=3.f;
	else if(gimbal_control_set->gimbal_cmd_slow_set_vx.out<-3.f)gimbal_control_set->gimbal_cmd_slow_set_vx.out=-3.f;
	
		
	if(rc_add_yaw_RC == 0) { gimbal_control_set->gimbal_cmd_slow_set_vx_RC.out=0;}
	if(rc_add_pit_RC == 0) { gimbal_control_set->gimbal_cmd_slow_set_vy_RC.out=0;}
   
	
  *add_yaw = gimbal_control_set->gimbal_cmd_slow_set_vx.out+yaw_turn+yaw_turn1+gimbal_control_set->gimbal_cmd_slow_set_vx_RC.out;
	if( gimbal_control_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		*add_pitch=gimbal_control_set->gimbal_cmd_slow_set_vy_gyro.out+pitch_turn+gimbal_control_set->gimbal_cmd_slow_set_vy_RC.out;
	}
	else
	{
	  *add_pitch=gimbal_control_set->gimbal_cmd_slow_set_vy.out+pitch_turn+gimbal_control_set->gimbal_cmd_slow_set_vy_RC.out;
	}
	
}

extern kalman auto_yaw;  extern kalman auto_pitch; fp32 yaw_set; fp32 pitch_set;
fp32 receive_yaw[3]={0},receive_pitch[3]={0};  fp32 compose_yaw=0,compose_pitch=0;
fp32 receive_yaw_add=0.0f; fp32 receive_pitch_add=0.0f;
fp32 vision_yaw_receive_last,vision_pitch_receive_last=0.0f;   fp32 vision_error_I[5]={0.0f};  fp32 vision_error_I_P[5]= {0.0f};
void gimbal_auto_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_auto_control_set)
{  
	fp32 yaw_auto_channel,pitch_auto_channel=0;
	if(vision_rxfifo.rx_flag==1)
	{
	if(vision_rxfifo.yaw_fifo==0)
	{
      vision_rxfifo.yaw_fifo=180;
	}
    if(vision_rxfifo.pitch_fifo==0)
	{
      vision_rxfifo.pitch_fifo=180;
	}		
	gimbal_auto_control_set->gimbal_Auto.yaw_offset=(180-vision_rxfifo.yaw_fifo);
	gimbal_auto_control_set->gimbal_Auto.pitch_offset=-(vision_rxfifo.pitch_fifo-186);
	
	fp32 yaw_auto=0.0f;
		
	rc_deadline_limit(gimbal_auto_control_set->gimbal_Auto.yaw_offset , yaw_auto_channel, AUTO_deadband-0.5f);
  rc_deadline_limit(gimbal_auto_control_set->gimbal_Auto.pitch_offset, pitch_auto_channel, AUTO_deadband-0.5f	);
	
	
	if(yaw_auto_channel<1) 
	{
		receive_yaw_add=0;
	}
	if(pitch_auto_channel<0.8f) 
	{
		receive_yaw_add=0;
	}
	yaw_auto=yaw_auto_channel/3.1415926f;
	pitch_auto_channel=pitch_auto_channel*22.752777777f*Motor_Ecd_to_Rad;
		
	
	yaw_set=yaw_auto*0.012f;
	pitch_set=pitch_auto_channel*0.12f;
	  
	VISION_DATA_USE(yaw_auto_channel,pitch_auto_channel);
	AUTO_PID();
//	 KalmanFilter(&auto_yaw,yaw_set)
//	 KalmanFilter(&auto_pitch,pitch_set);
//	
//	first_order_filter_cali(&gimbal_auto_control_set->gimbal_cmd_slow_set_vx_auto, auto_yaw.X_now);
//    first_order_filter_cali(&gimbal_auto_control_set->gimbal_cmd_slow_set_vy_auto, auto_pitch.X_now);
	
//    KalmanFilter(&auto_yaw,gimbal_auto_control_set->gimbal_cmd_slow_set_vx_auto.out);
//	KalmanFilter(&auto_pitch,gimbal_auto_control_set->gimbal_cmd_slow_set_vy_auto.out);
	
	//first_order_filter_cali(&gimbal_auto_control_set->gimbal_cmd_slow_set_vx_auto, yaw_set);
    //first_order_filter_cali(&gimbal_auto_control_set->gimbal_cmd_slow_set_vy_auto, pitch_set);
//	*add_yaw=auto_yaw.X_now;
//	*add_pitch=auto_pitch.X_now;
    
	*add_yaw=1.2f*yaw_set+(receive_yaw[0]-receive_yaw[1])*0.3f+receive_yaw_add*0.0001f+vision_error_I[4]*0.1f;
	KalmanFilter(&auto_yaw,*add_yaw);
	*add_yaw=auto_yaw.X_now;
	 
	*add_pitch=1.1f*pitch_set+(receive_pitch[0]-receive_pitch[1])*0.25f+receive_pitch_add*0.0001f+vision_error_I_P[4]*0.1f;
	KalmanFilter(&auto_pitch,*add_pitch);
  *add_pitch=auto_pitch.X_now;
	
	}
	else
	{
   *add_yaw=0;
   *add_pitch=0;
	}
	
	MOUSE_OF_AUTO(gimbal_auto_control_set);
	
  *add_yaw += gimbal_auto_control_set->gimbal_cmd_slow_set_vx.out;
	if( gimbal_auto_control_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		*add_pitch+=gimbal_auto_control_set->gimbal_cmd_slow_set_vy_gyro.out;
	}
	else
	{
	  *add_pitch+=gimbal_auto_control_set->gimbal_cmd_slow_set_vy.out;
	}	
//    vision_rxfifo.yaw_fifo=180;
//	vision_rxfifo.pitch_fifo=180;   	
}

void Fiter(fp32 pitch)
{
	Pitch_Set[7]=Pitch_Set[6];
	Pitch_Set[6]=Pitch_Set[5];
	Pitch_Set[5]=Pitch_Set[4];
	Pitch_Set[4]=Pitch_Set[3];
	Pitch_Set[3]=Pitch_Set[2];
	Pitch_Set[2]=Pitch_Set[1];
	Pitch_Set[1]=Pitch_Set[0];
	Pitch_Set[0]=pitch;
}

void Mean_Filtering_Y(fp32 Y)
{
	  vision_error_I[3]=vision_error_I[2];
		vision_error_I[2]=vision_error_I[1];
		vision_error_I[0]=Y;
		vision_error_I[4]=(vision_error_I[0]=vision_error_I[1]+vision_error_I[2]+vision_error_I[3])/4;
	
}
void Mean_Filtering_P(fp32 P)
{
	  vision_error_I_P[3]=vision_error_I_P[2];
		vision_error_I_P[2]=vision_error_I_P[1];
		vision_error_I_P[0]=P;
		vision_error_I_P[4]=(vision_error_I_P[0]=vision_error_I_P[1]+vision_error_I_P[2]+vision_error_I_P[3])/4;
	
}
void AUTO_PID(void)
{
	  receive_pitch[2]=receive_pitch[1];
	  receive_pitch[1]=receive_pitch[0];
	  receive_pitch[0]=pitch_set;  
	  receive_pitch_add+=pitch_set;
	 if(receive_pitch_add>0.01f)  receive_pitch_add=0.01f;
	 else if(receive_pitch_add<-0.01f)  receive_pitch_add=-0.01f;
	
	  receive_yaw[2]=receive_yaw[1];
	  receive_yaw[1]=receive_yaw[0];
	  receive_yaw[0]=yaw_set;
	  receive_yaw_add+=yaw_set;
	 if(receive_yaw_add>0.1f)  receive_yaw_add=0.1f;
	 else if(receive_yaw_add<-0.1f)  receive_yaw_add=-0.1f;
	
	
}
void VISION_DATA_USE(fp32 Y,fp32 P)
{
	if(Y==vision_yaw_receive_last)
	{
		yaw_set=0;
	}
	else
	{
		Mean_Filtering_Y(yaw_set);
	}
	
	if(P==vision_pitch_receive_last)
	{
		pitch_set=0;
	}
	else
	{
		Mean_Filtering_P(pitch_set);
	}
	  vision_yaw_receive_last=Y;  
    vision_pitch_receive_last=P;	
}

void MOUSE_OF_AUTO(Gimbal_Control_t *gimbal_auto_control_set)
{
	fp32  yaw_channel,pitch_channel=0.0f;  
	fp32  rc_add_yaw, rc_add_pit=0.0f;
  rc_deadline_limit(gimbal_auto_control_set->gimbal_rc_ctrl->mouse.x , yaw_channel, RC_deadband-6	);
  rc_deadline_limit(gimbal_auto_control_set->gimbal_rc_ctrl->mouse.y, pitch_channel, RC_deadband-5	);
	
     
  rc_add_yaw = -yaw_channel* Yaw_Mouse_Sen;
	
  if( gimbal_auto_control_set->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		rc_add_pit=  pitch_channel*Pitch_Mouse_Sen_Gro;
		first_order_filter_cali(&gimbal_auto_control_set->gimbal_cmd_slow_set_vy_gyro, rc_add_pit);		
	}
	else
	{
		rc_add_pit=  -pitch_channel*Pitch_Mouse_Sen;
		Fiter(rc_add_pit);
    rc_add_pit=(rc_add_pit*8+Pitch_Set[1]-Pitch_Set[7])/8;
		first_order_filter_cali(&gimbal_auto_control_set->gimbal_cmd_slow_set_vy, rc_add_pit);	
	}
   
    //一阶低通滤波代替斜波作为输入
    first_order_filter_cali(&gimbal_auto_control_set->gimbal_cmd_slow_set_vx, rc_add_yaw);	
  
  if(gimbal_auto_control_set->gimbal_cmd_slow_set_vx.out>2.f)gimbal_auto_control_set->gimbal_cmd_slow_set_vx.out=2.f;
	else if(gimbal_auto_control_set->gimbal_cmd_slow_set_vx.out<-2.f)gimbal_auto_control_set->gimbal_cmd_slow_set_vx.out=-2.f;
	
}

void shoot_auto_judge(Gimbal_Control_t *gimbal_auto_shoot_control_set)
{
	
	if(gimbal_auto_shoot_control_set->gimbal_Auto.yaw_offset<2&&gimbal_auto_shoot_control_set->gimbal_Auto.yaw_offset>-2)
	{
		if(gimbal_auto_shoot_control_set->gimbal_Auto.pitch_offset<1&&gimbal_auto_shoot_control_set->gimbal_Auto.pitch_offset>-1)
		{
			gimbal_auto_shoot_control_set->gimbal_Auto.shoot_ready=1;
		}
		
  }
	else
	{
		gimbal_auto_shoot_control_set->gimbal_Auto.shoot_ready=0;
	}
	
}
