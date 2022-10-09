/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot_task.c/h
  * @brief      �������.
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

#include "shoot_task.h"
#include "chassis_task.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "tim.h"
#include "stm32.h"
														//��������Ħ����
#define shoot_fric(speed) 	shoot_control.fric_left_speed_set = -speed;\
														shoot_control.fric_right_speed_set = speed    
#define trigger_motor(speed)				shoot_control.trigger_speed_set = -speed //�����������
#define third_fric(speed)		shoot_control.fric_b_speed_set = -speed 	//���������������
//�г̿���IO
#define BUTTEN_TRIG_PIN     HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)


static void shoot_init(void);
/**
  * @brief          Ħ����ģʽ�л�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ����ٶȼ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_control_loop(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);
/**
  * @brief          �������̻ز�
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);





shoot_control_t shoot_control;          //�������
static int16_t s_can_set_current = 0 ,left_can_set_current = 0, right_can_set_current = 0, trigger_can_set_current = 0;
/**
  * @brief          �������
  * @param[in]      void
  * @retval         ����can����ֵ
  */
void shoot_task(void const *pvParameters)
{
		vTaskDelay(SHOOT_TASK_INIT_TIME);
		shoot_init();
		while(1)
		{
				shoot_set_mode();
				shoot_feedback_update();
				shoot_control_loop();		 //���÷���������
			  if (!(toe_is_error(TRIGGER_MOTOR_TOE) && toe_is_error(FRIC_LEFT_MOTOR_TOE) && toe_is_error(FRIC_RIGHT_MOTOR_TOE)))
        {
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_shoot(0, 0, 0, 0);
            }
            else
            {
                CAN_cmd_shoot(s_can_set_current ,left_can_set_current, right_can_set_current, trigger_can_set_current);
            }
        }
				vTaskDelay(SHOOT_CONTROL_TIME_MS);
		}
}

/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
		//���PID��ʼ��
		static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 fric_b_pid[3] = {FRIC_S_MOTOR_SPEED_PID_KP, FRIC_S_MOTOR_SPEED_PID_KI, FRIC_S_MOTOR_SPEED_PID_KD};
    static const fp32 fric_left_pid[3] = {FRIC_LEFT_MOTOR_SPEED_PID_KP, FRIC_LEFT_MOTOR_SPEED_PID_KI, FRIC_LEFT_MOTOR_SPEED_PID_KD};
		static const fp32 fric_right_pid[3] = {FRIC_RIGHT_MOTOR_SPEED_PID_KP, FRIC_RIGHT_MOTOR_SPEED_PID_KI, FRIC_RIGHT_MOTOR_SPEED_PID_KD};
		
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
    shoot_control.trigger_motor_measure = get_trigger_motor_measure_point();
		shoot_control.fric_b_motor_measure = get_can_2006_measure_point();
		shoot_control.fric_left_motor_measure = get_can_3508_left_measure_point();
		shoot_control.fric_right_motor_measure = get_can_3508_right_measure_point();
    //��ʼ��PID
		PID_init(&shoot_control.trigger_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);		
		PID_init(&shoot_control.bullet_pid, PID_POSITION, fric_b_pid, FRIC_S_MOTOR_SPEED_PID_MAX_OUT, FRIC_S_MOTOR_SPEED_PID_MAX_IOUT);		
		PID_init(&shoot_control.fric_left_pid, PID_POSITION, fric_left_pid, FRIC_LEFT_MOTOR_SPEED_PID_MAX_OUT, FRIC_LEFT_MOTOR_SPEED_PID_MAX_IOUT);
		PID_init(&shoot_control.fric_right_pid, PID_POSITION, fric_right_pid, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_OUT, FRIC_RIGHT_MOTOR_SPEED_PID_MAX_IOUT);
    //��������
		shoot_control.shoot_flag = 0;
		shoot_control.bullet_flag = 0;
		shoot_control.shoot_continu_flag = 0;
		shoot_control.stuck_flag = 0;
		shoot_control.reverse_time = 0;
		shoot_control.shoot_time = 150;
		
		shoot_control.trigger_given_current = 0;
    shoot_control.trigger_speed = 0.0f;
    shoot_control.trigger_speed_set = 0.0f;
		shoot_control.fric_b_speed = 0.0f;
    shoot_control.fric_b_speed_set = 0.0f;
		shoot_control.fric_left_speed = 0.0f;
    shoot_control.fric_left_speed_set = 0.0f;
		shoot_control.fric_right_speed = 0.0f;
    shoot_control.fric_right_speed_set = 0.0f;
		
		shoot_control.trigger_angle = 0;
		shoot_control.trigger_angle_set = shoot_control.trigger_angle;
}
/**
  * @brief          ���״̬������
  * @param[in]      void
  * @retval         void
  */
int8_t R = 0;
int s=2000,l;
static void shoot_set_mode(void)
{
		static int8_t press_l_last_s = 0;
		static uint16_t press_R_time = 0;
		fp32 fric_speed;

		//���̿��Ƴ���R������Ħ����
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_R)
		{
				press_R_time ++;
		}
		if(press_R_time > 500) 
		{
				R=!R;
				press_R_time = 0;
		}
	#if SHOOT_THIRD_MODE
		
		if ((switch_is_up(shoot_control.shoot_rc->rc.s[1]) || R) && robot_state.mains_power_shooter_output)
    {
				laser_on();
				trigger_motor_turn_back();
				//���ݲ���ϵͳ ���Ƶ���
				switch (robot_state.shooter_id1_42mm_speed_limit)
				{
						case 10:
						{
								fric_speed = 4200;
								break;
						}					
						case 16:
						{
								fric_speed = 5900;
								break;
						}
						default:
						{
								fric_speed = 5900;
								break;
						}
				}
				shoot_fric(fric_speed);
				if(shoot_control.stuck_flag == 0)//�޿���
				{
						//����
						if(!BUTTEN_TRIG_PIN && shoot_control.fric_b_motor_measure->given_current >= 100)  trigger_motor(0.0f);
						else if(BUTTEN_TRIG_PIN)	trigger_motor(4.0f);
					
						if(shoot_control.shoot_rc->rc.ch[4] < 120)
						{
								shoot_control.bullet_flag = 1;
						}
						//����
						if(shoot_control.bullet_flag == 1 && (shoot_control.shoot_rc->rc.ch[4] > 600 || (!press_l_last_s && shoot_control.press_l)) &&
							!BUTTEN_TRIG_PIN && (robot_state.shooter_id1_42mm_cooling_limit - power_heat_data_t.shooter_id1_42mm_cooling_heat >= 100))
						{
								shoot_control.shoot_flag = 1;
								shoot_control.bullet_flag = 0;
						}
				}
				else if(shoot_control.stuck_flag == 1)//����
				{
						third_fric(0);	
						trigger_motor(-3.0f);
				}
    }
		else
		{
				laser_off();
				shoot_fric(0);
				third_fric(0);	
				trigger_motor(0.0f);
		}
		
		if(shoot_control.shoot_flag ==1)
		{
				shoot_control.shoot_time = 0;
				shoot_control.shoot_flag = 0;
		}
		//��������
		if(shoot_control.shoot_time < 50)
		{
				third_fric(15000);
				trigger_motor(8.0f);
		}
		else
		{
				third_fric(0);
		}
		
	#else
	
		if ((switch_is_up(shoot_control.shoot_rc->rc.s[1]) || R) && robot_state.mains_power_shooter_output)
    {
				laser_on();
				trigger_motor_turn_back();
				//���ݲ���ϵͳ ���Ƶ���
				switch (robot_state.shooter_id1_42mm_speed_limit)
				{
						case 10:
						{
								fric_speed = 4100;
								break;
						}					
						case 16:
						{
								fric_speed = 5900;//14->4900  16->5900
								break;
						}
						default:
						{
								fric_speed = 5900;
								break;
						}
				}
				shoot_fric(fric_speed);
				
				if(shoot_control.stuck_flag == 0)//�޿���
				{
						//����
						if(!BUTTEN_TRIG_PIN)  	trigger_motor(0.0f);
						else if(BUTTEN_TRIG_PIN)		trigger_motor(4.0f);;
					
						if(shoot_control.shoot_rc->rc.ch[4] < 120)
						{
								shoot_control.bullet_flag = 1;
						}
						//����
						if(shoot_control.bullet_flag == 1 && (shoot_control.shoot_rc->rc.ch[4] > 600 || (!press_l_last_s && shoot_control.press_l)) &&
							!BUTTEN_TRIG_PIN && (robot_state.shooter_id1_42mm_cooling_limit - power_heat_data_t.shooter_id1_42mm_cooling_heat >= 100)) 
						{
								shoot_control.shoot_flag = 1;
								shoot_control.bullet_flag = 0;
						}
				}
				else if(shoot_control.stuck_flag == 1)//����
				{
						third_fric(0);	
						trigger_motor(-3.0f);
				}
    }
		else
		{
				laser_off();
				shoot_fric(0);
				third_fric(0);	
				trigger_motor(0);
		}
		
		if(shoot_control.shoot_flag ==1)
		{
				shoot_control.shoot_time = 0;
				shoot_control.shoot_flag = 0;
		}
		//����
		if(shoot_control.shoot_time < 40)
		{
				trigger_motor(6.0f);
		}
		
	#endif
		
		shoot_control.shoot_time++;
		
		if(shoot_control.shoot_time >= 150) shoot_control.shoot_time = 150;
		
		press_l_last_s = shoot_control.press_l;

}

/**
  * @brief          ����ٶȼ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_control_loop(void)
{
			//����pid
			PID_calc(&shoot_control.trigger_pid, shoot_control.trigger_speed, shoot_control.trigger_speed_set);
			PID_calc(&shoot_control.bullet_pid, shoot_control.fric_b_speed, shoot_control.fric_b_speed_set);
			PID_calc(&shoot_control.fric_left_pid, shoot_control.fric_left_speed, shoot_control.fric_left_speed_set);
			PID_calc(&shoot_control.fric_right_pid, shoot_control.fric_right_speed, shoot_control.fric_right_speed_set);
			trigger_can_set_current = shoot_control.trigger_pid.out;
			s_can_set_current = shoot_control.bullet_pid.out;
			left_can_set_current = shoot_control.fric_left_pid.out;
			right_can_set_current = shoot_control.fric_right_pid.out;
}

/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
	
    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.trigger_speed = speed_fliter_3;
		
		//����ٶȸ���
		shoot_control.fric_b_speed = shoot_control.fric_b_motor_measure->speed_rpm;
		shoot_control.fric_left_speed = shoot_control.fric_left_motor_measure->speed_rpm;
		shoot_control.fric_right_speed = shoot_control.fric_right_motor_measure->speed_rpm;
		
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
		
}


/**
  * @brief          �������̻ز�
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void)
{
			//���ݵ���ֵ��ʱ���ж��Ƿ񿨵�
			if(trigger_can_set_current < -7000.0f)
			{
					shoot_control.block_time ++;
					if(shoot_control.block_time > 800)
					{	
							shoot_control.stuck_flag = 1;
							shoot_control.block_time = 0;
					}
			}
			else
			{
					shoot_control.block_time = 0;
			}
			//�����ز�ʱ��
			if(shoot_control.stuck_flag == 1)
			{
					shoot_control.reverse_time ++;
					if(shoot_control.reverse_time > 150)
					{
							shoot_control.reverse_time = 0;
							shoot_control.stuck_flag = 0;
					}
			}
}
