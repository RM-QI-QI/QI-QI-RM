#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"
#include "gimbal_task.h"

typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //��̨����
  GIMBAL_INIT,           //��̨��ʼ��
  GIMBAL_CALI,           //��̨У׼
  GIMBAL_RELATIVE_ANGLE, //��̨�������ֵ��ԽǶȿ���
  GIMBAL_ABSOLUTE_ANGLE, //��̨�����Ǿ��ԽǶȿ���
  GIMBAL_MOTIONLESS,     //��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��
  GIMBAL_AUTO,            //����
} gimbal_behaviour_e;

extern void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set);

extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set);
extern void gimbal_auto_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_auto_control_set);
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern void gimbal_auto_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_auto_control_set);
#endif
