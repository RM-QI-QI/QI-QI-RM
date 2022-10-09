#include "vision.h"
#include "math.h"
#include "gimbal_task.h"

#define G 9.8f
#define H 0.45f

fp32 Filter(fp32 get_ad);
fp32 distance_x;               	//水平距离
fp32 a,b,c,x1,x2,angle_set;			//一元二次方程系数、结果、目标角度

extern gimbal_control_t gimbal_control;

void vision_loop(vision_rxfifo_t *vision_rx)
{
		//接收更新
		vision_rx->yaw_add_new = (fp32)vision_rx->yaw_fifo/ 10000.0f - 180.0f;
		vision_rx->pitch_add_new = (fp32)vision_rx->pitch_fifo/ 10000.0f - 180.0f;
		vision_rx->distance_new = (fp32)vision_rx->distance_fifo/ 10000.0f;
		vision_rx->rev = (fp32)vision_rx->rev/ 10000.0f;
		vision_rx->change_flag_new = vision_rx->rx_change_flag;
		vision_rx->success_flag_new = vision_rx->rx_flag;
		
		//角度计算
		if(vision_rx->distance_new != vision_rx->distance_last && vision_rx->distance_new != 0)
		{
				vision_rx->distance = vision_rx->distance_new;
		}
		distance_x = sqrt(pow(vision_rx->distance,2) - pow(H,2));
		a = (-0.5 * G * pow(vision_rx->distance,2))/pow(9.5,2);
		b = distance_x;
		c = a - H;
		
		if (b*b - 4 * a*c < 0) 
		{
				x1 = 0;
				x2 = 0;
				angle_set = -3.14f; 
		}
		else if (b*b - 4 * a*c == 0) 
		{
				x1 = (-b + sqrt(b * b - 4 * a*c)) / (2 * a);
        x2 = (-b - sqrt(b * b - 4 * a*c)) / (2 * a);
			
				x1 = atan(x1);
				x2 = atan(x2);
			
				angle_set = -x1;
		}
		else if (b*b - 4 * a*c > 0) 
		{
				x1 = (-b + sqrt(b * b - 4 * a*c)) / (2 * a);
        x2 = (-b - sqrt(b * b - 4 * a*c)) / (2 * a);
			
				x1 = atan(x1);
				x2 = atan(x2);
				
				if(x1 < 3.14f && x1 > -3.14f)
				{
						angle_set = -x1;
				}
				else
				{
						angle_set = -x2;
				}
		}
		
		if(angle_set > 0.24f) angle_set = 0.24f;
		if(angle_set < -0.90f) angle_set = -0.90f;
	
		
		//增量计算
		if(!vision_rx->rx_update_flag)
		{
				vision_rx->yaw_add = vision_rx->yaw_add * 0.0001f;
		}
		if(vision_rx->rx_update_flag)
		{
				vision_rx->yaw_add = vision_rx->yaw_add_new * 0.003f;
				vision_rx->pitch_add = angle_set;
			
				vision_rx->rx_update_flag = 0;
		}

		
		if(vision_rx->change_flag)
		{
				vision_rx->yaw_add = 0.0f;
				vision_rx->pitch_add = 0.0f;
		}
		else if(!vision_rx->rx_flag)
		{
				vision_rx->yaw_add = 0.0f;
				vision_rx->pitch_add = 0.0f;
		}
		
		vision_rx->yaw_add = Filter(vision_rx->yaw_add);
		
		vision_rx->yaw_add_last = vision_rx->yaw_add_new;
		vision_rx->pitch_add_last = vision_rx->pitch_add_new;
		vision_rx->distance_last = vision_rx->distance_new;
		vision_rx->change_flag_last = vision_rx->change_flag_new;
		vision_rx->success_flag_last = vision_rx->success_flag_new;
		
}



fp32 coe[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // 加权系数表
fp32 sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // 加权系数和
fp32 filter_buf[12 + 1];
fp32 Filter(fp32 get_ad) {
  int i;
  fp32 filter_sum = 0;
  filter_buf[12] = get_ad;
  for(i = 0; i < 12; i++) {
    filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
    filter_sum += filter_buf[i] * coe[i];
  }
  filter_sum /= sum_coe;
  return filter_sum;
}

