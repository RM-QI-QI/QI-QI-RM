/**
  ******************************************************************************
  * File Name          : Moto.c
  * Description        : C620,C610_FOC Driver 
  ******************************************************************************
 **/
 
#include "Moto.h"
#include "bkp_sram.h"
#include "chassis_task.h"
 
 
Moto_DataTypedef Moto_Data1[8]={0};
Moto_DataTypedef Moto_Data2[8]={0};


	/**
  * @brief          电机控制函数
  * @param[in]      3508控制范围 -16384~0~16384 
  * @param[in]      6020控制范围 [-30000,30000]
  * @param[in]      2006控制范围  -10000~0~10000
  * @retval         none
	CAN_ID：
		CAN_MOTO_ALL_ID_LOW  0x200
    CAN_MOTO_ALL_ID_HIGH  0x1FF
    CAN_MOTO_ID_1  0x201
    CAN_MOTO_ID_2  0x202
    CAN_MOTO_ID_3  0x203
    CAN_MOTO_ID_4  0x204
    CAN_MOTO_ID_5  0x205
    CAN_MOTO_ID_6  0x206
    CAN_MOTO_ID_7  0x207
    CAN_MOTO_ID_8  0x208
  */
void CAN_CMD_MOTO(CAN_HandleTypeDef *CAN_ID,uint32_t MOTO_ID,int16_t data0, int16_t data1, int16_t data2, int16_t data3)
{
	  CAN_TxHeaderTypeDef  tx_message;
    uint8_t              can_send_data[8];
	
    uint32_t send_mail_box;
    tx_message.StdId = MOTO_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = (data0 >> 8);
    can_send_data[1] = data0;
    can_send_data[2] = (data1 >> 8);
    can_send_data[3] = data1;
    can_send_data[4] = (data2 >> 8);
    can_send_data[5] = data2;
    can_send_data[6] = (data3 >> 8);
    can_send_data[7] = data3;
    HAL_CAN_AddTxMessage(CAN_ID, &tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
	uint8_t CAN_rec[8];
	int32_t diff;

	if(hcan ->Instance == CAN1)    //判断是哪个CAN收到的消息
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, CAN_rec);
		
		Moto_Data1[rx_header.StdId-0x201].lastangle = Moto_Data1[rx_header.StdId-0x201].angle;
		
		Moto_Data1[rx_header.StdId-0x201].angle = CAN_rec[1]+(CAN_rec[0]<<8);
		Moto_Data1[rx_header.StdId-0x201].speed = CAN_rec[3]+(CAN_rec[2]<<8);
		Moto_Data1[rx_header.StdId-0x201].current = CAN_rec[5]+(CAN_rec[4]<<8);
		Moto_Data1[rx_header.StdId-0x201].temperate = CAN_rec[6];
		
		if(Moto_Data1[rx_header.StdId-0x201].empty == 0) 
		{
			Moto_Data1[rx_header.StdId-0x201].lastangle = Moto_Data1[rx_header.StdId-0x201].angle;
			Moto_Data1[rx_header.StdId-0x201].empty = 1;
		}
		diff = Moto_Data1[rx_header.StdId-0x201].angle - Moto_Data1[rx_header.StdId-0x201].lastangle;
		if(diff < -4096) diff += 8192;
		else if(diff > 4096) diff-=8192;
		Moto_Data1[rx_header.StdId-0x201].sum_angle += diff;
	}
	else if(hcan ->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, CAN_rec);
		
		Moto_Data2[rx_header.StdId-0x201].lastangle = Moto_Data2[rx_header.StdId-0x201].angle;
			
		Moto_Data2[rx_header.StdId-0x201].angle = CAN_rec[1]+(CAN_rec[0]<<8);
		Moto_Data2[rx_header.StdId-0x201].speed = CAN_rec[3]+(CAN_rec[2]<<8);
		Moto_Data2[rx_header.StdId-0x201].current = CAN_rec[5]+(CAN_rec[4]<<8);
		Moto_Data2[rx_header.StdId-0x201].temperate = CAN_rec[6];
		
		if(Moto_Data2[rx_header.StdId-0x201].empty == 0) 
		{
			Moto_Data2[rx_header.StdId-0x201].lastangle = Moto_Data2[rx_header.StdId-0x201].angle;
			Moto_Data2[rx_header.StdId-0x201].empty = 1;
		}

		diff = Moto_Data2[rx_header.StdId-0x201].angle - Moto_Data2[rx_header.StdId-0x201].lastangle;
		if(diff < -4096) diff += 8192;
		else if(diff > 4096) diff-=8192;
		Moto_Data2[rx_header.StdId-0x201].sum_angle += diff;
		
		if(rx_header.StdId == 0x203)
		{
			BKP_Handler->yaw_sum_angle = Moto_Data2[2].sum_angle;
			BKP_Handler->yaw_lastangle = Moto_Data2[2].lastangle;
			BKP_Handler->updatemark = 1;
		}
	}
}

/**
  * @brief          CAN过滤器初始化函数
  * @param[in]      h_can:CAN句柄指针
  * @retval         none
  */

HAL_StatusTypeDef CAN_Filter_Init(CAN_HandleTypeDef *h_can) 
{
  CAN_FilterTypeDef sFilterConfig;
 
  sFilterConfig.FilterBank = 2;   //chenal 0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //标识符屏蔽位模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //过滤器位宽为单个32位
  sFilterConfig.FilterIdHigh = 0x0000;  //标识符寄存器   
  sFilterConfig.FilterIdLow = 0x0000;   //标识符寄存器   
  //MASK bit 0 means don't care,bit 0 means match 
  sFilterConfig.FilterMaskIdHigh = 0x0000;   //屏蔽寄存器  //只存在于标识符屏蔽位模式中，在标识符列表模式中为标识符寄存器 
  sFilterConfig.FilterMaskIdLow = 0x0000;    //屏蔽寄存器                                 
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //FIFO0的中断和FIFO1的中断是不一样的，这里是把接收到的报文放入到FIFO0中
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;   //enable filter
  sFilterConfig.SlaveStartFilterBank = 0;    //为从属can选择开始的过滤库，对于单个CAN实例，这个参数没有意义
  if (HAL_CAN_ConfigFilter(h_can, &sFilterConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }
  //regist RX_IT
  if (HAL_CAN_ActivateNotification(h_can, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  //注册CAN_IT_RX_FIFO0_MSG_PENDING 对应的回调函数原型
  {
    return HAL_ERROR;
  }
 return HAL_OK;
}



