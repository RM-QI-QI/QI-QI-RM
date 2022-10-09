#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"

typedef struct {
	int32_t yaw_fifo;//YAW��Ϣ
	int32_t pitch_fifo;//PITCH��Ϣ
	int32_t yaw_speed_fifo;//YAW�ٶ���Ϣ
	int32_t pitch_speed_fifo;//PITCH�ٶ���Ϣ
	int32_t distance_fifo;
	bool_t rx_change_flag;//ʶ��Ŀ���л�
	bool_t rx_flag;//ʶ��Ŀ��
	
	bool_t rx_update_flag;//�Ӿ�����
	
	fp32 yaw_add;//YAW��Ϣ
	fp32 pitch_add;//PITCH��Ϣ
	fp32 distance;
	fp32 rev;
	bool_t change_flag;
	bool_t success_flag;//ʶ��ɹ���־
	
	fp32 yaw_add_last;//YAW��Ϣ
	fp32 pitch_add_last;//PITCH��Ϣ
	fp32 distance_last;
	bool_t change_flag_last;
	bool_t success_flag_last;//ʶ��ɹ���־
	
	fp32 yaw_add_new;//YAW��Ϣ
	fp32 pitch_add_new;//PITCH��Ϣ
	fp32 distance_new;
	bool_t change_flag_new;
	bool_t success_flag_new;//ʶ��ɹ���־
	
	bool_t vision_task_update_flag;
	
} vision_rxfifo_t;


#define VISION_RX_LEN_2 58u
#define VISION_RX_LEN 29u

#define HEAD0_BASE 0
#define HEAD1_BASE 4
#define YAW_FIFO_BASE 8
#define PITCH_FIFO_BASE 12
#define DISTANCE 16
#define REV 20
#define CHANGE_FLAG_FIFO_BASE 21

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);

extern vision_rxfifo_t vision_rxfifo;
extern uint8_t vision_rx_buf[2][VISION_RX_LEN_2];


extern void vision_init(void);
extern void vision_rx_decode(uint8_t *test_code);
extern vision_rxfifo_t *get_vision_fifo(void);
#endif
