/*********************************************************************************
FILE NAME:sdio_sdcard.c

SDIO�ӿ�SD�����������HAL��ʹ��
									��֧�ֵ���SD��
									��������ԭ��̽���߿���������
									
									��Fatfs�ǵ����ɹ���ʱ��Heap�ռ䣡������
									SDIOʱ��Ƶ�ʲ�Ҫ����25MHz
									����SDIOʱ�ǵ���CubeMX���̸߼�ѡ�ѡ��Ҫ����SDIO��ʼ��������
				
				From:@rjgawuie
*********************************************************************************/

#include "sdio_sdcard.h"
#include "string.h"
#include "sdio.h"


HAL_SD_CardInfoTypeDef  SDCardInfo;         //SD����Ϣ�ṹ��
//DMA_HandleTypeDef SDTxDMAHandler,SDRxDMAHandler;    //SD��DMA���ͺͽ��վ��

//SD_ReadDisk/SD_WriteDisk����ר��buf,�����������������ݻ�������ַ����4�ֽڶ����ʱ��,
//��Ҫ�õ�������,ȷ�����ݻ�������ַ��4�ֽڶ����.
//__align(4) uint8_t  SDIO_DATA_BUFFER[512];

//SD����ʼ��
//����ֵ:0 ��ʼ����ȷ������ֵ����ʼ������
uint8_t  SD_Init(void)
{
	HAL_SD_DeInit(&hsd);
  
	hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK) return 1;
	
	//��ȡSD����Ϣ
	if(HAL_SD_GetCardInfo(&hsd,&SDCardInfo)!=HAL_OK) return 2;
    
  if (HAL_SD_ConfigWideBusOperation(&hsd,SDIO_BUS_WIDE_4B)) return 2;//ʹ�ܿ�����ģʽ

  return 0;
}


//�õ�����Ϣ
//cardinfo:����Ϣ�洢��
//����ֵ:����״̬
uint8_t  SD_GetCardInfo(HAL_SD_CardInfoTypeDef *cardinfo)
{
    uint8_t  sta;
    sta=HAL_SD_GetCardInfo(&hsd,cardinfo);
    return sta;
}

//�ж�SD���Ƿ���Դ���(��д)����
//����ֵ:SD_TRANSFER_OK ������ɣ����Լ�����һ�δ���
//		 SD_TRANSFER_BUSY SD����æ�������Խ�����һ�δ���
uint8_t  SD_GetCardState(void)
{
  return((HAL_SD_GetCardState(&hsd)==HAL_SD_CARD_TRANSFER )?SD_TRANSFER_OK:SD_TRANSFER_BUSY);
}

 
//��SD��
//buf:�����ݻ�����
//sector:������ַ
//cnt:��������	
//����ֵ:����״̬;0,����;����,�������;
uint8_t  SD_ReadDisk(uint8_t * buf,uint32_t  sector,uint32_t  cnt)
{
	uint8_t  sta=HAL_OK;
	uint32_t  timeout=SD_TIMEOUT;
  long long lsector=sector;

	sta=HAL_SD_ReadBlocks(&hsd, (uint8_t*)buf,lsector,cnt,SD_TIMEOUT);//���sector�Ķ�����
	
	//�ȴ�SD������
	while(SD_GetCardState()!=SD_TRANSFER_OK)
    {
			if(timeout-- == 0)
			{	
				sta=SD_TRANSFER_BUSY;
			}
    }

    return sta;
}  


//дSD��
//buf:д���ݻ�����
//sector:������ַ
//cnt:��������	
//����ֵ:����״̬;0,����;����,�������;	
uint8_t  SD_WriteDisk(uint8_t  *buf,uint32_t  sector,uint32_t  cnt)
{   
    uint8_t  sta=HAL_OK;
	uint32_t  timeout=SD_TIMEOUT;
    long long lsector=sector;

	sta=HAL_SD_WriteBlocks(&hsd,(uint8_t*)buf,lsector,cnt,SD_TIMEOUT);//���sector��д����
		
	//�ȴ�SD��д��
	while(SD_GetCardState()!=SD_TRANSFER_OK)
    {
		if(timeout-- == 0)
		{	
			sta=SD_TRANSFER_BUSY;
		}
    }    

    return sta;
}

