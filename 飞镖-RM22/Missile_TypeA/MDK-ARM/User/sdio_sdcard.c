/*********************************************************************************
FILE NAME:sdio_sdcard.c

SDIO接口SD卡驱动，配合HAL库使用
									仅支持单颗SD卡
									改自正点原子探索者开发板例程
									
									用Fatfs记得生成工程时改Heap空间！！！！
									SDIO时钟频率不要超过25MHz
									配置SDIO时记得在CubeMX工程高级选项勾选不要调用SDIO初始化！！！
				
				From:@rjgawuie
*********************************************************************************/

#include "sdio_sdcard.h"
#include "string.h"
#include "sdio.h"


HAL_SD_CardInfoTypeDef  SDCardInfo;         //SD卡信息结构体
//DMA_HandleTypeDef SDTxDMAHandler,SDRxDMAHandler;    //SD卡DMA发送和接收句柄

//SD_ReadDisk/SD_WriteDisk函数专用buf,当这两个函数的数据缓存区地址不是4字节对齐的时候,
//需要用到该数组,确保数据缓存区地址是4字节对齐的.
//__align(4) uint8_t  SDIO_DATA_BUFFER[512];

//SD卡初始化
//返回值:0 初始化正确；其他值，初始化错误
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
	
	//获取SD卡信息
	if(HAL_SD_GetCardInfo(&hsd,&SDCardInfo)!=HAL_OK) return 2;
    
  if (HAL_SD_ConfigWideBusOperation(&hsd,SDIO_BUS_WIDE_4B)) return 2;//使能宽总线模式

  return 0;
}


//得到卡信息
//cardinfo:卡信息存储区
//返回值:错误状态
uint8_t  SD_GetCardInfo(HAL_SD_CardInfoTypeDef *cardinfo)
{
    uint8_t  sta;
    sta=HAL_SD_GetCardInfo(&hsd,cardinfo);
    return sta;
}

//判断SD卡是否可以传输(读写)数据
//返回值:SD_TRANSFER_OK 传输完成，可以继续下一次传输
//		 SD_TRANSFER_BUSY SD卡正忙，不可以进行下一次传输
uint8_t  SD_GetCardState(void)
{
  return((HAL_SD_GetCardState(&hsd)==HAL_SD_CARD_TRANSFER )?SD_TRANSFER_OK:SD_TRANSFER_BUSY);
}

 
//读SD卡
//buf:读数据缓存区
//sector:扇区地址
//cnt:扇区个数	
//返回值:错误状态;0,正常;其他,错误代码;
uint8_t  SD_ReadDisk(uint8_t * buf,uint32_t  sector,uint32_t  cnt)
{
	uint8_t  sta=HAL_OK;
	uint32_t  timeout=SD_TIMEOUT;
  long long lsector=sector;

	sta=HAL_SD_ReadBlocks(&hsd, (uint8_t*)buf,lsector,cnt,SD_TIMEOUT);//多个sector的读操作
	
	//等待SD卡读完
	while(SD_GetCardState()!=SD_TRANSFER_OK)
    {
			if(timeout-- == 0)
			{	
				sta=SD_TRANSFER_BUSY;
			}
    }

    return sta;
}  


//写SD卡
//buf:写数据缓存区
//sector:扇区地址
//cnt:扇区个数	
//返回值:错误状态;0,正常;其他,错误代码;	
uint8_t  SD_WriteDisk(uint8_t  *buf,uint32_t  sector,uint32_t  cnt)
{   
    uint8_t  sta=HAL_OK;
	uint32_t  timeout=SD_TIMEOUT;
    long long lsector=sector;

	sta=HAL_SD_WriteBlocks(&hsd,(uint8_t*)buf,lsector,cnt,SD_TIMEOUT);//多个sector的写操作
		
	//等待SD卡写完
	while(SD_GetCardState()!=SD_TRANSFER_OK)
    {
		if(timeout-- == 0)
		{	
			sta=SD_TRANSFER_BUSY;
		}
    }    

    return sta;
}

