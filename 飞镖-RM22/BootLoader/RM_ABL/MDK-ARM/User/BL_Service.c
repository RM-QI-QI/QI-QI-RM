/***********************************************************
文件名：BL_Service.c
描述：
		bootloader 功能实现
From:Qi-Q@Rjgawuie
***********************************************************/
#include "BL_Service.h"

Circle_Buf_Typedef cb;  //环形缓冲区句柄



/***********************************************************
函数名：Data_Send
功能：数据发送
参数：data 待发送数据
返回值: 无
***********************************************************/
void Data_Send(uint8_t data)
{
	LL_USART_TransmitData8(USED_UART_HANDLER,data);
	while(LL_USART_IsActiveFlag_TXE(USED_UART_HANDLER)==0);
}

/***********************************************************
函数名：FLASH_Check
功能：检查FLASH指定地址是否为空
参数：check_address 待检查的开始地址
			Size					待检查位置的大小
返回值: 0 FLASH空(全为0xFF填充)
				1 FLASH不为空
***********************************************************/
uint8_t FLASH_Check(uint32_t check_address,uint32_t Size)
{
	for(uint32_t i=0;i<Size;i++)
	{
		if(*(__IO uint8_t *)(check_address+i) != 0xFF) return 1;
	}
	return 0;
}

/***********************************************************
函数名：Sector_Select
功能：将地址转换为扇区号
参数：FL_addr 待转换的地址
返回值: 扇区号
***********************************************************/
uint32_t Sector_Select(uint32_t FL_addr)
{
	uint32_t sector_num;
	sector_num = (FL_addr - 0x08000000)/0x20000;
	
	if(sector_num == 0)
	{
		for(sector_num = 0;sector_num < 4;sector_num++)
		{
			if(FL_addr >= 0x08000000+0x4000*sector_num && FL_addr < 0x08004000+0x4000*sector_num)
			break;
		}
	}
	else if(sector_num == 8)
	{
		for(sector_num = 0;sector_num < 4;sector_num++)
		{
			if(FL_addr >= 0x08100000+0x4000*sector_num && FL_addr < 0x08104000+0x4000*sector_num)
				break;
		}
		sector_num += 12;
	}
	else if(sector_num >= 1 && sector_num <= 7)
		sector_num += 4;
	else
		sector_num += 8;
	return sector_num;
}

/***********************************************************
函数名：Data_recv
功能：数据接收及解算/烧录实现
参数：无
返回值: 无
***********************************************************/
void Data_recv()
{
	uint8_t Data[Sector_Size]; //数据缓冲区
	uint8_t Databuf[Circle_Buf_Size]; //串口环形缓冲区
	uint8_t Prg_Flag = 0;  //已编程标志
	uint32_t sector_size=Sector_Size;  //扇区大小变量
	uint16_t version = Version;
	
	uint32_t addrlast;  //最后地址
	uint32_t addr;
	
	uint8_t checksum = 0;  //校验和
	uint8_t seq=0,temp; //包序号，临时变量
	
	FLASH_EraseInitTypeDef earse;  //擦除函数句柄
	uint32_t error; //擦除错误标志
					
	//第一阶段，初始化/等待数据进入
	CircleBuf_Init(&cb,512,(char *)Databuf);
	LL_USART_EnableIT_RXNE(USED_UART_HANDLER);
	HAL_Delay(70);
	
	if(cb.count>0) //有数据进入，发送
	{
		
		while(CircleBuf_Read(&cb) != 0x03)
		{
			while(cb.count==0);
		}
		Data_Send(0x03);
		Data_Send((sector_size>>24)&0xFF);
		Data_Send((sector_size>>16)&0xFF);
		Data_Send((sector_size>>8)&0xFF);
		Data_Send(sector_size&0xFF);
		Data_Send((version>>8)&0xFF);
		Data_Send(version&0xFF);
		Data_Send((sector_size>>24)+(sector_size>>16)+(sector_size>>8)+(sector_size)+(version>>8)+version);

		
		while(1)
			{
				while(temp!=0x04&&temp!=0x05&&temp!=0x10) //滤掉开始标志前的字符
					{
						if(cb.count>0)
						{
							temp = CircleBuf_Read(&cb);
						}
					}
				if(temp == 0x04)  //检测到开始标志，开始接收下一包数据
					{
						
						for(int i=0;i<4;i++)
							{
								while(cb.count==0);
								
								temp = CircleBuf_Read(&cb);
								checksum += temp;
								addr = addr|(temp<<((3-i)*8));
							}
							addrlast = addr;
						while(cb.count == 0);
						
						seq = CircleBuf_Read(&cb);
						checksum += seq;
						
						for(int i=0;i<256;i++)
							{
								while(cb.count == 0);
								
								temp = CircleBuf_Read(&cb);
								checksum += temp;
								Data[i+seq*256] = temp;
							}
							
						while(cb.count == 0);
						
						temp = CircleBuf_Read(&cb);
						if(checksum == temp)
						{
							Data_Send(0x06);
						}
						else
						{
							Data_Send(0x07);
						}
						Prg_Flag = 0;
						addr = 0;
						checksum = 0;
					}
				else if(temp == 0x05&&Prg_Flag == 0)  //检测到包结束标志
					{//刷写部分，不同系列需要改写该部分代码
						if(addrlast >= FL_ADDR_START&&addrlast <= FL_ADDR_END)
						{
							HAL_FLASH_Unlock();
							
							if(FLASH_Check(addrlast,sector_size))  //检查不为空，擦除
							{
								earse.TypeErase = FLASH_TYPEERASE_SECTORS;
								earse.Sector = Sector_Select(addrlast);
								earse.NbSectors = 1;
								earse.VoltageRange = FLASH_VOLTAGE_RANGE_3;
								
								__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR|FLASH_FLAG_WRPERR);
								HAL_FLASHEx_Erase(&earse,&error);
							}
							
							
							for(uint32_t k=0;k<sector_size;k++) //开始编程
							{
									FLASH_WaitForLastOperation(50000U);
									HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,addrlast+k,Data[k]);
							}
							
							HAL_FLASH_Lock();
							
							//编程结束，发送标志
							Prg_Flag = 1;
							Data_Send(0x09);
						}
						else	Data_Send(0x11);
					}
				else if(temp == 0x10)  //检测到结束标志
					{
						HAL_NVIC_SystemReset();
						break;
					}
				temp = 0;
		}
	}
	
	LL_USART_DisableIT_RXNE(USED_UART_HANDLER);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	CircleBuf_DeInit(&cb);
}
/***********************************************************
函数名：TransferControl
功能：退出BootLoader并转交控制权
参数：无
返回值: 无
***********************************************************/
void TransferControl()
{
	static pFunction jump_to_application;
  uint32_t jump_address;
	uint32_t app_addr = APP_ADDRESS;
	
	/* 检查跳转的地址是否有效 */
	if (((*(__IO uint32_t*)app_addr) & 0x2FFE0000 ) == 0x20000000)
	{
		/* 关闭所有中断，并恢复所有外设初始化 */
		LL_USART_DeInit(USED_UART_HANDLER);
		LL_GPIO_DeInit(GPIOG);
		LL_RCC_DeInit();
		NVIC_DisableIRQ(SysTick_IRQn);
		/* 准备跳转函数指针及栈顶地址 */
		jump_address = *(__IO uint32_t*) (app_addr + 4);
		jump_to_application = (pFunction) jump_address;
		/* 设置偏移量、栈顶地址并跳转到指定地址 */
		SCB->VTOR = app_addr;
		__set_MSP(jump_address);
		jump_to_application();
	} 
}
/***********************************************************
函数名：IRQ_Rec
功能：串口中断处理,需要注册到中断函数
参数：无
返回值: 无
***********************************************************/
void IRQ_Rec()
{
	uint8_t tmp;
	if(USED_UART_HANDLER->SR&USART_SR_RXNE)
	{
		tmp = USED_UART_HANDLER->DR;
		CircleBuf_Write(&cb,tmp);
		LL_USART_ClearFlag_RXNE(USED_UART_HANDLER);
	}
}
