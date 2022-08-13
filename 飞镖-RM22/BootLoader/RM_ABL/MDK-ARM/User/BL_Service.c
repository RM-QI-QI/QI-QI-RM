/***********************************************************
�ļ�����BL_Service.c
������
		bootloader ����ʵ��
From:Qi-Q@Rjgawuie
***********************************************************/
#include "BL_Service.h"

Circle_Buf_Typedef cb;  //���λ��������



/***********************************************************
��������Data_Send
���ܣ����ݷ���
������data ����������
����ֵ: ��
***********************************************************/
void Data_Send(uint8_t data)
{
	LL_USART_TransmitData8(USED_UART_HANDLER,data);
	while(LL_USART_IsActiveFlag_TXE(USED_UART_HANDLER)==0);
}

/***********************************************************
��������FLASH_Check
���ܣ����FLASHָ����ַ�Ƿ�Ϊ��
������check_address �����Ŀ�ʼ��ַ
			Size					�����λ�õĴ�С
����ֵ: 0 FLASH��(ȫΪ0xFF���)
				1 FLASH��Ϊ��
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
��������Sector_Select
���ܣ�����ַת��Ϊ������
������FL_addr ��ת���ĵ�ַ
����ֵ: ������
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
��������Data_recv
���ܣ����ݽ��ռ�����/��¼ʵ��
��������
����ֵ: ��
***********************************************************/
void Data_recv()
{
	uint8_t Data[Sector_Size]; //���ݻ�����
	uint8_t Databuf[Circle_Buf_Size]; //���ڻ��λ�����
	uint8_t Prg_Flag = 0;  //�ѱ�̱�־
	uint32_t sector_size=Sector_Size;  //������С����
	uint16_t version = Version;
	
	uint32_t addrlast;  //����ַ
	uint32_t addr;
	
	uint8_t checksum = 0;  //У���
	uint8_t seq=0,temp; //����ţ���ʱ����
	
	FLASH_EraseInitTypeDef earse;  //�����������
	uint32_t error; //���������־
					
	//��һ�׶Σ���ʼ��/�ȴ����ݽ���
	CircleBuf_Init(&cb,512,(char *)Databuf);
	LL_USART_EnableIT_RXNE(USED_UART_HANDLER);
	HAL_Delay(70);
	
	if(cb.count>0) //�����ݽ��룬����
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
				while(temp!=0x04&&temp!=0x05&&temp!=0x10) //�˵���ʼ��־ǰ���ַ�
					{
						if(cb.count>0)
						{
							temp = CircleBuf_Read(&cb);
						}
					}
				if(temp == 0x04)  //��⵽��ʼ��־����ʼ������һ������
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
				else if(temp == 0x05&&Prg_Flag == 0)  //��⵽��������־
					{//ˢд���֣���ͬϵ����Ҫ��д�ò��ִ���
						if(addrlast >= FL_ADDR_START&&addrlast <= FL_ADDR_END)
						{
							HAL_FLASH_Unlock();
							
							if(FLASH_Check(addrlast,sector_size))  //��鲻Ϊ�գ�����
							{
								earse.TypeErase = FLASH_TYPEERASE_SECTORS;
								earse.Sector = Sector_Select(addrlast);
								earse.NbSectors = 1;
								earse.VoltageRange = FLASH_VOLTAGE_RANGE_3;
								
								__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR|FLASH_FLAG_WRPERR);
								HAL_FLASHEx_Erase(&earse,&error);
							}
							
							
							for(uint32_t k=0;k<sector_size;k++) //��ʼ���
							{
									FLASH_WaitForLastOperation(50000U);
									HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,addrlast+k,Data[k]);
							}
							
							HAL_FLASH_Lock();
							
							//��̽��������ͱ�־
							Prg_Flag = 1;
							Data_Send(0x09);
						}
						else	Data_Send(0x11);
					}
				else if(temp == 0x10)  //��⵽������־
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
��������TransferControl
���ܣ��˳�BootLoader��ת������Ȩ
��������
����ֵ: ��
***********************************************************/
void TransferControl()
{
	static pFunction jump_to_application;
  uint32_t jump_address;
	uint32_t app_addr = APP_ADDRESS;
	
	/* �����ת�ĵ�ַ�Ƿ���Ч */
	if (((*(__IO uint32_t*)app_addr) & 0x2FFE0000 ) == 0x20000000)
	{
		/* �ر������жϣ����ָ����������ʼ�� */
		LL_USART_DeInit(USED_UART_HANDLER);
		LL_GPIO_DeInit(GPIOG);
		LL_RCC_DeInit();
		NVIC_DisableIRQ(SysTick_IRQn);
		/* ׼����ת����ָ�뼰ջ����ַ */
		jump_address = *(__IO uint32_t*) (app_addr + 4);
		jump_to_application = (pFunction) jump_address;
		/* ����ƫ������ջ����ַ����ת��ָ����ַ */
		SCB->VTOR = app_addr;
		__set_MSP(jump_address);
		jump_to_application();
	} 
}
/***********************************************************
��������IRQ_Rec
���ܣ������жϴ���,��Ҫע�ᵽ�жϺ���
��������
����ֵ: ��
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
