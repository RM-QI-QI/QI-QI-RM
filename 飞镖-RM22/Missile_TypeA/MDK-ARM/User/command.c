/***********************************************************
�ļ�����command.c
������
		FreeRTOS-�����������ű�ִ��
From:Qi-Q@Rjgawuie
***********************************************************/

#include "command.h"

#include "Stepper.h"
#include "chassis_task.h"
#include "Moto.h"
#include "usart.h"
#include "stdio.h"
#include "ff.h"
#include "sdio_sdcard.h"
#include "sdio.h"
#include "string.h"

int SD_exist = 0; 								//SD�����ڱ�־
FATFS fs;													//�ļ�ϵͳ����

char CMD_Buf[100];         //�������
Circle_Buf_Typedef uartbuf={0,0,0,0,0};  //���ڻ��λ�����
uint8_t CMD_Point=0;
uint8_t Process_mark=0;

/***********************************************************
��������SD_Insert
���ܣ�����SD������ʱ����(���жϺ���)
������None
����ֵ:1 ��ʼ��ʧ��
***********************************************************/
uint16_t SD_Insert()
{
	if(SD_NOT_Exist)
	{
		HAL_SD_DeInit(&hsd);
		SD_exist=0;
		return 0;
	}
	else
	{
		if(SD_Init()==0&&f_mount(&fs,"0:",1)==0)
		{
			SD_exist=1;
			return 0;
		}
		return 1;
	}
}
/***********************************************************
��������Para_Save
���ܣ���SD���������
������None
����ֵ: 1 �ļ��򿪴���
				2 �ļ���ȡ����
				3 �ļ�д�����
				10 SD��������
***********************************************************/
int Para_Save()
{
	FIL datafile;
	
	if(SD_exist)
	{
		//���ļ�
		if(f_open(&datafile,"0:config.txt",FA_OPEN_ALWAYS|FA_WRITE|FA_READ)) return 1;
		if(f_rewind(&datafile)) return 1;
		//д������ģʽ
		if(f_printf(&datafile,"#����ģʽ\n")<0) return 3;
		if(f_printf(&datafile,"RunMode %d\n",RunMode)<0) return 3;
		//д��PID����
		if(f_printf(&datafile,"#PID��������\n")<0) return 3;
		for(uint8_t i=0;i<12;i++) if(f_printf(&datafile,"PID ID=%d Type=%s P=%f I=%f D=%f MAXout=%f MAXiout=%f\n",i+1,pid_mode[i]==PID_POSITION?"PID_POSITION":"PID_DELTA",P_I_D[i].Kp,P_I_D[i].Ki,P_I_D[i].Kd,Maxout[i],Maxiout[i])<0) return 3;
		//д��Ƕ����ֵ
		if(f_printf(&datafile,"#�Ƕ�ƫ��ֵ\n")<0) return 3;
		for(uint8_t i=0;i<3;i++) if(f_printf(&datafile,"AngleErr ID=%d value=%f\n\n",i+1,Angle_error[i])<0) return 3;
		//д����̨����
		if(f_printf(&datafile,"#����Ϊ��̨�ߴ����\n")<0) return 3;
		
		if(f_printf(&datafile,"#���̳���\n")<0) return 3;
		if(f_printf(&datafile,"Length_Chassis %f\n",length_chassis)<0) return 3;
		
		if(f_printf(&datafile,"#����ƽ�泤��\n")<0) return 3;
		if(f_printf(&datafile,"Length_Launch %f\n",length_launch)<0) return 3;
		
		if(f_printf(&datafile,"#��̨֧�Ÿ˳���\n")<0) return 3;
		if(f_printf(&datafile,"Length_Brace %f\n",length_brace)<0) return 3;
		
		if(f_printf(&datafile,"#PITCH��ת�����\n")<0) return 3;
		if(f_printf(&datafile,"Screw_Pitch %f\n",screw_pitch)<0) return 3;
		
		if(f_printf(&datafile,"#˿������\n")<0) return 3;
		if(f_printf(&datafile,"Pitch_Error %f\n",pitch_error)<0) return 3;
		
		if(f_printf(&datafile,"#�����г̷�Χ\n")<0) return 3;
		if(f_printf(&datafile,"MAX_Length %f\n",max_length)<0) return 3;
		if(f_printf(&datafile,"MIN_Length %f\n",min_length)<0) return 3;
		
		if(f_printf(&datafile,"#�ϳ��������������ٶȲ���\n")<0) return 3;
		if(f_printf(&datafile,"Outpost_Speed %d %d %d %d %d %d\n",outpost_speed[0],outpost_speed[1],outpost_speed[2],outpost_speed[3],outpost_speed[4],outpost_speed[5])<0) return 3;
		if(f_printf(&datafile,"Outpost_Current %d %d %d %d %d %d\n",outpost_current[0],outpost_current[1],outpost_current[2],outpost_current[3],outpost_current[4],outpost_current[5])<0) return 3;
		if(f_printf(&datafile,"Base_Speed %d %d %d %d %d %d\n",base_speed[0],base_speed[1],base_speed[2],base_speed[3],base_speed[4],base_speed[5])<0) return 3;
		if(f_printf(&datafile,"Base_Current %d %d %d %d %d %d\n",base_current[0],base_current[1],base_current[2],base_current[3],base_current[4],base_current[5])<0) return 3;
		
		
		if(f_truncate(&datafile)) return 3;
		if(f_close(&datafile)) return 1;
		return 0;
	}
	else return 10;
}
/***********************************************************
��������Para_Load
���ܣ���SD����ȡ����
������None
����ֵ: 1 �ļ��򿪴���
				2 �ļ���ȡ����
				3 �ļ�д�����
				4 �ļ���������
				10 SD��������
***********************************************************/
int Para_Load()
{
	FIL datafile;
	char databuf[150],datacmd[50],buf1[20];
	int ID;
	float p,i,d,max,imax;
	
	if(SD_exist)
	{
		//���ļ�
		if(f_open(&datafile,"0:config.txt",FA_OPEN_EXISTING|FA_READ)) return 1;
		if(f_rewind(&datafile)) return 1;
		
		while(f_eof(&datafile)==0)
		{
			if(f_gets(databuf,sizeof(databuf),&datafile)==NULL) continue;
			sscanf(databuf,"%s",datacmd);
			
			if(datacmd[0]=='#') continue;
			else if(strcmp(datacmd,"PID")==0)
			{
				sscanf(databuf,"PID ID=%d Type=%s P=%f I=%f D=%f MAXout=%f MAXiout=%f\n",&ID,buf1,&p,&i,&d,&max,&imax);
				P_I_D[ID-1].Kp = p;
				P_I_D[ID-1].Ki = i;
				P_I_D[ID-1].Kd = d;
				Maxout[ID-1] = max;
				Maxiout[ID-1] = imax;
				if(strcmp(buf1,"PID_POSITION")==0) pid_mode[ID-1]=PID_POSITION;
				else pid_mode[ID-1]=PID_DELTA;
			}
			else if(strcmp(datacmd,"AngleErr")==0)
			{
				sscanf(databuf,"AngleErr ID=%d value=%f\n\n",&ID,&p);
				Angle_error[ID-1] = p;
			}
			else if(strcmp(datacmd,"RunMode")==0)
			{
				sscanf(databuf,"RunMode %d\n",&RunMode);
			}
			else if(strcmp(datacmd,"Length_Chassis")==0)
			{
				sscanf(databuf,"Length_Chassis %lf\n",&length_chassis);
			}
			else if(strcmp(datacmd,"Length_Launch")==0)
			{
				sscanf(databuf,"Length_Launch %lf\n",&length_launch);
			}
			else if(strcmp(datacmd,"Length_Brace")==0)
			{
				sscanf(databuf,"Length_Brace %lf\n",&length_brace);
			}
			else if(strcmp(datacmd,"Pitch_Error")==0)
			{
				sscanf(databuf,"Pitch_Error %lf\n",&pitch_error);
			}
			else if(strcmp(datacmd,"Screw_Pitch")==0)
			{
				sscanf(databuf,"Screw_Pitch %lf\n",&screw_pitch);
			}
			else if(strcmp(datacmd,"MAX_Length")==0)
			{
				sscanf(databuf,"MAX_Length %lf\n",&max_length);
			}
			else if(strcmp(datacmd,"MIN_Length")==0)
			{
				sscanf(databuf,"MIN_Length %lf\n",&min_length);
			}
			else if(strcmp(datacmd,"Outpost_Speed")==0)
			{
				sscanf(databuf,"Outpost_Speed %d %d %d %d %d %d\n",&outpost_speed[0],&outpost_speed[1],&outpost_speed[2],&outpost_speed[3],&outpost_speed[4],&outpost_speed[5]);
			}
			else if(strcmp(datacmd,"Outpost_Current")==0)
			{
				sscanf(databuf,"Outpost_Current %d %d %d %d %d %d\n",&outpost_current[0],&outpost_current[1],&outpost_current[2],&outpost_current[3],&outpost_current[4],&outpost_current[5]);
			}
			else if(strcmp(datacmd,"Base_Speed")==0)
			{
				sscanf(databuf,"Base_Speed %d %d %d %d %d %d\n",&base_speed[0],&base_speed[1],&base_speed[2],&base_speed[3],&base_speed[4],&base_speed[5]);
			}
			else if(strcmp(datacmd,"Base_Current")==0)
			{
				sscanf(databuf,"Base_Current %d %d %d %d %d %d\n",&base_current[0],&base_current[1],&base_current[2],&base_current[3],&base_current[4],&base_current[5]);
			}
			else return 4;
		}
		return 0;
	}
	else return 10;
}

/***********************************************************
��������Data_Keep
���ܣ���SD���������в���
������None
����ֵ: 1 �ļ��򿪴���
				2 �ļ���ȡ����
				3 �ļ�д�����
				10 SD��������
***********************************************************/
int Data_Keep()
{
	FIL datafile;
	uint32_t wbyte;
	
	if(SD_exist)
	{
		//���ļ�
		if(f_open(&datafile,"0:Missile_PD.dat",FA_OPEN_ALWAYS|FA_WRITE)) return 1;
		if(f_rewind(&datafile)) return 1;
		
		if(f_write(&datafile,(uint8_t *)BKP_Handler,sizeof(Backup_Typedef),&wbyte)) return 3;
		if(wbyte!=sizeof(Backup_Typedef)) return 3;  //д�벻����Ԥ��
		
		if(f_truncate(&datafile)) return 3;
		if(f_close(&datafile)) return 1;
		return 0;
	}
	return 10;
}
/***********************************************************
��������Data_Load
���ܣ���SD����ȡ���в���������SRAM
������None
����ֵ: 1 �ļ��򿪴���
				2 �ļ���ȡ����
				3 �ļ�д�����
				10 SD��������
***********************************************************/
int Data_Load()
{
	FIL datafile;
	uint32_t rbyte;
	
	if(SD_exist)
	{
		//���ļ�
		if(f_open(&datafile,"0:Missile_PD.dat",FA_OPEN_ALWAYS|FA_READ|FA_WRITE)) return 1;
		if(f_rewind(&datafile)) return 1;
		
		if(f_read(&datafile,(uint8_t *)BKP_Handler,sizeof(Backup_Typedef),&rbyte)) return 2;
		if(rbyte!=sizeof(Backup_Typedef)) return 2;  //���벻����Ԥ��
		
		if(f_rewind(&datafile)) return 1;
		if(f_write(&datafile,0,1,&rbyte)) return 3;
		if(f_close(&datafile)) return 1;
		return 0;
	}
	return 10;
}
/***********************************************************
��������Run_recv
���ܣ���SRAM�ָ���������
������None
����ֵ: 1 ���ڵ�����
***********************************************************/
int Run_recv()
{
	if(BKP_Handler->updatemark==1)
	{
		stepper1.sum_Step = BKP_Handler->pitch_sum_step;
		Moto_Data2[2].angle = BKP_Handler->yaw_lastangle;
		Moto_Data2[2].sum_angle = BKP_Handler->yaw_sum_angle;
		BKP_Handler->updatemark = 0;
		if(BKP_Handler->pitchcorrectmark==1)
		{
			BKP_Handler->pitchcorrectmark=0;
			Pitch_Init();
		}
		return 0;
	}
	return 1;
}

/***********************************************************
��������UART8_IRQProcess
���ܣ������������
������None
***********************************************************/
void UART8_IRQProcess()
{
	char buf;
	
	if(uartbuf.size>1)
	if(__HAL_UART_GET_FLAG(&huart8,UART_FLAG_RXNE)==SET)
	{
		buf=UART8->DR;
		if(buf == '!') {CircleBuf_Write(&uartbuf,buf); CMD_Point=1;}
		else if(buf == '@') 
				{
					CircleBuf_Write(&uartbuf,buf); 
					CircleBuf_Write(&uartbuf,'\0');
					CMD_Point=0;
					Process_mark++;
				}
		else if(CMD_Point) CircleBuf_Write(&uartbuf,buf);
	}
}

/***********************************************************
��������change_all_para
���ܣ����������
������None
***********************************************************/
void change_all_para()
{
	int state_val;
	int32_t ID=0,set;
	int32_t set_speed,Mo,Mi;
	float Value,P,I,D,angle;
	char buf[60];
	char ch;
	
	do
	{
		ch = CircleBuf_Read(&uartbuf);
		CMD_Buf[ID] = ch;
		ID++;
	}while(ch!='\0');
	CMD_Buf[ID] = '\0';
	
	
	switch(CMD_Buf[1])
	{
		case 'P':
			sscanf(CMD_Buf,"!P+%d+%f+%f+%f+%d+%d@",&ID,&P,&I,&D,&Mo,&Mi);
			switch(ID)
			{
				case 0:
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
				case 7:
				case 8:
				case 9:
				case 10:
				case 11:
					P_I_D[ID].Kp=P;
					P_I_D[ID].Ki=I;
					P_I_D[ID].Kd=D;
				  Maxout[ID]=Mo;
				  Maxiout[ID]=Mi;
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				case 12:
					for(int i=0;i<6;i++)
						{
							P_I_D[i].Kp=P;
							P_I_D[i].Ki=I;
							P_I_D[i].Kd=D;
							Maxout[i]=Mo;
							Maxiout[i]=Mi;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
						}
					break;
				case 13:
					for(int i=6;i<8;i++)
						{
							P_I_D[i].Kp=P;
							P_I_D[i].Ki=I;
							P_I_D[i].Kd=D;
							Maxout[i]=Mo;
							Maxiout[i]=Mi;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
						}
					break;
				case 14:
					for(int i=9;i<11;i++)
						{
							P_I_D[i].Kp=P;
							P_I_D[i].Ki=I;
							P_I_D[i].Kd=D;
							Maxout[i]=Mo;
							Maxiout[i]=Mi;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
						}
					break;
				case 15:
					if(P==1.0f)
					{
						switch(Mi)
						{
							case 1:
								P_I_D[Mo].Kp+=D;
								break;
							case 2:
								P_I_D[Mo].Ki+=D;
								break;
							case 3:
								P_I_D[Mo].Kd+=D;
								break;
							default:
								HAL_UART_Transmit(&huart8,"Failed\n",7,100);
								break;
								
						}
						HAL_UART_Transmit(&huart8,"Success\n",8,100);
					}
					else if(P==2.0f)
					{
						switch(Mi)
						{
							case 1:
								P_I_D[Mo].Kp-=D;
								break;
							case 2:
								P_I_D[Mo].Ki-=D;
								break;
							case 3:
								P_I_D[Mo].Kd-=D;
								break;
							default:
								HAL_UART_Transmit(&huart8,"Failed\n",7,100);
								break;
						}
						HAL_UART_Transmit(&huart8,"Success\n",8,100);
					}
					break;
				default :
					HAL_UART_Transmit(&huart8,"Failed\n",7,100);
					break;
			}
			break;
		case 'S':
			if(strcmp("!Save@",CMD_Buf)==0)
			{
				state_val = Para_Save();
				if(state_val) printf("Failed with return %d\n",state_val);//HAL_UART_Transmit(&huart8,"Failed\n",7,100);
				else HAL_UART_Transmit(&huart8,"Saved\n",6,100);
				break;
			}
			sscanf(CMD_Buf,"!S+%d+%d+%d@",&ID,&set,&set_speed);
			switch(ID)
			{
				case 0:
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
					switch(set)
					{
						case 1:
							speed[ID]+=set_speed;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						case 2:
							speed[ID]-=set_speed;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						case 3:
							speed[ID]=set_speed;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						default:
							HAL_UART_Transmit(&huart8,"Failed\n",7,100);
							break;
					}
					break;
				case 6:
					for(int i=0;i<6;i++)
						switch(set)
							{
								case 1:
									speed[i]+=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 2:
									speed[i]-=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 3:
									speed[i]=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								default:
									HAL_UART_Transmit(&huart8,"Failed\n",7,100);
									break;
							}
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				case 7:
						switch(set)
							{
								case 1:
									speed[0]+=set_speed;
									speed[1]+=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 2:
									speed[0]-=set_speed;
									speed[1]-=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 3:
									speed[0]=set_speed;
									speed[1]=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								default:
									HAL_UART_Transmit(&huart8,"Failed\n",7,100);
									break;
							}
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				case 8:
						switch(set)
							{
								case 1:
									speed[2]+=set_speed;
									speed[3]+=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 2:
									speed[2]-=set_speed;
									speed[3]-=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 3:
									speed[2]=set_speed;
									speed[3]=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								default:
									HAL_UART_Transmit(&huart8,"Failed\n",7,100);
									break;
							}
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				case 9:
						switch(set)
							{
								case 1:
									speed[4]+=set_speed;
									speed[5]+=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 2:
									speed[4]-=set_speed;
									speed[5]-=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 3:
									speed[4]=set_speed;
									speed[5]=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								default:
									HAL_UART_Transmit(&huart8,"Failed\n",7,100);
									break;
							}
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				case 10:
					sscanf(CMD_Buf,"!S+%d+%f@",&ID,&angle);
					if(Pitch_AngleSet(angle) == 0)	printf("Set Success\n");
					else  printf("Out of adjustable range!!\n");
					break;
				case 11:
					Move_Step(&stepper1,set);
					break;
				default:
					HAL_UART_Transmit(&huart8,"Failed\n",7,100);
					break;
			}
			break;
		case 'I':
			sscanf(CMD_Buf,"!I+%d+%d+%d@",&ID,&set,&set_speed);
			switch(ID)
			{
				case 0:
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
					switch(set)
					{
						case 1:
							current[ID]+=set_speed;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						case 2:
							current[ID]-=set_speed;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						case 3:
							current[ID]=set_speed;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						default:
							HAL_UART_Transmit(&huart8,"Failed\n",7,100);
							break;
					}
					break;
				case 6:
					for(int i=0;i<6;i++)
						switch(set)
							{
								case 1:
									current[i]+=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 2:
									current[i]-=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 3:
									current[i]=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								default:
									HAL_UART_Transmit(&huart8,"Failed\n",7,100);
									break;
							}
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				case 7:
						switch(set)
							{
								case 1:
									current[0]+=set_speed;
									current[1]+=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 2:
									current[0]-=set_speed;
									current[1]-=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 3:
									current[0]=set_speed;
									current[1]=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								default:
									HAL_UART_Transmit(&huart8,"Failed\n",7,100);
									break;
							}
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				case 8:
						switch(set)
							{
								case 1:
									current[2]+=set_speed;
									current[3]+=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 2:
									current[2]-=set_speed;
									current[3]-=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 3:
									current[2]=set_speed;
									current[3]=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								default:
									HAL_UART_Transmit(&huart8,"Failed\n",7,100);
									break;
							}
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				case 9:
						switch(set)
							{
								case 1:
									current[4]+=set_speed;
									current[5]+=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 2:
									current[4]-=set_speed;
									current[5]-=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								case 3:
									current[4]=set_speed;
									current[5]=set_speed;
									HAL_UART_Transmit(&huart8,"Success\n",8,100);
									break;
								default:
									HAL_UART_Transmit(&huart8,"Failed\n",7,100);
									break;
							}
					HAL_UART_Transmit(&huart8,"Success\n",8,100);
					break;
				default:
					HAL_UART_Transmit(&huart8,"Failed\n",7,100);
					break;
			}
			break;
		case 'T':
			sscanf(CMD_Buf,"!T+%d+%d+%f@",&ID,&set,&Value);
			switch(ID)
			{
				case 0:
				case 1:
				case 2:
					switch(set)
					{
						case 1:
							Angle_target[ID]+=Value;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						case 2:
							Angle_target[ID]-=Value;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						case 3:
							Angle_target[ID]=Value;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						default:
							HAL_UART_Transmit(&huart8,"Failed\n",7,100);
							break;
					}
					break;
				default:
					HAL_UART_Transmit(&huart8,"Failed\n",7,100);
				break;
			}
			break;
		case 'E':
			sscanf(CMD_Buf,"!E+%d+%d+%f@",&ID,&set,&Value);
			switch(ID)
			{
				case 0:
				case 1:
				case 2:
					switch(set)
					{
						case 1:
							Angle_error[ID]+=Value;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						case 2:
							Angle_error[ID]-=Value;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						case 3:
							Angle_error[ID]=Value;
							HAL_UART_Transmit(&huart8,"Success\n",8,100);
							break;
						default:
							HAL_UART_Transmit(&huart8,"Failed\n",7,100);
							break;
					}
					break;
				default:
					HAL_UART_Transmit(&huart8,"Failed\n",7,100);
				break;
			}
			break;
		case '!':
			sscanf(CMD_Buf,"!!%s",buf);
			if(strcmp(buf,"Shut")==0)		//����ͣ��
			{
				if(RunMode == 0)
					for(int i=0;i<6;i++) speed[i] = 0;
				else if(RunMode == 1)
					for(int i=0;i<6;i++) current[i] = 0;
			}
			else if(strcmp(buf,"Backpara")==0)
			{
				for(int i=0;i<12;i++) printf("PID%d %.3f %.3f %.3f %.0f %.0f\n",i+1,
																															P_I_D[i].Kp,
																															P_I_D[i].Ki,
																															P_I_D[i].Kd,
																															Maxout[i],
																															Maxiout[i]);
				for(int i=0;i<6;i++) printf("SpeedSet%d %d\n",i+1,speed[i]);
				for(int i=0;i<6;i++) printf("Current%d %d\n",i+1,current[i]);
			}
			else if(strcmp(buf,"CorrectPitch")==0)
			{
				BKP_Handler->pitchcorrectmark = 1;
				HAL_NVIC_SystemReset();
			}
			else if(strcmp(buf,"TurnLock")==0)
			{
				TurnLock();
			}
			else if(strcmp(buf,"TurnUnlock")==0)
			{
				TurnUnlock();
			}
			else if(strcmp(buf,"LaserOn")==0)
			{
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
			}
			else if(strcmp(buf,"LaserOff")==0)
			{
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
			}
			else if(strcmp(buf,"PIDMode")==0)
			{
				for(int i=0;i<6;i++)
				{
					current[i] = 0;
					speed[i] = 0;
				}
				RunMode = 0;
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
			}
			else if(strcmp(buf,"CurrentMode")==0)
			{
				for(int i=0;i<6;i++)
				{
					current[i] = 0;
					speed[i] = 0;
				}
				RunMode = 1;
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
			}
			else if(strcmp(buf,"Reset")==0)
			{
				HAL_NVIC_SystemReset();
			}
			break;
		default:
			HAL_UART_Transmit(&huart8,"Wrong Para\n",11,100);
			break;
	}
//	__HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);
}

/***********************************************************
��������TurnLock
���ܣ����ڻ���������
������None
***********************************************************/
void TurnLock()
{
	Moto_Data2[0].sum_angle = 0;
	Angle_target[0] = 0;
	turn_lock = 1;
}

/***********************************************************
��������TurnUnlock
���ܣ����ڻ����̽���
������None
***********************************************************/
void TurnUnlock()
{
	turn_lock = 0;
}


/***********************************************************
��������Command_Setup
���ܣ����������ʼ�����֣�ֻ���ڿ�ʼʱ����һ�Σ�
������None
***********************************************************/
void Command_Setup()
{
//	CircleBuf_Init(&uartbuf,150);
//	HAL_NVIC_EnableIRQ(UART8_IRQn);
}
/***********************************************************
��������Command_Loop
���ܣ���������ѭ������
������None
***********************************************************/
void Command_Loop()
{
		if(Process_mark)
		{
			change_all_para();
			Process_mark--;
		}
		printf("Speed %d %d %d %d %d %d\n",Moto_Data1[0].speed,
																			 -Moto_Data1[1].speed,
																			 Moto_Data1[2].speed,
																			 -Moto_Data1[3].speed,
																			 Moto_Data1[4].speed,
																			 -Moto_Data1[5].speed);
	  osDelay(20);
}
