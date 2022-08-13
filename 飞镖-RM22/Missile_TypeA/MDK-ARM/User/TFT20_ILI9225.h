/*********************************************************************************
FILE NAME:TFT20_ILI9225.h

SPI�ӿ�LCD������	���HAL��ʹ��
									��֧�ֵ���LCD
									���������ṩ������
									
				
				From:@rjgawuie
*********************************************************************************/

#ifndef _TFT20_ILI9225_H
#define _TFT20_ILI9225_H
#include "sys.h"
#include "spi.h"
#include "main.h"
#include "stm32f4xx_hal.h"



#define RS_Port GPIOA                                   //RS RST CS ���ź�
#define RS_Pin  GPIO_PIN_6
#define RST_Port GPIOB                                   //RS RST CS ���ź�
#define RST_Pin  GPIO_PIN_10
#define CS_Port GPIOB                                 //RS RST CS ���ź�
#define CS_Pin  GPIO_PIN_9
#define TFT20_RS   PAout(6)  //�Ĵ���ѡ���ź�
#define TFT20_RST  PBout(10)  //��λ���� I6
#define TFT20_CS   PBout(9)  //Ƭѡ�ź� 

//-------------------------��������--------------------------------------//
#define LCD_X_SIZE	        176
#define LCD_Y_SIZE	        220

#define USE_HORIZONTAL  		1	//ʹ�ú��� 1:ʹ�� 	 0:��ʹ��.
#define USE_HARDWARE_SPI    0 //SPI      1:Ӳ��SPI 0:ģ��SPI

#if USE_HORIZONTAL //��������˺��� 
#define X_MAX_PIXEL	        LCD_Y_SIZE
#define Y_MAX_PIXEL	        LCD_X_SIZE
#else
#define X_MAX_PIXEL	        LCD_X_SIZE
#define Y_MAX_PIXEL	        LCD_Y_SIZE
#endif

//������ɫֵ����
#define RED  	  0xf800  //��ɫ
#define GREEN	  0x07e0  //��ɫ
#define BLUE 	  0x001f  //��ɫ
#define WHITE	  0xffff  //��ɫ
#define BLACK	  0x0000  //��ɫ
#define YELLOW  0xFFE0  //��ɫ
#define GRAY0   0xEF7D  //��ɫ0 0011 0001 0110 0101   3165 
#define GRAY1   0x8410  //��ɫ1 0000 0000 0000 0000
#define GRAY2   0x4208  //��ɫ2 1111 1111 1101 1111

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;


//------------------------------------------------------------------------
//������ʾ����
//void TFT20_TestDemo(void);  

//Һ������ʼ��
u16 TFT20_Init(void);

//Һ����Ӳ��λ
void TFT20_Reset(void); 

//��ʱ
void TFT20_delay_ms(u16 ms);

//����TFT����ʾ��ʼ��
void TFT20_SetXY(u16 XStart,u16 YStart);

//������ʾ���ڣ�������
void TFT20_SetRegion(u8 XStart,u8 YStart,u8 XEnd,u8 YEnd);

//ȫ�������ɫ������������
void TFT20_Clear(u16 Color); 

//�ֲ�����
void TFT20_LocalClear(u16 XStart,u16 YStart,u16 C,u16 H,u16 Color); 

//��������ɫ��
void TFT20_DrawPoint(u16 XStart,u16 YStart,u16 Color);

//BGR->RGB��ʽת��
u16 TFT20_BGR_RGB(u16 BGR);

//24λrgb����ת16λ
u16 TFT20_RGB24_RGB16(u32 rgb);

//R,G,B ת16λ
u16 RGB(u16 R,u16 G,u16 B);

//��Բ
void TFT20_DrawCircle(u16 X,u16 Y,u16 R,u16 Color);

//���ߺ���
void TFT20_DrawLine(u16 XStart, u16 YStart,u16 XEnd, u16 YEnd,u16 Color);  

//�����Σ���б�ߣ�
void TFT20_Drawbox(u16 x,u16 y,u16 w,u16 h,u16 Color);

//������ mode=2 ��
void TFT20_Drawbox2(u16 x,u16 y,u16 w,u16 h,u8 mode);

//��ʾͼƬ
void TFT20_DisplayImg(u16 XStart,u16 YStart,u8 *img);

////����Ļ��ʾһ���µİ�ť��
//void TFT20_DisplayButtonDown(u16 x0,u16 y0,u16 x1,u16 y1);

//����Ļ��ʾһ͹��İ�ť��
//void TFT20_DisplayButtonUp(u16 x0,u16 y0,u16 x1,u16 y1);

////��ʾ���ֻ��ַ� ��GBK16��
void TFT20_DrawFont_GBK16(u16 x,u16 y,u16 fc,u16 bc,u8 *Dat);

//��ʾ���ֻ��ַ� ��GBK24��
//void TFT20_DrawFont_GBK24(u16 x,u16 y,u16 fc,u16 bc,u8 *Dat);

//��ʾ����
void TFT20_DrawFormat_Float(u16 x,u16 y,u16 fc,u16 bc,const char *type,float num);
void TFT20_DrawFormat_Int(u16 x,u16 y,u16 fc,u16 bc,const char *type,int num);

//��ʾ�����������
void TFT20_DrawFont_Num32(u16 x,u16 y,u16 fc,u16 bc,u16 num);

//����SPI
void TFT20_SPI_Init(void);

//дһ���ֽ�����
void  TFT20_SPI_WriteData(u8 Dat);

//��Һ����дһ��8λָ��
void TFT20_WriteIndex(u8 Index);

//��Һ����д��һ��8λ����
void TFT20_WriteData(u8 Dat);

//д�Ĵ�������
void TFT20_WriteReg(u8 Index,u16 Dat);





#endif
