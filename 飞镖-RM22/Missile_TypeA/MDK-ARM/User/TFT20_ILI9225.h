/*********************************************************************************
FILE NAME:TFT20_ILI9225.h

SPI接口LCD驱动，	配合HAL库使用
									仅支持单颗LCD
									改自卖家提供的例程
									
				
				From:@rjgawuie
*********************************************************************************/

#ifndef _TFT20_ILI9225_H
#define _TFT20_ILI9225_H
#include "sys.h"
#include "spi.h"
#include "main.h"
#include "stm32f4xx_hal.h"



#define RS_Port GPIOA                                   //RS RST CS 引脚宏
#define RS_Pin  GPIO_PIN_6
#define RST_Port GPIOB                                   //RS RST CS 引脚宏
#define RST_Pin  GPIO_PIN_10
#define CS_Port GPIOB                                 //RS RST CS 引脚宏
#define CS_Pin  GPIO_PIN_9
#define TFT20_RS   PAout(6)  //寄存器选择信号
#define TFT20_RST  PBout(10)  //复位引脚 I6
#define TFT20_CS   PBout(9)  //片选信号 

//-------------------------参数设置--------------------------------------//
#define LCD_X_SIZE	        176
#define LCD_Y_SIZE	        220

#define USE_HORIZONTAL  		1	//使用横屏 1:使用 	 0:不使用.
#define USE_HARDWARE_SPI    0 //SPI      1:硬件SPI 0:模拟SPI

#if USE_HORIZONTAL //如果定义了横屏 
#define X_MAX_PIXEL	        LCD_Y_SIZE
#define Y_MAX_PIXEL	        LCD_X_SIZE
#else
#define X_MAX_PIXEL	        LCD_X_SIZE
#define Y_MAX_PIXEL	        LCD_Y_SIZE
#endif

//常用颜色值定义
#define RED  	  0xf800  //红色
#define GREEN	  0x07e0  //绿色
#define BLUE 	  0x001f  //蓝色
#define WHITE	  0xffff  //白色
#define BLACK	  0x0000  //黑色
#define YELLOW  0xFFE0  //黄色
#define GRAY0   0xEF7D  //灰色0 0011 0001 0110 0101   3165 
#define GRAY1   0x8410  //灰色1 0000 0000 0000 0000
#define GRAY2   0x4208  //灰色2 1111 1111 1101 1111

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;


//------------------------------------------------------------------------
//测试演示程序
//void TFT20_TestDemo(void);  

//液晶屏初始化
u16 TFT20_Init(void);

//液晶屏硬复位
void TFT20_Reset(void); 

//延时
void TFT20_delay_ms(u16 ms);

//设置TFT屏显示起始点
void TFT20_SetXY(u16 XStart,u16 YStart);

//设置显示窗口（开窗）
void TFT20_SetRegion(u8 XStart,u8 YStart,u8 XEnd,u8 YEnd);

//全屏填充颜色函数（清屏）
void TFT20_Clear(u16 Color); 

//局部清屏
void TFT20_LocalClear(u16 XStart,u16 YStart,u16 C,u16 H,u16 Color); 

//画任意颜色点
void TFT20_DrawPoint(u16 XStart,u16 YStart,u16 Color);

//BGR->RGB格式转换
u16 TFT20_BGR_RGB(u16 BGR);

//24位rgb编码转16位
u16 TFT20_RGB24_RGB16(u32 rgb);

//R,G,B 转16位
u16 RGB(u16 R,u16 G,u16 B);

//画圆
void TFT20_DrawCircle(u16 X,u16 Y,u16 R,u16 Color);

//画线函数
void TFT20_DrawLine(u16 XStart, u16 YStart,u16 XEnd, u16 YEnd,u16 Color);  

//画矩形（带斜线）
void TFT20_Drawbox(u16 x,u16 y,u16 w,u16 h,u16 Color);

//画矩形 mode=2 黑
void TFT20_Drawbox2(u16 x,u16 y,u16 w,u16 h,u8 mode);

//显示图片
void TFT20_DisplayImg(u16 XStart,u16 YStart,u8 *img);

////在屏幕显示一凹下的按钮框
//void TFT20_DisplayButtonDown(u16 x0,u16 y0,u16 x1,u16 y1);

//在屏幕显示一凸起的按钮框
//void TFT20_DisplayButtonUp(u16 x0,u16 y0,u16 x1,u16 y1);

////显示汉字或字符 （GBK16）
void TFT20_DrawFont_GBK16(u16 x,u16 y,u16 fc,u16 bc,u8 *Dat);

//显示汉字或字符 （GBK24）
//void TFT20_DrawFont_GBK24(u16 x,u16 y,u16 fc,u16 bc,u8 *Dat);

//显示数字
void TFT20_DrawFormat_Float(u16 x,u16 y,u16 fc,u16 bc,const char *type,float num);
void TFT20_DrawFormat_Int(u16 x,u16 y,u16 fc,u16 bc,const char *type,int num);

//显示数码管体数字
void TFT20_DrawFont_Num32(u16 x,u16 y,u16 fc,u16 bc,u16 num);

//配置SPI
void TFT20_SPI_Init(void);

//写一个字节数据
void  TFT20_SPI_WriteData(u8 Dat);

//向液晶屏写一个8位指令
void TFT20_WriteIndex(u8 Index);

//向液晶屏写入一个8位数据
void TFT20_WriteData(u8 Dat);

//写寄存器数据
void TFT20_WriteReg(u8 Index,u16 Dat);





#endif
