/*********************************************************************************
FILE NAME:TFT20_ILI9225.c

SPI接口LCD驱动，	配合HAL库使用
									仅支持单颗LCD
									改自卖家提供的例程
									
				
				From:@rjgawuie
*********************************************************************************/

#include "TFT20_ILI9225.h"
#include "Font.h"
#include "stdlib.h"
#include "stdio.h"






/*******************************************************************************
* 函 数 : RGB
* 功 能 : R,G,B 转16位
* 输 入 : 无
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
u16 RGB(u16 R,u16 G,u16 B)
{
	u16 RGB16;
	
	R>>=3;
	G>>=2;
	B>>=3;
	
	RGB16=R<<11|G<<5|B;
	
	return RGB16;
}

/*******************************************************************************
* 函 数 : TFT20_RGB24_RGB16
* 功 能 : 24位rgb编码转16位
* 输 入 : 无
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
u16 TFT20_RGB24_RGB16(u32 rgb)
{
	u16 R,G,B,RGB16;
	
	R=(rgb&0xff0000)>>19;
	G=(rgb&0x00ff00)>>10;
	B=(rgb&0x0000ff)>>3;
	
	RGB16=R<<11|G<<5|B;
	
	return RGB16;
}



/*******************************************************************************
* 函 数 : TFT20_delay_ms
* 功 能 : 延时
* 输 入 : 无
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_delay_ms(u16 ms)
{
	HAL_Delay(ms);
}


/*******************************************************************************
* 函 数 : TFT20_Reset
* 功 能 : 液晶屏硬复位
* 输 入 : 无
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_Reset()
{	
    TFT20_RST=0;
    TFT20_delay_ms(100);
    TFT20_RST=1;
    TFT20_delay_ms(50);
}


/*******************************************************************************
* 函 数 : TFT20_Init
* 功 能 : 液晶屏初始化
* 输 入 : 无
* 输 出 : 无
* 返 回 : 无
* 说 明 : 液晶屏初始化_ILI9225_176X220
*******************************************************************************/
u16 TFT20_Init()
{	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	switch((uint32_t)RS_Port)      //RS引脚使能
	{
		case (uint32_t)GPIOA:	__HAL_RCC_GPIOA_CLK_ENABLE();break;
		case (uint32_t)GPIOB:	__HAL_RCC_GPIOB_CLK_ENABLE();break;
		case (uint32_t)GPIOC:	__HAL_RCC_GPIOC_CLK_ENABLE();break;
		case (uint32_t)GPIOD:	__HAL_RCC_GPIOD_CLK_ENABLE();break;
		case (uint32_t)GPIOE:	__HAL_RCC_GPIOE_CLK_ENABLE();break;
		case (uint32_t)GPIOF:	__HAL_RCC_GPIOF_CLK_ENABLE();break;
		case (uint32_t)GPIOH:	__HAL_RCC_GPIOH_CLK_ENABLE();break;
		case (uint32_t)GPIOI:	__HAL_RCC_GPIOI_CLK_ENABLE();break;
		default: return 1;
	}
	switch((uint32_t)RST_Port)      //RST引脚使能
	{
		case (uint32_t)GPIOA:	__HAL_RCC_GPIOA_CLK_ENABLE();break;
		case (uint32_t)GPIOB:	__HAL_RCC_GPIOB_CLK_ENABLE();break;
		case (uint32_t)GPIOC:	__HAL_RCC_GPIOC_CLK_ENABLE();break;
		case (uint32_t)GPIOD:	__HAL_RCC_GPIOD_CLK_ENABLE();break;
		case (uint32_t)GPIOE:	__HAL_RCC_GPIOE_CLK_ENABLE();break;
		case (uint32_t)GPIOF:	__HAL_RCC_GPIOF_CLK_ENABLE();break;
		case (uint32_t)GPIOH:	__HAL_RCC_GPIOH_CLK_ENABLE();break;
		case (uint32_t)GPIOI:	__HAL_RCC_GPIOI_CLK_ENABLE();break;
		default: return 1;
	}
	switch((uint32_t)CS_Port)      //CS使能
	{
		case (uint32_t)GPIOA:	__HAL_RCC_GPIOA_CLK_ENABLE();break;
		case (uint32_t)GPIOB:	__HAL_RCC_GPIOB_CLK_ENABLE();break;
		case (uint32_t)GPIOC:	__HAL_RCC_GPIOC_CLK_ENABLE();break;
		case (uint32_t)GPIOD:	__HAL_RCC_GPIOD_CLK_ENABLE();break;
		case (uint32_t)GPIOE:	__HAL_RCC_GPIOE_CLK_ENABLE();break;
		case (uint32_t)GPIOF:	__HAL_RCC_GPIOF_CLK_ENABLE();break;
		case (uint32_t)GPIOH:	__HAL_RCC_GPIOH_CLK_ENABLE();break;
		case (uint32_t)GPIOI:	__HAL_RCC_GPIOI_CLK_ENABLE();break;
		default: return 1;
	}
	
	GPIO_InitStruct.Pin = RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  //GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(RS_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = RST_Pin;
	HAL_GPIO_Init(RST_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = CS_Pin;
	HAL_GPIO_Init(CS_Port, &GPIO_InitStruct);
	
	
	TFT20_Reset(); //Reset before LCD Init.

	//LCD Init For 2.2inch LCD Panel with ILI9225.	
	TFT20_WriteReg(0x10, 0x0000); // Set SAP,DSTB,STB
	TFT20_WriteReg(0x11, 0x0000); // Set APON,PON,AON,VCI1EN,VC
	TFT20_WriteReg(0x12, 0x0000); // Set BT,DC1,DC2,DC3
	TFT20_WriteReg(0x13, 0x0000); // Set GVDD
	TFT20_WriteReg(0x14, 0x0000); // Set VCOMH/VCOML voltage
	TFT20_delay_ms(40); // Delay 20 ms
	
	// Please follow this power on sequence
	TFT20_WriteReg(0x11, 0x0018); // Set APON,PON,AON,VCI1EN,VC
	TFT20_WriteReg(0x12, 0x1121); // Set BT,DC1,DC2,DC3
	TFT20_WriteReg(0x13, 0x0063); // Set GVDD
	TFT20_WriteReg(0x14, 0x3961); // Set VCOMH/VCOML voltage
	TFT20_WriteReg(0x10, 0x0800); // Set SAP,DSTB,STB
	TFT20_delay_ms(10); // Delay 10 ms
	TFT20_WriteReg(0x11, 0x1038); // Set APON,PON,AON,VCI1EN,VC
	TFT20_delay_ms(30); // Delay 30 ms
	
	TFT20_WriteReg(0x02, 0x0100); // set 1 line inversion

#if USE_HORIZONTAL//如果定义了横屏
	//R01H:SM=0,GS=0,SS=0 (for details,See the datasheet of ILI9225)
	TFT20_WriteReg(0x01, 0x001C); // set the display line number and display direction
	//R03H:BGR=1,ID0=1,ID1=1,AM=1 (for details,See the datasheet of ILI9225)
	TFT20_WriteReg(0x03, 0x1038); // set GRAM write direction .
#else//竖屏
	//R01H:SM=0,GS=0,SS=1 (for details,See the datasheet of ILI9225)
	TFT20_WriteReg(0x01, 0x011C); // set the display line number and display direction 
	//R03H:BGR=1,ID0=1,ID1=1,AM=0 (for details,See the datasheet of ILI9225)
	TFT20_WriteReg(0x03, 0x1030); // set GRAM write direction.
#endif

	TFT20_WriteReg(0x07, 0x0000); // Display off
	TFT20_WriteReg(0x08, 0x0808); // set the back porch and front porch
	TFT20_WriteReg(0x0B, 0x1100); // set the clocks number per line
	TFT20_WriteReg(0x0C, 0x0000); // CPU interface
	TFT20_WriteReg(0x0F, 0x0501); // Set Osc
	TFT20_WriteReg(0x15, 0x0020); // Set VCI recycling
	TFT20_WriteReg(0x20, 0x0000); // RAM Address
	TFT20_WriteReg(0x21, 0x0000); // RAM Address
	
	//------------------------ Set GRAM area --------------------------------//
	TFT20_WriteReg(0x30, 0x0000); 
	TFT20_WriteReg(0x31, 0x00DB); 
	TFT20_WriteReg(0x32, 0x0000); 
	TFT20_WriteReg(0x33, 0x0000); 
	TFT20_WriteReg(0x34, 0x00DB); 
	TFT20_WriteReg(0x35, 0x0000); 
	TFT20_WriteReg(0x36, 0x00AF); 
	TFT20_WriteReg(0x37, 0x0000); 
	TFT20_WriteReg(0x38, 0x00DB); 
	TFT20_WriteReg(0x39, 0x0000); 
	
	// ---------- Adjust the Gamma 2.2 Curve -------------------//
	TFT20_WriteReg(0x50, 0x0603); 
	TFT20_WriteReg(0x51, 0x080D); 
	TFT20_WriteReg(0x52, 0x0D0C); 
	TFT20_WriteReg(0x53, 0x0205); 
	TFT20_WriteReg(0x54, 0x040A); 
	TFT20_WriteReg(0x55, 0x0703); 
	TFT20_WriteReg(0x56, 0x0300); 
	TFT20_WriteReg(0x57, 0x0400); 
	TFT20_WriteReg(0x58, 0x0B00); 
	TFT20_WriteReg(0x59, 0x0017); 
	
	TFT20_WriteReg(0x0F, 0x0701); // Vertical RAM Address Position
	TFT20_WriteReg(0x07, 0x0012); // Vertical RAM Address Position
	TFT20_delay_ms(50); // Delay 50 ms
	TFT20_WriteReg(0x07, 0x1017); // Vertical RAM Address Position  
	
  TFT20_Clear(BLACK); //黑色 //清屏
	return 0;
}




/*******************************************************************************
* 函 数 : TFT20_SetXY
* 功 能 : 设置TFT屏显示起始点
* 输 入 : X Y 起始点
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_SetXY(u16 XStart,u16 YStart)
{	
#if USE_HORIZONTAL//如果定义了横屏  	    	
	TFT20_WriteReg(0x21,XStart);//设置XY起始地址
	TFT20_WriteReg(0x20,YStart);
#else             //竖屏	
	TFT20_WriteReg(0x20,XStart);
	TFT20_WriteReg(0x21,YStart);
#endif
	TFT20_WriteIndex(0x22);		//从GRAM读取数据寄存器的18位数据。
} 


/*******************************************************************************
* 函 数 : TFT20_SetRegion
* 功 能 : 设置显示窗口
* 输 入 : X Y 起点和终点
* 输 出 : 无
* 返 回 : 无
* 说 明 : 在此区域写点数据自动换行
*******************************************************************************/
void TFT20_SetRegion(u8 XStart,u8 YStart,u8 XEnd,u8 YEnd)
{
#if USE_HORIZONTAL//横屏	
	TFT20_WriteReg(0x38,XEnd);
	TFT20_WriteReg(0x39,XStart);
	TFT20_WriteReg(0x36,YEnd);
	TFT20_WriteReg(0x37,YStart);
	TFT20_WriteReg(0x21,XStart);
	TFT20_WriteReg(0x20,YStart);
#else             //竖屏	
	TFT20_WriteReg(0x36,XEnd);
	TFT20_WriteReg(0x37,XStart);
	TFT20_WriteReg(0x38,YEnd);
	TFT20_WriteReg(0x39,YStart);
	TFT20_WriteReg(0x20,XStart);
	TFT20_WriteReg(0x21,YStart);
#endif
	TFT20_WriteIndex(0x22);	
}


/*******************************************************************************
* 函 数 : TFT20_Clear
* 功 能 : 全屏填充颜色函数（清屏）
* 输 入 : Color[in] 颜色数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_Clear(u16 Color)               
{	
   u16 i,m;
	
   TFT20_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
	
   for(i=0;i<X_MAX_PIXEL;i++)
    for(m=0;m<Y_MAX_PIXEL;m++)
    {	
	  	TFT20_WriteData(Color>>8);
	    TFT20_WriteData(Color); 
    }   
}


/*******************************************************************************
* 函 数 : TFT20_LocalClear
* 功 能 : 局部清屏
* 输 入 : XY 显示起始点  Color 颜色数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_LocalClear(u16 XStart,u16 YStart,u16 C,u16 H,u16 Color)   //局部清屏
{
	u16 temp;
	
	TFT20_SetRegion(XStart,YStart,XStart+C-1,YStart+H-1); //设置显示窗口大小
	for(temp=0;temp<(C*H);temp++)
	 {	
		TFT20_WriteData(Color>>8);
		TFT20_WriteData(Color);				
	 }	
}


/*******************************************************************************
* 函 数 : TFT20_DrawPoint
* 功 能 : 画任意颜色点
* 输 入 : XStart[in] YStart[in] XY描点位置  Color[in] 颜色数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_DrawPoint(u16 XStart,u16 YStart,u16 Color)
{
	TFT20_SetXY(XStart,YStart);
	TFT20_WriteData(Color>>8);
	TFT20_WriteData(Color);

}    


/*******************************************************************************
* 函 数 : TFT20_BGR_RGB
* 功 能 : BGR->RGB格式转换
* 输 入 : 无
* 输 出 : 无
* 返 回 : RGB格式的颜色值
* 说 明 : 从ILI93xx读出的数据为GBR格式，而我们写入的时候为RGB格式。
*******************************************************************************/
u16 TFT20_BGR_RGB(u16 BGR)
{
  u16 R,G,B,RGB;   
  B=(BGR>>0)&0x1f;
  G=(BGR>>5)&0x3f;
  R=(BGR>>11)&0x1f;	
	
  RGB=(B<<11)+(G<<5)+(R<<0);	
	
  return (RGB);
}


/*******************************************************************************
* 函 数 : TFT20_DrawCircle
* 功 能 : 画圆
* 输 入 : x[in] y[in] XY圆心点位置 R[in] 半径 Color[in] 颜色数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : Bresenham算法 
*******************************************************************************/
void TFT20_DrawCircle(u16 X,u16 Y,u16 R,u16 Color) //画圆 
{
	unsigned short  a,b; //短整型
	int c; 
	a=0;   //必须赋值为零
	b=R; 
	c=3-2*R; 
	while(a<b) 
	{ 
		TFT20_DrawPoint(X+a,Y+b,Color); //7 
		TFT20_DrawPoint(X-a,Y+b,Color); //6 
		TFT20_DrawPoint(X+a,Y-b,Color); //2 
		TFT20_DrawPoint(X-a,Y-b,Color); //3 
		TFT20_DrawPoint(X+b,Y+a,Color); //8 
		TFT20_DrawPoint(X-b,Y+a,Color); //5 
		TFT20_DrawPoint(X+b,Y-a,Color); //1 
		TFT20_DrawPoint(X-b,Y-a,Color); //4 

		if(c<0) c=c+4*a+6; 
		else {c=c+4*(a-b)+10;b-=1;} 
		a+=1; 
	} 
	if(a==b) 
	{ 
		TFT20_DrawPoint(X+a,Y+b,Color); 
		TFT20_DrawPoint(X+a,Y+b,Color); 
		TFT20_DrawPoint(X+a,Y-b,Color); 
		TFT20_DrawPoint(X-a,Y-b,Color); 
		TFT20_DrawPoint(X+b,Y+a,Color); 
		TFT20_DrawPoint(X-b,Y+a,Color); 
		TFT20_DrawPoint(X+b,Y-a,Color); 
		TFT20_DrawPoint(X-b,Y-a,Color); 
	} 
} 


//，
/*******************************************************************************
* 函 数 : TFT20_DrawLine
* 功 能 : 画线函数
* 输 入 : X Y 起始点 终点坐标  Color[in] 颜色数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 使用Bresenham 画线算法
*******************************************************************************/
void TFT20_DrawLine(u16 XStart, u16 YStart,u16 XEnd, u16 YEnd,u16 Color)   
{
	int dx,dy,      //X Y 差值
			dx2,dy2,    //X Y 差值的两倍
			x_inc,y_inc,//在绘图过程中像素点移动的方向
			juece,      //决策变量
			temp;      //临时变量

	TFT20_SetXY(XStart,YStart);//设置TFT屏显示起始点
	
	dx = XEnd-XStart;//计算x距离
	dy = YEnd-YStart;//计算y距离

	if(dx>=0) x_inc= 1;  //确定方向
	else {x_inc= -1;dx= -dx;} 
	
	if(dy>=0) y_inc= 1;
	else {y_inc= -1;dy= -dy;} 

	dx2 = dx<<1; //乘以2
	dy2 = dy<<1;

	if(dx>dy)	     //x距离大于y距离,那么每个x轴上只有一个点,每个y轴上有若干个点
	{              //且线的点数等于x距离,以x轴递增画点
		juece= dy2-dx;  //初始化误差项
		for(temp=0;temp<=dx;temp++) //要画的点数不会超过x距离
		{
			TFT20_DrawPoint(XStart,YStart,Color);
			if(juece>=0) //是否需要增加y坐标值
			{
				juece -=dx2;
				YStart +=y_inc;//增加y坐标值
			} 
			juece +=dy2; //调整
			XStart +=x_inc;//x坐标值每次画点后都递增1（移动到下一个像素）
		}
	}
	else          //y轴大于x轴，则每个y轴上只有一个点，x轴若干个点
	{             //以y轴为递增画点
		juece= dx2-dy;  //初始化误差项
		for (temp=0;temp<=dy;temp++) //画线
		{
			TFT20_DrawPoint(XStart,YStart,Color);
			if(juece>=0)
			{
				juece -=dy2;
				XStart +=x_inc;
			}
			juece +=dx2;
			YStart +=y_inc;
		}
	}
}


/*******************************************************************************
* 函 数 : TFT20_Drawbox
* 功 能 : 画矩形（带斜线）
* 输 入 : xy 显示起始点 w[in]矩形宽度 h[in]高度  Color[in] 颜色
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_Drawbox(u16 x,u16 y,u16 w,u16 h,u16 Color)
{
	TFT20_DrawLine(x    ,y  ,x+w  ,y    ,0xEF7D);//上横
	TFT20_DrawLine(x+w	,y	,x+w	,y+h  ,0x2965);//右竖
	TFT20_DrawLine(x    ,y+h,x+w  ,y+h  ,0x2965);//下横
	TFT20_DrawLine(x    ,y  ,x    ,y+h  ,0xEF7D);//左竖
  TFT20_DrawLine(x+1  ,y+1,x+w-1,y+h-1,Color );//正斜
}


/*******************************************************************************
* 函 数 : TFT20_Drawbox2
* 功 能 : 画矩形
* 输 入 : xy 显示起始点 w[in]矩形宽度 h[in]高度
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_Drawbox2(u16 x,u16 y,u16 w,u16 h,u8 mode)
{
	if(mode==0){
		TFT20_DrawLine(x    ,y  ,x+w  ,y    ,0xEF7D);
		TFT20_DrawLine(x+w	,y	,x+w	,y+h  ,0x2965);
		TFT20_DrawLine(x    ,y+h,x+w  ,y+h  ,0x2965);
		TFT20_DrawLine(x    ,y  ,x    ,y+h  ,0xEF7D);
		}
	if(mode==1){
		TFT20_DrawLine(x    ,y  ,x+w  ,y    ,0x2965);
		TFT20_DrawLine(x+w	,y 	,x+w	,y+h  ,0xEF7D);
		TFT20_DrawLine(x    ,y+h,x+w  ,y+h  ,0xEF7D);
		TFT20_DrawLine(x    ,y  ,x    ,y+h  ,0x2965);
	}
	if(mode==2){
		TFT20_DrawLine(x    ,y  ,x+w  ,y    ,0x0000);
		TFT20_DrawLine(x+w  ,y  ,x+w  ,y+h  ,0x0000);
		TFT20_DrawLine(x    ,y+h,x+w  ,y+h  ,0x0000);
		TFT20_DrawLine(x    ,y  ,x    ,y+h  ,0x0000);
	}
}


/*******************************************************************************
* 函 数 : TFT20_DisplayImg
* 功 能 : 任意位置显示图片
* 输 入 : XY 显示起始点  *img[in] 图片数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 16位BMP/Image2LCD取模选项设置/水平扫描/16位/不包含图像头数据/自左至右
          自顶至底/低位在前/在图片数组前加上图片尺寸 例:u8 a[]={176,220,dat};
*******************************************************************************/
void TFT20_DisplayImg(u16 XStart,u16 YStart,u8 *img)
{
	u16 temp;
	
//	TFT20_Clear(GRAY0); //灰色0 //清屏
	TFT20_SetRegion(XStart,YStart,XStart+img[0]-1,YStart+img[1]-1); //设置显示窗口大小
	for(temp=0;temp<(img[0]*img[1]);temp++)
	 {	
		TFT20_WriteData(*(img+temp*2+3));	//数据低位在前
		TFT20_WriteData(*(img+temp*2+2));				
	 }	
	TFT20_SetRegion(0,0,X_MAX_PIXEL,Y_MAX_PIXEL);
}


/*******************************************************************************
* 函 数 : TFT20_DisplayButtonDown
* 功 能 : 在屏幕显示一凸起的按钮框
* 输 入 : x0,y0,x1,y1 按钮框左上角和右下角坐标
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_DisplayButtonDown(u16 x0,u16 y0,u16 x1,u16 y1)
{
	TFT20_DrawLine(x0  ,y0  ,x1  ,y0  ,GRAY2); //上上横
	TFT20_DrawLine(x0+1,y0+1,x1  ,y0+1,GRAY1); //上下横
	TFT20_DrawLine(x0  ,y0  ,x0  ,y1  ,GRAY2); //左左竖 
	TFT20_DrawLine(x0+1,y0+1,x0+1,y1  ,GRAY1); //左右竖
	TFT20_DrawLine(x0  ,y1  ,x1  ,y1  ,WHITE); //下横
	TFT20_DrawLine(x1  ,y0  ,x1  ,y1  ,WHITE); //右竖
}


/*******************************************************************************
* 函 数 : TFT20_DisplayButtonUp
* 功 能 : 在屏幕显示一凹下的按钮框
* 输 入 : x0,y0,x1,y1 按钮框左上角和右下角坐标
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_DisplayButtonUp(u16 x0,u16 y0,u16 x1,u16 y1)
{
	TFT20_DrawLine(x0  ,y0  ,x1  ,y0  ,WHITE); //上横
	TFT20_DrawLine(x0  ,y0  ,x0  ,y1  ,WHITE); //左竖
	TFT20_DrawLine(x0+1,y1-1,x1  ,y1-1,GRAY1); //下上横
	TFT20_DrawLine(x0  ,y1  ,x1  ,y1  ,GRAY2); //下下横
	TFT20_DrawLine(x1-1,y0+1,x1-1,y1  ,GRAY1); //右左竖
  TFT20_DrawLine(x1  ,y0  ,x1  ,y1  ,GRAY2); //右右竖
}


/*******************************************************************************
* 函 数 : TFT20_DrawFont_GBK16
* 功 能 : 显示汉字或字符 （GBK16）
* 输 入 : XY 显示起始点 fc[in]字体颜色 bc[in]背景色 *Dat[in] 显示数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_DrawFont_GBK16(u16 x,u16 y,u16 fc,u16 bc,u8 *Dat)
{
	u8  i,j;
	unsigned short k,x0=x;//记忆x0起点坐标

	while(*Dat) 
	{	
		if((*Dat) < 128) //小于128的为ASCLL字符
		{
			k=*Dat; //ASCLL字符编号索引
			if(k==0x0D) {x=x0;y+=16;} //换行  +16
			else 
			{
				if(k>32) k-=32; else k=0;//数组字符编号索引
			  for(i=0;i<16;i++)
				 for(j=0;j<8;j++) 
					{
				    if(asc16[k*16+i]&(0x80>>j))	TFT20_DrawPoint(x+j,y+i,fc); //字体颜色
						else {if(fc!=bc) TFT20_DrawPoint(x+j,y+i,bc);}  //背景色
					}
				x+=8;
			}
			Dat++;
		}
		else   //大于128的为GBK汉字内码(两字节)（在这里面是）
		{
			for(k=0;k<hz16_num;k++) 
			{
			  if((hz16[k].Index[0]==*(Dat))&&(hz16[k].Index[1]==*(Dat+1)))
			  { 
					for(i=0;i<16;i++)
					{
					 for(j=0;j<8;j++) 
						{
							if(hz16[k].Msk[i*2]&(0x80>>j)) TFT20_DrawPoint(x+j,y+i,fc); //字体颜色
							else {if(fc!=bc) TFT20_DrawPoint(x+j,y+i,bc);} //背景色
						}
					 for(j=0;j<8;j++) 
						{
							if(hz16[k].Msk[i*2+1]&(0x80>>j)) TFT20_DrawPoint(x+j+8,y+i,fc); //字体颜色
							else {if(fc!=bc) TFT20_DrawPoint(x+j+8,y+i,bc);} //背景色
						}
					}
				}
			 }
			Dat+=2;x+=16;
		} 
	}
}



/*******************************************************************************
* 函 数 : TFT20_DrawFormat_Float
* 功 能 : 格式化长度不超过20的浮点数并显示
* 输 入 : XY 显示起始点 fc[in]字体颜色 bc[in]背景色 type 希望格式化成的类型
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/

void TFT20_DrawFormat_Float(u16 x,u16 y,u16 fc,u16 bc,const char *type,float num)
{
	char buf[20];
	sprintf(buf,type,num);
	TFT20_DrawFont_GBK16(x,y,fc,bc,(u8 *)buf);
}

/*******************************************************************************
* 函 数 : TFT20_DrawFormat_Int
* 功 能 : 格式化长度不超过20的浮点数并显示
* 输 入 : XY 显示起始点 fc[in]字体颜色 bc[in]背景色 type 希望格式化成的类型
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/

void TFT20_DrawFormat_Int(u16 x,u16 y,u16 fc,u16 bc,const char *type,int num)
{
	char buf[20];
	sprintf(buf,type,num);
	TFT20_DrawFont_GBK16(x,y,fc,bc,(u8 *)buf);
}


/*******************************************************************************
* 函 数 : TFT20_DrawFont_Num32
* 功 能 : 显示数码管体数字
* 输 入 : XY 显示起始点 fc[in]字体颜色 bc[in]背景色 num[in]
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_DrawFont_Num32(u16 x,u16 y,u16 fc,u16 bc,u16 num)
{
	u8 i,j,k,c;

  for(i=0;i<32;i++)
	 {
		for(j=0;j<4;j++) 
		 {
			c=*(sz32+num*32*4+i*4+j);
			for(k=0;k<8;k++)	
			 {
	       if(c&(0x80>>k)) TFT20_DrawPoint(x+j*8+k,y+i,fc);
				 else {if(fc!=bc) TFT20_DrawPoint(x+j*8+k,y+i,bc);}
			}
		}
	}
}



/*******************************************************************************
* 函 数 : TFT20_SPI_WriteData
* 功 能 : 写一个字节数据
* 输 入 : Dat[in] 待写入数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void  TFT20_SPI_WriteData(u8 Dat)
{
	HAL_SPI_Transmit(&hspi1,&Dat,1,100);
}


/*******************************************************************************
* 函 数 : TFT20_WriteIndex
* 功 能 : 向液晶屏写一个8位指令
* 输 入 : Index[in] 指令
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_WriteIndex(u8 Index)
{
   TFT20_CS=0;
   TFT20_RS=0;
   TFT20_SPI_WriteData(Index);
   TFT20_CS=1;
}


/*******************************************************************************
* 函 数 : TFT20_WriteData
* 功 能 : 向液晶屏写入一个8位数据
* 输 入 : Dat[in] 待写入数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_WriteData(u8 Dat)
{
   TFT20_CS=0;
   TFT20_RS=1;
   TFT20_SPI_WriteData(Dat);
   TFT20_CS=1;
}


/*******************************************************************************
* 函 数 : TFT20_WriteReg
* 功 能 : 写寄存器数据
* 输 入 : Index[in] 寄存器地址 Dat[in] 待写入数据
* 输 出 : 无
* 返 回 : 无
* 说 明 : 无
*******************************************************************************/
void TFT20_WriteReg(u8 Index,u16 Dat)
{
	TFT20_WriteIndex(Index); //指令（寄存器地址）
	TFT20_WriteData(Dat>>8);
	TFT20_WriteData(Dat);	
}



