/***********************************************************
文件名：CircleBuf.c
描述：
		环形缓冲区实现
From:Qi-Q@Rjgawuie
***********************************************************/

#include "CircleBuf.h"

#include "stdlib.h"

/***********************************************************
函数名：CircleBuf_Melloc
功能：环形缓冲区内存申请
参数：size 	申请的内存大小
返回值: 无
***********************************************************/
void *CircleBuf_Melloc (uint16_t size)			
{
	return (void*)malloc(size);
}
/***********************************************************
函数名：CircleBuf_Free
功能：环形缓冲区内存释放
参数：mf 	要释放的内存指针
返回值: 无
***********************************************************/
void CircleBuf_Free (void* mf)		 
{
	free(mf);
}
/***********************************************************
函数名：CircleBuf_Init
功能：环形缓冲区初始化
参数：circlebuf	指向缓冲区句柄的指针
			bufsize		要开辟的缓冲区大小
返回值: 无
***********************************************************/
void CircleBuf_Init(Circle_Buf_Typedef *circlebuf,uint16_t bufsize)
{
	circlebuf->buf = (char *)CircleBuf_Melloc(bufsize);
	
	circlebuf->count = 0;
	circlebuf->read = circlebuf->buf;
	circlebuf->write = circlebuf->buf;
	circlebuf->size = bufsize;
}
/***********************************************************
函数名：CircleBuf_DeInit
功能：环形缓冲区去初始化
参数：circlebuf	指向缓冲区句柄的指针
返回值: 无
***********************************************************/
void CircleBuf_DeInit(Circle_Buf_Typedef *circlebuf)
{
	circlebuf->size = 0;
	CircleBuf_Free(circlebuf->buf);
	circlebuf->count = 0;
	circlebuf->read = NULL;
	circlebuf->write = NULL;
	circlebuf->buf = NULL;
}
/***********************************************************
函数名：CircleBuf_Write
功能：环形缓冲区写入1字节
参数：circlebuf	指向缓冲区句柄的指针
			ch				存入的数据
返回值: 无
***********************************************************/
void CircleBuf_Write(Circle_Buf_Typedef *circlebuf,char cbwch)
{
	*(circlebuf->write) = cbwch;
	
	circlebuf->write++;
	
	if(circlebuf->count == circlebuf->size)
	{
		circlebuf->read++;
		if(circlebuf->read == circlebuf->buf+circlebuf->size) 
			circlebuf->read = circlebuf->buf;
	}
	else circlebuf->count++;
	
	if(circlebuf->write == circlebuf->buf+circlebuf->size) 
		circlebuf->write = circlebuf->buf;
}
/***********************************************************
函数名：CircleBuf_Read
功能：环形缓冲区读出1字节
参数：circlebuf	指向缓冲区句柄的指针
返回值: 读出的字符，错误返回0
***********************************************************/
char CircleBuf_Read(Circle_Buf_Typedef *circlebuf)
{
	uint8_t cbrch;
	if(circlebuf->count>0)
	{
		cbrch = *circlebuf->read;
		circlebuf->read++;
		circlebuf->count--;
		if(circlebuf->read == circlebuf->buf+circlebuf->size) 
			circlebuf->read = circlebuf->buf;
		return cbrch;
	}
	return 0;
}
