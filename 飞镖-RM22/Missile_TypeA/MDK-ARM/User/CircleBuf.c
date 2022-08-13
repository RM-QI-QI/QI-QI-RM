/***********************************************************
�ļ�����CircleBuf.c
������
		���λ�����ʵ��
From:Qi-Q@Rjgawuie
***********************************************************/

#include "CircleBuf.h"

#include "stdlib.h"

/***********************************************************
��������CircleBuf_Melloc
���ܣ����λ������ڴ�����
������size 	������ڴ��С
����ֵ: ��
***********************************************************/
void *CircleBuf_Melloc (uint16_t size)			
{
	return (void*)malloc(size);
}
/***********************************************************
��������CircleBuf_Free
���ܣ����λ������ڴ��ͷ�
������mf 	Ҫ�ͷŵ��ڴ�ָ��
����ֵ: ��
***********************************************************/
void CircleBuf_Free (void* mf)		 
{
	free(mf);
}
/***********************************************************
��������CircleBuf_Init
���ܣ����λ�������ʼ��
������circlebuf	ָ�򻺳��������ָ��
			bufsize		Ҫ���ٵĻ�������С
����ֵ: ��
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
��������CircleBuf_DeInit
���ܣ����λ�����ȥ��ʼ��
������circlebuf	ָ�򻺳��������ָ��
����ֵ: ��
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
��������CircleBuf_Write
���ܣ����λ�����д��1�ֽ�
������circlebuf	ָ�򻺳��������ָ��
			ch				���������
����ֵ: ��
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
��������CircleBuf_Read
���ܣ����λ���������1�ֽ�
������circlebuf	ָ�򻺳��������ָ��
����ֵ: �������ַ������󷵻�0
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
