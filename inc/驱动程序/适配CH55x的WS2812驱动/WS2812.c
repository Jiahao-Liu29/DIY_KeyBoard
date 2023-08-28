/*
  CH552T 主频24M，发送一个灯的数据大概耗时22us
  2021-8-19修改
*/
#include "CH554.H"
#include "Debug.H"
#include "WS2812.H"
#include <intrins.h>
#define SEND1 WS2812_DI = 1;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();WS2812_DI = 0;
#define SEND0 WS2812_DI = 1;_nop_();_nop_();_nop_();_nop_();_nop_();WS2812_DI = 0;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();
sbit WS2812_DI = P3^3;
/***********WS2812写数据函数***********/
void WS2812DataTranslator(unsigned char value)
{
	if (value & 0x80)
	{
		SEND1;
	}
	else
	{
		SEND0;
	}
	if (value & 0x40)
	{
		SEND1;
	}
	else
	{
		SEND0;
	}
	if (value & 0x20)
	{
		SEND1;
	}
	else
	{
		SEND0;
	}
	if (value & 0x10)
	{
		SEND1;
	}
	else
	{
		SEND0;
	}
	if (value & 0x08)
	{
		SEND1;
	}
	else
	{
		SEND0;
	}
	if (value & 0x04)
	{
		SEND1;
	}
	else
	{
		SEND0;
	}
	if (value & 0x02)
	{
		SEND1;
	}
	else
	{
		SEND0;
	}
	if (value & 0x01)
	{
		SEND1;
	}
	else
	{
		SEND0;
	}
}
/***********WS2812写RGB值函数***********/
void WS2812RGBvalue(unsigned char Rvalue, unsigned char Gvalue, unsigned char Bvalue, unsigned char ledQuantity)
{
	while (ledQuantity--)
	{
		WS2812DataTranslator(Gvalue);
		WS2812DataTranslator(Rvalue);
		WS2812DataTranslator(Bvalue);
	}
	WS2812_DI = 0;
	//mDelayuS(300);
}
/***********WS2812初始化函数***********/
void WS2812Initialize(unsigned char ledQuantityum)
{
	while (ledQuantityum--)
	{
		WS2812DataTranslator(0);
		WS2812DataTranslator(0);
		WS2812DataTranslator(0);
	}
	WS2812_DI = 0;
	mDelayuS(300);
}