#include <ch552.h>
#include "uart1.h"
#include ".\Public\debug.h"
#include "stdio.h"
#include <string.h>
#include <intrins.h>
#include "stdlib.h"

#define uchar unsigned char
#define uint unsigned int

UINT8 T0RH = 0;	//T0高8位重载值
UINT8 T0RL = 0;	//T0低8位重载值
sbit P14 = P3^0;
sbit Key = P1^5;
int count = 0;
uint SEED = 0;
uint num_count;
code uchar HUXICODE[][3] = {{32,11,227}, {29,10,201},{23,7,165}, {21,7,143}, {17,5,122},{15,5,101},{12,4,81}, {6,2,43}, {3,1,18}, {0,0,0},
						{31,219,4}, {27,188,3},{23,167,3}, {21,146,3}, {18,125,2},{15,104,2},{12,84,1}, {7,52,1}, {5,33,1}, {0,0,0},
						{218,100,3}, {188,86,3},{167,77,3}, {149,68,2}, {125,57,2},{104,48,2},{84,38,1}, {63,29,1}, {26,12,0}, {0,0,0}};
//uchar send_buf1[17] = {32, 29, 23, 50, 20, 70, 110, 150, 160, 30, 88, 215, 255, 183, 167, 19, 231};
//uchar send_buf2[17] = {11, 10, 7, 75, 81, 255, 164, 137, 35, 98, 64, 25, 167, 178, 123, 45, 81};
//uchar send_buf3[17] = {227, 201, 165, 20, 164, 45, 198, 126, 210, 39, 172, 189, 67, 83, 23, 156, 31};

#define DAT1	P14=1;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();P14=0;
#define DAT0	P14=1;_nop_();_nop_();_nop_();_nop_();_nop_();P14=0;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();

// 解析一个 8bit 数据 0 或 1
void Anlx_bit(uchar* dat)
{
	uint i;
	for(i = 0; i < 8; i++)
	{
		if((*dat >> (7-i)) & 0x01)	// 为 1
		{
			DAT1;
		}
		else	
			DAT0;
	}
}

void WS2812Data(unsigned char value)
{
	if (value & 0x80)
	{
		DAT1;
	}
	else
	{
		DAT0;
	}
	if (value & 0x40)
	{
		DAT1;
	}
	else
	{
		DAT0;
	}
	if (value & 0x20)
	{
		DAT1;
	}
	else
	{
		DAT0;
	}
	if (value & 0x10)
	{
		DAT1;
	}
	else
	{
		DAT0;
	}
	if (value & 0x08)
	{
		DAT1;
	}
	else
	{
		DAT0;
	}
	if (value & 0x04)
	{
		DAT1;
	}
	else
	{
		DAT0;
	}
	if (value & 0x02)
	{
		DAT1;
	}
	else
	{
		DAT0;
	}
	if (value & 0x01)
	{
		DAT1;
	}
	else
	{
		DAT0;
	}
}

void display_rgb(uchar r, uchar g, uchar b) 
{
	WS2812Data(g);
	WS2812Data(r);
	WS2812Data(b);
}

// 随机颜色
void display_suiji(uchar num)
{
	uint i = 0;
	for(i = 0; i< num+1; i++)
	{
        //随机颜色
		uint green = SEED>>8;
		uint red = SEED;
		uint blue = ~SEED;
		SEED = SEED + 100;
		WS2812Data(green);
		WS2812Data(red);
		WS2812Data(blue);
	}
}

// 呼吸灯
void display_char(uint num)
{
	WS2812Data(HUXICODE[num][1]);
	WS2812Data(HUXICODE[num][0]);
	WS2812Data(HUXICODE[num][2]);
}

/**T0配置函数**/
void ConfigT0(UINT8 ms)
{
    unsigned long tmp = 0;

	tmp = 24000000/12;
	tmp = (tmp * ms)/1000;
	tmp = 65536 - tmp;
	tmp = tmp + 1;
    T0RH = (UINT8)(tmp >> 8);
	T0RL = (UINT8)tmp;

	TMOD = ( TMOD & ~( bT0_GATE | bT0_CT | bT0_M1 ) ) | bT0_M0;//* 模式1，16 位定时/计数器
	TH0 = T0RH;
	TL0 = T0RL;
	TF0 = 0;
	ET0 = 1;
}

UINT8 T2RH = 0;	//T0高8位重载值
UINT8 T2RL = 0;	//T0低8位重载值

/**T2配置函数**/
void ConfigT2(UINT8 ms)
{
    unsigned long tmp = 0;

	tmp = 24000000/12;
	tmp = (tmp * ms)/1000;
	tmp = 65536 - tmp;
	tmp = tmp + 1;
    T2RH = (UINT8)(tmp >> 8);
	T2RL = (UINT8)tmp;

//	T2MOD = ( T2MOD & ~( bT1_GATE | bT1_CT | bT1_M1 ) ) | bT1_M0;//* 模式1，16 位定时/计数器
	T2MOD |= (bTMR_CLK | bT2_CLK);
	C_T2=0;
	RCLK = 0;
	TCLK = 0;
	CP_RL2 = 0;
	TH2 = T2RH;
	TL2 = T2RL;
	ET2 = 1;
//	TR1 = 1;
}

pdata UINT8 First_Flag = 1;
pdata UINT8 Second_Flag = 0;
pdata UINT8 Time_Flag = 0;
pdata UINT8 End_Flag = 0;
UINT8 LED_AttitudeFlag = 0;
unsigned long pdata Double_TimeThr = 0;

pdata UINT8 Key_Flag = 0;
pdata UINT8 Key_DoubleFlag = 0;

/*十进制转BCD*/
UINT8 DEC_BCD(UINT8 DEC)
{
	UINT8 BCD;
	BCD = DEC / 10;
	DEC %= 10;
	DEC += BCD * 16;

	return DEC;
}

void Key_Driver()
{
	if(!Key)
	{
		mDelaymS(8);
		if(!Key)
		{
//			if(First_Flag)
//			{
//				Time_Flag = 1;
//				mTimer2RunCTL(1);
////					Double_TimeThr += 25;
//				First_Flag = 0;
//			}
//			else if(Second_Flag)
//			{
//				End_Flag = 1;
//			}
			if(Key_Flag == 0)
			{
				Key_Flag = 1;
				Time_Flag = 1;
				mTimer2RunCTL(1);
			}
		}
	}
	else	// 按键松开
	{
		if(Key_Flag == 1)
		{
			Key_Flag = 0;
			if(Key_DoubleFlag == 0)
			{
				Key_DoubleFlag = 1;
				Double_TimeThr = 0;
			}
			else if(Key_DoubleFlag == 1)
			{
				if(Double_TimeThr < 500)
				{
					Key_DoubleFlag = 0;
					CH554UART1SendByte(0x00);
//					return 1;
				}
			}
		}
		else if(Key_DoubleFlag == 1)
		{
			if(Double_TimeThr >= 500)
			{
				Key_DoubleFlag = 0;
				CH554UART1SendByte(0x01);
				Time_Flag = 0;
				Double_TimeThr = 0;
				mTimer2RunCTL(0);
//				return 2;
			}
		}
		
		/*
		if(End_Flag)
		{
			if(Double_TimeThr < 500)	// 1 秒内按下
			{
	//						mTimer1RunCTL(1);
				switch(LED_AttitudeFlag)
				{
					case 0 : LED_AttitudeFlag++;mTimer2RunCTL(0); break;
					case 1 : LED_AttitudeFlag++;mTimer2RunCTL(0); break;
					default: break;
				}
				CH554UART1SendByte(DEC_BCD(LED_AttitudeFlag));
				if(LED_AttitudeFlag >= 2)
					LED_AttitudeFlag = 0;
			}
	}
		*/
	}
//	return 0;
}




//timer2中断服务程序
void	mTimer2Interrupt( void ) interrupt INT_NO_TMR2 using 3
{
	mTimer2RunCTL(0);
	if(TF2)
    {
        TF2 = 0;                                                             //清空定时器2溢出中断	  
        if(Time_Flag)
		{
			Double_TimeThr++;
		}
		else
		{
			Double_TimeThr = 0;
		}
		
		
    }
	mTimer2RunCTL(1);
}


void main()
{
//	WS2812Initialize(4);
//	int i = 0;
//	int j = 0;
	CfgFsys();
	mDelaymS(8);
	ConfigT2(1);
	mTimer2RunCTL(0);
//	ConfigT0(10); // 10ms
	UART1Init( ); 
	EA = 1; //允许单片机中断	
	
	while(1)
	{
//		CH554UART1SendByte(0x00);
		Key_Driver();
//		WS2812RGBvalue(0, 255, 0, 1);
//		P14 = 1;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();P14 = 0;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();		
//		
//		P14 = 1;_nop_();_nop_();_nop_();_nop_();_nop_();P14 = 0;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();
//		display_rgb(255, 0, 0);
//		mDelaymS(1000);
//		display_rgb(0, 255, 0);
//		mDelaymS(1000);
//		for(j = 0; j < 16;j++)
//		{
//			for(i = j; i < 30; i++)
//			{
////				display_rgb(send_buf1[i], send_buf2[i], send_buf3[i]);
//				display_char(i);
////				display_suiji(i);
//			}
//			mDelaymS(100);
//		}
		
		
		
//		display_rgb(255, 0, 0);
//		if(Key == 0)
//		{
//			mDelaymS(10);
//			if(Key == 0)
//			{
//				mTimer0RunCTL(1);
//				send_buf[2] = 10;
				
//			}	
//		}
	}
}


///**T0中断函数**/
//void InterruptTimer0() interrupt INT_NO_TMR0 using 1
//{
//	uint i;
//	TH0 = T0RH;
//	TL0 = T0RL;
//	
//	count++;
//	
//	if(count >= 100)
//	{
//		num_count++;
//		if(num_count == 30)
//			num_count = 0;
//		for(i = num_count; i < 30; i++)
//		{
//			display_char(i);
//		}	
//		
//		mTimer0RunCTL(0);
//		count = 0;
//	}
//}


