#include "ds2812b.h"
#include <intrins.h>
#include "stdio.h"

sbit RGB = P3^0;
pdata UINT8 SEED = 0;
UINT8 T1RH = 0;	//T0高8位重载值
UINT8 T1RL = 0;	//T0低8位重载值
UINT8 count_num = 0;
int i = 0;

pdata UINT8 display_flag = 1;
UINT8 WS2812B_DisplayType = 0;

#define DAT1	RGB=1;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();RGB=0;
#define DAT0	RGB=1;_nop_();_nop_();_nop_();_nop_();_nop_();RGB=0;_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();

code UINT8 HUXICODE[][3] = {{32,11,227}, {29,10,201},{23,7,165}, {21,7,143}, {17,5,122},{15,5,101},{12,4,81}, {6,2,43}, {3,1,18}, {0,0,0},
						{31,219,4}, {27,188,3},{23,167,3}, {21,146,3}, {18,125,2},{15,104,2},{12,84,1}, {7,52,1}, {5,33,1}, {0,0,0},
						{218,100,3}, {188,86,3},{167,77,3}, {149,68,2}, {125,57,2},{104,48,2},{84,38,1}, {63,29,1}, {26,12,0}, {0,0,0}};
//code UINT8 HUXICODE[][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};
		
// size 65							
code UINT8 R_w[] = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
					240, 220, 200, 180, 160, 140, 120, 100, 80, 60, 40, 20, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240 };
code UINT8 G_w[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240,
					255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
					255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
code UINT8 B_w[] = { 0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240,
					255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
					255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
					240, 220, 200, 180, 160, 140, 120, 100, 80, 60, 40, 20, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	
// 解析一个 8bit 数据
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

// 随机颜色
void display_suiji(unsigned char num)
{
	pdata UINT8 i = 0;
	for(i = 0; i< num; i++)
	{
        //随机颜色
		pdata UINT8 green = SEED*num;
		pdata UINT8 red = SEED;
		pdata UINT8 blue = SEED/num;
		SEED = SEED + num*5* num;
		WS2812Data(green);
		WS2812Data(red);
		WS2812Data(blue);
	}
}

// 循环灯
void display_char(UINT8 num)
{
	pdata UINT8 j;
	for(j = 0; j < 17; j++)
	{
		WS2812Data(HUXICODE[num][1]);
		WS2812Data(HUXICODE[num][0]);
		WS2812Data(HUXICODE[num][2]);
		num++;
		if(num >= 30)
			num = 0;
	}
}

pdata UINT8 Opposite_Flag = 0;

// 呼吸灯
void display_Type2(UINT8 R_w[], UINT8 G_w[], UINT8 B_w[], UINT8 num)
{
	pdata UINT8 j;
		
//	for(j = num; j < (17+num); j++)
//	{
//		if(17+num >= 65)
//			num = 0;
//		WS2812Data(G_w[j]);
//		WS2812Data(R_w[j]);
//		WS2812Data(B_w[j]);
//	}
	
	for(j = 0; j < 17; j++)
	{
		
		WS2812Data(G_w[num]);
		WS2812Data(R_w[num]);
		WS2812Data(B_w[num]);
//		if(num >= 65)
//		{
//			num = 0;
//		}
	}
}

void display_Type3(UINT8 num)
{
	pdata UINT8 j;
		
	for(j = num; j < (17+num); j++)
	{
		if(17+num >= 65)
			num = 0;
		WS2812Data(G_w[j]);
		WS2812Data(R_w[j]);
		WS2812Data(B_w[j]);
	}
}

/**T1配置函数**/
void ConfigT1(UINT8 ms)
{
    unsigned long tmp = 0;

	tmp = 24000000/12;
	tmp = (tmp * ms)/1000;
	tmp = 65536 - tmp;
	tmp = tmp + 1;
    T1RH = (UINT8)(tmp >> 8);
	T1RL = (UINT8)tmp;

	TMOD = ( TMOD & ~( bT1_GATE | bT1_CT | bT1_M1 ) ) | bT1_M0;//* 模式1，16 位定时/计数器
//	TMOD = 0x11;
	TH1 = T1RH;
	TL1 = T1RL;
	TF1 = 0;
	ET1 = 1;
//	TR1 = 1;
}



// 滚动显示
void display_row()
{
	int i = 0;
	int j = 0;
	for(j = 0; j < 16;j++)
	{
		for(i = j; i < 30; i++)
		{
			display_char(i);
//				display_suiji(i);
		}
	}
}

/**T1中断函数**/
void InterruptTimer1() interrupt INT_NO_TMR1
{
	TH1 = T1RH;
	TL1 = T1RL;

	count_num++;
	// 计数40次为 1s
//	if(count_num >= 40)
	
	switch(WS2812B_DisplayType)
	{
		case 0:
		{
			if(count_num >= 8)
			{
				display_char(i);
				i++;
				if(i >= 30)	i = 0;
				count_num = 0;
			}
		}	break;
		case 1:
		{
			if(count_num >= 8)
			{
				count_num = 0;
				display_Type2(R_w, G_w, B_w, i);
				if(Opposite_Flag == 0)
				{
					i++;
					if(i >= 64)
					{
						Opposite_Flag = 1;
					}
				}
				else
				{
					i--;
					if(i == 0)
					{
						Opposite_Flag = 0;
					}
				}
			}
		}	break;
		case 2:
		{
			if(count_num >= 8)
			{
				display_Type3(i);
				i++;
				if(i >= 65) i = 0;
				count_num = 0;
			}
		}	break;
		default: break;
	}
	
// 倒序写法
/*
	if(count_num >= 5)
	{
		if(display_flag == 1 && i < 30)
		{
			i++;
		}
		else if(display_flag == 0)
		{
			i--;
			if(i <= 0)	
			{
				display_flag = 1;
				i = 0;
			}
		}
		display_char(i);
		if(i >= 30)
		{
			display_flag = 0;
			i = 30;
		}
		count_num = 0;
	}
*/
}
							
							