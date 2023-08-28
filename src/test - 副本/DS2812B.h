#ifndef __DS2812B_H
#define __DS2812B_H
#include ".\Public\CH554.H"
#include ".\Public\Debug.H"

extern UINT8 count_num;
extern UINT8 WS2812B_DisplayType;
extern int i;

void WS2812Data(unsigned char value);
void ConfigT1(UINT8 ms);
void display_char(UINT8 num);
void display_row();

#endif
