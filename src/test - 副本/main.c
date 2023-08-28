#include "./Public/CH554.H"                                                      
#include "./Public/DEBUG.H"
#include <string.h>
#include <stdio.h>
#include "ds2812b.h"



#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define CapsLockLED 0x02

UINT8X  Ep0Buffer[8>(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;    //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X  Ep1Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000a;  //端点1 IN缓冲区,必须是偶地址
UINT8X  Ep2Buffer[64<(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0050;  //端点2 IN缓冲区,必须是偶地址
UINT8   SetupReq,SetupLen,Ready,Count,FLAG,UsbConfig,LED_VALID;
PUINT8  pDescr;   //USB配置标志
USB_SETUP_REQ   SetupReqBuf;  //暂存Setup包
bit Ep2InKey;
#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0
#pragma  NOAREGS
 //定义的4个按键以及EC16的A、B脚
sbit LED1 = P3^4;
sbit LED2 = P1^5;
sbit LED3 = P1^4;
sbit LED4 = P1^1;
sbit LED5 = P1^7;
sbit LED6 = P3^1;
sbit LED7 = P3^2;
sbit LED8 = P3^3;
sbit LED17 = P1^6;

UINT8 key;

UINT8 EC16_A_State = 0;
UINT8 EC16_A_PState = 0;
UINT8 T0RH = 0;	//T0高8位重载值
UINT8 T0RL = 0;	//T0低8位重载值
//UINT8 KeyState[6] = {1,1,1,1,1,1}; //按键状态
//UINT8 BackState[6] = {1,1,1,1,1,1}; //按键上一次的状态
UINT8 KeyState = 1; //按键状态
UINT8 BackState = 1; //按键上一次的状态
//unsigned long pdata TimeThr[4] = {1000, 1000, 1000, 1000};
//unsigned long pdata KeyDownTime[4]= {0, 0, 0, 0};
unsigned long pdata TimeThr = 1000;
unsigned long pdata KeyDownTime = 0;
UINT8C key_code_map[6] = {
	0xE7,0x32,0x33,0x34,0x35,0x36         //按键1,按键2,按键3,按键4,按键3,按键4
};

// P8 端口写数据
void write_data(UINT8 dat)
{
	LED1 = dat & 0x01;
	LED2 = (dat >> 1) & 0x01;
	LED3 = (dat >> 2) & 0x01;
	LED4 = (dat >> 3) & 0x01;
	LED5 = (dat >> 4) & 0x01;
	LED6 = (dat >> 5) & 0x01;
	LED7 = (dat >> 6) & 0x01;
	LED8 = (dat >> 7) & 0x01;
}

// P8 端口读数据
UINT8 read_data()
{
	UINT8 dat = 0x00;
	dat|=LED8;
	dat=(dat<<1)|LED7;
	dat=(dat<<1)|LED6;
	dat=(dat<<1)|LED5;
	dat=(dat<<1)|LED4;
	dat=(dat<<1)|LED3;
	dat=(dat<<1)|LED2;
	dat=(dat<<1)|LED1;
	
	return dat;
}


/*普通键盘数据*/
UINT8 HIDKey[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/*多媒体按键数据*/
UINT8 HIDKeyMUL[4] = {0x00,0x00,0x00,0x00};

/****字符串描述符****/
/*HID类报表描述符*/
UINT8C KeyRepDesc[62] =
{
/******************************************************************
键盘发送给PC的数据每次8个字节：BYTE1 BYTE2 BYTE3 BYTE4 BYTE5 BYTE6 BYTE7 BYTE8。定义分别是：
BYTE1 --
       |--bit0:   Left Control 
       |--bit1:   Left Shift 
       |--bit2:   Left Alt 
       |--bit3:   Left GUI 
       |--bit4:   Right Control  
       |--bit5:   Right Shift 
       |--bit6:   Right Alt 
       |--bit7:   Right GUI 
BYTE2 -- 暂不清楚，有的地方说是保留位
BYTE3--BYTE8 -- 这六个为普通按键
*******************************************************************/
    0x05,0x01,0x09,0x06,0xA1,0x01,0x05,0x07,
    0x19,0xe0,0x29,0xe7,0x15,0x00,0x25,0x01,
    0x75,0x01,0x95,0x08,0x81,0x02,0x95,0x01,
    0x75,0x08,0x81,0x01,0x95,0x03,0x75,0x01,
    0x05,0x08,0x19,0x01,0x29,0x03,0x91,0x02,
    0x95,0x05,0x75,0x01,0x91,0x01,0x95,0x06,
    0x75,0x08,0x26,0xff,0x00,0x05,0x07,0x19,
    0x00,0x29,0x91,0x81,0x00,0xC0
};
/*多媒体键盘报表描述符*/
UINT8C KeyMULRepDesc[105] =	
{
/**********************************************************************************************
键盘发送给PC的数据每次4个字节：BYTE1 BYTE2 BYTE3 BYTE4
BYTE1 BYTE2 BYTE3 这3个字节分成24位，每个位代表一个按键，1代表按下，0抬起。
BYTE1 --
       |--bit0:  Vol-  
       |--bit1:  Vol+ 
       |--bit2:  Mute  
       |--bit3:  Email 
       |--bit4:  Media   
       |--bit5:  WWW Home 
       |--bit6:  Play/Pause 
       |--bit7:  Scan Pre Track 
BYTE2 BYTE3按下面的顺序排下去，BYTE3 bit7：最后一个Usage( NULL )。
BYTE4 --
    系统功能按键，关机(0x81)，休眠(0x82），唤醒（0x83）
***********************************************************************************************/
	0x05, 0x0C, //USAGE_PAGE 用途页选择0x0c(用户页)
	0x09, 0x01, //USAGE 接下来的应用集合用于用户控制
	0xA1, 0x01, //COLLECTION 开集合
		0x15, 0x00, //LOGICAL_MINIMUM (0)
		0x25, 0x01, //LOGICAL_MAXIMUM (1)
		0x0A, 0xEA, 0x00,		/* Usage( Vol- ) */
		0x0A, 0xE9, 0x00,		/* Usage( Vol+ ) */
		0x0A, 0xE2, 0x00,		/* Usage( Mute ) */
		0x0A, 0x8A, 0x01,		/* Usage( Email ) */
		0x0A, 0x83, 0x01,		/* Usage( Media ) */
		0x0A, 0x23, 0x02,		/* Usage( WWW Home ) */
		0x0A, 0xCD, 0x00,		/* Usage( Play/Pause ) */
		0x0A, 0xB6, 0x00,		/* Usage( Scan Pre Track ) */
		0x0A, 0xB5, 0x00,		/* Usage( Scan Next Track ) */
		0x0A, 0xB7, 0x00,		/* Usage( Stop ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x00, 0x00,		/* Usage( NULL ) */
		0x0A, 0x11, 0x22,		/* Usage( NULL ) */
		0x75, 0x01, //REPORT_SIZE (1)
		0x95, 0x18, //REPORT_COUNT (24)
		0x81, 0x02, //INPUT (Data,Var,Abs)输入24bit数据
		0x05, 0x01, //USAGE_PAGE 用途页0x01(普通桌面)
			0x19, 0x00, //USAGE_MINIMUM 用途最小值0x00(未定义)
			0x29, 0x83, //USAGE_MAXIMUM 用途最大值0x83(系统唤醒)
			0x15, 0x00, //LOGICAL_MINIMUM (0)
			0x25, 0x83, //LOGICAL_MAXIMUM (83)
			0x75, 0x08, //REPORT_SIZE (8)
			0x95, 0x01, //REPORT_COUNT (1)
			0x81, 0x00, //INPUT (Data,Ary,Abs)输入1字节数据
	0xC0//END_COLLECTION 闭合集合
};
/*设备描述符*/
UINT8C DevDesc[18] = {
   0x12,      //bLength字段。设备描述符的长度为18(0x12)字节
   0x01,	  //bDescriptorType字段。设备描述符的编号为0x01
   0x10,0x01, //bcdUSB字段。这里设置版本为USB1.1，即0x0110。
              //由于是小端结构，所以低字节在先，即0x10，0x01。
   0x00,	  //bDeviceClass字段。我们不在设备描述符中定义设备类，
              //而在接口描述符中定义设备类，所以该字段的值为0。
   0x00,	  //bDeviceSubClass字段。bDeviceClass字段为0时，该字段也为0。
   0x00,	  //bDeviceProtocol字段。bDeviceClass字段为0时，该字段也为0。
   0x08,	  //bMaxPacketSize0字段。 的端点0大小的8字节。
   0x3d,0x41, //idVender字段,注意小端模式，低字节在先。
   0x3a,0x55, //idProduct字段 产品ID号。注意小端模式，低字节应该在前。
   0x00,0x00, //bcdDevice字段。注意小端模式，低字节应该在前。
   0x00,	  //iManufacturer字段。厂商字符串的索引
   0x00,	  //iProduct字段。产品字符串的索引值,注意字符串索引值不要使用相同的值。
   0x00,	  //iSerialNumber字段。设备的序列号字符串索引值。
   0x01		  //bNumConfigurations字段。该设备所具有的配置数。
};
/*配置描述符*/
UINT8C CfgDesc[59] =
{
 /*配置描述符*/
    0x09, //bLength字段。配置描述符的长度为9字节
	0x02, //bDescriptorType字段。配置描述符编号为0x02
	0x3b, //wTotalLength字段。配置描述符集合的总长度0x003b，包括配置描述符本身、接口描述符、类描述符、端点描述符等，LSB
	0x00, 
	0x02, //bNumInterfaces字段。该配置包含的接口数，只有2个接口
	0x01, //bConfiguration字段。该配置的值为1
	0x01, //iConfigurationz字段，该配置的字符串索引。
	0xA0, //bmAttributes字段,bit4-bit7描述设备的特性
	0x64, //bMaxPower字段，该设备需要的最大电流量。每单位电流为 2 mA    
 /*接口描述符*/
    //接口1，普通键盘
    0x09,0x04,0x00,0x00,0x01,0x03,0x01,0x01,0x00, //接口描述符,键盘  HID设备的定义放置在接口描述符中
    0x09,0x21,0x11,0x01,0x00,0x01,0x22,0x3e,0x00, //HID类描述符
    0x07,0x05,0x81,0x03,0x08,0x00,0x0a, //端点描述符
	//接口2，多媒体按键
	0x09,0x04,0x01,0x00,0x01,0x03,0x00,0x00,0x00, // 接口描述符
	0x09,0x21,0x00,0x01,0x00,0x01,0x22,0x69,0x00, // HID类描述符
	0x07,0x05,0x82,0x03,0x04,0x00,0x0a,	// 端点描述符 
};
/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	  IE_USB = 0;
	  USB_CTRL = 0x00; //先设定USB设备模式
    UEP0_DMA = Ep0Buffer; //端点0数据传输地址
    UEP1_DMA = Ep1Buffer; //端点1数据传输地址
	UEP2_DMA = Ep2Buffer; //端点2数据传输地址
    UEP4_1_MOD = ~(bUEP4_RX_EN | bUEP4_TX_EN |bUEP1_RX_EN | bUEP1_BUF_MOD) | bUEP4_TX_EN;//端点1单64字节收发缓冲区,端点0收发
	UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; //OUT事务返回ACK，IN事务返回NAK
    UEP1_CTRL = bUEP_T_TOG | UEP_T_RES_NAK;	//端点1手动翻转同步标志位，IN事务返回NAK
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; //端点2自动翻转同步标志位，IN事务返回NAK
		
	  USB_DEV_AD = 0x00;
	  UDEV_CTRL = bUD_PD_DIS; // 禁止DP/DM下拉电阻
	  USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
	  UDEV_CTRL |= bUD_PORT_EN; // 允许USB端口
	  USB_INT_FG = 0xFF; // 清中断标志
	  USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	  IE_USB = 1;
}
/*******************************************************************************
* Function Name  : Enp1IntIn()
* Description    : USB设备模式端点1的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey)); //加载上传数据
    UEP1_T_LEN = sizeof(HIDKey); //上传数据长度
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //有数据时上传数据并应答ACK
}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB设备模式端点2的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2IntIn( )
{
    memcpy( Ep2Buffer, HIDKeyMUL, sizeof(HIDKeyMUL)); //加载上传数据
    UEP2_T_LEN = sizeof(HIDKeyMUL); //上传数据长度
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //有数据时上传数据并应答ACK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
void    DeviceInterrupt( void ) interrupt INT_NO_USB using 1                      //USB中断服务程序,使用寄存器组1
{
    UINT8 len;
    if(UIF_TRANSFER)                                                            //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# 中断端点上传
            UEP2_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP1_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
			FLAG = 1; 															/*传输完成标志*/
            break;
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# 中断端点上传
            UEP1_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            FLAG = 1;                                                           /*传输完成标志*/
            break;
        case UIS_TOKEN_SETUP | 0:                                                //SETUP事务
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // 限制总长度
                }
                len = 0;                                                        // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;								
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* HID类命令 */
                {
									switch( SetupReq ) 
									{
										case 0x01://GetReport
												 break;
										case 0x02://GetIdle
												 break;	
										case 0x03://GetProtocol
												 break;				
										case 0x09://SetReport										
												 break;
										case 0x0A://SetIdle
												 break;	
										case 0x0B://SetProtocol
												 break;
										default:
												 len = 0xFF;  								 					            /*命令不支持*/					
												 break;
								  }	
                }
                else
                {//标准请求
                    switch(SetupReq)                                        //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                             //设备描述符
                            pDescr = DevDesc;                               //把设备描述符送到要发送的缓冲区
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                             //配置描述符
                            pDescr = CfgDesc;                               //把设备描述符送到要发送的缓冲区
                            len = sizeof(CfgDesc);
                            break;
                        case 0x22:                                          //报表描述符
                            if(UsbSetupBuf->wIndexL == 0)                   //接口0报表描述符
                            {
                                pDescr = KeyRepDesc;                        //数据准备上传
                                len = sizeof(KeyRepDesc);								
                            }
							if(UsbSetupBuf->wIndexL == 1)                   //接口0报表描述符
                            {
                                pDescr = KeyMULRepDesc;                        //数据准备上传
                                len = sizeof(KeyMULRepDesc);
                                Ready = 1;                                  //如果有更多接口，该标准位应该在最后一个接口配置完成后有效
                                //IE_UART1 = 1;//开启串口中断															
															
                            }
                            else
                            {
                                len = 0xff;                                 //本程序只有2个接口，这句话正常不可能执行
                            }
                            break;
                        default:
                            len = 0xff;                                     //不支持的命令或者出错
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //限制总长度
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen;                  //本次传输长度
                        memcpy(Ep0Buffer,pDescr,len);                        //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if ( SetupLen >= 1 )
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                            //Clear Feature
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
                        {
                            switch( UsbSetupBuf->wIndexL )
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF;                                            // 不支持的端点
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF;                                                // 不是端点不支持
                        }
                        break;
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )             /* 设置设备 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    /* 设置唤醒使能标志 */
                                }
                                else
                                {
                                    len = 0xFF;                                        /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* 操作失败 */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        /* 设置端点 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                               //操作失败
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //操作失败
                            }
                        }
                        else
                        {
                            len = 0xFF;                                      //操作失败
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 )
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff;                                           //操作失败
                        break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                   //包长度错误
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len <= 8)                                                //上传数据或者状态阶段返回0长度包
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
            }
            else
            {
                UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen;                          //本次传输长度
                memcpy( Ep0Buffer, pDescr, len );                            //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                              //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            if((SetupReq == 0x09)&& (len == 1))
            {
              LED_VALID = Ep0Buffer[0];
				/*
				// NUM_CLOCK 键判断
				if(LED_VALID)	// 未被按下
					mTimer1RunCTL(1);
				else	// 按下
					mTimer1RunCTL(0);
				*/
				if(LED_VALID)
				{
					WS2812Data(0);
					WS2812Data(0);
					WS2812Data(0);
				}
				else
				{
					WS2812Data(255);
					WS2812Data(0);
					WS2812Data(0);
				}
            }
            else if((SetupReq == 0x09) && (len == 8)){//SetReport						 
            }							
            UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA0,返回应答ACK
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //写0清空中断
    }
    if(UIF_BUS_RST)                                                       //设备模式USB总线复位中断
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //清中断标志
    }
    if (UIF_SUSPEND)                                                     //USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
    }
    else {                                                               //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF;                                               //清中断标志
    }
}
/**键盘HID值上传函数**/
void HIDValueHandle1()
{
    //TR0 = 0; //发送前关定时器中断
	FLAG = 0; //清空USB中断传输完成标志，准备发送按键按下数据
	Enp1IntIn(); //USB设备模式端点1的中断上传
	while(FLAG == 0); //等待USB中断数据传输完成
	FLAG = 0; //清空USB中断传输完成标志，准备发送按键抬起数据	
	memset(&HIDKey[0],0,8);	//把HIDkey置0，发送0表示按键抬起
	Enp1IntIn(); //USB设备模式端点1的中断上传		
	while(FLAG == 0); //等待USB中断数据传输完成
	//TR0 = 1; //发送完打开定时器中断		
}
/**多媒体按键HID值上传函数**/
void HIDValueHandle2()
{
    //TR0 = 0; //发送前关定时器中断
	FLAG = 0; //清空USB中断传输完成标志，准备发送按键按下数据
	Enp2IntIn(); //USB设备模式端点2的中断上传
	while(FLAG == 0); //等待USB中断数据传输完成
	FLAG = 0; //清空USB中断传输完成标志，准备发送按键抬起数据	
	memset(&HIDKeyMUL[0],0,4); //把HIDKeyMUL置0，发送0表示按键抬起
	Enp2IntIn(); //USB设备模式端点2的中断上传		
	while(FLAG == 0); //等待USB中断数据传输
	//TR0 = 1; //发送完打开定时器中断		
}
/**按键行为函数**/
/*找到按键的HID值自由发挥部分*/
/*普通按键
  例如ctrl + c :
  HIDKey[0] = 0x01;
  HIDKey[2] = 0x06;
  if(Ready) //枚举成功
	{
	    HIDValueHandle1();
	}
*/

// 用于上传功能
void Ready_function(void)
{
	if(Ready) //枚举成功
	{
		HIDValueHandle1(); //普通按键HID值上传
	}
}

/*************************************************************************************************************************************************/
/*
*	修改时间：2022.8.28
*	修改人：刘佳豪
*	修改内容：1.增加在NUM_CLOCK下双击 "5" 键切换灯光效果
*			  2.增加定时2用于检测第一次按下与第二次按下的时间间隔
*/

pdata UINT8 Time_Flag = 0; // 计时 开始/关闭 标志位
UINT8 LED_AttitudeFlag = 0;	// 灯效切换标志位

unsigned long pdata Double_TimeThr = 0;	// 计时时间（采用1ms中断的方式）

pdata UINT8 Key_Flag = 0;	// 第一次按下标志位
pdata UINT8 Key_DoubleFlag = 0;	// 第二次按下标志位

pdata UINT8 Add_Flag = 0;	// 用于灯效切换的限定标志位，使得 LED_AttitudeFlag 只加一次

// 关闭灯光函数
/*
*	对17个按键赋 0 值，同时对第一个按键赋值，使得只亮第一个按键
* 	[说明]：使用次函数时 NUM_CLOCK 必然按下
*/
void LED_Close()
{
	int j;
	for(j = 0; j < 17; j++)
	{
		if(j == 0)
		{
			WS2812Data(255);
			WS2812Data(0);
			WS2812Data(0);
		}
		else
		{
			WS2812Data(0);
			WS2812Data(0);
			WS2812Data(0);
		}
	}
}

// 按键行为调用函数
/*
*	通过获取相应键值，采用 switch 对每一个键值，进行对应的函数调用
*	[说明]：switch 比 if 效率更高
*			该部分对 "5" 所在的键值进行处理，有效防止其他按键冲突问题
*/
void KeyAction(unsigned char keyCode)
{
	switch(keyCode)
	{
		case 0xE7:
		{
			HIDKey[2] = 0x53; 
			Ready_function();
		}	break;	 // NUM CLOCK
		
		case 0xEB: HIDKey[2] = 0x54; Ready_function();	break;	// /
		case 0xED: HIDKey[2] = 0x55; Ready_function();	break;	// *
		case 0xEE: HIDKey[2] = 0x56; Ready_function();	break;	// -
		
		case 0xD7: HIDKey[2] = 0x5F; Ready_function();	break;	// 7
		case 0xDB: HIDKey[2] = 0x60; Ready_function();	break;	// 8
		case 0xDD: HIDKey[2] = 0x61; Ready_function();	break;	// 9
		case 0xDE: HIDKey[2] = 0x57; Ready_function();	break;	// +
		
		case 0xB7: HIDKey[2] = 0x5C; Ready_function();	break;	// 4
		
		case 0xBB:
		{			
			HIDKey[2] = 0x5D; 
			Ready_function();
			if(LED_VALID == 0)										// 当 NUM_CLOCK 按下时
			{
				if(Key_Flag == 0)									// 如果是第一次按下
				{
					Key_Flag = 1;	// 将第一次按下的标志更改，用于后面松手检测部分的函数使用
					Time_Flag = 1;	// 定时器计时开启
					mTimer2RunCTL(1);	// 定时器2开启
					Add_Flag = 1;	// 累加标志开启
				}
			}
		}	break;	 // 5
		
		case 0xBD: HIDKey[2] = 0x5E; Ready_function();	break;	// 6
		case 0xBE: HIDKey[2] = 0x58; Ready_function();	break;	// enter
		
		case 0x77: HIDKey[2] = 0x59; Ready_function();	break;	// 1
		case 0x7B: HIDKey[2] = 0x5A; Ready_function();	break;	// 2
		case 0x7D: HIDKey[2] = 0x5B; Ready_function();	break;	// 3
		case 0x7E: HIDKey[2] = 0x63; Ready_function();	break;	// .
		
		case 0x80: HIDKey[2] = 0x62; Ready_function();	break;	// 0
		default: break;
	} 
}

/*
按键动作函数8
void KeyAction(unsigned char keyCode)
{   
	if(keyCode == 0xE7)//按键1
	{
	    HIDKey[2] = 0x05;  //打开媒体播放器
		if(Ready) //枚举成功
        {
            HIDValueHandle1(); //多媒体按键HID值上传
        }
	}
	if(keyCode == 0x32)//按键
	{
		HIDKey[2] = 0x5C;  //下一曲
		if(Ready) //枚举成功
        {
            HIDValueHandle1(); //多媒体按键HID值上传
        }
	}
	if(keyCode == 0x33)	//按键3	
	{
		HIDKey[2] = 0x06;  //上一曲
		if(Ready) //枚举成功
        {
            HIDValueHandle1(); //普通按键HID值上传
        }
	}
	if(keyCode == 0x34)	//按键4	
	{
		HIDKey[2] = 0x07; //播放/暂停

		if(Ready) //枚举成功
        {
            HIDValueHandle1(); //多媒体按键HID值上传
        }
	}
//	if(keyCode == 0x35)	//按键5	
//	{
//		HIDKeyMUL[0] = 0x02; //音量+

//		if(Ready) //枚举成功
//        {
//            HIDValueHandle2(); //多媒体按键HID值上传
//        }
//	}
//	if(keyCode == 0x36)	//按键6	
//	{
//		HIDKeyMUL[0] = 0x01; //音量-

//		if(Ready) //枚举成功
//        {
//            HIDValueHandle2(); //多媒体按键HID值上传
//        }
//	}
}
*/

// 按键驱动
// 用途：放在main函数中读取按键状态，进行相应的处理
void KeyDrive()
{
	if(KeyState != BackState)										// 如果当前按键状态与上一次的按键状态不相等，即按键被按下
	{
		if(BackState != 0)											// 如果上一次按键状态不为0，即再次判断是否被按下
		{
			KeyAction(key); // 调用按键功能函数
		}
		BackState = KeyState;										// 按键状态刷新
	}
	if(KeyDownTime > 0)												// 此种情况是长按，如果按下的时间大于0
	{
		if(KeyDownTime >=  TimeThr)									// 如果按下的时间达到设定值
		{
			KeyAction(key);	// 调用按键功能函数
			TimeThr += 100; // 增大设定值，使得调用功能函数没那么快
		}
	}
	else															// 无按键按下
	{
		TimeThr = 1000; // 重置设定时间
		if(Key_Flag == 1)											// 按键第一次按下
		{
			Key_Flag = 0;	// 清空标志，等待第二次按下
			if(Key_DoubleFlag == 0)									// 如果第二次未被按下
			{
				Key_DoubleFlag = 1;	// 激活第二次按下的标志
				Double_TimeThr = 0;	// 重置松手计时
			}
			else if(Key_DoubleFlag == 1)							// 如果第二次按下
			{
				if(Double_TimeThr < 500 && Add_Flag == 1)			// 如果第一次与第二次按下的时间间隔小于 0.5s 并且累加标记为1
				{
					Key_DoubleFlag = 0;	// 第二次按下的标志清空
					Add_Flag = 0;	// 累加标志清空
					if(LED_AttitudeFlag >= 4)						// 如果灯效累加标志超过4
						LED_AttitudeFlag = 0;	// 重置为0
					switch(LED_AttitudeFlag)						// 用于切换灯效（默认无灯光）
					{
						case 0: 									// 第一种等效
						{
							i = 0;	// 重置 DS2812B 累加计数器
							LED_AttitudeFlag++;	// 等效累加标志加一
							WS2812B_DisplayType = 0; // 等效标志置为0
							mTimer1RunCTL(1); // 开启定时器1，开始显示灯光
						}	break;
						case 1:										// 第二种等效
						{
							mTimer1RunCTL(0); // 关闭定时器1（防止定时器内部计时不准的问题）
							i = 0; // 重置 DS2812B 累加计数器
							count_num = 0; // 重置定时器内部计数累加器
							LED_AttitudeFlag++; // 等效累加标志加一
							WS2812B_DisplayType = 1; // 等效标志置为1
							mTimer1RunCTL(1); // 开启定时器1，开始显示灯光
						}	break;
						case 2:										// 第三种等效
						{
							mTimer1RunCTL(0); // 关闭定时器1（防止定时器内部计时不准的问题）
							i = 0; // 重置 DS2812B 累加计数器
							count_num = 0; // 重置定时器内部计数累加器
							LED_AttitudeFlag++; // 等效累加标志加一
							WS2812B_DisplayType = 2; // 等效标志置为1
							mTimer1RunCTL(1); // 开启定时器1，开始显示灯光
						}	break;
						case 3: 									// 关闭灯光
						{
							mTimer1RunCTL(0); // 关闭定时器1（防止定时器内部计时不准的问题）
							count_num = 0; // 重置定时器内部计数累加器
							LED_Close(); // 调用关闭灯光函数 TP 642
							LED_AttitudeFlag++; // 等效累加标志加一
						}	break;
						default: break;
					}
				}
			}
		}
		else if(Key_DoubleFlag == 1)								// 如果第一次已经按下
		{
			if(Double_TimeThr >= 500)								// 第二次未在 0.5s 内按下
			{
				Key_DoubleFlag = 0;	// 第二次按下的标志位清空
				
				/* 此处逻辑有待完善 */
//				LED_AttitudeFlag = 0; // 等效累加 清空
				
				Time_Flag = 0; // 计时标志清空
				Double_TimeThr = 0;	// 计时时间清空
				mTimer2RunCTL(0);	// 关闭定时器2
			}
		}
	}
}
///**按键驱动**/
//void KeyDrive()
//{
//	unsigned char j;

//	for(j=0;j<4;j++)
//	{
//		if(KeyState[j] != BackState[j])
//		{
//	    	if(BackState[j] != 0)
//			{
//				KeyAction(key_code_map[j]);
//			}
//			BackState[j] = KeyState[j]; 
//		}
//		if(KeyDownTime[j] > 0)
//		{
//			if(KeyDownTime[j] >= TimeThr[j])
//			{
//				KeyAction(key_code_map[j]);
//				TimeThr[j] += 100;
//			} 
//		}
//		else
//		{
//			TimeThr[j] = 1000;
//		}		
//	}
//}

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
//	TMOD = 0x11;
	TH0 = T0RH;
	TL0 = T0RL;
	TF0 = 0;
	ET0 = 1;
	TR0 = 1;
}

UINT8 T2RH = 0;	//T2高8位重载值
UINT8 T2RL = 0;	//T2低8位重载值

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

//timer2中断服务程序
void	mTimer2Interrupt( void ) interrupt INT_NO_TMR2 using 3
{
	mTimer2RunCTL(0);
	if(TF2)
    {
        TF2 = 0;  // 清空定时器2溢出中断	  
        if(Time_Flag)												// 如果计时开始
		{
			Double_TimeThr++; // 时间累加
		}
		else														// 计时未开始
		{
			Double_TimeThr = 0;	// 时间置空
		}
    }
	mTimer2RunCTL(1);
}


/*****************主函数**********************/
main()
{
    CfgFsys(); //CH552时钟选择12M配置
    mDelaymS(5); //修改主频等待内部晶振稳定,必加	
	ConfigT0(10); //配置10ms T0中断
	ConfigT1(25); //配置1000ms T1中断
	ConfigT2(1);
	mTimer1RunCTL(0); //定时器1默认关闭
	mTimer2RunCTL(0);
	USBDeviceInit(); //USB设备模式初始化
    EA = 1; //允许单片机中断
    UEP1_T_LEN = 0; //预使用发送长度一定要清空
  	UEP2_T_LEN = 0;	//清空端点2发送长度
    FLAG = 0; //清空USB中断传输完成标志
    Ready = 0;
//	LED_VALID = 1;   //给一个默认值

	while(1)
	{
	    KeyDrive();	//按键驱动
		
	}
}

// 按键扫描
// 行列反扫描法
void KeyScan()
{
	UINT8 temp; 
	write_data(0xf0);	// 高4位置 1 
	
	if(read_data() != 0xf0 || LED17 != 1)							// 如果有按键被按下 或者 按键17被按下
	{
		if(read_data() != 0xf0 || LED17 != 1)						// 再次确认
		{
			KeyState = 0;	// 按键状态置0
			if(read_data() != 0xf0)									// 如果是矩阵按键被按下
			{
				temp = read_data();	// 读出键值
				write_data(0x0F);	// 反转	低4位置 1
				key = temp | read_data();	// 读出两值进行或运算得出键值
			}
			if(LED17 != 1)											// 如果按键17被按下
			{
				key = 0x80;											// 更改键值读数为 0x80
			}
			KeyDownTime += 25;										// 只要有按键按下，按下时间就加25
		}
	}
	else 															// 无按键按下
	{
		key = 0; // 键值缓存清空
		KeyState = 1;	// 按键状态置1
		KeyDownTime = 0;	// 按下时间清空
	}
}
/**T0中断函数**/
void InterruptTimer0() interrupt INT_NO_TMR0 using 2
{
	TH0 = T0RH;
	TL0 = T0RL;

	KeyScan(); //按键扫描
}



/*****************************************************************************************************************/


