/********************************** (C) COPYRIGHT *******************************
* File Name		  : CDC.C
* Author			 : Kongou Hikari
* Version			: V1.0
* Date			   : 2019/02/16
* Description		: CH552 USB to JTAG with FTDI Protocol
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <ch554.h>
#include <ch554_usb.h>
#include <debug.h>

/*
Memory map:
EP0 Buf		00 - 3f
EP4 Buf 	40 - 7f
EP1 Buf		80 - bf

EP2 Buf		300 - 33f
EP3 Buf 	380 - 3bf
*/

__xdata __at (0x0000) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];	   //端点0 OUT&IN缓冲区，必须是偶地址

__xdata __at (0x0080) uint8_t  Ep1Buffer[MAX_PACKET_SIZE];		//端点1 IN 发送缓冲区
__xdata __at (0x0300) uint8_t  Ep2Buffer[MAX_PACKET_SIZE * 2];	  //端点2 OUT接收缓冲区

__xdata __at (0x0380) uint8_t  Ep3Buffer[MAX_PACKET_SIZE];		//端点3 IN 发送缓冲区
__xdata __at (0x0040) uint8_t  Ep4Buffer[MAX_PACKET_SIZE];	  //端点4 OUT接收缓冲区


uint16_t SetupLen;
uint8_t   SetupReq, Count, UsbConfig;
uint8_t   VendorControl;

__code uint8_t *  pDescr;													   //USB配置标志
uint8_t pDescr_Index = 0;
USB_SETUP_REQ   SetupReqBuf;												   //暂存Setup包
#define UsbSetupBuf	 ((PUSB_SETUP_REQ)Ep0Buffer)


/*设备描述符*/
__code uint8_t DevDesc[] = {0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, DEFAULT_ENDP0_SIZE,
							0x03, 0x04, 0x10, 0x60, 0x00, 0x05, 0x01, 0x02,
							0x03, 0x01
						   };
__code uint8_t CfgDesc[] =
{
	0x09, 0x02, sizeof(CfgDesc) & 0xff, sizeof(CfgDesc) >> 8,
	0x02, 0x01, 0x00, 0x80, 0x32,		 //配置描述符（1个接口）
	//以下为接口0（数据接口）描述符
	0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0xff, 0xff, 0x03,	 //数据接口描述符
	0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,				 //端点描述符 EP1 BULK IN
	0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,				 //端点描述符 EP2 BULK OUT
	//以下为接口1（数据接口）描述符
	0x09, 0x04, 0x01, 0x00, 0x02, 0xff, 0xff, 0xff, 0x00,	 //数据接口描述符
	0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00,				 //端点描述符 EP3 BULK IN
	0x07, 0x05, 0x04, 0x02, 0x40, 0x00, 0x00,				 //端点描述符 EP4 BULK OUT
};
/*字符串描述符*/
unsigned char  __code LangDes[] = {0x04, 0x03, 0x09, 0x04};	  //语言描述符

unsigned char  __code Prod_Des[] =								//产品字符串描述符
{
	sizeof(Prod_Des), 0x03,
	'S', 0x00, 'i', 0x00, 'p', 0x00, 'e', 0x00, 'e', 0x00, 'd', 0x00,
	'-', 0x00, 'U', 0x00, 'A', 0x00, 'R', 0x00, 'T', 0x00,
};
unsigned char  __code Jtag_Des[] =								//产品字符串描述符
{
	sizeof(Jtag_Des), 0x03,
	'S', 0x00, 'i', 0x00, 'p', 0x00, 'e', 0x00, 'e', 0x00, 'd', 0x00,
	'-', 0x00, 'J', 0x00, 'T', 0x00, 'A', 0x00, 'G', 0x00
};
unsigned char  __code Manuf_Des[] =
{
	sizeof(Manuf_Des), 0x03,
	'K', 0x00, 'o', 0x00, 'n', 0x00, 'g', 0x00, 'o', 0x00, 'u', 0x00,
	' ', 0x00, 'H', 0x00, 'i', 0x00, 'k', 0x00, 'a', 0x00, 'r', 0x00, 'i', 0x00
};

/* 下载控制 */
volatile __idata uint8_t USBOutLength = 0;
volatile __idata uint8_t USBOutPtr = 0;
volatile __idata uint8_t USBReceived = 0;

volatile __idata uint8_t USBRecvLen_A = 0;
volatile __idata uint8_t USBRecvLen_B = 0;
volatile __idata uint8_t USBRecvBuf = 0;
volatile __idata uint8_t Serial_Done = 0;
volatile __idata uint8_t USBBufState = 0;
volatile __idata uint8_t SerialSendBuf = 0;
volatile __idata uint8_t USB_Require_Data = 0;

volatile __idata uint8_t USBOutLength_1 = 0;
volatile __idata uint8_t USBOutPtr_1 = 0;
volatile __idata uint8_t USBReceived_1 = 0;
/* 上传控制 */
volatile __idata uint8_t UpPoint1_Busy = 0;   //上传端点是否忙标志
volatile __idata uint8_t UpPoint1_LenA = 2;
volatile __idata uint8_t UpPoint1_LenB = 2;

volatile __idata uint8_t UpPoint3_Busy = 0;   //上传端点是否忙标志

/* 杂项 */
volatile __idata uint16_t SOF_Count = 0;
volatile __idata uint8_t Latency_Timer = 4; //Latency Timer
volatile __idata uint8_t Latency_Timer1 = 4;
volatile __idata uint8_t Require_DFU = 0;


#define HARD_ESP_CTRL 1

#ifndef HARD_ESP_CTRL
volatile __idata uint8_t Esp_Boot_Chk = 0;
volatile __idata uint8_t Esp_Require_Reset = 0;
#endif

/*******************************************************************************
* Function Name  : USBDeviceCfg()
* Description	: USB设备模式配置
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceCfg()
{
	USB_CTRL = 0x00;														   //清空USB控制寄存器
	USB_CTRL &= ~bUC_HOST_MODE;												//该位为选择设备模式
	USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;					//USB设备和内部上拉使能,在中断期间中断标志未清除前自动返回NAK
	USB_DEV_AD = 0x00;														 //设备地址初始化
	//	 USB_CTRL |= bUC_LOW_SPEED;
	//	 UDEV_CTRL |= bUD_LOW_SPEED;												//选择低速1.5M模式
	USB_CTRL &= ~bUC_LOW_SPEED;
	UDEV_CTRL &= ~bUD_LOW_SPEED;											 //选择全速12M模式，默认方式
	UDEV_CTRL = bUD_PD_DIS;  // 禁止DP/DM下拉电阻
	UDEV_CTRL |= bUD_PORT_EN;												  //使能物理端口
}

void Jump_to_BL()
{
	ES = 0;
	PS = 0;

	USB_INT_EN = 0;
	USB_CTRL = 0x06;
	//UDEV_CTRL = 0x80;

	mDelaymS(100);

	EA = 0;

	while(1)
	{
		__asm
		LJMP 0x3800
		__endasm;
	}
}
/*******************************************************************************
* Function Name  : USBDeviceIntCfg()
* Description	: USB设备模式中断初始化
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceIntCfg()
{
	USB_INT_EN |= bUIE_SUSPEND;											   //使能设备挂起中断
	USB_INT_EN |= bUIE_TRANSFER;											  //使能USB传输完成中断
	USB_INT_EN |= bUIE_BUS_RST;											   //使能设备模式USB总线复位中断
	USB_INT_EN |= bUIE_DEV_SOF;													//打开SOF中断
	USB_INT_FG |= 0x1F;													   //清中断标志
	IE_USB = 1;															   //使能USB中断
	EA = 1;																   //允许单片机中断
}
/*******************************************************************************
* Function Name  : USBDeviceEndPointCfg()
* Description	: USB设备模式端点配置，模拟兼容HID设备，除了端点0的控制传输，还包括端点2批量上下传
* Input		  : None
* Output		 : None
* Return		 : None
*******************************************************************************/
void USBDeviceEndPointCfg()
{
	// TODO: Is casting the right thing here? What about endianness?
	UEP2_DMA = (uint16_t) Ep2Buffer;											//端点2 OUT接收数据传输地址
	UEP3_DMA = (uint16_t) Ep3Buffer;
	//UEP2_3_MOD = 0x48;															//端点2 单缓冲接收, 端点3单缓冲发送
	UEP2_3_MOD = 0x49;				//端点3单缓冲发送,端点2双缓冲接收

	UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;									//端点2 自动翻转同步标志位，OUT返回ACK
	UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; //端点3发送返回NAK

	//UEP4_DMA = (uint16_t) Ep4Buffer; //Ep4Buffer = Ep0Buffer + 64
	UEP1_DMA = (uint16_t) Ep1Buffer;										   //端点1 IN 发送数据传输地址
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;								 //端点1 自动翻转同步标志位，IN事务返回NAK
	UEP4_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK; //端点4接收返回ACK, 无法自动翻转
	UEP4_1_MOD = 0x48;														 //端点1 单缓冲发送, 端点4单缓冲接收

	UEP0_DMA = (uint16_t) Ep0Buffer;													  //端点0数据传输地址
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;								 //手动翻转，OUT事务返回ACK，IN事务返回NAK
}

__code uint8_t HexToAscTab[] = "0123456789ABCDEF";

void uuidcpy(__xdata uint8_t *dest, uint8_t index, uint8_t len) /* 使用UUID生成USB Serial Number */
{
	uint8_t i;
	uint8_t p = 0; /* UUID格式, 十位十六进制数 */
	__code uint8_t *puuid;
	for(i = index; i < (index + len); i++)
	{
		if(i == 0)
			dest[p++] = 22; //10 * 2 + 2
		else if(i == 1)
			dest[p++] = 0x03;
		else
		{
			if(i & 0x01) //奇数
			{
				dest[p++] = 0x00;
			}
			else
			{
				puuid = (__code uint8_t *) (0x3ffa + (i - 2) / 4);
				if(i & 0x02)
					dest[p++] = HexToAscTab[(*puuid) >> 4];
				else
					dest[p++] = HexToAscTab[(*puuid) & 0x0f];
			}
		}
	}
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description	: CH559USB中断处理函数
*******************************************************************************/
void DeviceInterrupt(void) __interrupt (INT_NO_USB)					   //USB中断服务程序,使用寄存器组1
{
	uint16_t len;
	uint16_t divisor;
	if ((USB_INT_ST & MASK_UIS_TOKEN) == UIS_TOKEN_SOF)
	{
		SOF_Count ++;
	}
	if(UIF_TRANSFER)															//USB传输完成标志
	{
		switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
		{
		case UIS_TOKEN_IN | 1:												  //endpoint 1# 端点批量上传
			UEP1_T_LEN = 0;
			UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;		   //默认应答NAK
			UpPoint1_Busy = 0;												  //清除忙标志
			break;
		case UIS_TOKEN_OUT | 2:												 //endpoint 2# 端点批量下传
			if ( U_TOG_OK )													 // 不同步的数据包将丢弃
			{
				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;	   //收到一包数据就NAK，主函数处理完，由主函数修改响应方式
				USBReceived = 1;
				if(UEP2_CTRL & bUEP_R_TOG)
				{
					USBRecvBuf = 0; //缓冲2
                    USBRecvLen_A = USB_RX_LEN;
				}
				else
				{
					USBRecvBuf = 1; //缓冲1
					USBRecvLen_B = USB_RX_LEN;
				}
				USB_Require_Data = 0;
			}
			break;
		case UIS_TOKEN_IN | 3:												  //endpoint 3# 端点批量上传
			UEP3_T_LEN = 0;
			UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;		   //默认应答NAK
			UpPoint3_Busy = 0;												  //清除忙标志
			break;
		case UIS_TOKEN_OUT | 4:												 //endpoint 4# 端点批量下传
			if ( U_TOG_OK )													 // 不同步的数据包将丢弃
			{
				UEP4_CTRL ^= bUEP_R_TOG;	//同步标志位翻转
				UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;	   //收到一包数据就NAK，主函数处理完，由主函数修改响应方式
				USBOutPtr_1 = 64; //TODO: Nasty Solution
				USBOutLength_1 = USB_RX_LEN + 64;
				USBReceived_1 = 1;
			}
			break;
		case UIS_TOKEN_SETUP | 0:												//SETUP事务
			len = USB_RX_LEN;
			if(len == (sizeof(USB_SETUP_REQ)))
			{
				SetupLen = ((uint16_t)UsbSetupBuf->wLengthH << 8) | (UsbSetupBuf->wLengthL);
				len = 0;													  // 默认为成功并且上传0长度
				VendorControl = 0;
				SetupReq = UsbSetupBuf->bRequest;
				if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )//非标准请求
				{
					//TODO: 重写
					VendorControl = 1;
					if(UsbSetupBuf->bRequestType & USB_REQ_TYP_READ)
					{
						//读
						switch( SetupReq )
						{
						case 0x90: //READ EEPROM
							Ep0Buffer[0] = 0xff;
							Ep0Buffer[1] = 0xff;
							len = 2;
							break;
						case 0x0a:
							if(UsbSetupBuf->wIndexL == 2)
								Ep0Buffer[0] = Latency_Timer1;
							else
								Ep0Buffer[0] = Latency_Timer;
							len = 1;
							break;
						case 0x05:
							Ep0Buffer[0] = 0x01;
							Ep0Buffer[1] = 0x60;
							len = 2;
							break;
						default:
							len = 0xFF;	 /*命令不支持*/
							break;
						}
					}
					else
					{
						//写
						switch( SetupReq )
						{
						case 0x02:
						case 0x04:
						case 0x06:
						case 0x07:
						case 0x0b:
						case 0x92:
							len = 0;
							break;
						case 0x91: //WRITE EEPROM, FT_PROG动作,直接跳转BL
							Require_DFU = 1;
							len = 0;
							break;
						case 0x00:
							if(UsbSetupBuf->wIndexL == 1)
								UpPoint1_Busy = 0;
							if(UsbSetupBuf->wIndexL == 2)
							{
								UpPoint3_Busy = 0;
								UEP4_CTRL &= ~(bUEP_R_TOG);
							}
							len = 0;
							break;
						case 0x09: //SET LATENCY TIMER
							if(UsbSetupBuf->wIndexL == 1)
								Latency_Timer = UsbSetupBuf->wValueL;
							else
								Latency_Timer1 = UsbSetupBuf->wValueL;
							len = 0;
							break;
						case 0x03:
							//divisor = wValue
							len = 0;
							break;
						case 0x01: //MODEM Control
							len = 0;
							break;
						default:
							len = 0xFF;		 /*命令不支持*/
							break;
						}
					}

				}
				else															 //标准请求
				{
					switch(SetupReq)											 //请求码
					{
					case USB_GET_DESCRIPTOR:
						switch(UsbSetupBuf->wValueH)
						{
						case 1:													   //设备描述符
							pDescr = DevDesc;										 //把设备描述符送到要发送的缓冲区
							len = sizeof(DevDesc);
							break;
						case 2:														//配置描述符
							pDescr = CfgDesc;										  //把设备描述符送到要发送的缓冲区
							len = sizeof(CfgDesc);
							break;
						case 3:
							if(UsbSetupBuf->wValueL == 0)
							{
								pDescr = LangDes;
								len = sizeof(LangDes);
							}
							else if(UsbSetupBuf->wValueL == 1)
							{
								pDescr = Manuf_Des;
								len = sizeof(Manuf_Des);
							}
							else if(UsbSetupBuf->wValueL == 2)
							{
								pDescr = Prod_Des;
								len = sizeof(Prod_Des);
							}
							else if(UsbSetupBuf->wValueL == 3)
							{
								pDescr = Jtag_Des;
								len = sizeof(Jtag_Des);
							}
							else
							{
								pDescr = (__code uint8_t *)0xffff;
								len = 22; /* 10位ASCII序列号 */
							}
							break;
						default:
							len = 0xff;												//不支持的命令或者出错
							break;
						}

						if ( SetupLen > len )
						{
							SetupLen = len;	//限制总长度
						}
						len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;							//本次传输长度
						if(pDescr == (__code uint8_t *) 0xffff) /* 取序列号的话 */
						{
							uuidcpy(Ep0Buffer, 0, len);
						}
						else
						{
							memcpy(Ep0Buffer, pDescr, len);								//加载上传数据
						}
						SetupLen -= len;
						pDescr_Index = len;
						break;
					case USB_SET_ADDRESS:
						SetupLen = UsbSetupBuf->wValueL;							  //暂存USB设备地址
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
					case USB_GET_INTERFACE:
						break;
					case USB_CLEAR_FEATURE:											//Clear Feature
						if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )				  /* 清除设备 */
						{
							if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
							{
								if( CfgDesc[ 7 ] & 0x20 )
								{
									/* 唤醒 */
								}
								else
								{
									len = 0xFF;										/* 操作失败 */
								}
							}
							else
							{
								len = 0xFF;											/* 操作失败 */
							}
						}
						else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
						{
							switch( UsbSetupBuf->wIndexL )
							{
							case 0x83:
								UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
								break;
							case 0x03:
								UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
								break;
							case 0x82:
								UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
								break;
							case 0x02:
								UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
								break;
							case 0x81:
								UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
								break;
							case 0x01:
								UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
								break;
							default:
								len = 0xFF;										 // 不支持的端点
								break;
							}
							UpPoint1_Busy = 0;
							UpPoint3_Busy = 0;
						}
						else
						{
							len = 0xFF;												// 不是端点不支持
						}
						break;
					case USB_SET_FEATURE:										  /* Set Feature */
						if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )				  /* 设置设备 */
						{
							if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
							{
								if( CfgDesc[ 7 ] & 0x20 )
								{
									/* 休眠 */
#ifdef DE_PRINTF
									printf( "suspend\n" );															 //睡眠状态

									while ( XBUS_AUX & bUART0_TX )
									{
										;	//等待发送完成
									}
#endif
									SAFE_MOD = 0x55;
									SAFE_MOD = 0xAA;
									WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;					  //USB或者RXD0/1有信号时可被唤醒
									PCON |= PD;																 //睡眠
									SAFE_MOD = 0x55;
									SAFE_MOD = 0xAA;
									WAKE_CTRL = 0x00;
								}
								else
								{
									len = 0xFF;										/* 操作失败 */
								}
							}
							else
							{
								len = 0xFF;											/* 操作失败 */
							}
						}
						else if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_ENDP )			 /* 设置端点 */
						{
							if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
							{
								switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
								{
								case 0x83:
									UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点3 IN STALL */
									break;
								case 0x03:
									UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点3 OUT Stall */
									break;
								case 0x82:
									UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
									break;
								case 0x02:
									UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
									break;
								case 0x81:
									UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
									break;
								case 0x01:
									UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点1 OUT Stall */
								default:
									len = 0xFF;									/* 操作失败 */
									break;
								}
							}
							else
							{
								len = 0xFF;									  /* 操作失败 */
							}
						}
						else
						{
							len = 0xFF;										  /* 操作失败 */
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
						len = 0xff;													//操作失败
						break;
					}
				}
			}
			else
			{
				len = 0xff;														 //包长度错误
			}
			if(len == 0xff)
			{
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
			}
			else if(len <= DEFAULT_ENDP0_SIZE)													   //上传数据或者状态阶段返回0长度包
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
		case UIS_TOKEN_IN | 0:													  //endpoint0 IN
			switch(SetupReq)
			{
			case USB_GET_DESCRIPTOR:
				len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;			  //本次传输长度
				if(pDescr == (__code uint8_t *)0xffff)
				{
					uuidcpy(Ep0Buffer, pDescr_Index, len);
				}
				else
				{
					memcpy( Ep0Buffer, pDescr + pDescr_Index, len );								   //加载上传数据
				}
				SetupLen -= len;
				pDescr_Index += len;
				UEP0_T_LEN = len;
				UEP0_CTRL ^= bUEP_T_TOG;											 //同步标志位翻转
				break;
			case USB_SET_ADDRESS:
				if(VendorControl == 0)
				{
					USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				}
				break;
			default:
				UEP0_T_LEN = 0;													  //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
			break;
		case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
			if(SetupReq == 0x22) //设置串口属性
			{

			}
			else
			{
				UEP0_T_LEN = 0;
				UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;  //状态阶段，对IN响应NAK
			}
			break;

		default:
			break;
		}
		UIF_TRANSFER = 0;														   //写0清空中断
	}
	if(UIF_BUS_RST)																 //设备模式USB总线复位中断
	{
#ifdef DE_PRINTF
		printf( "reset\n" );															 //睡眠状态
#endif
		UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
		UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
		USB_DEV_AD = 0x00;
		UIF_SUSPEND = 0;
		UIF_TRANSFER = 0;
		UIF_BUS_RST = 0;															 //清中断标志
		UsbConfig = 0;		  //清除配置值
		UpPoint1_Busy = 0;
		UpPoint3_Busy = 0;

		USBOutLength = 0;
		USBOutPtr = 0;
		USBReceived = 0;

		USBOutLength_1 = 0;
		USBOutPtr_1 = 0;
		USBReceived_1 = 0;

		UpPoint1_LenA = 2;
		UpPoint1_LenB = 2;

		USBRecvLen_A = 0;
		USBRecvLen_B = 0;
		USBRecvBuf = 0;
		SerialSendBuf = 0;
		USBBufState = 0;
		Serial_Done = 0;
		USB_Require_Data = 0;
	}
	if (UIF_SUSPEND)																 //USB总线挂起/唤醒完成
	{
		UIF_SUSPEND = 0;
		if ( USB_MIS_ST & bUMS_SUSPEND )											 //挂起
		{
#ifdef DE_PRINTF
			printf( "suspend\n" );															 //睡眠状态
#endif
			while ( XBUS_AUX & bUART0_TX )
			{
				;	//等待发送完成
			}
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;					  //USB或者RXD0/1有信号时可被唤醒
			PCON |= PD;																 //睡眠
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = 0x00;
		}
	}
	else																			   //意外的中断,不可能发生的情况
	{
		USB_INT_FG = 0xFF;															 //清中断标志

	}
}

void SerialPort_Config()
{
	volatile uint32_t x;
	volatile uint8_t x2;

    P1_MOD_OC = 0x7f;
    P1_DIR_PU = 0xcc; //P1.4 P1.5 P1.0 P1.1开漏
    P3_MOD_OC = 0xfd;

	SM0 = 0;
	SM1 = 1;
	SM2 = 0;																   //串口0使用模式1
	//使用Timer1作为波特率发生器
	RCLK = 0;																  //UART0接收时钟
	TCLK = 0;																  //UART0发送时钟
	PCON |= SMOD;
	x = 10 * FREQ_SYS / 115200 / 16;									   //如果更改主频，注意x的值不要溢出
	x2 = x % 10;
	x /= 10;
	if ( x2 >= 5 ) x ++;													   //四舍五入

	TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1;			  //0X20，Timer1作为8位自动重载定时器
	T2MOD = T2MOD | bTMR_CLK | bT1_CLK;										//Timer1时钟选择
	TH1 = 0 - x;															   //12MHz晶振,buad/12为实际需设置波特率
	TR1 = 1;																   //启动定时器1
	TI = 0;
	REN = 1;																   //串口0接收使能
	ES = 1; //开串口中断
	PS = 1; //中断优先级最高

	//串口2配置
    SCON1 = 0x70; //8bit, fast, receive enable
	SBAUD1 = 0 - x;
	IE_UART1 = 1;
	IP_EX |= bIP_UART1;
}

void Xtal_Enable(void) //使能外部时钟
{
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
	CLOCK_CFG |= bOSC_EN_XT;                          //使能外部24M晶振
	SAFE_MOD = 0x00;
	mDelaymS(5);
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
	CLOCK_CFG &= ~bOSC_EN_INT;                        //关闭内部RC
	SAFE_MOD = 0x00;
}

void CLKO_Enable(void) //打开T2输出
{
	ET2 = 0;
	T2MOD |= bTMR_CLK | bT2_CLK | T2OE;
	RCAP2H = 0xff;
	RCAP2L = 0xff;
	T2CON |= TR2;
	P1_MOD_OC &= ~(0x01); //P1.0推挽输出
}

//主函数
main()
{
	uint8_t i;
	volatile uint16_t Uart_Timeout = 0;
	volatile uint16_t Uart_Timeout1 = 0;
	uint16_t Esp_Stage = 0;
	int8_t size;

	
	Xtal_Enable();	//启动振荡器
	CfgFsys( );														   //CH552时钟选择配置
	mDelaymS(5);														  //修改主频等待内部时钟稳定,必加
	CLKO_Enable();

#ifdef DE_PRINTF
	printf("start ...\n");
#endif
	USBDeviceCfg();
	USBDeviceEndPointCfg();											   //端点配置
	USBDeviceIntCfg();													//中断初始化
	UEP0_T_LEN = 0;
	UEP1_T_LEN = 0;													   //预使用发送长度一定要清空
	UEP2_T_LEN = 0;													   //预使用发送长度一定要清空

	/* 预先填充 Modem Status */
	Ep1Buffer[0] = 0x01;
	Ep1Buffer[1] = 0x60;
	Ep3Buffer[0] = 0x01;
	Ep3Buffer[1] = 0x60;
	XBUS_AUX = 0;
	while(1)
	{
		if(UsbConfig)
		{
			if(UpPoint1_Busy == 0)
			{
				size = 0;

				if(size >= 62)
				{
				#if 0
					for(i = 0; i < 62; i++)
					{
						Ep1Buffer[2 + i] = RingBuf[ReadPtr++];
						ReadPtr %= sizeof(RingBuf);
					}
				#endif

					UpPoint1_Busy = 1;
					UEP1_T_LEN = 64;
					UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;

				}
				else if((uint16_t) (SOF_Count - Uart_Timeout) >= Latency_Timer) //超时
				{
					Uart_Timeout = SOF_Count;
					if(size > 62) size = 62;
				#if 0
					for(i = 0; i < (uint8_t)size; i++)
					{
						Ep1Buffer[2 + i] = RingBuf[ReadPtr++];
						ReadPtr %= sizeof(RingBuf);
					}
				#endif

					UpPoint1_Busy = 1;
					UEP1_T_LEN = 2 + size;
					UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;			//应答ACK
				}
			}

			if(UpPoint3_Busy == 0)
			{
				size = 0;

				if(size >= 62)
				{
				#if 0
					for(i = 0; i < 62; i++)
					{
						Ep3Buffer[2 + i] = RingBuf_1[ReadPtr_1++];
						ReadPtr_1 %= sizeof(RingBuf_1);
					}
				#endif

					UpPoint3_Busy = 1;
					UEP3_T_LEN = 64;
					UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;


				}
				else if((uint16_t) (SOF_Count - Uart_Timeout1) >= Latency_Timer1) //超时
				{
					Uart_Timeout1 = SOF_Count;
					if(size > 62) size = 62;
				#if 0
					for(i = 0; i < (uint8_t)size; i++)
					{
						Ep3Buffer[2 + i] = RingBuf_1[ReadPtr_1++];
						ReadPtr_1 %= sizeof(RingBuf_1);
					}
				#endif

					UpPoint3_Busy = 1;
					UEP3_T_LEN = 2 + size;
					UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;			//应答ACK
				}
			}

			if(Require_DFU)
			{
				Require_DFU = 0;
				Jump_to_BL();
			}
		}
	}
}
