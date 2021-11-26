/********************************** (C) COPYRIGHT *******************************
* File Name		: CDC.C
* Author		: Kongou Hikari
* Version		: V1.0
* Date			: 2019/02/16
* Description	: CH552 USB to JTAG with FTDI Protocol
*******************************************************************************/
#include "hal.h"

/*
Memory map:
EP0 Buf	00 - 3f
EP4 Buf 	40 - 7f
EP1 Buf	80 - bf
RingBuf	100 - 1ff
EP2 Buf	300 - 37f
EP3 Buf 	380 - 3bf
*/

__xdata __at (0x0000) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];	// Endpoint 0 OUT & IN buffer must be an even address
__xdata __at (0x0080) uint8_t  Ep1Buffer[MAX_PACKET_SIZE];		// Endpoint 1 IN transmit buffer
__xdata __at (0x0300) uint8_t  Ep2Buffer[MAX_PACKET_SIZE * 2];	// Endpoint 2 OUT Receive Buffer
__xdata __at (0x0380) uint8_t  Ep3Buffer[MAX_PACKET_SIZE];		// Endpoint 3 IN transmit buffer
__xdata __at (0x0040) uint8_t  Ep4Buffer[MAX_PACKET_SIZE];		// Endpoint 4 OUT Receive Buffer
__xdata __at (0x0100) uint8_t  RingBuf[128];

uint16_t SetupLen;
uint8_t   SetupReq, Count, UsbConfig;
uint8_t   VendorControl;

__code uint8_t *  pDescr;										// USB configuration flag
uint8_t pDescr_Index = 0;
USB_SETUP_REQ   SetupReqBuf;									// temporarily store the Setup package
#define UsbSetupBuf	((PUSB_SETUP_REQ)Ep0Buffer)

/* Device descriptor */
__code uint8_t DevDesc[] = {0x12, 0x01, 0x00, 0x02,
                            0x00, 0x00, 0x00, DEFAULT_ENDP0_SIZE,
                            0x03, 0x04, 0x10, 0x60, 0x00, 0x05, 0x01, 0x02,
                            0x03, 0x01
                           };
__code uint16_t itdf_eeprom [] = {
	0x0800, 0x0403, 0x6010, 0x0500, 0x3280, 0x0000, 0x0200, 0x1096,
	0x1aa6, 0x0000, 0x0046, 0x0310, 0x004f, 0x0070, 0x0065, 0x006e,
	0x002d, 0x0045, 0x0043, 0x031a, 0x0055, 0x0053, 0x0042, 0x0020,
	0x0044, 0x0065, 0x0062, 0x0075, 0x0067, 0x0067, 0x0065, 0x0072,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1027
};
__code uint8_t CfgDesc[] = {
	0x09, 0x02, sizeof(CfgDesc) & 0xff, sizeof(CfgDesc) >> 8,
	0x02, 0x01, 0x00, 0x80, 0x32,		// Configuration descriptor (1 interface)

	// The following is the interface 0 (data interface) descriptor
	0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0xff, 0xff, 0x04,	// Data interface descriptor
	0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,				// Endpoint descriptor EP1 BULK IN
	0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,				// Endpoint descriptor EP2 BULK OUT

	//The following is the interface 1 (data interface) descriptor
	0x09, 0x04, 0x01, 0x00, 0x02, 0xff, 0xff, 0xff, 0x00,	// Data interface descriptor
	0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00,				// Endpoint descriptor EP3 BULK IN
	0x07, 0x05, 0x04, 0x02, 0x40, 0x00, 0x00,				// Endpoint descriptor EP4 BULK OUT
};
/* String descriptor */
unsigned char  __code LangDes[] = {0x04, 0x03, 0x09, 0x04};	// Language descriptor

unsigned char  __code Prod_Des[] = {						// Product string descriptor
	sizeof(Prod_Des), 0x03,
	'S', 0x00, 'i', 0x00, 'p', 0x00, 'e', 0x00, 'e', 0x00, 'd', 0x00,
	'-', 0x00, 'D', 0x00, 'e', 0x00, 'b', 0x00, 'u', 0x00, 'g', 0x00
};
unsigned char  __code Jtag_Des[] = {						// Product string descriptor
	sizeof(Jtag_Des), 0x03,
	'S', 0x00, 'i', 0x00, 'p', 0x00, 'e', 0x00, 'e', 0x00, 'd', 0x00,
	'-', 0x00, 'J', 0x00, 'T', 0x00, 'A', 0x00, 'G', 0x00
};
unsigned char  __code Manuf_Des[] = {
	sizeof(Manuf_Des), 0x03,
	'K', 0x00, 'o', 0x00, 'n', 0x00, 'g', 0x00, 'o', 0x00, 'u', 0x00,
	' ', 0x00, 'H', 0x00, 'i', 0x00, 'k', 0x00, 'a', 0x00, 'r', 0x00, 'i', 0x00
};

__code uint8_t QualifierDesc[] = {
	10,         	/* bLength */
	USB_DESCR_TYP_QUALIF,	/* bDescriptorType */

	0x00, 0x02,				/* bcdUSB */

	0xff,                   /* bDeviceClass */
	0xff,                   /* bDeviceSubClass */
	0xff,                   /* bDeviceProtocol */

	DEFAULT_ENDP0_SIZE,     /* bMaxPacketSize0 */
	0x00,                   /* bNumOtherSpeedConfigurations */
	0x00                    /* bReserved */
};

/* Download control */
volatile __idata uint8_t USBOutLength	= 0;
volatile __idata uint8_t USBOutPtr		= 0;
volatile __idata uint8_t USBReceived	= 0;

volatile __idata uint8_t Serial_Done	= 0;
volatile __idata uint8_t USB_Require_Data = 0;

volatile __idata uint8_t USBOutLength_1	= 0;
volatile __idata uint8_t USBOutPtr_1	= 0;
volatile __idata uint8_t USBReceived_1	= 0;
/* Upload control */
volatile __idata uint8_t UpPoint1_Busy	= 0;   //Is the upload endpoint busy flag
volatile __idata uint8_t UpPoint1_Ptr	= 2;

volatile __idata uint8_t UpPoint3_Busy	= 0;   //Is the upload endpoint busy flag
volatile __idata uint8_t UpPoint3_Ptr	= 2;

/* Miscellaneous */
volatile __idata uint16_t SOF_Count		= 0;
volatile __idata uint8_t Latency_Timer	= 4; // Latency Timer
volatile __idata uint8_t Latency_Timer1	= 4;
volatile __idata uint8_t Require_DFU	= 0;

/* Flow control */
volatile __idata uint8_t soft_dtr = 0;
volatile __idata uint8_t soft_rts = 0;

/* MPSSE settings */

volatile __idata uint8_t Mpsse_Status	= 0;
volatile __idata uint16_t Mpsse_LongLen = 0;
volatile __idata uint8_t Mpsse_ShortLen = 0;

#ifdef DE_PRINTF
#define dbg_printf(...) printf(s__VA_ARGS__)
#else
#define	dbg_printf(...)
#endif

/*******************************************************************************
* Function Name  : USBDeviceCfg()
* Description	: USB device mode configuration
* Input		: None
* Output		: None
* Return		: None
*******************************************************************************/
void USBDeviceCfg() {
	USB_CTRL = 0x00;														// clear the USB control register
	USB_CTRL &= ~bUC_HOST_MODE;												// This bit is to select the device mode
	USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;					// USB device and internal pull-up enable, automatically return to NAK before the interrupt flag is cleared during the interrupt
	USB_DEV_AD = 0x00;														// Device address initialization
	// USB_CTRL |= bUC_LOW_SPEED;
	//	UDEV_CTRL |= bUD_LOW_SPEED;											//Select low speed 1.5M mode
	USB_CTRL &= ~bUC_LOW_SPEED;
	UDEV_CTRL &= ~bUD_LOW_SPEED;											// Select the full speed 12M mode, the default mode
	UDEV_CTRL = bUD_PD_DIS;  // Prohibit DP/DM pull-down resistor
	UDEV_CTRL |= bUD_PORT_EN;												// Enable physical port
}

/*******************************************************************************
* Function Name  : USBDeviceIntCfg()
* Description	: USB device mode interrupt initialization
* Input		: None
* Output		: None
* Return		: None
*******************************************************************************/
void USBDeviceIntCfg() {
	USB_INT_EN |= bUIE_SUSPEND;											// Enable device suspension interrupt
	USB_INT_EN |= bUIE_TRANSFER;										// Enable USB transfer completion interrupt
	USB_INT_EN |= bUIE_BUS_RST;											// Enable device mode USB bus reset interrupt
	USB_INT_EN |= bUIE_DEV_SOF;											// Open SOF interrupt
	USB_INT_FG |= 0x1F;													// Clear interrupt flag
	IE_USB = 1;															// Enable USB interrupt
	EA = 1;																// Allow MCU interrupt
}
/*******************************************************************************
* Function Name  : USBDeviceEndPointCfg()
* Description	: USB device mode endpoint configuration, simulation compatible HID device, in addition to endpoint 0 control transmission, also includes endpoint 2 batch upload and download
* Input		: None
* Output		: None
* Return		: None
*******************************************************************************/
void USBDeviceEndPointCfg() {
	// TODO: Is casting the right thing here? What about endianness?
	UEP2_DMA = (uint16_t) Ep2Buffer;										// Endpoint 2 OUT receiving data transmission address
	UEP3_DMA = (uint16_t) Ep3Buffer;
	UEP2_3_MOD = 0x48;														// Endpoint 2 single-buffered reception, endpoint 3 single-buffered transmission
	//UEP2_3_MOD = 0x49;													//Endpoint 3 single-buffered transmission, endpoint 2 double-buffered reception

	UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;								// Endpoint 2 automatically flips the synchronization flag, and OUT returns ACK
	UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; 								// Endpoint 3 sends back NAK

	//UEP4_DMA = (uint16_t) Ep4Buffer; //Ep4Buffer = Ep0Buffer + 64
	UEP1_DMA = (uint16_t) Ep1Buffer;										// Endpoint 1 IN send data transmission address
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;								// Endpoint 1 automatically flips the synchronization flag, and the IN transaction returns NAK
	UEP4_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK; 								// Endpoint 4 receives and returns ACK, which cannot be reversed automatically
	UEP4_1_MOD = 0x48;														// Endpoint 1 single-buffered transmission, endpoint 4 single-buffered reception

	UEP0_DMA = (uint16_t) Ep0Buffer;													// Endpoint 0 data transmission address
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;								// Manually flip, OUT transaction returns ACK, IN transaction returns NAK
}

__code uint8_t HexToAscTab[] = "0123456789ABCDEF";

void uuidcpy(__xdata uint8_t *dest, uint8_t index, uint8_t len) { /* Use UUID to generate USB Serial Number */
	uint8_t i;
	uint8_t p = 0; /* UUID format, decimal hexadecimal number */
	__code uint8_t *puuid;
	for(i = index; i < (index + len); i++) {
	if(i == 0)
		dest[p++] = 22; // 10 * 2 + 2
	else if(i == 1)
		dest[p++] = 0x03;
	else {
		if(i & 0x01) { // odd
			dest[p++] = 0x00;
		} else {
			puuid = (__code uint8_t *) (0x3ffa + (i - 2) / 4);
			if(i & 0x02)
				dest[p++] = HexToAscTab[(*puuid) >> 4];
			else
				dest[p++] = HexToAscTab[(*puuid) & 0x0f];
		}
	}
	}
}

#define INTF1_DTR	TIN1
#define INTF1_RTS	TIN0

#define INTF2_DTR	TIN3
#define INTF2_RTS	TIN2

//volatile __idata uint8_t DTR_State = 0;
volatile __idata uint8_t Modem_Count = 0;

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description	: CH559USB interrupt handler
*******************************************************************************/
void DeviceInterrupt(void) __interrupt (INT_NO_USB) {				// USB interrupt service routine, using register set 1
	uint16_t len;
	uint16_t divisor;
	if ((USB_INT_ST & MASK_UIS_TOKEN) == UIS_TOKEN_SOF) {
#ifdef SOF_NO_TIMER
	SOF_Count ++;
	if(Modem_Count)
		Modem_Count --;
	if(Modem_Count == 1) {
		if(soft_dtr == 0 && soft_rts == 1) {
			INTF1_RTS = 1;
			INTF1_DTR = 0;
		}
		if(soft_dtr == 1 && soft_rts == 0) {
			INTF1_RTS = 0;
			INTF1_DTR = 1;
		}
		if(soft_dtr == soft_rts) {
			INTF1_DTR = 1;
			INTF1_RTS = 0;
			INTF1_RTS = 1;
		}
	}
	if(SOF_Count % 16 == 0)
		PWM2 = 1;
#endif
	}
	if(UIF_TRANSFER) {														// USB transfer complete flag
	switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP)) {
		case UIS_TOKEN_IN | 1:												// endpoint 1# endpoint bulk upload
			UEP1_T_LEN = 0;
			UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;		// Default response NAK
			UpPoint1_Busy = 0;												// Clear busy flag
			break;
		case UIS_TOKEN_OUT | 2:												// endpoint 2# Endpoint batch download
			if ( U_TOG_OK ) {												// out-of-sync packets will be discarded
				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;	// NAK after receiving a packet of data, the main function is processed, the main function will modify the response mode
				USBOutLength = USB_RX_LEN;
				USBOutPtr = 0;
				USBReceived = 1;
			}
			break;
		case UIS_TOKEN_IN | 3:												// endpoint 3# endpoint bulk upload
			UEP3_T_LEN = 0;
			UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;		// Default response NAK
			UpPoint3_Busy = 0;												// Clear busy flag
			break;
		case UIS_TOKEN_OUT | 4:												// endpoint 4# Endpoint batch download
			if ( U_TOG_OK ) {												// out-of-sync packets will be discarded
				UEP4_CTRL ^= bUEP_R_TOG;	// Synchronization flag bit flip
				UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;	// NAK after receiving a packet of data, the main function is processed, the main function will modify the response mode
				USBOutLength_1 = USB_RX_LEN + 64;
				USBOutPtr_1 = 64;
				USBReceived_1 = 1;
			}
			break;
		case UIS_TOKEN_SETUP | 0:											// SETUP transaction
			len = USB_RX_LEN;
			if(len == (sizeof(USB_SETUP_REQ))) {
				SetupLen = ((uint16_t)UsbSetupBuf->wLengthH << 8) | (UsbSetupBuf->wLengthL);
				len = 0;													//The default is success and upload 0 length
				VendorControl = 0;
				SetupReq = UsbSetupBuf->bRequest;
				if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD ) { // Non-standard request
					// TODO: rewrite
					VendorControl = 1;
					if(UsbSetupBuf->bRequestType & USB_REQ_TYP_READ) {
						// Read
						switch( SetupReq ) {
							case 0x90: // READ EEPROM
								divisor = UsbSetupBuf->wIndexL & 0x3f;
								Ep0Buffer[0] = itdf_eeprom[divisor] & 0xff;
								Ep0Buffer[1] = itdf_eeprom[divisor] >> 8;
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
								len = 0xFF;	/* Command not supported */
								break;
						}
					} else {
						// Write
						switch( SetupReq ) {
							case 0x02:
							case 0x04:
							case 0x06:
							case 0x07:
							case 0x0b:
							case 0x92:
								len = 0;
								break;
							case 0x91: // WRITE EEPROM, FT_PROG action, jump directly to BL
								Require_DFU = 1;
								len = 0;
								break;
							case 0x00:
								if(UsbSetupBuf->wIndexL == 1)
									UpPoint1_Busy = 0;
								if(UsbSetupBuf->wIndexL == 2) {
									UpPoint3_Busy = 0;
									UEP4_CTRL &= ~(bUEP_R_TOG);
								}
								len = 0;
								break;
							case 0x09: // SET LATENCY TIMER
								if(UsbSetupBuf->wIndexL == 1)
									Latency_Timer = UsbSetupBuf->wValueL;
								else
									Latency_Timer1 = UsbSetupBuf->wValueL;
								len = 0;
								break;
							case 0x03:
								// divisor = wValue
								// U1SMOD = 1;
								//PCON |= SMOD; //Baud rate doubled
								//T2MOD |= bTMR_CLK; //Highest counting clock
								PCON |= SMOD;
								T2MOD |= bT1_CLK;

								divisor = UsbSetupBuf->wValueL |
									(UsbSetupBuf->wValueH << 8);
								divisor &= 0x3fff; //There is no way to take the integer part of the decimal, baudrate = 48M/16/divisor

								if(divisor == 0 || divisor == 1) { // baudrate> 3M
									if(UsbSetupBuf->wIndexL == 2)
										TH1 = 0xff; //I ca n't hold back 1M
								} else {
									uint16_t div_tmp = 0;
									div_tmp = 10 * divisor / 3; // 16M CPU clock
									if (div_tmp % 10 >= 5) 	divisor = div_tmp / 10 + 1;
									else 				divisor = div_tmp / 10;

									if(divisor > 256) {
										//TH1 = 0 - SBAUD_TH; //all use the default baud rate
										if(UsbSetupBuf->wIndexL == 2) {
											divisor /= 12;
											if(divisor > 256) { // Set the baud rate to be less than 488
												TH1 = (0 - SBAUD_TH)&0xff; // 9600bps
											} else {
												// PCON &= ~(SMOD);
												T2MOD &= ~(bT1_CLK); // low baud rate
												TH1 = 0 - divisor;
											}
										}
									} else {
										if(UsbSetupBuf->wIndexL == 2)
											TH1 = 0 - divisor;
#if 0
										else //intf2
											SBAUD1 = 0 - divisor;
#endif
									}
								}
								len = 0;
								break;
							case 0x01: // MODEM Control
#if HARD_ESP_CTRL
								if(UsbSetupBuf->wIndexL == 2) {
									if(UsbSetupBuf->wValueH & 0x01) {
										if(UsbSetupBuf->wValueL & 0x01) { // DTR
											soft_dtr = 1;
											// INTF1_DTR = 0;
										} else {
											soft_dtr = 0;
											// INTF1_DTR = 1;
										}
									}
									if(UsbSetupBuf->wValueH & 0x02) {
										if(UsbSetupBuf->wValueL & 0x02) { // RTS
											soft_rts = 1;
											// INTF1_RTS = 0;
										} else {
											soft_rts = 0;
											// INTF1_RTS = 1;
										}
									}
									Modem_Count = 20;
								}
#else
								if(Esp_Require_Reset == 3) {
									CAP1 = 0;
									Esp_Require_Reset = 4;
								}
#endif
								len = 0;
								break;
							default:
								len = 0xFF;		/* Command not supported */
								break;
						}
					}

				} else {														// standard request
					switch(SetupReq) {										// Request code
						case USB_GET_DESCRIPTOR:
							switch(UsbSetupBuf->wValueH) {
								case USB_DESCR_TYP_DEVICE:													// Device descriptor
									pDescr = DevDesc;										// Send the device descriptor to the buffer to be sent
									len = sizeof(DevDesc);
									break;
								case USB_DESCR_TYP_CONFIG:													// Configuration descriptor
									pDescr = CfgDesc;										// Send the device descriptor to the buffer to be sent
									len = sizeof(CfgDesc);
									break;
								case USB_DESCR_TYP_STRING:
									if(UsbSetupBuf->wValueL == 0) {
										pDescr = LangDes;
										len = sizeof(LangDes);
									} else if(UsbSetupBuf->wValueL == 1) {
										pDescr = Manuf_Des;
										len = sizeof(Manuf_Des);
									} else if(UsbSetupBuf->wValueL == 2) {
										pDescr = Prod_Des;
										len = sizeof(Prod_Des);
									} else if(UsbSetupBuf->wValueL == 4) {
										pDescr = Jtag_Des;
										len = sizeof(Jtag_Des);
									} else {
										pDescr = (__code uint8_t *)0xffff;
										len = 22; /* 10-bit ASCII serial number */
									}
									break;
								case USB_DESCR_TYP_QUALIF:
									// pDescr = QualifierDesc;
									// len = sizeof(QualifierDesc);
									len = 0xff;
									break;
								default:
									len = 0xff;											// unsupported command or error
									break;
							}

							if ( SetupLen > len ) {
								SetupLen = len;	// Limit the total length
							}
							if (len != 0xff) {
								len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;						// This transmission length

								if(pDescr == (__code uint8_t *) 0xffff) { /*If you take the serial number */
									uuidcpy(Ep0Buffer, 0, len);
								} else {
									memcpy(Ep0Buffer, pDescr, len);							// Load upload data
								}
								SetupLen -= len;
								pDescr_Index = len;
							}
							break;
						case USB_SET_ADDRESS:
							SetupLen = UsbSetupBuf->wValueL;							// temporarily store the USB device address
							break;
						case USB_GET_CONFIGURATION:
							Ep0Buffer[0] = UsbConfig;
							if ( SetupLen >= 1 ) {
								len = 1;
							}
							break;
						case USB_SET_CONFIGURATION:
							UsbConfig = UsbSetupBuf->wValueL;
							break;
						case USB_GET_INTERFACE:
							break;
						case USB_CLEAR_FEATURE:										// Clear Feature
							if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE ) {			/* Clear device */
								if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 ) {
									if( CfgDesc[ 7 ] & 0x20 ) {
										/* Wake up */
									} else {
										len = 0xFF;									/*The operation failed */
									}
								} else {
									len = 0xFF;										/*The operation failed */
								}
							} else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP ) { // Endpoint
								switch( UsbSetupBuf->wIndexL ) {
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
										len = 0xFF;										// Unsupported endpoint
										break;
								}
								UpPoint1_Busy = 0;
								UpPoint3_Busy = 0;
							} else {
								len = 0xFF;											//It is not that the endpoint does not support
							}
							break;
						case USB_SET_FEATURE:										/* Set Feature */
							if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE ) {			/* Set up the device */
								if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 ) {
									if( CfgDesc[ 7 ] & 0x20 ) {
										/* Sleep */
										dbg_printf( "suspend\n" );												// sleep state
										#ifdef DE_PRINTF
										while ( XBUS_AUX & bUART0_TX ) {};	// Wait for sending completion
										#endif
#if 0
										SAFE_MOD = 0x55;
										SAFE_MOD = 0xAA;
										WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;					//USB or RXD0/1 can be woken up when there is a signal
										PCON |= PD;																//sleep
										SAFE_MOD = 0x55;
										SAFE_MOD = 0xAA;
										WAKE_CTRL = 0x00;
#endif
									} else {
										len = 0xFF;									/*The operation failed */
									}
								} else {
									len = 0xFF;										/*The operation failed */
								}
							} else if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_ENDP ) {		/* Set the endpoint */
								if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 ) {
									switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL ) {
										case 0x83:
											UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* Set endpoint 3 IN STALL */
											break;
										case 0x03:
											UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* Set endpoint 3 OUT Stall */
											break;
										case 0x82:
											UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* Set endpoint 2 IN STALL */
											break;
										case 0x02:
											UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* Set endpoint 2 OUT Stall */
											break;
										case 0x81:
											UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* Set endpoint 1 IN STALL */
											break;
										case 0x01:
											UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* Set endpoint 1 OUT Stall */
										default:
											len = 0xFF;								/*The operation failed */
											break;
									}
								} else {
									len = 0xFF;									/*The operation failed */
								}
							} else {
								len = 0xFF;										/*The operation failed */
							}
							break;
						case USB_GET_STATUS:
							Ep0Buffer[0] = 0x00;
							Ep0Buffer[1] = 0x00;
							if ( SetupLen >= 2 ) {
								len = 2;
							} else {
								len = SetupLen;
							}
							break;
						default:
							len = 0xff;												//The operation failed
							break;
					}
				}
			} else {
				len = 0xff;														// Package length error
			}
			if(len == 0xff) {
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;// STALL
			} else if(len <= DEFAULT_ENDP0_SIZE) {												// Upload data or return 0 length package in status stage
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default data packet is DATA1, and the response ACK is returned
			} else {
				UEP0_T_LEN = 0;  // Although it has not yet reached the status stage, it is preset to upload a 0-length data packet in advance to prevent the host from entering the status stage in advance
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default data packet is DATA1, and the response ACK is returned
			}
			break;
		case UIS_TOKEN_IN | 0:													// endpoint0 IN
			switch(SetupReq) {
				case USB_GET_DESCRIPTOR:
					len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;			// This transmission length
					if(pDescr == (__code uint8_t *)0xffff) {
						uuidcpy(Ep0Buffer, pDescr_Index, len);
					} else {
						memcpy( Ep0Buffer, pDescr + pDescr_Index, len );								// Load upload data
					}
					SetupLen -= len;
					pDescr_Index += len;
					UEP0_T_LEN = len;
					UEP0_CTRL ^= bUEP_T_TOG;											// Synchronization flag bit flip
					break;
				case USB_SET_ADDRESS:
					if(VendorControl == 0) {
						USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
						UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
					}
					break;
				default:
					UEP0_T_LEN = 0;													//The status phase is completed interrupted or a 0-length data packet is forced to upload to end the control transmission
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
			}
			break;
		case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
			if(SetupReq == 0x22) { // Set serial port properties

			} else {
				UEP0_T_LEN = 0;
				UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;  //In the status phase, respond NAK to IN
			}
			break;

		default:
			break;
	}
	UIF_TRANSFER = 0;															// write 0 to clear interrupt
	}
	if(UIF_BUS_RST) {															// device mode USB bus reset interrupt
	dbg_printf( "reset\n" );													// sleep state
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
	USB_DEV_AD = 0x00;
	UIF_SUSPEND = 0;
	UIF_TRANSFER = 0;
	UIF_BUS_RST = 0;															// Clear the interrupt flag
	UsbConfig = 0;		// Clear configuration value
	UpPoint1_Busy = 0;
	UpPoint3_Busy = 0;

	USBOutLength = 0;
	USBOutPtr = 0;
	USBReceived = 0;

	USBOutLength_1 = 0;
	USBOutPtr_1 = 0;
	USBReceived_1 = 0;

	Mpsse_ShortLen = 0;
	Mpsse_LongLen = 0;

	Mpsse_Status = 0;
	UpPoint1_Ptr = 2;
	UpPoint3_Ptr = 2;

	Serial_Done = 0;
	USB_Require_Data = 0;
	}
	if (UIF_SUSPEND) {															// USB bus suspend/wake up completed
	UIF_SUSPEND = 0;
	if ( USB_MIS_ST & bUMS_SUSPEND ) {										// suspend
#ifdef USB_SLEEP
		dbg_printf( "suspend\n" );															// sleep state
		while ( XBUS_AUX & bUART0_TX ) {
			;	// Wait for sending completion
		}
		SAFE_MOD = 0x55;
		SAFE_MOD = 0xAA;
		WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;					// USB or RXD0/1 can be woken up when there is a signal
		PCON |= PD;																// sleep
		SAFE_MOD = 0x55;
		SAFE_MOD = 0xAA;
		WAKE_CTRL = 0x00;
#endif
	}
	} else {																		// Unexpected interruption, impossible situation
	USB_INT_FG = 0xFF;															// Clear interrupt flag

	}
}


// #define FAST_COPY_2
// #define FAST_COPY_1

void CLKO_Enable(void) { // Turn on T2 output
	ET2 = 0;
	T2CON = 0;
	T2MOD = 0;
	T2MOD |= bTMR_CLK | bT2_CLK | T2OE;
	RCAP2H = 0xff;
	RCAP2L = 0xfe;
	TH2 = 0xff;
	TL2 = 0xfe;
	TR2 = 1;
	P1_MOD_OC &= ~(0x01); // P1.0 push-pull output
	P1_DIR_PU |= 0x01;
}

#define TMS T2EX
#define TDI MOSI
#define TDO MISO
#define TCK SCK
#define TCK_CONT SCS

void JTAG_IO_Config(void) {
	P1_DIR_PU |= ((1 << 1) | (1 << 5) | (1 << 7));
	P1_DIR_PU &= ~((1 << 6) | (1 << 4));
	P1_MOD_OC &= ~((1 << 1) | (1 << 5) | (1 << 7) | (1 << 6) | (1 << 4));

	TMS = 0;
	TDI = 0;
	TDO = 0;
	TCK = 0;
	TCK_CONT = 0;
	/* P1.1 TMS, P1.5 TDI(MOSI), P1.7 TCK PP */
	/* P1.6 TDO(MISO) INPUT */
	/* P1.4 INPUT */
}

void Run_Test_Start() {
	/* P1.7 INPUT, P1.4 PP */
	PIN_FUNC |= bT2_PIN_X;
	P1_DIR_PU &= ~((1 << 7));
	P1_MOD_OC &= ~((1 << 7));
	// TCK = 1;

	RCAP2L = 0xfd;

	P1_DIR_PU |= ((1 << 4));
	P1_MOD_OC &= ~((1 << 4));
}

void Run_Test_Stop() {
	P1_DIR_PU &= ~((1 << 4));
	P1_MOD_OC &= ~((1 << 4)); // P1.4 INPUT

	RCAP2L = 0xfe;
	PIN_FUNC &= ~bT2_PIN_X;

	P1_DIR_PU |= ((1 << 7));
	P1_MOD_OC &= ~((1 << 7)); // P1.7 OUTPUT
}

#define MPSSE_IDLE		0
#define MPSSE_RCV_LENGTH_L	1
#define MPSSE_RCV_LENGTH_H	2
#define MPSSE_TRANSMIT_BYTE 3
#define MPSSE_RCV_LENGTH	4
#define MPSSE_TRANSMIT_BIT	5
#define MPSSE_ERROR		6
#define MPSSE_TRANSMIT_BIT_MSB 7
#define MPSSE_TMS_OUT	8
#define MPSSE_NO_OP_1	9
#define MPSSE_NO_OP_2	10
#define MPSSE_TRANSMIT_BYTE_MSB	11
#define MPSSE_RUN_TEST	12

// Main function
main() {
	uint8_t i;
	uint8_t Purge_Buffer = 0;
	uint8_t data, rcvdata;
	uint8_t instr = 0;
	volatile uint16_t Uart_Timeout = 0;
	volatile uint16_t Uart_Timeout1 = 0;
	uint16_t Esp_Stage = 0;
	// int8_t size;


	Xtal_Enable();	// Start the oscillator
	CfgFsys( );														// CH552 clock selection configuration
	mDelaymS(5);														// Modify the main frequency and wait for the internal clock to stabilize, must be added
	CLKO_Enable();
	JTAG_IO_Config();
	SerialPort_Config();

	PWM2 = 1;

#if MPSSE_HWSPI
	SPI_Init();
#endif

	dbg_printf("start ...\n");
	USBDeviceCfg();
	USBDeviceEndPointCfg();											// Endpoint configuration
	USBDeviceIntCfg();												// Interrupt initialization
	UEP0_T_LEN = 0;
	UEP1_T_LEN = 0;													//The pre-used transmission length must be cleared
	UEP2_T_LEN = 0;													//The pre-used transmission length must be cleared

	/* Pre-fill Modem Status */
	Ep1Buffer[0] = 0x01;
	Ep1Buffer[1] = 0x60;
	Ep3Buffer[0] = 0x01;
	Ep3Buffer[1] = 0x60;
	UpPoint1_Ptr = 2;
	UpPoint3_Ptr = 2;
	XBUS_AUX = 0;
#ifndef SOF_NO_TIMER
	init_timer();                                                              // Every 1ms SOF_Count increases by 1
	SOF_Count = 0;
#endif
	T1 = 0;
	while(1) {
	if(UsbConfig) {
		if(USBReceived == 1) {
			// Received a packet
#if MPSSE_DEBUG
			if(UpPoint1_Ptr < 64 && UpPoint1_Busy == 0 && UpPoint3_Busy == 0 && UpPoint3_Ptr < 64) /* Can send */
#else
			if(UpPoint1_Ptr < 64 && UpPoint1_Busy == 0)
#endif
			{
				PWM2 = !PWM2;
				switch(Mpsse_Status) {
					case MPSSE_IDLE:
						instr = Ep2Buffer[USBOutPtr];
#if MPSSE_DEBUG
						Ep3Buffer[UpPoint3_Ptr++] = instr;
#endif
						switch(instr) {
							case 0x80:
							case 0x82: /* Fake Bit bang mode */
								Mpsse_Status = MPSSE_NO_OP_1;
								USBOutPtr++;
								break;
							case 0x81:
							case 0x83: /* False state */
								Ep1Buffer[UpPoint1_Ptr++] = Ep2Buffer[USBOutPtr] - 0x80;
								USBOutPtr++;
								break;
							case 0x84:
							case 0x85: /* Loopback */
								USBOutPtr++;
								break;
							case 0x86: /* Speed adjustment, temporarily not supported */
								Mpsse_Status = MPSSE_NO_OP_1;
								USBOutPtr++;
								break;
							case 0x87: /* Refresh the buffer immediately */
								Purge_Buffer = 1;
								USBOutPtr++;
								break;
							case 0x19:
							case 0x39:
							case 0x11:
							case 0x31:
								SPI_ON();
								Mpsse_Status = MPSSE_RCV_LENGTH_L;
								USBOutPtr++;
								break;
							case 0x6b:
							case 0x4b:
							case 0x3b:
							case 0x1b:
							case 0x13:
								SPI_OFF();
								Mpsse_Status = MPSSE_RCV_LENGTH;
								USBOutPtr++;
								break;
							default:	/* Unsupported command */
								Ep1Buffer[UpPoint1_Ptr++] = 0xfa;
								Mpsse_Status = MPSSE_ERROR;
								break;
						}
						break;
					case MPSSE_RCV_LENGTH_L: /* Receive length */
						Mpsse_LongLen = Ep2Buffer[USBOutPtr];
						Mpsse_Status ++;
						USBOutPtr++;
						break;
					case MPSSE_RCV_LENGTH_H:
						Mpsse_LongLen |= (Ep2Buffer[USBOutPtr] << 8) & 0xff00;
						USBOutPtr++;
#if GOWIN_INT_FLASH_QUIRK
						if((Mpsse_LongLen == 25000 || Mpsse_LongLen == 750 || Mpsse_LongLen == 2968) && (instr & (1 << 5)) == 0) {
							SPI_OFF();
							Run_Test_Start();
							Mpsse_Status = MPSSE_RUN_TEST;
						} else if(instr == 0x11 || instr == 0x31)
#else
						if (instr == 0x11 || instr == 0x31)
#endif
						{
							Mpsse_Status = MPSSE_TRANSMIT_BYTE_MSB;
							SPI_MSBFIRST();
						} else {
							Mpsse_Status ++;
							SPI_LSBFIRST();
						}
						break;
					case MPSSE_TRANSMIT_BYTE:
						data = Ep2Buffer[USBOutPtr];
#if MPSSE_HWSPI
						SPI0_DATA = data;
						while(S0_FREE == 0);
						rcvdata = SPI0_DATA;
#else
						rcvdata = 0;
						for(i = 0; i < 8; i++) {
							SCK = 0;
							MOSI = (data & 0x01);
							data >>= 1;
							rcvdata >>= 1;
							__asm nop __endasm;
							__asm nop __endasm;
							SCK = 1;
							if(MISO == 1)
								rcvdata |= 0x80;
							__asm nop __endasm;
							__asm nop __endasm;
						}
						SCK = 0;
#endif
						if(instr == 0x39)
							Ep1Buffer[UpPoint1_Ptr++] = rcvdata;
						USBOutPtr++;
						if(Mpsse_LongLen == 0)
							Mpsse_Status = MPSSE_IDLE;
						Mpsse_LongLen --;
						break;
					case MPSSE_TRANSMIT_BYTE_MSB:
						data = Ep2Buffer[USBOutPtr];
#if MPSSE_HWSPI
						SPI0_DATA = data;
						while(S0_FREE == 0);
						rcvdata = SPI0_DATA;
#else
						rcvdata = 0;
						for(i = 0; i < 8; i++) {
							SCK = 0;
							MOSI = (data & 0x80);
							data <<= 1;
							rcvdata <<= 1;
							__asm nop __endasm;
							__asm nop __endasm;
							SCK = 1;
							if(MISO == 1)
								rcvdata |= 0x01;
							__asm nop __endasm;
							__asm nop __endasm;
						}
						SCK = 0;
#endif
						if(instr == 0x31)
							Ep1Buffer[UpPoint1_Ptr++] = rcvdata;
						USBOutPtr++;
						if(Mpsse_LongLen == 0)
							Mpsse_Status = MPSSE_IDLE;
						Mpsse_LongLen --;
						break;
					case MPSSE_RCV_LENGTH:
						Mpsse_ShortLen = Ep2Buffer[USBOutPtr];
						if(instr == 0x6b || instr == 0x4b)
							Mpsse_Status = MPSSE_TMS_OUT;
						else if(instr == 0x13)
							Mpsse_Status = MPSSE_TRANSMIT_BIT_MSB;
						else
							Mpsse_Status++;
						USBOutPtr++;
						break;
					case MPSSE_TRANSMIT_BIT:
						data = Ep2Buffer[USBOutPtr];
						rcvdata = 0;
						do {
							SCK = 0;
							MOSI = (data & 0x01);
							data >>= 1;
							rcvdata >>= 1;
							__asm nop __endasm;
							__asm nop __endasm;
							SCK = 1;
							if(MISO)
								rcvdata |= 0x80;// (1 << (Mpsse_ShortLen));
							__asm nop __endasm;
							__asm nop __endasm;
						} while((Mpsse_ShortLen--) > 0);
						SCK = 0;
						if(instr == 0x3b)
							Ep1Buffer[UpPoint1_Ptr++] = rcvdata;
						Mpsse_Status = MPSSE_IDLE;
						USBOutPtr++;
						break;
					case MPSSE_TRANSMIT_BIT_MSB:
						data = Ep2Buffer[USBOutPtr];
						rcvdata = 0;
						do {
							SCK = 0;
							MOSI = (data & 0x80);
							data <<= 1;
							__asm nop __endasm;
							__asm nop __endasm;
							SCK = 1;
							__asm nop __endasm;
							__asm nop __endasm;
						} while((Mpsse_ShortLen--) > 0);
						SCK = 0;

						Mpsse_Status = MPSSE_IDLE;
						USBOutPtr++;

						break;
					case MPSSE_ERROR:
						Ep1Buffer[UpPoint1_Ptr++] = Ep2Buffer[USBOutPtr];
						Mpsse_Status = MPSSE_IDLE;
						USBOutPtr++;
						break;
					case MPSSE_TMS_OUT:
						data = Ep2Buffer[USBOutPtr];
						if(data & 0x80)
							TDI = 1;
						else
							TDI = 0;
						rcvdata = 0;
						do {
							TCK = 0;
							TMS = (data & 0x01);
							data >>= 1;
							rcvdata >>= 1;
							__asm nop __endasm;
							__asm nop __endasm;
							SCK = 1;
							if(TDO)
								rcvdata |= 0x80;// (1 << (Mpsse_ShortLen));
							__asm nop __endasm;
							__asm nop __endasm;
						} while((Mpsse_ShortLen--) > 0);
						TCK = 0;
						if(instr == 0x6b)
							Ep1Buffer[UpPoint1_Ptr++] = rcvdata;
						Mpsse_Status = MPSSE_IDLE;
						USBOutPtr++;
						break;
					case MPSSE_NO_OP_1:
						Mpsse_Status ++;
						USBOutPtr++;
						break;
					case MPSSE_NO_OP_2:
						Mpsse_Status = MPSSE_IDLE;
						USBOutPtr++;
						break;
#if GOWIN_INT_FLASH_QUIRK
					case MPSSE_RUN_TEST:
						if(Mpsse_LongLen == 0) {
							Mpsse_Status = MPSSE_IDLE;
							Run_Test_Stop();
						}

						USBOutPtr++;
						Mpsse_LongLen --;
						break;
#endif
					default:
						Mpsse_Status = MPSSE_IDLE;
						break;
				}


				if(USBOutPtr >= USBOutLength) {
					// Received
					USBReceived = 0;
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
					// Open reception
				}
			}
		}

		if(UpPoint1_Busy == 0) {
			if(UpPoint1_Ptr == 64) {
				UpPoint1_Busy = 1;
				UEP1_T_LEN = 64;
				UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
				UpPoint1_Ptr = 2;
			} else if((uint16_t) (SOF_Count - Uart_Timeout) >= Latency_Timer || Purge_Buffer == 1) { // Timeout
				Uart_Timeout = SOF_Count;

				UpPoint1_Busy = 1;
				UEP1_T_LEN = UpPoint1_Ptr;
				UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;		// Response to ACK
				UpPoint1_Ptr = 2;
				Purge_Buffer = 0;
			}
		}

		if(UpPoint3_Busy == 0) {
			int8_t size = WritePtr - ReadPtr;
			if(size < 0) size = size + sizeof(RingBuf);// Find the remainder

			if(size >= 62) {
				for(i = 0; i < 62; i++) {
					Ep3Buffer[2 + i] = RingBuf[ReadPtr++];
					ReadPtr %= sizeof(RingBuf);
				}
				UpPoint3_Busy = 1;
				UEP3_T_LEN = 64;
				UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
				UpPoint3_Ptr = 2;

			} else if((uint16_t) (SOF_Count - Uart_Timeout1) >= Latency_Timer1) { // Timeout
				Uart_Timeout1 = SOF_Count;
				if(size > 62) size = 62;
				for(i = 0; i < (uint8_t)size; i++) {
					Ep3Buffer[2 + i] = RingBuf[ReadPtr++];
					ReadPtr %= sizeof(RingBuf);
				}
				UpPoint3_Busy = 1;
				// UEP3_T_LEN = UpPoint3_Ptr;
				UEP3_T_LEN = 2 + size;
				UpPoint3_Ptr = 2;
				UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;		// Response to ACK
			}
		}

		if(USBReceived_1) { // IDLE status
			if(Serial_Done == 0) { // Serial port IDLE
				Serial_Done = 2; // Serial port sending
				TI = 1;
			}
			if(UEP4_CTRL & MASK_UEP_R_RES != UEP_R_RES_ACK)
				UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
			USBReceived_1 = 0;
		}

		if(Serial_Done == 1) {
			Serial_Done = 2; // Serial port sending
			TI = 1;

			Serial_Done = 0;
			// if(UEP4_CTRL & MASK_UEP_R_RES != UEP_R_RES_ACK)
			UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
		}

		if(Require_DFU) {
			Require_DFU = 0;
			Jump_to_BL();
		}
	}
	}
}

/*******************************************************************************
* Function Name  : mTimer0Interrupt()
* Description    : CH554 timer counter 0 timer counter interrupt processing function
*******************************************************************************/
void mTimer0Interrupt(void) __interrupt (INT_NO_TMR0) {                        //timer0 interrupt service routine
	mTimer_x_SetData(0, 1000);                                                 //For non-auto reload mode, TH0 and TL0 need to be re-assigned, 1MHz/1000=1000Hz, 1ms
	SOF_Count ++;
	if(Modem_Count)
		Modem_Count --;
	if(Modem_Count == 1) {
		if(soft_dtr == 0 && soft_rts == 1) {
			INTF1_RTS = 1;
			INTF1_DTR = 0;
		}
		if(soft_dtr == 1 && soft_rts == 0) {
			INTF1_RTS = 0;
			INTF1_DTR = 1;
		}
		if(soft_dtr == soft_rts) {
			INTF1_DTR = 1;
			INTF1_RTS = 0;
			INTF1_RTS = 1;
		}
	}
	if(SOF_Count % 16 == 0)
		PWM2 = 1;
}

