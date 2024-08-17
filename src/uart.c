
/*******************************************************************************
* Function Name  : Uart0_ISR()
* Description	: Serial port receive interrupt function to achieve circular buffer reception
*******************************************************************************/
#include "hal.h"

// Ring Buf

volatile __idata uint8_t WritePtr = 0;
volatile __idata uint8_t ReadPtr = 0;

#ifndef HARD_ESP_CTRL
	volatile __idata uint8_t Esp_Boot_Chk = 0;
	volatile __idata uint8_t Esp_Require_Reset = 0;
#endif

#ifndef HARD_ESP_CTRL
__code uint8_t ESP_Boot_Sequence[] = {
	0x07, 0x07, 0x12, 0x20,
	0x55, 0x55, 0x55, 0x55,
	0x55, 0x55, 0x55, 0x55,
	0x55, 0x55, 0x55, 0x55,
	0x55, 0x55, 0x55, 0x55
};
#endif

#ifndef FAST_RECEIVE /* Old disrepair code, don't maintain it anymore */
void Uart0_ISR(void) __interrupt (INT_NO_UART0) __using 1 {
	if(RI) { // Receive data
	if((WritePtr + 1) % sizeof(RingBuf) != ReadPtr) {
		// Ring buffer write
		RingBuf[WritePtr++] = SBUF;
		WritePtr %= sizeof(RingBuf);
	}
	RI = 0;
	}
	if (TI) {
	if(USBOutPtr_1 >= USBOutLength_1) {
		UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
		TI = 0;
	} else {
		uint8_t ch = Ep2Buffer[USBOutPtr_1];
		SBUF = ch;
		TI = 0;
#ifndef HARD_ESP_CTRL
		if(ESP_Boot_Sequence[Esp_Boot_Chk] == ch)
			Esp_Boot_Chk ++;
		else
			Esp_Boot_Chk = 0;

		if(Esp_Boot_Chk >= (sizeof(ESP_Boot_Sequence) - 1)) {
			if(Esp_Require_Reset == 0)
				Esp_Require_Reset = 1;
			Esp_Boot_Chk = 0;
		}
#endif
		USBOutPtr_1++;
	}
	}

}
#else

// Assemble receive data, select register group 1, DPTR1 1.5M~150kHz~160 cycles
void Uart0_ISR(void) __interrupt (INT_NO_UART0) __using 1 __naked {
	__asm
	push psw ; 2
	push a
	push dph
	push dpl

	ReadFromSerial:
	jnb _RI, SendToSerial ; 7

	mov a, _WritePtr ; 2
	mov dpl, _ReadPtr

	inc a ; 1
	anl dpl, #0x7f
	anl a, #0x7f ; 2

	xrl a, dpl
	jz SendToSerial

	mov dph, #(_RingBuf >> 8) ; 3
	mov dpl, _WritePtr ; 3
	mov a, _SBUF ; 2
	movx @dptr, a ; 1

	inc _WritePtr ; 1
	anl _WritePtr, #0x7f ; 2

	SendToSerial:
	clr _RI ; 2

	jnb _TI, ISR_End

	clr c
	mov a, _USBOutPtr_1
	subb a, _USBOutLength_1
	jc SerialTx

	UsbEpAck:
	mov _Serial_Done, #1
	sjmp Tx_End
	SerialTx:
	mov dph, #(_Ep4Buffer >> 8)
	mov dpl, _USBOutPtr_1
	movx a, @dptr
	mov _SBUF, a
	inc _USBOutPtr_1

	Tx_End:
	clr _TI

	ISR_End:

	pop dpl
	pop dph
	pop a
	pop psw
	reti
	__endasm;
}
#endif

void SerialPort_Config() {
	volatile uint32_t x;
	volatile uint8_t x2;

	/* P3.0 input */
	P3_DIR_PU &= ~((1 << 0));
	P3_MOD_OC &= ~((1 << 0));

	/* P3.1 output */
	P3_DIR_PU &= ((1 << 1));
	P3_MOD_OC |= ~((1 << 1));

	SM0 = 0;
	SM1 = 1;
	SM2 = 0;																// Serial port 0 use mode 1
	// Use Timer1 as baud rate generator
	RCLK = 0;																// UART0 receive clock
	TCLK = 0;																// UART0 sending clock
	PCON |= SMOD;
	x = 10 * FREQ_SYS / SBAUD_SET / 16;									// Calculation of baud rate: 16M/16/baud rate
	// If you change the main frequency, pay attention to the value of x not to overflow
	x2 = x % 10;
	x /= 10;
	if ( x2 >= 5 ) x ++;													// rounding

	TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1;			// 0X20, Timer1 as an 8-bit automatic reload timer
	T2MOD = T2MOD | bTMR_CLK | bT1_CLK;									// Timer1 clock selection
	TH1 = 0 - x;															// 12MHz crystal oscillator, buad/12 is the actual baud rate that needs to be set
	TR1 = 1;																// Start timer 1
	TI = 0;
	REN = 1;																// Serial port 0 receiving enable
	ES = 1; // Open serial port interrupt
	PS = 1; // Interrupt priority is the highest
}
