#include "hal.h"

void Xtal_Enable(void) { // Enable external clock using xtal crystal
	USB_INT_EN = 0;
	USB_CTRL = 0x06;

	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
	CLOCK_CFG |= bOSC_EN_XT;                       // Enable external 24M crystal oscillator
	SAFE_MOD = 0x00;
	mDelaymS(50);

// SAFE_MOD = 0x55;
// SAFE_MOD = 0xAA;
//	CLOCK_CFG &= ~bOSC_EN_INT;                        //Turn off internal RC
// SAFE_MOD = 0x00;
	mDelaymS(250);
}

void Jump_to_BL() {
	ES = 0;
	PS = 0;

	P1_DIR_PU = 0;
	P1_MOD_OC = 0;
	P1 = 0xff;

	USB_INT_EN = 0;
	USB_CTRL = 0x06;
	// UDEV_CTRL = 0x80;

	mDelaymS(100);

	EA = 0;

	while(1) {
		__asm
		LJMP 0x3800
		__endasm;
	}
}


//--------------------- Timer ---------------------------------
/*******************************************************************************
* Function Name  : mTimer_x_ModInit(uint8_t x ,uint8_t mode)
* Description    : CH554 timer counter x mode setting
* Input          : uint8_t mode,Timer mode selection
                   0: Mode 0, 13-bit timer, the upper 3 bits of TLn are invalid
                   1: Mode 1, 16-bit timer
                   2: Mode 2, 8-bit automatic reload timer
                   3: Mode 3, two 8-bit timers Timer0
     (FIXME)       3: Mode 3, Timer1 stops
* Output         : None
* Return         : SUCCESS  SUCCESS
                   FAIL     FAIL
*******************************************************************************/
uint8_t mTimer_x_ModInit(uint8_t x, uint8_t mode) {
	if(x == 0) {TMOD = TMOD & 0xf0 | mode;}
	else if(x == 1) {TMOD = TMOD & 0x0f | (mode<<4);}
	else if(x == 2) {RCLK = 0; TCLK = 0; CP_RL2 = 0;}                            //16-bit automatic reload timer
	else return FAIL;
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : mTimer_x_SetData(uint8_t x,uint16_t dat)
* Description    : CH554Timer0 TH0 and TL0 assignment
* Input          : uint16_t dat;timer assignment
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer_x_SetData(uint8_t x, uint16_t dat) {
	uint16_t tmp;
	tmp = 65536 - dat;
	if(x == 0) {TL0 = tmp & 0xff; TH0 = (tmp>>8) & 0xff;}
	else if(x == 1) {TL1 = tmp & 0xff; TH1 = (tmp>>8) & 0xff;}
	else if(x == 2) {
		RCAP2L = TL2 = tmp & 0xff;                                               //16-bit automatic reload timer
		RCAP2H = TH2 = (tmp>>8) & 0xff;
	}
}

/*******************************************************************************
* Function Name  : CAP2Init(uint8_t mode)
* Description    : CH554 timer counter 2 T2EX pin capture function initialization
                   uint8_t mode, edge capture mode selection
                   0: T2ex from falling edge to next falling edge
                   1: T2ex Between any edges
                   3: T2ex from rising edge to next rising edge
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP2Init(uint8_t mode) {
	RCLK = 0;
	TCLK = 0;
	C_T2  = 0;
	EXEN2 = 1;
	CP_RL2 = 1;                                                                //Start the capture function of T2ex
	T2MOD |= mode << 2;                                                        //Edge capture mode selection
}

/*******************************************************************************
* Function Name  : CAP1Init(uint8_t mode)
* Description    : CH554timer counter 2 T2 pin capture function initialization T2
                   uint8_t mode, edge capture mode selection
                   0: T2ex from falling edge to next falling edge
                   1: T2ex between any edges
                   3: T2ex from rising edge to next rising edge
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP1Init(uint8_t mode) {
	RCLK = 0;
	TCLK = 0;
	CP_RL2 = 1;
	C_T2 = 0;
	T2MOD = T2MOD & ~T2OE | (mode << 2) | bT2_CAP1_EN;                         //Enable T2 pin capture function, edge capture mode selection
}

void init_timer() {
	mTimer0Clk12DivFsys();		//T0 timer clock setting, 12MHz/12=1MHz
	mTimer_x_ModInit(0, 1);		//T0 timer mode setting
	mTimer_x_SetData(0, 1000);	//T0 timer assignment, 1MHz/1000=1000Hz, 1ms
	mTimer0RunCTL(1);			//T0 timer start
	ET0 = 1;					//T0 timer interrupt is on
	EA = 1;
}
