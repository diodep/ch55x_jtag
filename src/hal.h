#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <ch554.h>
#include <ch554_usb.h>
#include <debug.h>

#include "config.h"

void Xtal_Enable(void);	// Enable external clock using xtal crystal
void Jump_to_BL();

//Define the function return value
#ifndef  SUCCESS
	#define  SUCCESS  0
#endif
#ifndef  FAIL
	#define  FAIL    0xFF
#endif


// -- timer --

//Define the start of the timer
#ifndef  START
	#define  START  1
#endif
#ifndef  STOP
	#define  STOP    0
#endif

//CH554 Timer0 clock selection
//bTMR_CLK affects Timer0&1&2 at the same time, please pay attention when using it (except for the standard clock used for timing)
#define mTimer0Clk12DivFsys( ) (T2MOD &= ~bT0_CLK)                          //Timer, clock=Fsys/12 T0 standard clock
#define mTimer0ClkFsys( )      (T2MOD |= bTMR_CLK | bT0_CLK)                //Timer, clock=Fsys
#define mTimer0Clk4DivFsys( )  (T2MOD &= ~bTMR_CLK;T2MOD |=  bT0_CLK)       //Timer, clock=Fsys/4
#define mTimer0CountClk( )     (TMOD |= bT0_CT)                             //Counter, the falling edge of T0 pin is valid

//CH554 Timer0 start (SS=1)/end (SS=0)
#define mTimer0RunCTL( SS )    (TR0 = SS ? START : STOP)


#define mTimer1Clk12DivFsys( ) (T2MOD &= ~bT1_CLK)                          //Timer, clock=Fsys/12 T1 standard clock
#define mTimer1ClkFsys( )      (T2MOD |= bTMR_CLK | bT1_CLK)                //Timer, clock=Fsys
#define mTimer1Clk4DivFsys( )  (T2MOD &= ~bTMR_CLK;T2MOD |=  bT1_CLK)       //Timer, clock=Fsys/4
#define mTimer1CountClk( )     (TMOD |= bT1_CT)                             //Counter, the falling edge of T0 pin is valid

//CH554 Timer1 start (SS=1)/end (SS=0)
#define mTimer1RunCTL( SS )    (TR1 = SS ? START : STOP)


#define mTimer2Clk12DivFsys( ) {T2MOD &= ~ bT2_CLK;C_T2 = 0;}      //Timer, clock=Fsys/12 T2 standard clock
#define mTimer2ClkFsys( )      {T2MOD |= (bTMR_CLK | bT2_CLK);C_T2=0;}         //Timer, clock=Fsys
#define mTimer2Clk4DivFsys( )  {T2MOD &= ~bTMR_CLK;T2MOD |=  bT2_CLK;C_T2 = 0;}//Timer, clock=Fsys/4
#define mTimer2CountClk( )     {C_T2 = 1;}                                     //Counter, the falling edge of T2 pin is valid

//CH554 Timer2 start (SS=1)/end (SS=0)
#define mTimer2RunCTL( SS )    {TR2 = SS ? START : STOP;}
#define mTimer2OutCTL( )       (T2MOD |= T2OE)                               //T2 output frequency TF2/2
#define CAP1Alter( )           (PIN_FUNC |= bT2_PIN_X;)                      //CAP1 is mapped from P10 to P14
#define CAP2Alter( )           (PIN_FUNC |= bT2EX_PIN_X;)                    //CAP2 is mapped by P11 RST

/*******************************************************************************
* Function Name  : mTimer_x_ModInit(uint8_t x ,uint8_t mode)
* Description    : CH554 timer counter x mode setting
* Input          : uint8_t mode,Timer mode selection
                   0: Mode 0, 13-bit timer, the upper 3 bits of TLn are invalid
                   1: Mode 1, 16-bit timer
                   2: Mode 2, 8-bit automatic reload timer
                   3: Mode 3, two 8-bit timers Timer0
       (FIXME)     3: Mode 3, Timer1 stops
                   uint8_t x timer 0 1 2
* Output         : None
* Return         : SUCCESS  SUCCESS
                   FAIL     FAIL
*******************************************************************************/
uint8_t mTimer_x_ModInit(uint8_t x, uint8_t mode);

/*******************************************************************************
* Function Name  : mTimer_x_SetData(uint8_t x,uint16_t dat)
* Description    : CH554Timer
* Input          : uint16_t dat;timer assignment
                   uint8_t x timer 0 1 2
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer_x_SetData(uint8_t x, uint16_t dat);

/*******************************************************************************
* Function Name  : CAP2Init(uint8_t mode)
* Description    : CH554 timer counter 2 T2EX pin capture function initialization
                   uint8_t mode, edge capture mode selection
                   0: T2ex from falling edge to next falling edge
                   1: T2ex between any edges
                   3: T2ex from rising edge to next rising edge
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP2Init(uint8_t mode);

/*******************************************************************************
* Function Name  : CAP1Init(uint8_t mode)
* Description    : CH554 timer counter 2 T2 pin capture function initialization T2
                   uint8_t mode, edge capture mode selection
                   0: T2ex from falling edge to next falling edge
                   1: T2ex between any edges
                   3: T2ex from rising edge to next rising edge
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAP1Init(uint8_t mode);
void init_timer();


// -- uart --
void SerialPort_Config();
extern volatile __idata uint8_t Esp_Require_Reset;
extern volatile __idata uint8_t WritePtr;	//FIXME: hideme
extern volatile __idata uint8_t ReadPtr;    //FIXME: hideme


// -- spi --
void SPI_Init();

#if MPSSE_HWSPI
	#define SPI_LSBFIRST() SPI0_SETUP |= bS0_BIT_ORDER
	#define SPI_MSBFIRST() SPI0_SETUP &= ~bS0_BIT_ORDER
	#define SPI_ON() SPI0_CTRL = bS0_MISO_OE | bS0_MOSI_OE | bS0_SCK_OE;
	#define SPI_OFF() SPI0_CTRL = 0;
#else
	#define SPI_LSBFIRST()
	#define SPI_MSBFIRST()
	#define SPI_ON()
	#define SPI_OFF()
#endif
