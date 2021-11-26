#define SBAUD_TH		104U	// 16M/16/9600
#define SBAUD_SET		128000U	// Baud rate of serial port 0

#define HARD_ESP_CTRL 1

#define MPSSE_DEBUG	0
#define MPSSE_HWSPI	1

#define GOWIN_INT_FLASH_QUIRK 1

#define FAST_RECEIVE		//UART

/*
 * Use T0 to count the SOF_Count every 1ms
 * If you doesn't like this feature, define SOF_NO_TIMER
 * Background: The usb host must to send SOF every 1ms, but some USB host don't really do that
 * FTDI's driver has some bug, if it doesn't received empty packet with modem status,
 * it will causes BSoD, so highly recommended use T0 instead of SOF packet to generate empty packet report.
 */
//#define SOF_NO_TIMER

//#define USB_SLEEP

