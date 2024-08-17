#include "hal.h"

void SPI_Init() {
	SPI0_CK_SE = 0x06;
}
