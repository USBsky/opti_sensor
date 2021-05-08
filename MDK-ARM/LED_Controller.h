#ifndef main
#include "stm32f4xx_hal.h"
#endif


struct LED {
	uint8_t led_sel[4];
	uint8_t ledx[7];
};

struct LED_Controller {
	uint8_t data[4];
	
};