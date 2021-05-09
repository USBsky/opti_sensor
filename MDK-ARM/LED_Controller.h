#ifndef __MAIN_H
#include "stm32f4xx_hal.h"
#endif


typedef enum  Init_Status{
	ok = 0,
	error = 1
}Init_Status;

typedef struct LED {
	uint16_t led_sel[4];
	uint16_t ledx[8];
}LED;

typedef struct LED_Controller {
	uint8_t data[4];
	uint8_t ptr;
	uint16_t led_encode;
	uint16_t led_sel;
}LED_Controller;

volatile LED_Controller led_ctrl;
volatile LED led;




Init_Status LED_init();
Init_Status LED_Controller_init();
Init_Status LED_Controller_setdata(float data);
void LED_dy_ctrl();
void LED_dis();