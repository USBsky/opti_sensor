#include "LED_Controller.h"


Init_Status LED_init() {
	led.led_sel[0] = GPIO_PIN_6;
	led.led_sel[1] = GPIO_PIN_7;
	led.led_sel[2] = GPIO_PIN_8;
	led.led_sel[3] = GPIO_PIN_9;
	led.ledx[0] = GPIO_PIN_1;
	led.ledx[1] = GPIO_PIN_2;
	led.ledx[2] = GPIO_PIN_3;
	led.ledx[3] = GPIO_PIN_4;
	led.ledx[4] = GPIO_PIN_5;
	led.ledx[5] = GPIO_PIN_6;
	led.ledx[6] = GPIO_PIN_7;
	led.ledx[7] = GPIO_PIN_8;
}

Init_Status LED_Controller_init() {
		led_ctrl.ptr = 0;
}

Init_Status LED_Controller_setdata(float data) {
		unsigned char *ptr = (unsigned char*)&data;
		for(int i = 0; i < 4; i++) {
			 led_ctrl.data[i] = *ptr++;
		}
}


void LED_dis() {
	HAL_GPIO_WritePin(GPIOB, led_ctrl.led_encode, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, led_ctrl.led_sel, GPIO_PIN_RESET);
	uint8_t data = led.ledx[led_ctrl.ptr];
	switch(data) {
		case 0 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2] | led.ledx[3] | led.ledx[4] | led.ledx[5];
		break;
		case 1 : led_ctrl.led_encode = led.ledx[1] | led.ledx[2];
		break;
		case 2 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[3] | led.ledx[4] | led.ledx[6];
		break;
		case 3 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2] | led.ledx[3] | led.ledx[6];
		break;
		case 4 : led_ctrl.led_encode = led.ledx[1] | led.ledx[2] | led.ledx[5] | led.ledx[6];
		break;
		case 5 : led_ctrl.led_encode = led.ledx[0] | led.ledx[2] | led.ledx[3] | led.ledx[5] | led.ledx[6];
		break;
		case 6 : led_ctrl.led_encode = led.ledx[0] | led.ledx[2] | led.ledx[3] | led.ledx[4] | led.ledx[5] | led.ledx[6];
		break;
		case 7 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2];
		break;
		case 8 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2] | led.ledx[3] | led.ledx[4] | led.ledx[6] \
		|	led.ledx[5];
		break;
		case 9 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2] | led.ledx[3] | led.ledx[4] | led.ledx[6];
	}
	switch(led_ctrl.ptr) {
		case 1 : led_ctrl.led_sel = led.led_sel[0]; led_ctrl.led_encode = led_ctrl.led_encode | led.ledx[7]; led_ctrl.ptr = 2; break;
		case 2 : led_ctrl.led_sel = led.led_sel[1]; led_ctrl.ptr = 3; break;
		case 3 : led_ctrl.led_sel = led.led_sel[2]; led_ctrl.ptr = 4; break;
		case 4 : led_ctrl.led_sel = led.led_sel[3]; led_ctrl.ptr = 1; break;
	}
	HAL_GPIO_WritePin(GPIOB, led_ctrl.led_encode, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, led_ctrl.led_sel, GPIO_PIN_SET);
}

