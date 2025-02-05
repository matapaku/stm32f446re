/*
 * 001led_toogle.c
 *
 *  Created on: Feb 1, 2025
 *      Author: ogawarinkyuu
 */

#include "stm32f446xx.h"


void delay(void){
	for(uint32_t i=0;i<500000;i++);
}


int main(void){

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToogleOutputPin(GPIOD, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
