/*
 * 002led_button.c
 *
 *  Created on: Feb 1, 2025
 *      Author: ogawarinkyuu
 */


#include "stm32f446xx.h"


void delay(void){
	for(uint32_t i=0;i<500000;i++);
}


int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn;


	//led setup
	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);

	//button setup,

	GPIOBtn.pGPIOx = GPIOC;

	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0)
		{
			delay();
			GPIO_ToogleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}
	return 0;
}
