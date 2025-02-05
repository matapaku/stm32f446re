/*
 * 005button_interrupt.c
 *
 *  Created on: Feb 1, 2025
 *      Author: ogawarinkyuu
 */


#include "stm32f446xx.h"
#include <string.h>


void delay(void){
	for(uint32_t i=0;i<500000;i++);
}


int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn;

	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

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

	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);


	//IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1)
	{

	}


	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(IRQ_NO_EXTI9_5);
	GPIO_ToogleOutputPin(GPIOC, GPIO_PIN_NO_13);
}
