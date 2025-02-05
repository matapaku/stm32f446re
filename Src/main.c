/*
 * main.c
 *
 *  Created on: Feb 1, 2025
 *      Author: ogawarinkyuu
 */


#include "stm32f446xx.h"

int main(void)
{
	return 0;
}

void EXTIO_IRQHandling(void)
{
	GPIO_IRQHandling(0);
}
