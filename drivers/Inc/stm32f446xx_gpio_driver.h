/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jan 31, 2025
 *      Author: ogawarinkyuu
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;                    //   values from @GPIO_PINS_MODES
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunmode;

}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/*
 * GPIPO_PIN_NO
 */

#define GPIO_PIN_NO_0   0
#define GPIO_PIN_NO_1   1
#define GPIO_PIN_NO_2   2
#define GPIO_PIN_NO_3   3
#define GPIO_PIN_NO_4   4
#define GPIO_PIN_NO_5   5
#define GPIO_PIN_NO_6   6
#define GPIO_PIN_NO_7   7
#define GPIO_PIN_NO_8   8
#define GPIO_PIN_NO_9   9
#define GPIO_PIN_NO_10  10
#define GPIO_PIN_NO_11  11
#define GPIO_PIN_NO_12  12
#define GPIO_PIN_NO_13  13
#define GPIO_PIN_NO_14  14
#define GPIO_PIN_NO_15  15


/*
 * @GPIO_PINS_MODES
 * GPIO possible modes
 */

#define GPIO_IN      0
#define GPIO_OUT     1
#define GPIO_ALTFN   2
#define GPIO_ANALOG  3
#define GPIO_IT_FT   4
#define GPIO_IT_RT   5
#define GPIO_IT_FRT  6


/*
 * GPIO possible output types
 */

#define GPIO_OP_TYPE_PP    0
#define GPIO_OP_TYPE_OD    1

/*
 * GPIO possible speed types
 */

#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3

/*
 * GPIO pull up and pull down types
 */

#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2

/*** APIs setup ****/


//Clock setup

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


// init and de-init

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//data read and write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WritToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToogleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
