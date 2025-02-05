/*
 * stm32f446xx.h
 *
 *  Created on: Jan 31, 2025
 *      Author: ogawarinkyuu
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>                      // to use uint32_t
#include <stddef.h>


//  generic macro
#define __vo                      volatile

#define ENABLE         1
#define DISABLE        0
#define SET            1
#define RESET          0
#define GPIO_PIN_SET   1
#define GPIO_PIN_RESET 0

#define FLAG_RESET     0
#define FLAG_SET       1

#define NO_PR_BITS_IMPLEMENTED  4



// macros for address



		#define NVIC_ISER0 ((__vo uint32_t *)0xE000E100)
        #define NVIC_ISER1 ((__vo uint32_t *)0xE000E104)
        #define NVIC_ISER2 ((__vo uint32_t *)0xE000E108)
        #define NVIC_ISER3 ((__vo uint32_t *)0xE000E10C)

        #define NVIC_ICER0 ((__vo uint32_t *)0xE000E180)
        #define NVIC_ICER1 ((__vo uint32_t *)0xE000E184)
        #define NVIC_ICER2 ((__vo uint32_t *)0xE000E188)
        #define NVIC_ICER3 ((__vo uint32_t *)0xE000E18C)

#define NVIC_PR_BASE_ADDR       ((__vo uint32_t *)0xE000E400)


#define FLASH_BASEADDR            0x08000000U
#define SRMA1_BASEADDR            0x00000000U
#define SRMA2_BASEADDR            (0x0001C000U)
#define ROM_BASEADDR              0x1FFF0000U
#define SRAM                      SRMA1_BASEADDR

#define RCC_BASEADDR              0x40023800U

#define PERIPH_BASE               0x40000000
#define APB1PERIPH_BASE           PERIPH_BASE
#define APB2PERIPH_BASE           0x40010000
#define AHB1PERIPH_BASE           0x40020000
#define AHB2PERIPH_BASE           0x50000000

/////// AHB address

#define GPIOA_BASEADDR                     (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR                     (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR                     (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR                     (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR                     (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR                     (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR                     (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR                     (AHB1PERIPH_BASE + 0x1C00)


/*
 * BASE address of peripherals which is hanging  on APB1 bus
 */

#define TIM2_BASEADDR                     (AHB1PERIPH_BASE + 0x0000U)
#define TIM3_BASEADDR                     (AHB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR                     (AHB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR                     (AHB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR                     (AHB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR                     (AHB1PERIPH_BASE + 0x1400)
#define TIM12_BASEADDR                     (AHB1PERIPH_BASE + 0x1800)
#define TIM13_BASEADDR                     (AHB1PERIPH_BASE + 0x1C00)
#define TIM14_BASEADDR                     (AHB1PERIPH_BASE + 0x1800)

#define I2C1_BASEADDR                       (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR                       (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR                       (APB1PERIPH_BASE + 0x5C00)

#define SPI1_BASEADDR                         0x40013000U
#define SPI2_BASEADDR                         0x40003800U
#define SPI3_BASEADDR                         0x40003C00U
#define SPI4_BASEADDR                         0x40013400U

#define USART2_BASEADDR                       0x40004400U
#define USART3_BASEADDR                       0x40004800U
#define UART4_BASEADDR                        0x40004C00U
#define UART5_BASEADDR                        0x40005000U



/*
 * BASE address of peripherals which is hanging  on APB2 bus
 */

#define USART1_BASEADDR                       0x40011000U
#define EXTI_BASEADDR                         0x40013C00U
#define SYSCFG_BASEADDR                       0x40013800U

/*********************** peripheral register definition structure **********************************/

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDER;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];                    /* AFR[0]: GPIO alternate function low register
										        AFR[1]: GPIO alternate function high register */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t      Reserved0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t      Reserved1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t      Reserved2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t      Reserved3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t      Reserved4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	 uint32_t     Reserved5[2];
	__vo uint32_t BDCR;
	 uint32_t Reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;

}RCC_RegDef_t;


//peripheral register definition structure for exti

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;


//peripheral register definition structure for syscfg

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;

//peripheral register definition structure for spi

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */

typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;




/*
 *                      peripheral definitions
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB           ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC           ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD           ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE           ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF           ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG           ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH           ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC             ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI            ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG          ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1            ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2            ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3            ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4            ((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)


/*
 *  Clock enable macros for GPIO peripherals
 */

#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= ( 1 << 0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= ( 1 << 1))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= ( 1 << 2))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= ( 1 << 3))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= ( 1 << 4))
#define GPIOF_PCLK_EN()         (RCC->AHB1ENR |= ( 1 << 5))
#define GPIOG_PCLK_EN()         (RCC->AHB1ENR |= ( 1 << 6))
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= ( 1 << 7))

/*
 *  Clock enable macros for I2C peripherals
 */

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= ( 1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= ( 1 << 22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= ( 1 << 23))
#define I2C4_PCLK_EN()          (RCC->APB1ENR |= ( 1 << 24))

/*
 *  Clock enable macros for SPI peripherals
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= ( 1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= ( 1 << 15))
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= ( 1 << 13))

/*
 *  Clock enable macros for USART peripherals
 */

#define USART1_PCLK_EN()          (RCC->APB2ENR |= ( 1 << 4 ))
#define USART2_PCLK_EN()          (RCC->APB1ENR |= ( 1 << 17))
#define USART3_PCLK_EN()          (RCC->APB1ENR |= ( 1 << 18))
#define UART4_PCLK_EN()           (RCC->APB1ENR |= ( 1 << 19))
#define UART5_PCLK_EN()           (RCC->APB1ENR |= ( 1 << 20))
#define USART6_PCLK_EN()          (RCC->APB2ENR |= ( 1 << 5 ))

/*
 *  Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()          (RCC->APB2ENR |= ( 1 << 14 ))



////// -CLOCK DISABLE- ////////////////////////////////////////////////////////////////////////////////

/*
 *  Clock disable macros for GPIO peripherals
 */

#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~( 1 << 0))
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~( 1 << 1))
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~( 1 << 2))
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~( 1 << 3))
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~( 1 << 4))
#define GPIOF_PCLK_DI()         (RCC->AHB1ENR &= ~( 1 << 5))
#define GPIOG_PCLK_DI()         (RCC->AHB1ENR &= ~( 1 << 6))
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~( 1 << 7))

/*
 *  Clock disable macros for I2C peripherals
 */

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~( 1 << 21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~( 1 << 22))
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~( 1 << 23))
#define I2C4_PCLK_DI()          (RCC->APB1ENR &= ~( 1 << 24))


/*
 *  Clock disable macros for SPI peripherals
 */

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~( 1 << 12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~( 1 << 14))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~( 1 << 15))
#define SPI4_PCLK_DI()          (RCC->APB2ENR &= ~( 1 << 13))

/*
 *  Clock disable macros for USART peripherals
 */

#define USART1_PCLK_DI()          (RCC->APB2ENR &= ~( 1 << 4 ))
#define USART2_PCLK_DI()          (RCC->APB1ENR &= ~( 1 << 17))
#define USART3_PCLK_DI()          (RCC->APB1ENR &= ~( 1 << 18))
#define UART4_PCLK_DI()           (RCC->APB1ENR &= ~( 1 << 19))
#define UART5_PCLK_DI()           (RCC->APB1ENR &= ~( 1 << 20))
#define USART6_PCLK_DI()          (RCC->APB2ENR &= ~( 1 << 5 ))

/*
 *  Clock disable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()          (RCC->APB2ENR &= ~( 1 << 14 ))

/*
 * Macros to reset generic GPIOX
 */

#define GPIOA_REG_RESET()            do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= (1 << 0)); }while(0)
#define GPIOB_REG_RESET()            do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= (1 << 1)); }while(0)
#define GPIOC_REG_RESET()            do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= (1 << 2)); }while(0)
#define GPIOD_REG_RESET()            do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= (1 << 3)); }while(0)
#define GPIOE_REG_RESET()            do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= (1 << 4)); }while(0)
#define GPIOF_REG_RESET()            do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= (1 << 5)); }while(0)
#define GPIOG_REG_RESET()            do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= (1 << 6)); }while(0)
#define GPIOH_REG_RESET()            do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= (1 << 7)); }while(0)

#define GPIO_BASE_ADDR_TO_CODE(x)            (  (x == GPIOA) ? 0:\
												(x == GPIOB) ? 1:\
												(x == GPIOC) ? 2:\
												(x == GPIOD) ? 3:\
												(x == GPIOE) ? 4:\
												(x == GPIOF) ? 5:\
												(x == GPIOG) ? 6:\
												(x == GPIOH) ? 7:0 )

// IRQ number

#define IRQ_NO_EXTI0         6
#define IRQ_NO_EXTI1         7
#define IRQ_NO_EXTI2         8
#define IRQ_NO_EXTI3         9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8


/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15




#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

#endif /* INC_STM32F446XX_H_ */
