/*
 * stm32f407xx.h
 *
 *  Created on: Jan 27, 2021
 *      Author: urmil
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_


#include <stdint.h>
#define __vo volatile

/*********************************Processor Specific Details ***************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Address
 * */
#define NVIC_ISER0    (( __vo uint32_t*)0xE000E100 )
#define NVIC_ISER1    (( __vo uint32_t*)0xE000E104 )
#define NVIC_ISER2    (( __vo uint32_t*)0xE000E108 )
#define NVIC_ISER3    (( __vo uint32_t*)0xE000E10c )

/*
 * ARM Cortex Mx Processor NVIC ICERx Register Address
 */

#define NVIC_ICER0   (( __vo uint32_t*)0xE000E180 )
#define NVIC_ICER1   (( __vo uint32_t*)0xE000E184 )
#define NVIC_ICER2   (( __vo uint32_t*)0xE000E188 )
#define NVIC_ICER3   (( __vo uint32_t*)0xE000E18C )


/*
 * ARM Cortex MX processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED    4




#define FLASH_BASEADDR 				0x08000000U  /*This is the location of the main memory */
#define SRAM1_BASEADDR				0x20000000U  /* Ram starts from 0x20000000. It is set to unsigned integer*/
#define SRAM2_BASEADDR 				0x2001C000U /* Since SRAM1 is 112kb. When added to the base address of SRAM1 and converted to hex. We get 1C000U. The SRAM2_BaseAddr is the addition of SRAM1 base_address +112kb which will be 0x2001C000U*/
#define ROM_BASEADDR				0x1FFF0000U /* From the refe3 reference manual. The System memory is the BaseAddr for ROM.*/

/* Peripheral bus base address*/
#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASE
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define	AHB2PERIPH_BASEADDR 			0x50000000U


/* the base addresses of GPIO peripherals on the AHB1 bus*/
#define GPIOA_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR  			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0200)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)


/* Base addresses of peripherals on the APB1 bus*/
#define I2C1_BASEADDR 				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

/* Base addresses of peripherals on APB2 bus */
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

/* GPIO Peripheral register Structure*/
typedef struct
{
	__vo uint32_t MODER;               	/* GPIO Port Mode Register Address offset: 0x00*/
	__vo uint32_t OTYPER;					/* GPIO Port Output Register Address offset: 0x04*/
	__vo uint32_t OSPEEDR;				/* GPIO Port Output speed Register Address offset: 0x08*/
	__vo uint32_t PUPDR;					/* GPIO Port Pull-up/down Register Address offset: 0x0C*/
	__vo uint32_t IDR;					/* GPIO Port Input data Register Address offset: 0x10*/
	__vo uint32_t ODR;					/* GPIO Port Output data Register Address offset: 0x14*/
	__vo uint32_t BSSR;					/* GPIO Port Bit set/reset Register Address offset: 0x18*/
	__vo uint32_t LCKR;					/* GPIO Port Configuration lock Register Address offset: 0x1C*/
	__vo uint32_t AFR[2];					/* GPIO Port Alternate function low Register Address offset: 0x20*/
}GPIO_RegDef_t;

/*EXTI Peripheral register Structure*/
typedef struct
{
	__vo uint32_t IMR;               	/* GPIO Port Mode Register Address offset: 0x00*/
	__vo uint32_t EMR;					/* GPIO Port Output Register Address offset: 0x04*/
	__vo uint32_t RTSR;				/* GPIO Port Output speed Register Address offset: 0x08*/
	__vo uint32_t FTSR;					/* GPIO Port Pull-up/down Register Address offset: 0x0C*/
	__vo uint32_t SWIER;					/* GPIO Port Input data Register Address offset: 0x10*/
	__vo uint32_t PR;					/* GPIO Port Output data Register Address offset: 0x14*/
}EXTI_RegDef_t;

/*
 * SYSCFG Peripheral register Structure
 * */
typedef struct
{
  __vo uint32_t MEMRMP;
  __vo uint32_t PMC;
  __vo uint32_t EXTICR[4];
  uint32_t 	   RESERVED1[2];
  __vo uint32_t CMPCR;
  uint32_t      RESERVED2[2];
  __vo uint32_t  CFGR;
}SYSCFG_RegDef_t;

/*
* RCC Peripheral register Structure
*/

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t      RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t 	  RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t 	  RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t      RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t      RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t 	  RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t 	  RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;


/*
 * SPI Peripheral register Configuration
 * */

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



/* Peripheral Definitions (Peripheral base addresses type casted to xxx_RegDef_t)*/


#define GPIOA 				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI    			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG 				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1   				((SPI_RegDef_t*)SPI1_BASE)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASE)
#define SPI3   				((SPI_RegDef_t*)SPI3_BASE)


/* Clock enable macros for GPIOx peripherals*/
#define GPIOA_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 0))
#define GPIOB_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 1))
#define GPIOC_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 2))
#define GPIOD_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 3))
#define GPIOE_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 4))
#define GPIOF_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 5))
#define GPIOG_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 6))
#define GPIOH_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 7))
#define GPIOI_PCLK_EN()   (RCC ->AHB1ENR |= ( 1 << 8))


/*Clock Enable for I2Cx peripherals*/
#define I2C1_PCLK_EN() 	 ( RCC ->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()	 ( RCC ->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()	 ( RCC ->APB1ENR |= (1 << 23) )



/*Clock enable for SPIx peripherals*/
#define SPI1_PCLK_EN()  ( RCC ->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN() 	( RCC ->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()  ( RCC ->APB1ENR |= (1 << 15) )

/*Clock enable for USARTx Peripherals*/
#define USART1_PCLK_EN()  ( RCC ->APB2ENR |= (1 << 4) )
#define USART6_PCLK_EN()  ( RCC ->APB2ENR |= (1 << 5) )


/* Clock enable for SYSCFG peripherals */
#define SYSCFG_PCLK_EN() ( RCC ->APB2ENR |= (1 << 14) )




/*Clock disable for GPIOx peripherals */
#define GPIOA_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()  ( RCC ->AHB1ENR &= ~( 1 << 8 ) )


/*Clock disable for I2Cx peripherals*/
#define I2C1_PCLK_DI() 	 (RCC ->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()	 (RCC ->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()	 (RCC ->APB1ENR &= ~( 1 << 23 ) )



/*Clock disable for SPIx peripherals*/
#define SPI1_PCLK_DI()  (RCC ->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() 	(RCC ->APB1ENR &= ~(1 << 15))
#define SPI3_PCLK_DI()  (RCC ->APB1ENR &= ~(1 << 16))

/*Clock disable for USARTx Peripherals*/
#define USART1_PCLK_DI()  (RCC ->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()  (RCC ->APB2ENR &= ~(1 << 5))


/* Clock disable for SYSCFG peripherals */
#define SYSCFG_PCLK_DI() (RCC ->APB2ENR &= ~(1 << 14))

/* Macros to reset the GPIO peripherals*/
#define GPIOA_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 0)); (RCC -> AHB1RSTR &= ~(1 << 0));} while (0)
#define GPIOB_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 1)); (RCC -> AHB1RSTR &= ~(1 << 1));} while (0)
#define GPIOC_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 2)); (RCC -> AHB1RSTR &= ~(1 << 2));} while (0)
#define GPIOD_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 3)); (RCC -> AHB1RSTR &= ~(1 << 3));} while (0)
#define GPIOE_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 4)); (RCC -> AHB1RSTR &= ~(1 << 4));} while (0)
#define GPIOF_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 5)); (RCC -> AHB1RSTR &= ~(1 << 5));} while (0)
#define GPIOG_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 6)); (RCC -> AHB1RSTR &= ~(1 << 6));} while (0)
#define GPIOH_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 7)); (RCC -> AHB1RSTR &= ~(1 << 7));} while (0)
#define GPIOI_REG_RESET() do{ ( RCC ->AHB1RSTR |= (1 << 8)); (RCC -> AHB1RSTR &= ~(1 << 8));} while (0)






/*
 * returns port code for given GPIOx base address
 * */
#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOB) ? 3 :\
									(x == GPIOA) ? 4 :\
									(x == GPIOB) ? 5 :\
									(x == GPIOA) ? 6 :\
									(x == GPIOB) ? 7 :0 )
/*
 * Interrupt request of STM32F407x MCU
 * This is MCU specific macros for interrupts
 * */

#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40



/*NVIC IRQ Priority macros*/

#define NVIC_IRQ_PRIO0  0
#define NVIC_IRQ_PRIO1  1
#define NVIC_IRQ_PRIO2  2
#define NVIC_IRQ_PRIO3  3
#define NVIC_IRQ_PRIO4  4
#define NVIC_IRQ_PRIO5  5
#define NVIC_IRQ_PRIO6  6
#define NVIC_IRQ_PRIO7  7
#define NVIC_IRQ_PRIO8  8
#define NVIC_IRQ_PRIO9  9
#define NVIC_IRQ_PRIO10  10
#define NVIC_IRQ_PRIO11 11
#define NVIC_IRQ_PRIO12  12
#define NVIC_IRQ_PRIO13  13
#define NVIC_IRQ_PRIO14  14
#define NVIC_IRQ_PRIO15  15




/* Some Generic macros*/
#define Enable 			1
#define Disable 		0
#define SET 			Enable
#define RESET 			Disable
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET  RESET




#include "stm32f407xx_gpio_driver.h"


#endif /* STM32F407XX_H_ */






