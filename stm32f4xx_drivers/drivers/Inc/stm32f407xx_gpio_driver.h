/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 01-Feb-2021
 *      Author: urmil
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


typedef struct
{
	uint8_t GPIO_PinNumber; /*This can be configured from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;  /* This can be configured from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed; /* This can be configured from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl; /*This can be configured from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPtype; /* This can be configured from @GPIO_PIN_OUT_TYPE*/
	uint8_t GPIO_PinPullup;
	uint8_t GPIO_PinAltFunMode;  /* This can be configured from */
}GPIO_PinConfig_t;



typedef struct
{
	/* Pointer to hold the base address of GPIO peripheral*/
	GPIO_RegDef_t *pGPIOx; 			 /*This holds the Base address of GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;




/*@GPIO_PIN_NUMBER
 * GPIO Possible Pin Numbers
 * */
#define GPIO_PIN_NO_0 		0
#define GPIO_PIN_NO_1 		1
#define GPIO_PIN_NO_2 		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4 		4
#define GPIO_PIN_NO_5 		5
#define GPIO_PIN_NO_6 		6
#define GPIO_PIN_NO_7 		7
#define GPIO_PIN_NO_8 		8
#define GPIO_PIN_NO_9 		9
#define GPIO_PIN_NO_10 		10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12 		12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15 		15



/* @GPIO_PIN_MODES
 * GPIO pin possible modes
 * */
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4  // Interrupt Falling Edge Trigger
#define GPIO_MODE_IT_RT 5 // Interrupt Rising Edge Trigger
#define GPIO_MODE_IT_RFFT 6 //Interrupt Rising Edge Falling Edge Trigger

/* @GPIO_PIN_OUT_TYPE
 * GPIO Output type Push Pull
 * */
#define GPIO_OP_TYPE_PP 0  //Push Pull
#define GPIO_OP_TYPE_OD 1  //Open Drain

/* @GPIO_PIN_SPEED
 *  GPIO Possible Output Speeds
 *  */

#define GPIO_SPEED_LOW  0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

/*@GPIO_PIN_PUPD
 *  GPIO Pull up Pull Down configuration Macros
 *  */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2


/*Peripheral Clock Setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*Init and De-init */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Data Read and Write */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOuputPort(GPIO_RegDef_t *GPIOx, uint16_t Value);
void GPIO_ToggleOuputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);


/*IRQ configuration and handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber );



#endif /* STM32F407XX_GPIO_DRIVER_H_ */
