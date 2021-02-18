/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 01-Feb-2021
 *      Author: urmil
 */


#include "stm32f407xx_gpio_driver.h"

/*Peripheral Clock Setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == Enable)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
				else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}
				else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}
				else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}
				else if (pGPIOx == GPIOE)
				{
					GPIOD_PCLK_DI();
				}
				else if (pGPIOx == GPIOF)
				{
					GPIOE_PCLK_DI();
				}
				else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}
				else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}
				else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}
}

/*Init and De-init */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1. Configure the mode of GPIO pin
	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//Non interrupt mode
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle ->pGPIOx -> MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing a bit
		pGPIOHandle ->pGPIOx -> MODER |= temp; //setting a bit
	}
else
{
	//Interrupt mode
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
	{
		//Configure the FTSR
		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Clear the corresponding RTSR bit
		EXTI ->RTSR &= ~ (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT )
	{
		//Configure the RTSR
		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//Clear the corresponding RTSR bit
		EXTI ->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFFT)
	{

	 //1. Configure both FTSR and RTSR
		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		EXTI ->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	//2. Configure the GPIO port selection in SYSCFG_EXTICR
	uint8_t temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 4 ;
	uint8_t temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4 ;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle ->pGPIOx) ;
	SYSCFG_PCLK_EN();
	SYSCFG -> EXTICR[temp1] = portcode << (temp2 *4);

	//3. Enable the exti interrupt delivery using IMR
	EXTI -> IMR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
}
	temp = 0;
	//2. Configure the speed of the GPIO pin
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber ) );
	pGPIOHandle ->pGPIOx -> OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing a bit
	pGPIOHandle ->pGPIOx ->OSPEEDR |= temp; // Setting a bit


	temp = 0;
	//3. Configure the PuPd settings
	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle ->pGPIOx -> PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clearing a bit
	pGPIOHandle -> pGPIOx ->PUPDR |= temp; //Setting a bit

	temp = 0;
	//4. Configure the output type
	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinOPtype << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle ->pGPIOx -> OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //clearing a bit
	pGPIOHandle -> pGPIOx ->OTYPER |= temp; //Setting a bit

	temp = 0;
	//5. Configure Alternate Functionality
	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//Configure Alternate Function Mode
		uint32_t temp1 = 0;
		uint32_t temp2 = 0;
		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle -> pGPIOx-> AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle -> pGPIOx-> AFR[temp1] |= (pGPIOHandle ->GPIO_PinConfig.GPIO_PinAltFunMode<< (4*temp2));


	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if (pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if (pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}
/*Data Read and Write */


/*
 * @return type will be 0 or 1
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	 uint8_t value;
 	 value =  (uint8_t)((pGPIOx ->IDR >> PinNumber) & 0x00000001 );
	 return value;

}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	 uint16_t value;
	 value =  (uint16_t)pGPIOx ->IDR;
	 return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) // Value can be either 0 or 1
{
		if(Value == GPIO_PIN_SET)
		{
			//Write 1 to the output data register at the bit field corresponding to the pin number
			pGPIOx ->ODR |= (1 << PinNumber);
		}else
		{
			//Write 0 to the output data register at the bit field corresponding to the pin number
			pGPIOx ->ODR &= ~(1 << PinNumber);
		}
	}

void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx ->ODR = Value;
}


void GPIO_ToggleOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx ->ODR ^= (1 << PinNumber);
	}

/*IRQ configuration and handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == Enable)
	{
		if(IRQNumber <=31 )
		{
			//Program ISER0 Register //0 to 31
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 Register //32 to 64
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 Register //64 to 95
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if (IRQNumber <=31 )
		{
		 //Program ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber < 64)
		{
			//Program ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber <96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber %64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// Find the IPR Registers
	uint8_t iprx = IRQNumber / 4 ;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 *iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx * 4) |= (IRQPriority << shift_amount);


}




void GPIO_IRQHandling(uint8_t PinNumber )
{
	if(EXTI ->PR & (1 <<PinNumber ))
	{
		//clear
		EXTI ->PR |= (1 << PinNumber);
	}
}
