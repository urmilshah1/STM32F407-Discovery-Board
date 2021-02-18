/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 11-Feb-2021
 *      Author: urmil
 */

#ifndef STM32F407XX_SPI_DRIVER_H_
#define STM32F407XX_SPI_DRIVER_H_


typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Cofig_t;


/*
 * Handle structure for SPIx Peripheral
 * */
typedef struct
{
	SPI_RegDef_t  *pSPIx;
	SPI_Config_t  SPIConfig;
}SPI_Handle_t;


/*Peripheral Clock Setup*/
void SPI_PeriClockControl(SPI_RegDef_t *pGPIOx, uint8_t EnorDi);

/*Init and De-init */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPI);

/*
 * Data send and Receive
 * */

void SPI_SendData(SPI_RegDef_t *pSPI, uint8_t *pTxBuffer, uint32_t Len);  //Standard practice to define Len as uint32_t



void SPI_ReceiveData(SPI_RegDef_t *pSPI, uint8_t *pRxBuffer, uint32_t Len);

/*
/*IRQ configuration and handling*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other peripheral control API's
 * */






#endif /* STM32F407XX_SPI_DRIVER_H_ */
