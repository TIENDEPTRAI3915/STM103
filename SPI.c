#include "SPI.h"
SPI_InitTypeDef spi;

void SPIinit(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	
	spi.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_128;
	spi.SPI_CPHA=SPI_CPHA_2Edge;
	spi.SPI_CPOL=SPI_CPOL_High;
	spi.SPI_DataSize=SPI_DataSize_8b;
	spi.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	spi.SPI_FirstBit=SPI_FirstBit_MSB;
	spi.SPI_Mode=SPI_Mode_Master;
	spi.SPI_NSS=SPI_NSS_Soft;
	SPI_Init(SPI2,&spi); 
	
	 SPI_SSOutputCmd(SPI2,DISABLE);
	SPI_Cmd(SPI2,ENABLE);
}

void SPI_write(uint8_t data,uint8_t len)
{
	while(len)
	{
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE))
			;
		SPI2->DR=data;
		len--;
	}
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY))
			;
	uint8_t temp = SPI2->DR;
	temp= SPI2->SR;	
}

void SPI_led_write(uint8_t data,uint8_t reg)
{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
//		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE))
//			;		
//		SPI2->DR= reg;
//	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY))
//			;
////	uint8_t temp = SPI2->DR;
////	temp= SPI2->SR;	
//	
//	while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE))
//			;		
//		SPI2->DR= data;
//	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY))
//			;
////		temp = SPI2->DR;
////	temp= SPI2->SR;	
	// Wait until the transmit buffer is empty
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	// Send the data
	SPI_I2S_SendData(SPI2, reg);
	// Wait until the data is completely sent
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
	SPI_I2S_SendData(SPI2, data);
	// Wait until the data is completely sent
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
}

void SPI_read(uint8_t *array ,uint8_t len)
{
	while(len)
	{
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY))
			;
		SPI2->DR=0;
				while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE))
			;
		*array=SPI2->DR;
		array++;
		len--;
	}
}

void SPI_MPU_read_lientuc(uint8_t data,uint8_t *array ,uint8_t len)
{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);

	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY))
			;
		SPI2->DR=data|(0x80);
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE))
		;
		uint8_t DUMPDATA = SPI2->DR;
		DUMPDATA= SPI2->SR;	
			while(len)
		{			
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE))
			;			
		SPI2->DR=0;
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE))
			;
		*array=SPI2->DR;
		array++;
		len--;
		}
		GPIO_SetBits(GPIOB,GPIO_Pin_12);	
}
void SPI_MPU_write (uint8_t reg,uint8_t data)
{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE))
			;
		SPI2->DR=reg;	
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE))
			;
		SPI2->DR=data;
		
		while(!SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE))
			;
		uint8_t temp = SPI2->DR;	
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY))
				;
		temp = SPI2->DR;
		temp= SPI2->SR;	
		GPIO_SetBits(GPIOB,GPIO_Pin_12);		
}
