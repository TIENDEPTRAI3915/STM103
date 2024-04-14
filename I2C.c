#include "I2C.h"
volatile uint32_t sys;
void I2Cinit(void)
{ 

	GPIO_InitTypeDef gpio;
	I2C_InitTypeDef I2C_Inittructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	gpio.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	gpio.GPIO_Mode=GPIO_Mode_AF_OD;//note full duplex IN_FLOATING, IN_PU
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	
	I2C_Inittructure.I2C_Mode=I2C_Mode_I2C;
	I2C_Inittructure.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_Inittructure.I2C_OwnAddress1=0x00;
	I2C_Inittructure.I2C_Ack=I2C_Ack_Enable;
	I2C_Inittructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_Inittructure.I2C_ClockSpeed=50000;
	
	I2C_Init(I2C1,&I2C_Inittructure);
	I2C_Cmd(I2C1,ENABLE);
	
}	
	void I2C_Write1(uint8_t deviceAddr, uint8_t regAddr, uint8_t data)
{
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, deviceAddr<<1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, regAddr);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(I2C1, data);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
			I2C_AcknowledgeConfig(I2C1,DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
}
	
void I2C_Read1(uint8_t deviceAddr, uint8_t regAddr,uint16_t *diachi,uint8_t len) 
{while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
	;
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, (deviceAddr<<1),I2C_Direction_Transmitter)
	;
    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
			;
	    I2C_SendData(I2C1, regAddr);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	  I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, (deviceAddr<<1), I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
			I2C_AcknowledgeConfig(I2C1,ENABLE);
		while(len>0)
	{
		if(len==1)
		{	I2C_AcknowledgeConfig(I2C1,DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
		}
		
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)))
		;

  *diachi=I2C_ReceiveData(I2C1);
	len--;
	diachi++;
	}
}




void I2CWrite(uint8_t address,uint8_t Data)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		;
			I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
		;
			I2C_Send7bitAddress(I2C1,address,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;
			I2C_SendData(I2C1,Data);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTING))
		;
			I2C_GenerateSTOP(I2C1,ENABLE);
	
}
void LCDIni(void)
{
	delayms(40);
		I2CWrite(0x4E,(0<<0)|(0<<1)|(1<<2)|(1<<3)|(0x33&0xf0));
		I2CWrite(0x4E,(0<<0)|(0<<1)|(0<<2)|(1<<3)|(0x33&0xf0));
		delayms(6);
		I2CWrite(0x4E,(0<<0)|(0<<1)|(1<<2)|(1<<3)|((0x33&0x0F)<<4));
		I2CWrite(0x4E,(0<<0)|(0<<1)|(0<<2)|(1<<3)|((0x33&0x0F)<<4));
		delayus(110);
		WriteLCD(0x4E,0,0x32);
		delayus(110);
		WriteLCD(0x4E,0,0x28);
	delayms(2);
		WriteLCD(0x4E,0,0x08);
	delayms(2);
		WriteLCD(0x4E,0,0x01);
	delayms(2);
		WriteLCD(0x4E,0,0x06);
	delayms(2);
		WriteLCD(0x4E,0,0x0C);
		delayms(2);
}

void WriteLCD(uint8_t address,uint8_t COMAND_DATA,uint8_t Data)
{
	if(COMAND_DATA==0)
	{
		I2CWrite(address,(0<<0)|(0<<1)|(1<<2)|(1<<3)|(Data&0xf0));
		delayus(110);
		I2CWrite(address,(0<<0)|(0<<1)|(0<<2)|(1<<3)|(Data&0xf0));
		delayus(110);
		I2CWrite(address,(0<<0)|(0<<1)|(1<<2)|(1<<3)|((Data&0x0F)<<4));
		delayus(110);
		I2CWrite(address,(0<<0)|(0<<1)|(0<<2)|(1<<3)|((Data&0x0F)<<4));
		delayus(110);
	}
	else
	{
		I2CWrite(address,(1<<0)|(0<<1)|(1<<2)|(1<<3)|(Data&0xf0));
		delayus(110);
		I2CWrite(address,(1<<0)|(0<<1)|(0<<2)|(1<<3)|(Data&0xf0));
		delayus(110);
		I2CWrite(address,(1<<0)|(0<<1)|(1<<2)|(1<<3)|((Data&0x0F)<<4));
		delayus(110);
		I2CWrite(address,(1<<0)|(0<<1)|(0<<2)|(1<<3)|((Data&0x0F)<<4));
		delayus(110);
	}
	
}

int I2C_EEPROMREAD(uint8_t deviceAddr, uint8_t regAddr,uint8_t *diachi,uint8_t len)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
	  {
		}
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		{
		}
  I2C_Send7bitAddress(I2C1, (deviceAddr<<1),I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
		}
					
	    I2C_SendData(I2C1, regAddr);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
		}
			I2C_SendData(I2C1, regAddr);

		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
		}			
	  
		I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		{
		}	
		I2C_Send7bitAddress(I2C1, (deviceAddr<<1), I2C_Direction_Receiver);	
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
		}				
			I2C_AcknowledgeConfig(I2C1,ENABLE);
		while(len>0)
	{
		if(len==1)
		{	
		}
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)))
		{
		}	
  *diachi=I2C_ReceiveData(I2C1);
	len--;
	diachi++;
	}

}

int I2C_EEPROMWRITE(uint8_t deviceAddr, uint8_t regAddr,uint8_t *diachi,uint8_t len)
{
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
	  {
		}
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		{
		}
  I2C_Send7bitAddress(I2C1, (deviceAddr<<1),I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
		}
	    I2C_SendData(I2C1, regAddr);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
		}
			I2C_SendData(I2C1, regAddr);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
		}
		for(int i=0;i<len;i++)
		{
					I2C_SendData(I2C1,*diachi);
			while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
			}				
			diachi++;			
		}
			I2C_GenerateSTOP(I2C1, ENABLE);
}
