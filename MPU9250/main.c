#include "stm32f10x.h"
#include "GPIO.h"
#include "delay.h"
#include "SPI.h"
uint8_t accel[6];
uint8_t gyro[6];
float giatocx,giatocy,giatocz;
int main()
{
	GPIOinit();
	delayinit();
	SPIinit();
	//init MPU9250
//	GPIO_ResetBits(GPIOB,GPIO_Pin_12);	
//	SPI_write(0x6B,1);
//		SPI_write(0,1);
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);
//	
//	GPIO_ResetBits(GPIOB,GPIO_Pin_12);	
//	SPI_write(0x19,1);
//		SPI_write(0x07,1);
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);
//	
//	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
//	SPI_write(0x1C,1);
//			SPI_write(0,1);
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);
//	
//		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
//	SPI_write(0x1B,1);//read add 1<<7 write clear bit 7
//				SPI_write(0,1);
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);

	SPI_MPU_write(0x6B,0);
	SPI_MPU_write(0x19,0x07);
	SPI_MPU_write(0x1B,0);
	SPI_MPU_write(0x1C,0);
	while(1)
	{
//		GPIO_ResetBits(GPIOB,GPIO_Pin_12);	
			SPI_MPU_read_lientuc(0x3B,accel,6);
//		SPI_write(0x3B|(1<<7),1);
//		SPI_read(accel,1);
//		SPI_read(accel,1);
//		SPI_read(accel,1);
//		SPI_read(accel,1);
//		SPI_read(accel,1);
//		SPI_read(accel,1);
//		GPIO_SetBits(GPIOB,GPIO_Pin_12);
		SPI_MPU_read_lientuc(0x43,gyro,6);	
		giatocx=((int16_t)(accel[0]<<8|accel[1]))/16384.0;
		giatocy=((int16_t)(accel[2]<<8|accel[3]))/16384.0;
		giatocz=((int16_t)(accel[4]<<8|accel[5]))/16384.0;		
		
		
		delayms(3000);
	}

}