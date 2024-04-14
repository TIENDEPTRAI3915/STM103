#include "GPIO.h"

void GPIOinit()
{	
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	gpio.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio.GPIO_Pin=GPIO_Pin_14|GPIO_Pin_13 ;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init  (GPIOC,&gpio); 
	
	//USART1 init gpio PA9,PA10
	GPIO_PinRemapConfig(GPIO_Remap_USART1 ,DISABLE);
	//USART1 RX
	gpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;//note full duplex IN_FLOATING, IN_PU
	gpio.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&gpio);
	//USART1 TX
	gpio.GPIO_Mode= GPIO_Mode_AF_PP;// note full duplex AF_PP ; Half duplex synchronous mode AF_PP
	gpio.GPIO_Pin=GPIO_Pin_9;
	GPIO_Init(GPIOA,&gpio);
	//ADC1chanell0 GPIO config
	gpio.GPIO_Pin=GPIO_Pin_0;
	gpio.GPIO_Mode=GPIO_Mode_AIN;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);
	//SPI 
		gpio.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_15;
	gpio.GPIO_Mode=GPIO_Mode_AF_PP;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	gpio.GPIO_Pin=GPIO_Pin_14;
	gpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	gpio.GPIO_Pin=GPIO_Pin_12;
	gpio.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	
	
	
	/*---------------------------------------*/
	//---EXTI PA0 config---
	
	
	
//	//GPIO PA0 CONFIG
//	gpio.GPIO_Mode=GPIO_Mode_IPU;
//	gpio.GPIO_Pin=GPIO_Pin_0;
//	GPIO_Init(GPIOA,&gpio);
//	
//	//Line 0 PA
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
// 
//	
//   EXTI_InitStructure.EXTI_Line = EXTI_Line0;
//   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//   EXTI_Init(&EXTI_InitStructure);
// 

//   NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0E;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 
//   NVIC_Init(&NVIC_InitStructure);
}
