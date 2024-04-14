#include "ENCODER.h"

void encodeconfig(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
	GPIO_InitTypeDef gpioa;
	TIM_TimeBaseInitTypeDef encode;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	gpioa.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	gpioa.GPIO_Pin|=GPIO_Pin_0|GPIO_Pin_1;
	gpioa.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioa);
	
	
	encode.TIM_ClockDivision=0;
	encode.TIM_CounterMode=TIM_CounterMode_Up;
	encode.TIM_Period=0xffff;
	encode.TIM_Prescaler=0;
	TIM_TimeBaseInit(TIM2,&encode);
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);

	
//	TIM_ITConfig(TIM2,TIM_IT_Trigger,ENABLE);	
	//TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
	//TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn ;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	 
//	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM2,ENABLE);
}
