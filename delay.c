#include "delay.h"

void delayinit(void)
{ RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStucture;
	TIM_TimeBaseInitStucture.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStucture.TIM_CounterMode=TIM_CounterMode_Up ;
	TIM_TimeBaseInitStucture.TIM_Period=5000;
	TIM_TimeBaseInitStucture.TIM_Prescaler=71;
	TIM_TimeBaseInitStucture.TIM_RepetitionCounter=0;
	
   TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStucture);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x07;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);
}
void delayms(uint16_t timems)
{
	while(timems--)
		{	TIM1->CNT=0;
			while(TIM1->CNT<1000)
			;
		}
}
void delayus(uint16_t timeus)
{
			TIM1->CNT=0;
			while(TIM1->CNT<timeus)
			;

}
