
#include "TIMER.h"
void TIMinit(void)
{
	RCC->APB2ENR|=1<<11;//TIM1 CLOCK ENABLE
	TIM1->CR1 =0;
	TIM1->CR1 |=(1<<7);
	TIM1->PSC=72-1;
	TIM1->ARR=0xffff;
	TIM1->CR1 |=(1<<0);
}

__STATIC_INLINE void TIMER_DELAYUS(uint16_t count)
{
	TIM1->CNT=0;
	while(TIM1->CNT<count)
		;	
}
void TIMER_DELAYMS(uint16_t count)
{
	while(count--)
	{
		TIMER_DELAYUS(1000);
	}

}

