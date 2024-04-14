#include "GPIO.h"

void GPIOinit(void)
{	
	RCC->APB2ENR|=(1<<4);//GPIOC CLOCK ENABLE
	//GPIOC OUTPUT PP 50MHZ
	GPIOC->CRH |=(3<<20);
	GPIOC->CRH &=~(3<<22);
	
}
