
#include "stm32f10x.h"
#include "GPIO.h"
#include "TIMER.h"
#include "UART.h"
#include "string.h"
#include <stdio.h>
#include <math.h>
typedef union {
    float floatValue;
    uint8_t byteArray[4];
} FloatByteArray;
char mang[6];
char mang2[4];
float a;
char r;
volatile uint32_t delay;
int lock=0;
void systick_config(void)
{
	SysTick_Config(SystemCoreClock/1000);
	SysTick->CTRL&=~(1<<0);
}
void sysdelay(uint32_t count)
{
	delay=count;
	SysTick->VAL=0;
	SysTick->CTRL|=(1<<0);
	while(delay)
		;
}
void SysTick_Handler(void)
{
	delay--;
}

int main()
{ 
	//SystemInit();

	GPIOinit();
	TIMinit();
	UART_CONFIG();
	systick_config();
	while(1)
	{		
//	if(lock==0)
//	{
//		lock=UART_RXchar();
//	}
//	else
//	{
//	GPIOC->ODR^=(1<<13);
//	UART_TXstring("HELLO");
//	TIMER_DELAYMS(1000);
		
		int n=0;
		while(1)
		{
			FloatByteArray data;
			r=UART_RXchar();
			a=sin(1*n*0.01);
			data.floatValue=a;
		//sprintf(mang,"%1.3lf",a);
			if(mang[5]==0)
			{
			mang[5]='0';
			}
				n++;
			//UART_TXstring(mang);
			for(int i=0;i<4;i++)
			{
			UART_TX(data.byteArray[i]);
			}
				a=cos(0.1*n);
			data.floatValue=a;
			for(int i=0;i<4;i++)
			{
			UART_TX(data.byteArray[i]);
			}
			//TIMER_DELAYMS(10);
		}

//}
	}
}
