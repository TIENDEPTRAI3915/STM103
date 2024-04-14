#include "stm32f10x.h"
#include "GPIO.h"
#include "UART.h"
#include "delay.h"
#include "string.h"
#include "ADC.h"
#include "I2C.h"
#include "SPI.h"
#include <math.h>
unsigned char const LEDcode[]=
{
0x7e,0x30,0x6d,0x79,0x33,0x5b,0x5f,0x70,0x7f,0x7b,0x01		//0..9 - DPabcdefg
};
extern volatile char USART_DATA[5];
volatile char TXfame[5]={'*',0,0,0,0};
volatile char temp;
extern uint16_t ADC_data;
extern float ADC_BUFFER[10]={0};
extern float LM35_TEMP=0;
extern float LM32_MEANTEMP=0;
extern double dolechchuan=0;
extern uint8_t timcouter=0;
extern uint8_t LM_35_disflag=0;
uint8_t LM35_Hbyte=0;
uint8_t LM35_Lbyte=0;
uint8_t TIME[5]={0};
volatile uint8_t TIMEWRITE[5]={0xFF,0xFF,0xFF,0xFF,0xFF};
volatile uint8_t 	enwrite=0;
void DMA1_Channel5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC5)==ENABLE)
	{
		if( USART_DATA[0]!='*' || USART_DATA[4]!='*'||((USART_DATA[1]+USART_DATA[2])&0xff)!=USART_DATA[3] )
		{
			if( USART_DATA[0] == ((USART_DATA[1]+ USART_DATA[2]+ USART_DATA[3]+ USART_DATA[4])&0xFF))
			{
				TIMEWRITE[0]=USART_DATA[1];
				TIMEWRITE[1]=(USART_DATA[2]>>4)&0x0F;
				TIMEWRITE[2]=((USART_DATA[2]&0x0F)<<1)|((USART_DATA[3]>>7)&0x01);
				TIMEWRITE[3]=(USART_DATA[3]&0x1F);
				TIMEWRITE[4]=USART_DATA[4];
				enwrite=1;
			}
			else if( USART_DATA[0]=='*'&& USART_DATA[1]==0&& USART_DATA[2]==0&& USART_DATA[3]==0&& USART_DATA[4]==0)
			{
				TXfame[1]=TIME[0];
        TXfame[2]=((TIME[1]&0x0F)<<4)|((TIME[2]>>1)&0x0F);
        TXfame[3]=((TIME[2]&0x01)<<7)|(TIME[3]&0x1F);
        TXfame[4]=TIME[4];
				for(int i=0;i<5;i++)
				{
					gui(TXfame[i]);
				}
			}
			else
			{
				TXfame[1]='E';
				TXfame[2]=USART_DATA[1];
				TXfame[3]=USART_DATA[2];
				TXfame[4]='*';
					for(int i=0;i<5;i++)
					{
						gui(TXfame[i]);
					}	
			}				
		}
		else
		{
				switch (USART_DATA[1]){
					case 'A':
								switch (USART_DATA[2]){
									case 'O':GPIO_ResetBits(GPIOC,GPIO_Pin_13);
													temp=(GPIOC->ODR>>13)&0x01;
													switch (temp){
														case 0:
															TXfame[1]='A';
															TXfame[2]='O';
															TXfame[3]='*';
															TXfame[4]=0;
															break;															
														case 1:
															TXfame[1]='A';
															TXfame[2]=0xFE;
															TXfame[3]='*';
															TXfame[4]=0;																												
															break;
													}
									break;
									case 'C':GPIO_SetBits(GPIOC,GPIO_Pin_13);
													temp=(GPIOC->ODR>>13)&0x01;
													switch (temp){
														case 0:
															TXfame[1]='A';
															TXfame[2]=0xFD;
															TXfame[3]='*';
															TXfame[4]=0;															
															break;
														case 1:
															TXfame[1]='A';
															TXfame[2]='C';
															TXfame[3]='*';
															TXfame[4]=0;															
															break;
													}
									break;
									default:
										TXfame[1]='A';
										TXfame[2]=0xFF;
										TXfame[3]='*';
										TXfame[4]=0;
								}
								for(int i=0;i<5;i++)
								{
									gui(TXfame[i]);
								}
					break;
					case 'B':
								switch (USART_DATA[2]){
									case 'O':GPIO_ResetBits(GPIOC,GPIO_Pin_14);
													temp=(GPIOC->ODR>>14)&0x01;
													switch (temp){
														case 0:
															TXfame[1]='B';
															TXfame[2]='O';
															TXfame[3]='*';
															TXfame[4]=0;														
															break;
														case 1:
															TXfame[1]='B';
															TXfame[2]=0xFE;
															TXfame[3]='*';
															TXfame[4]=0;	
															break;
													}
									break;
									case 'C':GPIO_SetBits(GPIOC,GPIO_Pin_14);
													temp=(GPIOC->ODR>>14)&0x01;
													switch (temp){
														case 0:
															TXfame[1]='B';
															TXfame[2]=0xFD;
															TXfame[3]='*';
															TXfame[4]=0;															
															break;
														case 1:
															TXfame[1]='B';
															TXfame[2]='C';
															TXfame[3]='*';
															TXfame[4]=0;															
															break;
													}
									break;
									default:
										TXfame[1]='B';
										TXfame[2]=0xFF;
										TXfame[3]='*';
										TXfame[4]=0;
								}
								for(int i=0;i<5;i++)
								{
									gui(TXfame[i]);
								}
					break;
					case 'T':
								if(LM_35_disflag==0)
								{
								switch (USART_DATA[2]){
									case 'G'://ADC
										TXfame[1]='T';
										TXfame[2]=LM32_MEANTEMP/1;
										TXfame[3]=LM32_MEANTEMP*100-TXfame[2]*100;
										TXfame[4]='*';
									for(int i=0;i<5;i++)
										{
											gui(TXfame[i]);
										}
										break;
									default:
										TXfame[1]='T';
										TXfame[2]=0xFF;
										TXfame[3]=0xFF;
										TXfame[4]='*';
									for(int i=0;i<5;i++)
										{
											gui(TXfame[i]);
										}	
									}									
								}
								else
								{
										TXfame[1]='T';
										TXfame[2]=0xFD;
										TXfame[3]=0xFD;
										TXfame[4]='*';
									for(int i=0;i<5;i++)
										{
											gui(TXfame[i]);
										}
								}
					break;
					case 'N':
						if(USART_DATA[2]=='N')
						{
										TXfame[1]='N';
										TXfame[2]='O';
										TXfame[3]='K';
										TXfame[4]='*';
						}
								for(int i=0;i<5;i++)
								{
									gui(TXfame[i]);
								}
					break;			
					default:
					TXfame[1]='E';
					TXfame[2]=USART_DATA[1];
					TXfame[3]=USART_DATA[2];
					TXfame[4]='*';
					for(int i=0;i<5;i++)
					{
						gui(TXfame[i]);
					}
				}
		}
	}
	DMA_ClearITPendingBit( DMA1_IT_TC5 );
}

void USART1_IRQHandler(void)
{		 
		if(USART_GetITStatus(USART1,USART_IT_IDLE)==SET)
		{
					DMA_Cmd(DMA1_Channel5, DISABLE);
					DMA_SetCurrDataCounter(DMA1_Channel5,5);
					DMA_Cmd(DMA1_Channel5, ENABLE);	     	
		}
	temp=USART1->SR;
	temp=USART1->DR;// xoa co IDLE
}


void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
	{

		ADC_BUFFER[timcouter]=ADC_data*3*100/4096;
		LM35_TEMP+=ADC_BUFFER[timcouter];
		timcouter++;
		if(timcouter>=10)
		{		
				timcouter=0;
				LM32_MEANTEMP=LM35_TEMP/10;
				dolechchuan=0;
				for(int i=0;i<10;i++)
				{
					dolechchuan+=(LM32_MEANTEMP-ADC_BUFFER[i])*(LM32_MEANTEMP-ADC_BUFFER[i]);
				}
				dolechchuan=sqrt(dolechchuan)/10;
				if(dolechchuan<3)
				{
					LM_35_disflag=0;
				}
				else
				{
					LM_35_disflag=1;
				}
				LM35_TEMP=0;
				uint16_t TEMPDIS=(LM32_MEANTEMP*100);
			 SPI_led_write(0,0x08);
			 SPI_led_write(0,0x07);		
			 SPI_led_write(0,0x06);		
			 SPI_led_write(0,0x05);
			 SPI_led_write(LEDcode[TEMPDIS/1000],0x04);		
				TEMPDIS%=1000;
			 SPI_led_write(LEDcode[TEMPDIS/100]|1<<7,0x03);	
				TEMPDIS%=100;			
			 SPI_led_write(LEDcode[TEMPDIS/10],0x02);		
			 SPI_led_write(	LEDcode[TEMPDIS%10]	,0x01);	
		}

	}
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
}

extern volatile uint32_t sys;
void systick_config(void)
{
	SysTick_Config(SystemCoreClock/1000);
	SysTick->CTRL&=~(1<<0);
}
void sysdelay(uint32_t count)
{
	sys=count;
	SysTick->VAL=0;
	SysTick->CTRL|=(1<<0);
	while(sys)
		;
}
void SysTick_Handler(void)
{
	sys++;
	//GPIOC->ODR^=(1<<14);//test systick inter
}

int g=0;
int main(){
	GPIOinit();
	delayinit();
	UARTinit();
	ADC_CONFIG();
	I2Cinit();
	SPIinit();
	SPI_led_write(0x01,0x0C); //Normal Operation
	SPI_led_write(0x00,0x0A); //Intensity (do sang)
	SPI_led_write(0x07,0x0B); //All digits on
	SPI_led_write(0x00,0x09); //Decoding
	SPI_led_write(0x00,0x0F); //Display test off

		while(1)
		{
			g=I2C_EEPROMREAD(0X50,0,TIME,5);
			I2C_GenerateSTOP(I2C1,DISABLE);
			I2C_GenerateSTOP(I2C1,ENABLE);
			if(enwrite==1)
			{
				enwrite=0;
				g=I2C_EEPROMWRITE(0x50,0,TIMEWRITE,5);
				delayms(10);
			}
//test usart
//		GPIOC->ODR ^=(1<<13);
//			delayms(1000);		
//			guichu("chao");
		}
}
