#include "stm32f10x.h"
#include "GPIO.h"
#include "UART.h"
#include "delay.h"
#include "I2C.h"
#include "string.h"
#include "encode.h"
#include <math.h>
char ten[16]="NGUYEN PHU TIEN";
char USARTREAD[30]={0,0,0};
char command[8]="COMMAND";
char nhap[13]="NHAP PASS:";
char ok[3]="OK";
char notok[9]="SAI PASS";
char dulieu[5]="DATA";
char mode[5]="MODE";
char clear[6]="CLEAR";
char hang1[6]="HANG1";
char hang2[6]="HANG2";
char wrongcommand[5]="EROR";
char help[5]="HELP";
char send[5]="SEND";
uint8_t modebit=1;
char PASS[6]={'2','8','1','1','0','3'};
int USARTREADCNT=0;
volatile int passcheck=0;
//----------------------------------------
extern volatile char USART_DATA[4];

//void DMA1_Channel5_IRQHandler(void)
//{
//	if(DMA_GetITStatus(DMA1_IT_TC5)==ENABLE)
//	{
//		if( USART_DATA[0]!='*' || USART_DATA[3]!='*' )
//		{
//			
//		}
//		else
//		{
//				switch (USART_DATA[1]){
//				case 'A':
//					GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//					break;
//				case 'B':
//					GPIO_SetBits(GPIOC,GPIO_Pin_13);
//					break;
//				case 'C':
//					break;
//				default:
//					guichu("chao");	
//				}
//		}

//	}
//	DMA_ClearITPendingBit( DMA1_IT_TC5 );
//}

//void USART1_IRQHandler(void)
//{
//		if(USART_GetITStatus(USART1,USART_IT_IDLE)==ENABLE)
//		{
//					DMA_Cmd(DMA1_Channel5, DISABLE);
//					DMA_SetCurrDataCounter(DMA1_Channel5,4);
//		}
//		DMA_Cmd(DMA1_Channel5, ENABLE);
//		USART_ClearITPendingBit(USART1,USART_IT_IDLE);	
//}

//int main(){
//	GPIOinit();
//	UARTinit();
//	delayinit();
//	guichu("chao");
//		while(1);
//}

















//------------------------------------------------------------------
//void USART1_IRQHandler(void)
//{
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)==ENABLE)
//	{
//	USARTREAD[USARTREADCNT++]=USART_ReceiveData(USART1);
//		if((USARTREAD[USARTREADCNT-2])==0x0D&&(USARTREAD[USARTREADCNT-1]==0x0A))
//			{
//				for(int i=USARTREADCNT-2;i<30;i++)
//					{
//					USARTREAD[i]=0;
//					}
//			USARTREADCNT=0;
//						if(passcheck)
//							{
//								GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//								if(strcmp(USARTREAD,command)==0)
//								{
//										guichu(command);
//										guichu(mode);
//										gui('\n');
//										modebit=3;
//								}
//								else if(strcmp(USARTREAD,dulieu)==0)
//								{
//										guichu(dulieu);
//										guichu(mode);
//										gui('\n');
//										modebit=4;
//								}
//								else if(strcmp(USARTREAD,help)==0)
//								{
//										guichu(command);
//										gui(' ');
//										gui('L');
//										gui('I');
//										gui('S');
//										gui('T');
//										gui('\n');
//										guichu(clear);
//										gui('\n');
//										guichu(hang1);
//										gui('\n');
//										guichu(hang2);
//										gui('\n');
//								}
//								if(modebit==0)
//								{				if(strcmp(USARTREAD,clear)==0)
//													{
//														WriteLCD(0x4E,modebit,0x01);
//													}
//													else if(strcmp(USARTREAD,hang1)==0)
//													{
//														WriteLCD(0x4E,modebit,0x80);
//													}
//													else if(strcmp(USARTREAD,hang2)==0)
//													{
//														WriteLCD(0x4E,modebit,0xC0);
//													}
//													else
//													{
//														guichu(wrongcommand);
//														gui('\n');
//														guichu(send);
//														gui(' ');
//														guichu(help);
//														gui('\n');
//													}
//								}
//								else if(modebit==1) 
//								{
//									for(int i=0;i<strlen(USARTREAD);i++)
//										{
//											WriteLCD(0x4E,modebit,USARTREAD[i]);								
//										}

//								}
//								if(modebit==3)
//								{
//									modebit=0;
//								}
//								if(modebit==4)
//								{
//									modebit=1;
//								}
//	}
//					if(passcheck==0)
//					{	for(int i=0;i<6;i++)
//							{
//							if(USARTREAD[i]!=PASS[i])
//								{
//								guichu(notok);
//								gui('\n');
//								guichu(nhap);
//								break;
//								}
//							if(i==5)
//							{
//							passcheck=1;
//							guichu(ok);
//							gui('\n');
//							guichu(dulieu);
//							gui('O');
//							gui('R');
//							guichu(command);
//							gui('O');
//							gui('R');
//							guichu(help);								
//							gui('?');
//							gui('\n');
//							}		
//							}
//					}

//			}
//					USART_ClearITPendingBit(USART1,USART_IT_RXNE);
//	}
//}

//void EXTI0_IRQHandler(void)
//{
//	if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13)==1)
//	{
//	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//	}
//	else
//	{
//	GPIO_SetBits(GPIOC,GPIO_Pin_13);	
//	}
//	//gui uart
//	guichu(ten);
//	EXTI_ClearITPendingBit(EXTI_Line0); 
//}


//int main()
//{RCC_ClocksTypeDef rcc;
//		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStucture;
//	GPIOinit();
//	GPIO_SetBits(GPIOC,GPIO_Pin_13);
//	UARTinit();
//	delayinit();
//	I2Cinit();
//	LCDIni();
//	guichu(nhap);
////clock	
//RCC_GetClocksFreq  (&rcc);
//volatile uint32_t a=rcc.ADCCLK_Frequency;
//volatile uint32_t b=	rcc.HCLK_Frequency;
//volatile uint32_t c=	rcc.PCLK1_Frequency;
//volatile uint32_t d=	rcc.PCLK2_Frequency;
//volatile uint32_t e=	rcc.SYSCLK_Frequency;
//	while(1)
//	{
//		if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13)==1)
//	{
//	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//	}
//	else
//	{
//	GPIO_SetBits(GPIOC,GPIO_Pin_13);	
//	}
//	delayms(1000);

//	TIM_TimeBaseInitStucture.TIM_ClockDivision=TIM_CKD_DIV1;
//	TIM_TimeBaseInitStucture.TIM_CounterMode=TIM_CounterMode_Up ;
//	TIM_TimeBaseInitStucture.TIM_Period=0xffff;
//	TIM_TimeBaseInitStucture.TIM_Prescaler=719;
//	TIM_TimeBaseInitStucture.TIM_RepetitionCounter=0;
//	
//   TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStucture);
//  }
//}
//void TIM2_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM2,TIM_IT_CC1)!=RESET)
//	{
//	 TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);
//	}
//	if(TIM_GetITStatus(TIM2,TIM_IT_Trigger)!=RESET)
//	{
//	 TIM_ClearITPendingBit(TIM2,TIM_IT_Trigger);
//	}
//}
double tocdo;
double tam;
#define dodai 6
uint16_t data[dodai];
uint16_t giatoc[6];
double tocdox;
double tocdoy;
double tocdoz;
double giatocx;
double giatocy;
double giatocz;
float AccErrorX,AccErrorY,AccErrorZ,GyroErrorX,GyroErrorY,GyroErrorZ;
void MPU_Error_Cal(void) {
		uint8_t dem;
		for(dem = 0; dem < 200; dem++) {
			I2C_Read1(0x68,0x3B,data,dodai);//16384,131
			giatocx=((int16_t)(data[0]<<8|data[1]))/16384.0;
			giatocy=((int16_t)(data[2]<<8|data[3]))/16384.0;
			giatocz=((int16_t)(data[4]<<8|data[5]))/16384.0;
			I2C_Read1(0x68,0x43,giatoc,6);
			tocdox=((int16_t)(giatoc[0]<<8|giatoc[1]))/131.0;
			tocdoy=((int16_t)(giatoc[2]<<8|giatoc[3]))/131.0;
			tocdoz=((int16_t)(giatoc[4]<<8|giatoc[5]))/131.0;
			AccErrorX += giatocx;
			AccErrorY += giatocy;
			AccErrorZ += giatocz;
			GyroErrorX += tocdox;
			GyroErrorY += tocdoy;
			GyroErrorZ += tocdoz;
		}
		AccErrorX/=200;
		AccErrorY/=200;
		AccErrorZ/=200;
		GyroErrorX/=200;
		GyroErrorY/=200;
		GyroErrorZ/=200;
}
double angleAccX,angleAccY, angleGyroX, angleGyroY, angleGyroZ,angleX,angleY,angleZ;
int main(){
	GPIOinit();
	//encodeconfig();
	delayinit();
	I2Cinit();
		I2C_Write1(0x68,0x6B,1<<7);
	delayms(1000);
	I2C_Write1(0x68,0x6B,0);
	I2C_Write1(0x68,0x19,0x07);
	I2C_Write1(0x68,0x1C,0);
	I2C_Write1(0x68,0x1B,0);
//	MPU_Error_Cal();
	while(1)
	{	
		I2C_Read1(0x68,0x3B,data,dodai);//16384,131
	giatocx=((int16_t)(data[0]<<8|data[1]))/16384.0;
	giatocy=((int16_t)(data[2]<<8|data[3]))/16384.0;
	giatocz=((int16_t)(data[4]<<8|data[5]))/16384.0;
	I2C_Read1(0x68,0x43,giatoc,6);
	tocdox=((int16_t)(giatoc[0]<<8|giatoc[1]))/131.0-GyroErrorX;
	tocdoy=((int16_t)(giatoc[2]<<8|giatoc[3]))/131.0-GyroErrorY;
	tocdoz=((int16_t)(giatoc[4]<<8|giatoc[5]))/131.0-GyroErrorZ;
//		
//	angleAccX = atan2(giatocy, sqrt(giatocz * giatocz + giatocx * giatocx)) * 360 / 2.0 /3.14;
//  angleAccY = atan2(giatocx, sqrt(giatocz * giatocz + giatocy * giatocy)) * 360 / -2.0 /3.14;


//  angleGyroX += tocdox * 0.001;
//  angleGyroY += tocdoy * 0.001;
//  angleGyroZ += tocdoz * 0.001;

//  angleX = ( 0.98f * (angleX + tocdox * 0.001)) + (0.02f * angleAccX);
//  angleY = ( 0.98f * (angleY + tocdoy * 0.001)) + (0.02f * angleAccY);
//  angleZ = angleGyroZ;

		
		GPIOC->ODR^=(1<<13);
		
		delayms(1000);
//	TIM2->CNT=0;
//	delayms(500);
//	tam=TIM2->CNT;
//	tocdo=(tam/18)*30;
	}
}
