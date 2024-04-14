#include "UART.h"
char USART_DATA[5]={0};


void UARTinit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA;
	
	//DMA config
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA.DMA_BufferSize=5;
	DMA.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA.DMA_M2M=DMA_M2M_Disable;
	DMA.DMA_MemoryBaseAddr=(uint32_t)USART_DATA;
	DMA.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA.DMA_Mode=DMA_Mode_Circular;
	DMA.DMA_PeripheralBaseAddr=(uint32_t)(&(USART1->DR));
	DMA.DMA_PeripheralDataSize=DMA_MemoryDataSize_Byte;
	DMA.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA.DMA_Priority=DMA_Priority_VeryHigh;
	
	DMA_Init(DMA1_Channel5,&DMA);
	
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	//USART CONFIG
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO, ENABLE);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

	USART_ClearFlag(USART1,USART_IT_TC);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	  USART1->CR1|=(1<<4);
	DMA_Cmd(DMA1_Channel5,ENABLE);
	USART_Cmd(USART1, ENABLE);

//-----------------------------------------------------------------------------

//		//GPIO_InitTypeDef    GPIO_InitStructure;
//    USART_InitTypeDef   USART_InitStructure;
//		DMA_InitTypeDef 		DMA_InitStructure;
//		NVIC_InitTypeDef 		NVIC_InitStructure;
//	
//		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE ); /*enable clock*/
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );
//	
//		//DMA Config
//		
//		DMA_DeInit(DMA1_Channel5);
//		DMA_InitStructure.DMA_BufferSize=4;
//		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
//		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART_DATA;
//		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//		DMA_InitStructure.DMA_BufferSize = sizeof(USART_DATA);
//		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//		DMA_Init(DMA1_Channel5, &DMA_InitStructure);

//		/* Enable DMA1 channel5 */
//		DMA_Cmd(DMA1_Channel5, ENABLE);
//    /* 
//        USART1_Rx : PA10  input floating 
//        USART1_Tx : PA9  alternate function push-pull
//    */	
////    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
////    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
////    GPIO_Init(GPIOA, &GPIO_InitStructure );

////    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
////    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
////    GPIO_Init(GPIOA, &GPIO_InitStructure );

//    USART_InitStructure.USART_BaudRate = 9600;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//    USART_Init(USART1, &USART_InitStructure);
//		
//		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
//		
//		DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
//		
//		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn ;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
//		
//		
//		//Ket noi DMA va USART_RX
//		USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
//		
//		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
//    USART_Cmd(USART1, ENABLE); //kich hoat usart1
	
}

void gui(uint8_t c)
{
while( USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET)
	;
	USART_SendData(USART1,c);
while( USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET)
	;
	}
void guichu(char *text)
{
	while(*text) gui(*text++);
}
