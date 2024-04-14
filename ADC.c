#include "ADC.h"
uint16_t ADC_data;
void ADC_CONFIG(void){
	ADC_InitTypeDef adc;
	DMA_InitTypeDef dma;
	//RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	//dmasetup
	dma.DMA_BufferSize=1;
	dma.DMA_DIR=DMA_DIR_PeripheralSRC;
	dma.DMA_M2M=DMA_M2M_Disable;
	dma.DMA_MemoryBaseAddr=(uint32_t)&ADC_data;
	dma.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	dma.DMA_MemoryInc=DMA_MemoryInc_Disable;
	dma.DMA_Mode=DMA_Mode_Circular;
	dma.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;
	dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	dma.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Channel1,&dma);
	//adc setup
	adc.ADC_ContinuousConvMode= ENABLE ;
	adc.ADC_DataAlign=ADC_DataAlign_Right;
	adc.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	adc.ADC_Mode=ADC_Mode_Independent;
	adc.ADC_NbrOfChannel=1;
	adc.ADC_ScanConvMode=ENABLE;
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_239Cycles5);

	ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Init(ADC1,&adc);
	DMA_Cmd(DMA1_Channel1,ENABLE);
	ADC_Cmd(ADC1,ENABLE);
//	ADC_StartCalibration(ADC1);

//	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}
