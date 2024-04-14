#include "PWM.h"

void PWMconfig(void)
{
	//TIME BASE CONFIG
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStucture;
	TIM_TimeBaseInitStucture.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStucture.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStucture.TIM_Period=7200;
	TIM_TimeBaseInitStucture.TIM_Prescaler=0;
	TIM_TimeBaseInitStucture.TIM_RepetitionCounter=0;
	
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStucture);
	
	//PWMCONFIG
	TIM_OCInitTypeDef PWM;
	PWM.TIM_OCMode=TIM_OCMode_PWM1;
	PWM.TIM_OutputState=TIM_OutputNState_Disable;
	PWM.TIM_OCPolarity=TIM_OCPolarity_High;
	PWM.TIM_Pulse=0;
  TIM_OC1Init(TIM3,&PWM);
	
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_CCxCmd(TIM3,TIM_Channel_1, TIM_CCx_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	// PWM ENABLE
	TIM_Cmd(TIM3,ENABLE);
}
void PWM_PULSEWITH(float CC_compare)
{
	if(CC_compare>100)
	{
		CC_compare=80;
	}
	if(CC_compare<0)
	{
		CC_compare=0;
	}
	uint16_t a= (float )CC_compare*(7200/100);
	TIM3->CCR1=a;
}
