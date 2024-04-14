#include "main.h"
#define ENCODER_RE 100;
typedef union {
  float floatValue;
  uint8_t byteArray[4];
} FloatByteArray;
int u=0;
int piden=0;
		volatile  FloatByteArray PWM;
		volatile  FloatByteArray data;	
		
	


float K_p = 0.04; 	//k_p = 0.28
float K_i = 0.02; //k_i = 0.0001
float K_d = 0.0001; 		//k_d = 0.1

// PID_related
//float e_dot = 0;
//float error_I = 0;

//float pre_control_signal = 0; // pre u
//float pre_previous_error = 0; // for calculating the derivative (edot)

float error_value = 0; //error
float previous_error = 0; //for calculating the derivative (edot)
float P_part, I_part, D_part;
float control_signal = 0; //u - Also called as process variable (PV)
float T = 0.005; // control period in s (1ms)
volatile float pre_mathlab=0;




uint8_t dir = 0; //Bit huong xoay 
//float speed,speedset;
volatile float SpeedSet=600;
volatile float t=0.01;
volatile float ek,ek_1,ek_2,uk,uk_1,Kp,Ki,Kd;



void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update))
	{
		//data.floatValue=(float)((int16_t)TIM2->CNT)*15;
		//plain A
		
		data.floatValue=((int16_t)(TIM2->CNT))*30;
		TIM2->CNT=0;
		//GPIOC->ODR^=(1<<13);//test interupt
		
		if(piden==1)
		{
			ek_2=ek_1;
			ek_1=ek;
			ek=SpeedSet-data.floatValue;
			uk_1=uk;
			uk=uk_1+Kp*(ek-ek_1)+Ki*(T/2)*(ek+ek_1)+Kd/T*(ek-2*ek_1+ek_2);
			if(uk<0)
			{
				uk=0;
			}
			if(uk>80)
			{
				uk=80;
			}
			
			PWM_PULSEWITH(uk);
			PWM.floatValue=uk;
			
		for(int i=0;i<4;i++)
		{
			gui(data.byteArray[i]);
		}
				for(int i=0;i<4;i++)
		{
			gui(PWM.byteArray[i]);
		}
		
		}
		else
		{
			PWM_PULSEWITH(0);
			ek=0;
			ek_1=0;
			ek_2=0;
			uk=0;
			uk_1=0;
		}




//		if(piden==1)
//		{			
//		error_value =   SpeedSet - data.floatValue ; 
//		P_part = (K_p*error_value);
//		I_part = I_part + K_i*error_value*T;
//		D_part = K_d*(error_value - previous_error)/T;
//		control_signal = P_part + D_part + I_part;
//		previous_error = error_value;
//			PWM.floatValue=control_signal;
//		PWM_PULSEWITH(control_signal);
//			
//		for(int i=0;i<4;i++)
//		{
//			gui(data.byteArray[i]);
//		}
//				for(int i=0;i<4;i++)
//		{
//			gui(PWM.byteArray[i]);
//		}
//		
//		}
//		else
//		{
//		PWM_PULSEWITH(0);
//		I_part=0;
//		error_value=0;
//		D_part=0;
//		control_signal=0;
//		previous_error=0;
//		}
			
//			for(int i=0;i<4;i++)
//			{
//				if((data.floatValue<=5000)&&data.floatValue>=0)
//				{
//				gui(data.byteArray[i]);
//					pre_mathlab=data.floatValue;
//				}
//				else
//				{
//					data.floatValue=pre_mathlab;
//					gui(data.byteArray[i]);
//				}
//				
//			}

//			pre_mathlab=data.floatValue;
			
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}

int main()
{
	GPIOinit();
	UARTinit();
	PWMconfig();
	encodeconfig();
	delayinit();
//PWM_PULSEWITH(0);
	//PWM_PULSEWITH(50);
	while(1)
	{
		//plain A
//		nhanchu(PWM.byteArray,4);
//	//	SpeedSet=PWM.floatValue;
//		PWM_PULSEWITH(PWM.floatValue);
//		
//		piden=1;
		
//					for(int i=0;i<4;i++)
//			{
//				gui(data.byteArray[i]);
//			}
		//plain B

//		nhanchu(PWM.byteArray,4);
//		PWM_PULSEWITH(PWM.floatValue);

		//delayms(100);
		
		//PLAN A
//			for(int i=0;i<4;i++)
//			{
//				gui(data.byteArray[i]);
//			}
			//PLANB
//						for(int i=0;i<4;i++)
//			{
//				gui(data.byteArray[i]);
//			}
//			delayms(100);




	}
	
}
