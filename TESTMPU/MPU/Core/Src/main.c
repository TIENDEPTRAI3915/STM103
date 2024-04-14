/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include <string.h>
#include "mpu9250.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4;
//TIM_HandleTypeDef htim5;
//TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
//Variables for transmit data
typedef union {
    float floatValue;
    uint8_t byteArray[4];
} 	FloatByteArray;

FloatByteArray data;
///////////////

//Variable for PID control
//Bo cu
//Kp = 0.02
//Ki = 0.02
//Kp = 0.0001
volatile uint8_t On = 0;
volatile float Kp,Ki,Kd,T = 0.005;
volatile float P_part = 0, I_part = 0, D_part = 0;
volatile float Error = 0, pre_Error, pre_pre_Error = 0;
volatile float pre_out, Out;
volatile float speed,speedset;
volatile uint8_t Resolution = 100,mode = 4;
//Variable for PWM control

//Varialble for UART 
uint8_t DataToSend[36];
///////////
//Uart DMA Data
#define RX_BUFF_SIZE 36
#define TX_BUFF_SIZE 36
uint8_t Rx_buff[RX_BUFF_SIZE];
uint8_t Tx_buff[TX_BUFF_SIZE];
uint8_t main_buff[RX_BUFF_SIZE];

//MPU9250 variables 
//I2C Variable
imu_9250_t* imu_9250_0;
//Struct_Angle Angle;
uint16_t error;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_TIM3_Init(void);
//static void MX_TIM5_Init(void);
//static void MX_TIM4_Init(void);
//static void MX_TIM9_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//PID calculation
//uint8_t PID_Crl(float SpeedSet, float Speed) {
//		Error = SpeedSet - Speed;
//		P_part = Kp*(Error - pre_Error);
//		I_part = 0.5*Ki*T*(Error + pre_Error);
//		D_part = Kd/T*( Error - 2*pre_Error+ pre_pre_Error);
//		Out = pre_out + P_part + I_part + D_part ;
//		pre_pre_Error = pre_Error;
//		pre_Error = Error;
//		pre_out = Out; 
//	
//		if(Out > 100) {DataToSend[9] = 97; return 97;}
//		else if(Out < 0) {DataToSend[9] = 3 ;return 3;}
//		else {DataToSend[9] = Out ;return Out;}
//}

////Left motor control
//void PWM_Left_Motor(uint8_t PWM_Val) {
//		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PWM_Val);
//		//TIM3->CCR1 = PWM_Val;
//}
////Right motor control
//void PWM_Right_Motor(uint8_t PWM_Val) {
//		//TIM3->CCR2 = PWM_Val;
//		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,PWM_Val);
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
//  MX_TIM2_Init();
//  MX_TIM3_Init();
//  MX_TIM5_Init();
//  MX_TIM4_Init();
//  MX_TIM9_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  imu_9250_0 = IMU_9250_Create();
  calibrateGyro(imu_9250_0, 500);

	
	//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

	//HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
	//HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
	//__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,50);
	//HAL_TIM_Base_Start_IT(&htim4);
//	HAL_TIM_Base_Start_IT(&htim9);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rx_buff, RX_BUFF_SIZE); // start uart2 rx dma idle 
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	uint8_t i = 0;
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
			//Tranfer data every 10ms
			
			imu_9250_0->get_data(imu_9250_0);
			HAL_Delay(10);
			//0->3
			data.floatValue = imu_9250_0->pt1_p.acc_x*0.86;
			for(i = 0; i < 4; i++) {
				DataToSend[i] = data.byteArray[i];
			}
			//4->7
			data.floatValue = imu_9250_0->pt1_p.acc_y*0.875;
			for(i = 0; i < 4; i++) {
				DataToSend[i+4] = data.byteArray[i];
			}
			//8->11
			data.floatValue = imu_9250_0->pt1_p.acc_z*1.452;
			for(i = 0; i < 4; i++) {
				DataToSend[i+8] = data.byteArray[i];
			}
			//12->15
			data.floatValue = imu_9250_0->pt1_p.gyro_x;
			for(i = 0; i < 4; i++) {
				DataToSend[i+12] = data.byteArray[i];
			}
			//16->19
			data.floatValue = imu_9250_0->pt1_p.gyro_y;
			for(i = 0; i < 4; i++) {
				DataToSend[i+16] = data.byteArray[i];
			}
			//20->23
			data.floatValue = imu_9250_0->pt1_p.gyro_z;
			for(i = 0; i < 3; i++) {
				DataToSend[i+20] = data.byteArray[i];
			}
			//24->27
			data.floatValue = imu_9250_0->pt1_p.mag_x;
			for(i = 0; i < 4; i++) {
				DataToSend[i+24] = data.byteArray[i];
			}
			//28->31
			data.floatValue = imu_9250_0->pt1_p.mag_y;
			for(i = 0; i < 4; i++) {
				DataToSend[i+28] = data.byteArray[i];
			}
			//32->35
			data.floatValue = imu_9250_0->pt1_p.mag_z;
			for(i = 0; i < 4; i++) {
				DataToSend[i+32] = data.byteArray[i];
			}
			HAL_UART_Transmit(&huart2,DataToSend,36,100);

		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM2_Init(void)
//{

//  /* USER CODE BEGIN TIM2_Init 0 */

//  /* USER CODE END TIM2_Init 0 */

//  TIM_Encoder_InitTypeDef sConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  /* USER CODE BEGIN TIM2_Init 1 */

//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 0;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 4294967295;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
//  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = 0;
//  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC2Filter = 0;
//  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */

//  /* USER CODE END TIM2_Init 2 */

//}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
//  */
//static void MX_TIM3_Init(void)
//{

//  /* USER CODE BEGIN TIM3_Init 0 */

//  /* USER CODE END TIM3_Init 0 */

//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};

//  /* USER CODE BEGIN TIM3_Init 1 */

//  /* USER CODE END TIM3_Init 1 */
//  htim3.Instance = TIM3;
//  htim3.Init.Prescaler = 99;
//  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim3.Init.Period = 99;
//  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM3_Init 2 */

//  /* USER CODE END TIM3_Init 2 */
//  HAL_TIM_MspPostInit(&htim3);

//}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM4_Init(void)
//{

//  /* USER CODE BEGIN TIM4_Init 0 */

//  /* USER CODE END TIM4_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  /* USER CODE BEGIN TIM4_Init 1 */

//  /* USER CODE END TIM4_Init 1 */
//  htim4.Instance = TIM4;
//  htim4.Init.Prescaler = 999;
//  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim4.Init.Period = 499;
//  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM4_Init 2 */

//  /* USER CODE END TIM4_Init 2 */

//}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM5_Init(void)
//{

//  /* USER CODE BEGIN TIM5_Init 0 */

//  /* USER CODE END TIM5_Init 0 */

//  TIM_Encoder_InitTypeDef sConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  /* USER CODE BEGIN TIM5_Init 1 */

//  /* USER CODE END TIM5_Init 1 */
//  htim5.Instance = TIM5;
//  htim5.Init.Prescaler = 0;
//  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim5.Init.Period = 4294967295;
//  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
//  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = 0;
//  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC2Filter = 0;
//  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM5_Init 2 */

//  /* USER CODE END TIM5_Init 2 */

//}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM9_Init(void)
//{

//  /* USER CODE BEGIN TIM9_Init 0 */

//  /* USER CODE END TIM9_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

//  /* USER CODE BEGIN TIM9_Init 1 */

//  /* USER CODE END TIM9_Init 1 */
//  htim9.Instance = TIM9;
//  htim9.Init.Prescaler = 999;
//  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim9.Init.Period = 999;
//  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM9_Init 2 */

//  /* USER CODE END TIM9_Init 2 */

//}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//	
//	if(On) {
//		if(htim->Instance == TIM4) {
//		//Speed measure every 5ms and PID calculation
////			speed = (int16_t)TIM5->CNT/mode/T/Resolution*60;
////			TIM5->CNT = 0;
////			TIM4->CNT = 0;
//			//PWM_Left_Motor(PID_Crl(speedset,speed));
//		}
//		if(htim->Instance == TIM9) {
//			//
//		}
//	}
//}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//		if(huart->Instance == huart2.Instance)
//		{
//		//Uart Rx receive data
//			memcpy(main_buff, Rx_buff, Size); // get data
//			if (main_buff[0] == 0xEE && main_buff[17] == 0xEF) // check header end stop byte
//			{
//				if(main_buff[1] == 0xFF && main_buff[16] == 0xFF) {
//				//Start PID control
//						On = 1;
//						for(uint8_t i = 0; i < 18; i++) {
//								main_buff[i] = 0;
//						}
//				}
//				else if(main_buff[1] == 0xCC && main_buff[16] == 0xCC) {
//				//Start PID control
//						On = 0;
//						for(uint8_t i = 0; i < 18; i++) {
//								main_buff[i] = 0;
//						}
//				}
//				else {
//						//Renew SpeedSet, Kp, Ki, Kd
//						for(uint8_t i = 0; i < 4; i++) {
//								data.byteArray[i] = main_buff[i+1];
//						}
//						speedset = data.floatValue;
//						
//						for(i = 0; i < 4; i++) {
//								data.byteArray[i] = main_buff[i+5];
//						}
//						Kp = data.floatValue;
//						
//						for(i = 0; i < 4; i++) {
//								data.byteArray[i] = main_buff[i+9];
//						}
//						Ki = data.floatValue;
//						
//						for(i = 0; i < 4; i++) {
//								data.byteArray[i] = main_buff[i+13];
//						}
//						Kd = data.floatValue;
//				}
//			}
//			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rx_buff, RX_BUFF_SIZE); // start uart rx DMA again
//		}
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
