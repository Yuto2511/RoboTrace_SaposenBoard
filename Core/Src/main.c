/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM05 0.5
#define TIM005 0.05
#define TIRE 14.0
#define V_MAX 1.2
#define V_MIN 0.8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
int _write(int, char*, int);
void user_init();
void loop_init();
int start_stop();
void side_sens();
void adc_getValues();
void Sens_Calibration();
void updateValues();
double degree();
void LineTrace(uint8_t);
void motor_R(double);
void motor_L(double);
double Speed_R();
double Speed_L();
double SpeedContorol_R(double Speed_Ref);
double SpeedContorol_L(double Speed_Ref);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//ADC
uint16_t ADC1_Sensor_Values[4];
uint16_t ADC2_Sensor_Value[1];

float Line_Sens_1;
float Line_Sens_2;
float Line_Sens_3;
float Line_Sens_4;
float Line_Sens_5;

uint16_t line_1_[10];
uint16_t line_2_[10];
uint16_t line_3_[10];
uint16_t line_4_[10];
uint16_t line_5_[10];

float Line_Calib_[5];

float Line_Sort_[5][5];
float ratio_buf;

float Line_1_Max;
float Line_1_Min;
float Line_2_Max;
float Line_2_Min;
float Line_3_Max;
float Line_3_Min;
float Line_4_Max;
float Line_4_Min;
float Line_5_Max;
float Line_5_Min;

//Motor
double SR;
double SL;
double speedR_I_buff;
double speedL_I_buff;
double Speed_Ref = 1.0;
double Speed_input_R;
double Speed_input_L;
double tread = 40;

//else
uint16_t num;
uint8_t linetrace_flag;
int time;
int side_r_time, side_l_time;
int side_r_flag, side_l_flag;
int side_r_count;
uint8_t cross_flag;
uint8_t start_flag;
uint8_t stop_flag;


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
  setbuf(stdout, NULL);
  user_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  //Motor
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //buzzer
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  //Encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


  //timer
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);


  //ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC1_Sensor_Values, 4);
  hdma_adc1.Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *) ADC2_Sensor_Value, 1);
  hdma_adc2.Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  Sens_Calibration();

	  if( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5) == 1 ){
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 500); //R
		  printf("Hight\r\n");
	  }
	  else if( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5) == 0 ) __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);

	  if( HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1) == 0 ) start_flag = 1;
	  */

	  if( HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1) == 0 ){
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 500);
		  HAL_Delay(1500);
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
		  Sens_Calibration();
		  start_flag = 1;
	  }

	  if(start_flag > 0){
		  loop_init();
		  HAL_Delay(4000);
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 500);
		  HAL_Delay(1000);
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
		  linetrace_flag = 1;
		  cross_flag = 0;
	  }

	  while(linetrace_flag){
		  stop_flag = start_stop();
		  //printf("%d\r\n", stop_flag);
		  if(start_flag > 5){
			  start_flag = linetrace_flag = 0;
			  break;
		  }
		  if(stop_flag >= 2){
			  linetrace_flag = 2;
			  start_flag++;
			  break;
		  }
		  HAL_Delay(1.5);
	  }

	  HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 11;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 15;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 199;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 799;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//printf
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 0xFFFF);
	return len;
}

//Init
void user_init()
{
	linetrace_flag = 0;
	num = 0;
	Line_1_Max = Line_2_Max = Line_3_Max = Line_4_Max = Line_5_Max = 0;
	Line_1_Min = Line_2_Min = Line_3_Min = Line_4_Min = Line_5_Min = 1000;
	side_r_time = side_l_time = 0;
	side_r_flag = side_l_flag = 0;
	side_r_count = 0;
	cross_flag = 0;
	start_flag = stop_flag = 0;
}

void loop_init()
{
	side_r_time = side_l_time = 0;
	side_r_flag = side_l_flag = 0;
	side_r_count = 0;
	cross_flag = 0;
	stop_flag = 0;
}

int start_stop()
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1){
		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 500);
		side_r_flag = 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) != 1) side_r_flag = 0;

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1){
		side_l_flag = 1;
		cross_flag = 0;
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != 1) side_l_flag = 0;

	if(!side_r_flag && !side_l_flag && !cross_flag){
		if( side_r_time - side_l_time > 0 ) side_r_count += 1;
		else if( side_l_time - side_r_time >= 20);
		side_r_time = side_l_time = 0;
		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
	}
	else if(cross_flag){
		side_r_flag = side_l_flag = 0;
		side_r_time = side_l_time = 0;
	}

	if(side_r_count >= 2) return 2;
	else if(side_r_count >= 1) return 1;
	else return 0;
}

//ADC_Vaulues_Get
void adc_getValues()
{
	if (num >= 9) num = 0;
	line_1_[num] = ADC2_Sensor_Value[0];
	line_2_[num] = ADC1_Sensor_Values[2];
	line_3_[num] = ADC1_Sensor_Values[3];
	line_4_[num] = ADC1_Sensor_Values[1];
	line_5_[num] = ADC1_Sensor_Values[0];
	num++;
}

//Sensor_Calibration
void Sens_Calibration()
{
	/*
	if(Line_Sens_1 <= Line_1_Min) Line_1_Min = Line_Sens_1;
	if(Line_Sens_1 >= Line_1_Max) Line_1_Max = Line_Sens_1;
	if(Line_Sens_2 <= Line_2_Min) Line_2_Min = Line_Sens_2;
	if(Line_Sens_2 >= Line_2_Max) Line_2_Max = Line_Sens_2;
	if(Line_Sens_3 <= Line_3_Min) Line_3_Min = Line_Sens_3;
	if(Line_Sens_3 >= Line_3_Max) Line_3_Max = Line_Sens_3;
	if(Line_Sens_4 <= Line_4_Min) Line_4_Min = Line_Sens_4;
	if(Line_Sens_4 >= Line_4_Max) Line_4_Max = Line_Sens_4;
	if(Line_Sens_5 <= Line_5_Min) Line_5_Min = Line_Sens_5;
	if(Line_Sens_5 >= Line_5_Max) Line_5_Max = Line_Sens_5;
	*/
	double Cal_SR, Cal_SL;
	int count = 0;

	while(count < 270)
	{
		Cal_SR = SpeedContorol_R(0.25);
		Cal_SL = SpeedContorol_L(0.25);
		motor_R(Cal_SR);
		motor_L(Cal_SL);

		if(Line_Sens_1 <= Line_1_Min) Line_1_Min = Line_Sens_1;
		if(Line_Sens_1 >= Line_1_Max) Line_1_Max = Line_Sens_1;
		if(Line_Sens_2 <= Line_2_Min) Line_2_Min = Line_Sens_2;
		if(Line_Sens_2 >= Line_2_Max) Line_2_Max = Line_Sens_2;
		if(Line_Sens_3 <= Line_3_Min) Line_3_Min = Line_Sens_3;
		if(Line_Sens_3 >= Line_3_Max) Line_3_Max = Line_Sens_3;
		if(Line_Sens_4 <= Line_4_Min) Line_4_Min = Line_Sens_4;
		if(Line_Sens_4 >= Line_4_Max) Line_4_Max = Line_Sens_4;
		if(Line_Sens_5 <= Line_5_Min) Line_5_Min = Line_Sens_5;
		if(Line_Sens_5 >= Line_5_Max) Line_5_Max = Line_Sens_5;

		count++;
		HAL_Delay(1);
	}
	while(1)
	{
		Cal_SR = SpeedContorol_R(0.25);
		Cal_SL = SpeedContorol_L(-0.25);
		motor_R(Cal_SR);
		motor_L(Cal_SL);

		if(Line_Sens_3 > 800){
			motor_R(0);
			motor_L(0);
			break;
		}
		HAL_Delay(1);
	}


}

//ADC_Values_Sort
void updateValues()
{
	uint16_t tmp;
	for(int i = 0; i < 10; i++){
		for(int j = i+1; j < 10; j++){
			if(line_1_[i] > line_1_[j]){
				tmp = line_1_[i];
				line_1_[i] = line_1_[j];
				line_1_[j] = tmp;
			}
			if(line_2_[i] > line_2_[j]){
				tmp = line_2_[i];
				line_2_[i] = line_2_[j];
				line_2_[j] = tmp;
			}
			if(line_3_[i] > line_3_[j]){
				tmp = line_3_[i];
				line_3_[i] = line_3_[j];
				line_3_[j] = tmp;
			}
			if(line_4_[i] > line_4_[j]){
				tmp = line_4_[i];
				line_4_[i] = line_4_[j];
				line_4_[j] = tmp;
			}
			if(line_5_[i] > line_5_[j]){
				tmp = line_5_[i];
				line_5_[i] = line_5_[j];
				line_5_[j] = tmp;
			}
		}
	}
	Line_Sens_1 = (line_1_[4] + line_1_[5]) / 2;
	Line_Sens_2 = (line_2_[4] + line_2_[5]) / 2;
	Line_Sens_3 = (line_3_[4] + line_3_[5]) / 2;
	Line_Sens_4 = (line_4_[4] + line_4_[5]) / 2;
	Line_Sens_5 = (line_5_[4] + line_5_[5]) / 2;

	Line_Sort_[0][1] = ((Line_Sens_1 - (double)Line_1_Min) / (double)(Line_1_Max - Line_1_Min)) * 1000;
	Line_Sort_[1][1] = ((Line_Sens_2 - (double)Line_2_Min) / (double)(Line_2_Max - Line_2_Min)) * 1000;
	Line_Sort_[2][1] = ((Line_Sens_3 - (double)Line_3_Min) / (double)(Line_3_Max - Line_3_Min)) * 1000;
	Line_Sort_[3][1] = ((Line_Sens_4 - (double)Line_4_Min) / (double)(Line_4_Max - Line_4_Min)) * 1000;
	Line_Sort_[4][1] = ((Line_Sens_5 - (double)Line_5_Min) / (double)(Line_5_Max - Line_5_Min)) * 1000;

	for(int i = 0; i < 5; i++){
		Line_Sort_[i][0] = i;
		Line_Calib_[i] = Line_Sort_[i][1];
	}
	int tmp_1;
	float tmp_2;
	for (int i = 0; i < 5; i++) {
		for (int j = i+1; j < 5; j++) {
			if (Line_Sort_[i][1] > Line_Sort_[j][1]) {
				tmp_1 = Line_Sort_[i][0];
				tmp_2 = Line_Sort_[i][1];
				Line_Sort_[i][0] = Line_Sort_[j][0];
				Line_Sort_[i][1] = Line_Sort_[j][1];
				Line_Sort_[j][0] = tmp_1;
				Line_Sort_[j][1] = tmp_2;
			}
		}
	}



}

//degree
double degree()
{
	double ratio;
	int sort_1, sort_2;

	sort_1 = Line_Sort_[4][0];

	switch(sort_1){
	case 0:
		sort_2 = 1;
		break;
	case 1:
		if(Line_Calib_[0] >= Line_Calib_[2]) sort_2 = 0;
		else if(Line_Calib_[0] < Line_Calib_[2]) sort_2 = 2;
		break;
	case 2:
		if(Line_Calib_[1] >= Line_Calib_[3]) sort_2 = 1;
		else if(Line_Calib_[1] < Line_Calib_[3]) sort_2 = 3;
		break;
	case 3:
		if(Line_Calib_[2] >= Line_Calib_[4]) sort_2 = 2;
		else if(Line_Calib_[2] < Line_Calib_[4]) sort_2 = 4;
		break;
	case 4:
		sort_2 = 3;
		break;
	}

	if(Line_Calib_[1] > 100 && Line_Calib_[2] > 100 && Line_Calib_[3] > 100){
		cross_flag = 1;
		return 0;
	}

	//0-1
	if(sort_1 == 1 && sort_2 == 0){
		ratio = 30 * (Line_Calib_[sort_2] / (Line_Calib_[sort_1] + Line_Calib_[sort_2]));
		return 30+ratio;
	}
	if(sort_1 == 0 && sort_2 == 1){
		ratio = 30 * (Line_Calib_[sort_1] / (Line_Calib_[sort_2] + Line_Calib_[sort_1]));
		return 30+ratio;
	}
	//1-2
	if(sort_1 == 2 && sort_2 == 1){
		ratio = 30 * (Line_Calib_[sort_2] / (Line_Calib_[sort_1] + Line_Calib_[sort_2]));
		return ratio;
	}
	if(sort_1 == 1 && sort_2 == 2){
		ratio = 30 * (Line_Calib_[sort_1] / (Line_Calib_[sort_2] + Line_Calib_[sort_1]));
		return ratio;
	}
	//2-3
	if(sort_1 == 2 && sort_2 == 3){
		ratio = 30 * (Line_Calib_[sort_2] / (Line_Calib_[sort_1] + Line_Calib_[sort_2]));
		return -ratio;
	}
	if(sort_1 == 3 && sort_2 == 2){
		ratio = 30 * (Line_Calib_[sort_1] / (Line_Calib_[sort_2] + Line_Calib_[sort_1]));
		return -ratio;
	}
	//3-4
	if(sort_1 == 3 && sort_2 == 4){
		ratio = 30 * (Line_Calib_[sort_2] / (Line_Calib_[sort_1] + Line_Calib_[sort_2]));
		return -(30+ratio);
	}
	if(sort_1 == 4 && sort_2 == 3){
		ratio = 30 * (Line_Calib_[sort_1] / (Line_Calib_[sort_2] + Line_Calib_[sort_1]));
		return -(30+ratio);
	}

	else return 0;

}


void LineTrace(uint8_t motor_flag)
{
	double deg = degree();
	double rad = deg * (M_PI / 180);
	double Turning_Radius = 35 / tan(rad);

	if(motor_flag == 2){
		Speed_Ref = Speed_Ref - 0.002;
		if(0 >= Speed_Ref) Speed_Ref = 0;
	}
	else{
		Speed_Ref = ((V_MIN - V_MAX)/30) * fabs(deg) + V_MAX;
		if(V_MIN > Speed_Ref) Speed_Ref = V_MIN;
	}

	Speed_input_R = SpeedContorol_R( (Turning_Radius - tread) * (Speed_Ref / Turning_Radius) );
	Speed_input_L = SpeedContorol_L( (Turning_Radius + tread) * (Speed_Ref / Turning_Radius) );


	switch(motor_flag){
	case 0:
		//motor_R(0);
		//motor_L(0);
		break;
	case 1:
		motor_R( Speed_input_R );
		motor_L( Speed_input_L );
		break;
	case 2:
		motor_R( Speed_input_R );
		motor_L( Speed_input_L );
		break;
	}
}

//Motor_R
void motor_R( double duty )
{
	int countorperiod = 0;
	if( duty < 0 ){
		countorperiod = duty * -1;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, countorperiod);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	}
	else if( duty >= 0 ){
		countorperiod = duty;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, countorperiod);
	}
}

//Motor_L
void motor_L( double duty )
{
	int countorperiod = 0;
	if( duty < 0 ){
		countorperiod = duty * -1;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, countorperiod);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	}
	else if( duty >= 0 ){
		countorperiod = duty;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, countorperiod);
	}
}

//Motor_Speed_R
double Speed_R()
{
	int Encoder = (TIM2 -> CNT) - 32767;
	TIM2 -> CNT = 32767;
	double speed = (7. * TIRE * M_PI * (double)Encoder) / (40960. * TIM05);
	return speed;
}

//Motor_Speed_L
double Speed_L()
{
	int Encoder = (TIM3 -> CNT) - 32767;
	TIM3 -> CNT = 32767;
	double speed = (7. * TIRE * M_PI * (double)Encoder) / (40960. * TIM05);
	return speed;
}

double SpeedContorol_R(double Speed_Ref)
{
	double PGain = 700.0, duty;
	double delta_speed = Speed_Ref - SR;

	speedR_I_buff += delta_speed * TIM05 * 0.001;
	if(speedR_I_buff >= 1000000) speedR_I_buff = 1000000;
	if(speedR_I_buff <= -1000000) speedR_I_buff = -1000000;

	duty = ( delta_speed * PGain ) /*+ ( speedR_I_buff * IGain )*/;

	return duty;
}

double SpeedContorol_L(double Speed_Ref)
{
	double PGain = 700.0, duty;
	double delta_speed = Speed_Ref - SL;

	speedL_I_buff += delta_speed * TIM05 * 0.001;
	if(speedL_I_buff >= 1000000) speedL_I_buff = 1000000;
	if(speedL_I_buff <= -1000000) speedL_I_buff = -1000000;

	duty = ( delta_speed * PGain ) /*+ ( speedL_I_buff * IGain )*/;

	return duty;
}

//Timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == TIM6){
	   adc_getValues();
   }
   if(htim->Instance == TIM7){
	   updateValues();
	   degree();
	   SR = Speed_R();
	   SL = Speed_L();
	   LineTrace(linetrace_flag);
	   if(side_r_flag) side_r_time++;
	   if(side_l_flag) side_l_time++;
   }
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
