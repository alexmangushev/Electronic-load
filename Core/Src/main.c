/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Nokia_LCD.h"

#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define BUF_SIZE 3 //size of buffer for ADC

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

uint8_t cur_menu_pos = 0; //save numbers of click on encoder's button
uint8_t change_value = 0; //1 - change value, 0 - selection position of change
uint16_t count_encoder = 0; //save numbers of rotation of encoder
uint8_t show_menu_flag = 0; //when flag is set, screen updates
uint8_t sd_detect = 0; //flag that show, have we SD card in device


/*
 * 0 - out current zero (����)
 * 1 - start (���)
 * 2 - measurement of capacity (���)
 */
uint8_t mode_of_work = 0;
uint16_t tmp;

//strings for showings data
uint8_t U_sh[9] = {};
uint8_t I_sh[9] = {};
//uint8_t I_sh_exposed[12] = {};

//buffers for sampling data by ADC
uint16_t buf_I[BUF_SIZE] = {};
uint16_t buf_U[BUF_SIZE] = {};

//sum of elements in the buffer
uint16_t sum_I = 0;
uint16_t sum_U = 0;

//counter of the number of elements in the buffer
uint16_t kol_I = 0;
uint16_t kol_U = 0;

//last filtered measurements
uint8_t last_I = 0;
uint8_t last_U = 0;

//start and end of buffer
uint16_t start_I = 0;
uint16_t end_I = 0;

uint16_t start_U = 0;
uint16_t end_U = 0;

//filtered values
uint16_t filtered_I = 0;
uint16_t filtered_U = 0;

//raw values
uint16_t raw_I = 0;
uint16_t raw_U = 0;

//values for protection
uint16_t out_I = 0;
uint16_t minimum_U = 0;

//calibration number for DAC
uint16_t calibration = 0;

//power, that conversion on the transistor
uint32_t transistor_power = 0;

//uint8_t sect[512];
//extern char str1[60];
uint32_t byteswritten;
uint32_t U_DAC; //U for sending to DAC
extern char USERPath[4]; /* logical drive path */

//FATFS SDFatFs;
//FIL MyFile;

//FRESULT res; //��������� ����������
//uint8_t wtext[]="Hello from STM32!!!";

volatile uint16_t array_I[256] = {};
volatile uint16_t array_U[256] = {};

/*
 * 0 - 11 bits - data
 * 12 - 1
 * 13 - 1
 * 14 - 1
 * 15 - 0
 */
uint16_t ADC_data= 0;
uint16_t counter = 2000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_IT(&hadc1);

  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_ADC_Start_IT(&hadc2);

  //PWM for FAN
  HAL_TIM_PWM_Init(&htim2);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  TIM2->CCR1 = 0;

  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_1);

  sd_detect = (HAL_GPIO_ReadPin(SD_detected_GPIO_Port, SD_detected_Pin)) ? 0 : 1;

  HAL_SPI_Init(&hspi2);

  HAL_RTCEx_SetSecond_IT(&hrtc);

  display_init();

  HAL_Delay(100);

  //disk_initialize(SDFatFs.drv);

  //write
  /*if(f_mount(&SDFatFs,(TCHAR const*)USERPath,0) != FR_OK)
  {
	  Error_Handler();
  }
  else
  {
	  if(f_open(&MyFile,"mywrite.txt",FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK)
	  {
		  Error_Handler();
	  }
	  else
	  {
		  res=f_write(&MyFile,wtext,sizeof(wtext),(void*)&byteswritten);
		  if((byteswritten==0)||(res!=FR_OK))
		  {
			  Error_Handler();
		  }
		  f_close(&MyFile);
	  }

  }*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	if (show_menu_flag && cur_menu_pos <= 4)
	  {
		  display_show(1);
		  show_menu_flag = 0;
	  }
	  else if (show_menu_flag && cur_menu_pos > 4)
	  {
		  display_show(2);
		  show_menu_flag = 0;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
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
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RST_Pin|CLK_Pin|SPI_CC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin|DIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_detected_Pin */
  GPIO_InitStruct.Pin = SD_detected_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_detected_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin LDAC_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin DC_Pin DIN_Pin CLK_Pin
                           SPI_CC_Pin */
  GPIO_InitStruct.Pin = RST_Pin|DC_Pin|DIN_Pin|CLK_Pin
                          |SPI_CC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_Button_Pin */
  GPIO_InitStruct.Pin = ENC_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_Button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
	HAL_GPIO_WritePin(SPI_CC_GPIO_Port, SPI_CC_Pin, 1);

	HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, 0);
	HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, 1);
}

//Interrupt one time per second from RTC
//sending data to DAC
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
    show_menu_flag = 1;

    //0,109i + 4.19
    U_DAC = (out_I * 109 + 4190) / 1000;

    if (out_I != 0)// && (mode_of_work == 0 || mode_of_work == 1)
    	ADC_data = 28672 + ((uint16_t)U_DAC + 1) * 4096 / 330 + calibration;
    else
    	ADC_data = 28672;
    //ADC_data = 28672 + (out_I + 1) * 4096 / 330;

    HAL_GPIO_WritePin(SPI_CC_GPIO_Port, SPI_CC_Pin, 0);
    HAL_SPI_Transmit_IT(&hspi2, (uint8_t*) &ADC_data, 1);

    transistor_power = filtered_I * filtered_U;

    if (transistor_power < 30000)
    {
    	TIM2->CCR1 = 0;
    }
    else if (transistor_power > 80000)
    {
    	TIM2->CCR1 = 2500;
    }
    else
    {
    	//TIM2->CCR1 = transistor_power / 45;
    	TIM2->CCR1 = 1500;
    }

}

//Data from ADC
//ADC1 - I
//ADC2 - U
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
    	HAL_ADC_Stop_IT(&hadc1);
    	HAL_ADC_Stop_IT(&hadc2);

    	raw_I = HAL_ADC_GetValue(&hadc1);

    	if (kol_I < BUF_SIZE)
		{
			buf_I[end_I] = raw_I;
			kol_I++;
			end_I++;

			sum_I+=raw_I;

		}
		else
		{
			sum_I -= buf_I[start_I];

			if (start_I + 1 >= BUF_SIZE)
				start_I = 0;
			else
				start_I++;

			if (end_I + 1 >= BUF_SIZE)
				end_I = 0;
			else
				end_I++;

			buf_I[end_I] = raw_I;
			sum_I+=raw_I;

		}

    	filtered_I = sum_I / kol_I;
    	filtered_I = filtered_I * 206 / 2778;

    	if (filtered_I < 5)
    		calibration = 0;

    	if (abs(filtered_I - last_I) < 5 && abs(filtered_I - out_I) >= 3)
    	{

			if (out_I > filtered_I)
			{
				calibration++;
			}
			else
			{
				calibration--;
			}

    	}

    	last_I = filtered_I;

    }


    if(hadc->Instance == ADC2) //check if the interrupt comes from ACD2
	{
    	HAL_ADC_Stop_IT(&hadc1);
    	HAL_ADC_Stop_IT(&hadc2);

    	raw_U = HAL_ADC_GetValue(&hadc2);

    	if (kol_U < BUF_SIZE)
		{
			buf_U[end_U] = raw_U;
			kol_U++;
			end_U++;

			sum_U+=raw_U;

			HAL_ADC_Start_IT(&hadc1);
			HAL_ADC_Start_IT(&hadc2);
		}
		else
		{
			sum_U -= buf_U[start_U];

			if (start_U + 1 >= BUF_SIZE)
				start_U = 0;
			else
				start_U++;

			if (end_U + 1 >= BUF_SIZE)
				end_U = 0;
			else
				end_U++;

			buf_U[end_U] = raw_U;
			sum_U+=raw_U;

			HAL_ADC_Start_IT(&hadc1);
			HAL_ADC_Start_IT(&hadc2);
		}

    	filtered_U = sum_U / kol_U;
    	filtered_U = filtered_U * 2178 / 4096;
	}


}

//Processing of the encoder rotation
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		if (change_value)
		{
			calibration = 0;

			count_encoder = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;
			if (!cur_menu_pos)
				show_menu_flag = 1;

			switch (cur_menu_pos)
			{
				case 1:
				{

					mode_of_work = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;

					if (mode_of_work == 2 && !sd_detect)
					{
						mode_of_work = 0;
						TIM4->CNT = 0;
					}

					if (mode_of_work >= 3 && mode_of_work < 5)
					{
						mode_of_work = 0;
						TIM4->CNT = 0;
					}
					else if (mode_of_work >= 3)
					{
						if (sd_detect)
						{
							mode_of_work = 2;
							TIM4->CNT = 4;
						}
						else
						{
							mode_of_work = 1;
							TIM4->CNT = 2;
						}
					}

					show_menu_flag = 1;
					break;
				}
				case 2:
				{
					count_encoder = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;
					out_I/=10; out_I*=10; out_I+=count_encoder;
					show_menu_flag = 1;
					break;
				}
				case 3:
				{
					count_encoder = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;
					tmp = out_I / 100;
					out_I %= 10;
					out_I += tmp * 100 + count_encoder * 10;
					show_menu_flag = 1;
					break;
				}
				case 4:
				{
					count_encoder = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;
					if (count_encoder > 2)
					{
						count_encoder = 2;
						TIM4->CNT = 4;
					}

					out_I %= 100; out_I = out_I + count_encoder * 100;
					show_menu_flag = 1;
					break;
				}

				case 5:
				{
					count_encoder = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;
					minimum_U/=10; minimum_U*=10; minimum_U+=count_encoder;
					show_menu_flag = 1;
					break;
				}
				case 6:
				{
					count_encoder = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;
					tmp = minimum_U / 100;
					minimum_U %= 10;
					minimum_U += tmp * 100 + count_encoder * 10;
					show_menu_flag = 1;
					break;
				}
				case 7:
				{
					count_encoder = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;
					minimum_U %= 100; minimum_U = minimum_U + count_encoder * 100;
					show_menu_flag = 1;
					break;
				}
			}
		}
		else
		{
			cur_menu_pos = __HAL_TIM_GET_COUNTER(htim) / 2 % 10;

			if (cur_menu_pos > 7)
			{
			  cur_menu_pos = 0;
			  TIM4->CNT = 0;

			  //HAL_ADC_Start_IT(&hadc1);
			  //HAL_ADC_Start_IT(&hadc2);

			  //HAL_RTCEx_SetSecond_IT(&hrtc);
			}
			else if (cur_menu_pos)
			{
			  //HAL_RTCEx_DeactivateSecond(&hrtc);

			  //HAL_ADC_Stop_IT(&hadc1);
			  //HAL_ADC_Stop_IT(&hadc2);

			  show_menu_flag = 1;
			}
		}
	}
}

/*
 * type:
 * 1 - draw main window
 * 2 - draw setting menu
 */
void display_show(uint8_t type)
{
	HAL_ADC_Stop_IT(&hadc1);
	HAL_ADC_Stop_IT(&hadc2);

	switch (type)
	{
/*-----------PAGE_1--------------*/
		case 1:
		{

			sprintf(U_sh,"U=%d.%02d�",filtered_U / 100, filtered_U % 100 );
			sprintf(I_sh,"I=%d.%02d�",filtered_I / 100, filtered_I % 100 );
			//10.8

			//sprintf(U_sh,"U=%d.%02d�",(filtered_U * 330 / 4096) / 100, (filtered_U * 330 / 4096) % 100 );
			//sprintf(I_sh,"I=%d.%02d�",(filtered_I * 330 / 4096) / 100, (filtered_I * 330 / 4096) % 100 );

			display_clear();

			display_set_cursor(0,0);  display_string("��. ��������");
			display_set_cursor(0,1); display_string("�����: ");
			display_set_cursor(7,1);

			if (cur_menu_pos != 1)
			{
				switch (mode_of_work)
				{
					case 0:
					{
						display_string("����");
						break;
					}
					case 1:
					{
						display_string("���");
						break;
					}
					case 2:
					{
						display_string("���");
						break;
					}
				}
			}
			else
			{
				switch (mode_of_work)
				{
					case 0:
					{
						display_char_underline('�');
						display_set_cursor(8,1);
						display_char_underline('�');
						display_set_cursor(9,1);
						display_char_underline('�');
						display_set_cursor(10,1);
						display_char_underline('�');
						break;
					}
					case 1:
					{
						display_char_underline('�');
						display_set_cursor(8,1);
						display_char_underline('�');
						display_set_cursor(9,1);
						display_char_underline('�');
						break;
					}
					case 2:
					{
						display_char_underline('�');
						display_set_cursor(8,1);
						display_char_underline('�');
						display_set_cursor(9,1);
						display_char_underline('�');
						break;
					}
				}
			}

			display_set_cursor(0,2);  display_string("I_load=");
			display_set_cursor(7,2);

			if (cur_menu_pos > 1)
			{
				switch (cur_menu_pos)
				{
					case 2:
					{
						display_char( out_I / 100 + 48);
						display_set_cursor(8,2); display_char('.');
						display_set_cursor(9,2); display_char(out_I % 100 / 10 + 48);
						display_set_cursor(10,2); display_char_underline(out_I % 10  + 48);
						break;
					}
					case 3:
					{
						display_char( out_I / 100 + 48);
						display_set_cursor(8,2); display_char('.');
						display_set_cursor(9,2); display_char_underline(out_I % 100 / 10 + 48);
						display_set_cursor(10,2); display_char(out_I % 10  + 48);
						break;
					}
					case 4:
					{
						display_char_underline( out_I / 100 + 48);
						display_set_cursor(8,2); display_char('.');
						display_set_cursor(9,2); display_char(out_I % 100 / 10 + 48);
						display_set_cursor(10,2); display_char(out_I % 10  + 48);
						break;
					}
				}
			}
			else
			{
				display_char( out_I / 100 + 48);
				display_set_cursor(8,2); display_char('.');
				display_set_cursor(9,2); display_char(out_I % 100 / 10 + 48);
				display_set_cursor(10,2); display_char(out_I % 10  + 48);
			}
			display_set_cursor(11,2);	display_char('A');

			display_set_cursor(0,3);  display_string(I_sh);

			display_set_cursor(0,4);  display_string(U_sh);

			display_set_cursor(10,4);
			if (sd_detect)
			{
				display_char('V');//detected
			}
			else
			{
				display_char('X');//not detected
			}

			display_set_cursor(11,4);  display_char(0x80);//sd

			break;
		}

/*-----------PAGE_2--------------*/
		case 2:
		{

			display_clear();

			display_set_cursor(0,0);  display_string("�����. �����");
			display_set_cursor(0,2);  display_string("U_���=");
			display_set_cursor(6,2);

			if (cur_menu_pos >= 5 && cur_menu_pos <= 7)
			{
				switch (cur_menu_pos)
				{
					case 5:
					{
						display_char( minimum_U / 100 + 48);
						display_set_cursor(7,2); display_char('.');
						display_set_cursor(8,2); display_char(minimum_U % 100 / 10 + 48);
						display_set_cursor(9,2); display_char_underline(minimum_U % 10  + 48);
						break;
					}
					case 6:
					{
						display_char( minimum_U / 100 + 48);
						display_set_cursor(7,2); display_char('.');
						display_set_cursor(8,2); display_char_underline(minimum_U % 100 / 10 + 48);
						display_set_cursor(9,2); display_char(minimum_U % 10  + 48);
						break;
					}
					case 7:
					{
						display_char_underline( minimum_U / 100 + 48);
						display_set_cursor(7,2); display_char('.');
						display_set_cursor(8,2); display_char(minimum_U % 100 / 10 + 48);
						display_set_cursor(9,2); display_char(minimum_U % 10  + 48);
						break;
					}
				}
			}
			else
			{
				display_char( minimum_U / 100 + 48);
				display_set_cursor(7,2); display_char('.');
				display_set_cursor(8,2); display_char(minimum_U % 100 / 10 + 48);
				display_set_cursor(9,2); display_char(minimum_U % 10  + 48);
			}

			display_set_cursor(10,2);  display_string("�");

			break;
		}

	}
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	return;
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
