/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <arm_math.h>
#include <string.h>
#include <stdio.h>
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

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define NUM_TAPS 10

float low_pass_3khz[NUM_TAPS] =
{
		-96.37464579060073790E-6,
		-239.5922410224250710E-6,
		-236.3866644854411160E-6,
		-2.158076902079195220E-6,
		 510.2559562923468660E-6,
		 0.001286383953416845,
		 0.002234094331159058,
		 0.003181184355529571,
		 0.003890215348743426,
		 0.004091359454488967,
		 0.003530532804035990,
		 0.002026581946409395,
		-471.5498250157864960E-6,
		-0.003838285809652535,
		-0.007745892812315048,
		-0.011668195105453104,
		-0.014917825986515122,
		-0.016716148646595486,
		-0.016289357550435389,
		-0.012979146956684395,
		-0.006352643733446109,
		 0.003705193386918102,
		 0.016932376001823603,
		 0.032675798700162108,
		 0.049928231758077674,
		 0.067415089353529495,
		 0.083722175327715726,
		 0.097449098668476702,
		 0.107368611573056216,
		 0.112570535158906665,
		 0.112570535158906665,
		 0.107368611573056216,
		 0.097449098668476702,
		 0.083722175327715726,
		 0.067415089353529495,
		 0.049928231758077674,
		 0.032675798700162108,
		 0.016932376001823603,
		 0.003705193386918102,
		-0.006352643733446109,
		-0.012979146956684395,
		-0.016289357550435389,
		-0.016716148646595486,
		-0.014917825986515122,
		-0.011668195105453104,
		-0.007745892812315048,
		-0.003838285809652535,
		-471.5498250157864960E-6,
		 0.002026581946409395,
		 0.003530532804035990,
		 0.004091359454488967,
		 0.003890215348743426,
		 0.003181184355529571,
		 0.002234094331159058,
		 0.001286383953416845,
		 510.2559562923468660E-6,
		-2.158076902079195220E-6,
		-236.3866644854411160E-6,
		-239.5922410224250710E-6,
		-96.37464579060073790E-6
};

#define LEFT_RIGHT_SAMPLES	80
#define LEFT 				0
#define RIGHT				1
#define TOTAL_SAMPLES		487

q15_t	state[NUM_TAPS+LEFT_RIGHT_SAMPLES];
q15_t	src[LEFT_RIGHT_SAMPLES];
q15_t	dest[LEFT_RIGHT_SAMPLES];
q15_t	coeffs[NUM_TAPS];


#define BUFFER_COUNT		2

#define MAX_CONVERSIONS		2000

uint16_t rxBuf[LEFT_RIGHT_SAMPLES*4];
uint16_t txBuf[LEFT_RIGHT_SAMPLES*4];

//uint16_t txBuf[TOTAL_SAMPLES*4];
int samples[2][TOTAL_SAMPLES];

int receiveBuffer[BUFFER_COUNT][2][LEFT_RIGHT_SAMPLES];

volatile int receiveBufferPtr = 1;
volatile int sendBufferPtr = 0;

int completed = 0;
volatile int sampleCount = 0;
volatile int conversionCount = 0;

arm_status stat;
arm_fir_instance_q15 arm_inst;

/*
void populateTxBuf()
{
	for ( int i = 0 ; i < TOTAL_SAMPLES ; i++ )
	{
		int val = 500000000 * (
				sin( 2 * PI * i/TOTAL_SAMPLES* 5 ) +
				sin( 2 * PI * i/TOTAL_SAMPLES* 7 ) +
				sin( 2 * PI * i/TOTAL_SAMPLES* 9 ) +
				sin( 2 * PI * i/TOTAL_SAMPLES* 0 )
		);

		txBuf[i*4] = (val>>16)&0xFFFF;
		txBuf[i*4+1] = val&0xFFFF;
		txBuf[i*4+2] = (val>>16)&0xFFFF;
		txBuf[i*4+3] = val&0xFFFF;
	}
}
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART3_UART_Init(void);
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
  //populateTxBuf();

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
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  for ( int i = 0 ; i < NUM_TAPS ; i++ )
	  coeffs[i] = low_pass_3khz[i] * 32768;

  arm_fir_init_q15(
		  &arm_inst,
		  NUM_TAPS,
		  &coeffs[0],
		  &state[0],
		  LEFT_RIGHT_SAMPLES
  );
  HAL_I2S_Transmit_DMA(&hi2s3, txBuf, LEFT_RIGHT_SAMPLES*2 );
  HAL_I2S_Receive_DMA(&hi2s2, rxBuf, LEFT_RIGHT_SAMPLES*2 );

  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 430;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 7;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_SLAVE_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);

  HAL_GPIO_WritePin( GPIOE, GPIO_PIN_5, GPIO_PIN_SET );

  for ( int i = 0 ; i < LEFT_RIGHT_SAMPLES ; i++ )
  {
	  int lval = receiveBuffer[sendBufferPtr][LEFT][i];
	  int rval = receiveBuffer[sendBufferPtr][RIGHT][i];

	  txBuf[i*4] = (lval>>16)&0xFFFF;
	  txBuf[i*4+1] = lval&0xFFFF;
	  txBuf[i*4+2] = (rval>>16)&0xFFFF;
	  txBuf[i*4+3] = rval&0xFFFF;
  }
/*
	arm_fir_fast_q15(
			&arm_inst,
			&src[0],
			&dest[0],
			LEFT_RIGHT_SAMPLES
	);
*/

  HAL_GPIO_WritePin( GPIOE, GPIO_PIN_5, GPIO_PIN_RESET );


  sendBufferPtr++;
  if ( sendBufferPtr >= BUFFER_COUNT )
	  sendBufferPtr = 0;

}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    UNUSED(hi2s);

    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_4, GPIO_PIN_SET );

  int pos = 0;
  for ( int i = 0 ; i < LEFT_RIGHT_SAMPLES ; i++ )
  {
	  receiveBuffer[receiveBufferPtr][LEFT][i] = (int) ( (rxBuf[pos]<<16)|rxBuf[pos+1] );
	  receiveBuffer[receiveBufferPtr][RIGHT][i] = (int) ( (rxBuf[pos+2]<<16)|rxBuf[pos+3] );
	  pos += 4;
  }

  receiveBufferPtr++;
  if ( receiveBufferPtr >= BUFFER_COUNT )
	  receiveBufferPtr = 0;

  HAL_GPIO_WritePin( GPIOE, GPIO_PIN_4, GPIO_PIN_RESET );

}


/*
 *
void printLatest()
{
	char c[20];

	conversionCount = 0;

	for ( int i = 0 ; i < 2 ; i++ )
	{
		for ( int j = 0 ; j < TOTAL_SAMPLES ; j++ )
		{
			sprintf( c, "%d,", samples[i][j]);
			HAL_UART_Transmit(&huart3, (uint8_t *)c, strlen(c), HAL_MAX_DELAY );
		}
		sprintf( c, "\r\n\r\n\r\n" );
		HAL_UART_Transmit(&huart3, (uint8_t *)c, strlen(c), HAL_MAX_DELAY );
	}


}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);

  for ( int i = 0 ; i < LEFT_RIGHT_SAMPLES ; i++ )
  {
	  int pos = i*4;
	  samples[LEFT][sampleCount] = (int) ( (rxBuf[pos]<<16)|rxBuf[pos+1] );
	  samples[RIGHT][sampleCount] = (int) ( (rxBuf[pos+2]<<16)|rxBuf[pos+3] );

	  sampleCount++;
	  if ( sampleCount >= TOTAL_SAMPLES )
	  {
		  if ( ++conversionCount > MAX_CONVERSIONS )
			  printLatest();
		  sampleCount = 0;
	  }
  }

  completed = 1;

}

*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
