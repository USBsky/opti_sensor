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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct LED {
	uint16_t led_sel[4];
	uint16_t ledx[8];
}LED;

typedef struct LED_Controller {
	unsigned char data[4];
	uint8_t ptr;
	uint16_t led_encode;
	uint16_t led_sel;
	uint8_t dot_plc;
}LED_Controller;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
volatile LED_Controller led_ctrl;
volatile LED led;
float adc_value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void LED_init(void);
void LED_Controller_init(void);
void LED_Controller_setdata(float data);
void LED_dy_ctrl(void);
void LED_dis(void);
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
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	LED_init();
	LED_Controller_init();
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 99;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 9999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_SEL_1_Pin|LED_SEL_2_Pin|LED_SEL_3_Pin|LED_SEL_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED5_Pin LED6_Pin LED7_Pin LED8_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_SEL_1_Pin LED_SEL_2_Pin LED_SEL_3_Pin LED_SEL_4_Pin */
  GPIO_InitStruct.Pin = LED_SEL_1_Pin|LED_SEL_2_Pin|LED_SEL_3_Pin|LED_SEL_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM10) {
		LED_dis();
	}
	if(htim->Instance == TIM11) {
		HAL_ADC_Start_IT(&hadc1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
		HAL_ADC_Stop_IT(&hadc1);
		uint32_t AD_value = HAL_ADC_GetValue(&hadc1);
		adc_value = (double) AD_value * 3.3 / 4096;
	  const double A = 0.0009321, B = -0.5941, C = 111.5, R = 250;
		volatile double Rx = (5.0 * R) / adc_value - R, distance = A * Rx * Rx + B * Rx + C;
		LED_Controller_setdata(distance);
}



void LED_init() {
	led.led_sel[0] = GPIO_PIN_6;
	led.led_sel[1] = GPIO_PIN_7;
	led.led_sel[2] = GPIO_PIN_8;
	led.led_sel[3] = GPIO_PIN_9;
	led.ledx[0] = GPIO_PIN_1;
	led.ledx[1] = GPIO_PIN_2;
	led.ledx[2] = GPIO_PIN_3;
	led.ledx[3] = GPIO_PIN_4;
	led.ledx[4] = GPIO_PIN_5;
	led.ledx[5] = GPIO_PIN_6;
	led.ledx[6] = GPIO_PIN_7;
	led.ledx[7] = GPIO_PIN_8;
}

void LED_Controller_init() {
		led_ctrl.ptr = 0;
		led_ctrl.led_encode = 0;
		led_ctrl.led_sel = 0;
		for(int i = 0; i < 8; i++) {
			led_ctrl.led_encode = led_ctrl.led_encode | led.ledx[i];
		}
		for(int i = 0; i < 3; i++) {
			led_ctrl.led_sel = led_ctrl.led_sel | led.led_sel[i];
		}
		HAL_GPIO_WritePin(GPIOB, led_ctrl.led_encode, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, led_ctrl.led_sel, GPIO_PIN_RESET);
}

void LED_Controller_setdata(float data) {
		int inter = (int) data;
		int ptr  = 0;
	while (inter > 0 && ptr < 4) {
		led_ctrl.dot_plc = ptr;
		led_ctrl.data[ptr++] = inter % 10;
		inter /= 10;
	}
	data = data - (int) data;
	while (ptr < 4) {
		data = data * 10;
		led_ctrl.data[ptr++] = (int) data;
		data = data - (int)data;
	}
}


void LED_dis() {
	HAL_GPIO_WritePin(GPIOB, led_ctrl.led_encode, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, led_ctrl.led_sel, GPIO_PIN_RESET);
	unsigned char data = led_ctrl.data[led_ctrl.ptr];
	switch(data) {
		case 0 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2] | led.ledx[3] | led.ledx[4] | led.ledx[5];
		break;
		case 1 : led_ctrl.led_encode = led.ledx[1] | led.ledx[2];
		break;
		case 2 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[3] | led.ledx[4] | led.ledx[6];
		break;
		case 3 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2] | led.ledx[3] | led.ledx[6];
		break;
		case 4 : led_ctrl.led_encode = led.ledx[1] | led.ledx[2] | led.ledx[5] | led.ledx[6];
		break;
		case 5 : led_ctrl.led_encode = led.ledx[0] | led.ledx[2] | led.ledx[3] | led.ledx[5] | led.ledx[6];
		break;
		case 6 : led_ctrl.led_encode = led.ledx[0] | led.ledx[2] | led.ledx[3] | led.ledx[4] | led.ledx[5] | led.ledx[6];
		break;
		case 7 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2];
		break;
		case 8 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2] | led.ledx[3] | led.ledx[4] | led.ledx[5] \
		|	led.ledx[6];
		break;
		case 9 : led_ctrl.led_encode = led.ledx[0] | led.ledx[1] | led.ledx[2] | led.ledx[3] | led.ledx[5] | led.ledx[6];
	}
	if(led_ctrl.ptr == led_ctrl.dot_plc) {
		led_ctrl.led_encode = led_ctrl.led_encode | led.ledx[7];
	}
	switch(led_ctrl.ptr) {
		case 0 : led_ctrl.led_sel = led.led_sel[0]; led_ctrl.ptr = 1; break;
		case 1 : led_ctrl.led_sel = led.led_sel[1]; led_ctrl.ptr = 2; break;
		case 2 : led_ctrl.led_sel = led.led_sel[2]; led_ctrl.ptr = 3; break;
		case 3 : led_ctrl.led_sel = led.led_sel[3]; led_ctrl.ptr = 0; break;
	}

	HAL_GPIO_WritePin(GPIOB, led_ctrl.led_encode, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, led_ctrl.led_sel, GPIO_PIN_SET);
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
