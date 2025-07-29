/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "LCD_Blio.h"
#include "Utility.h"
#include <stdint.h>
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint16_t valor_dac = 0;
uint8_t up = 1;

uint16_t dac_min = (0.25 / 3.3) * 4095; // marromenos 310
uint16_t dac_max = (1.75 / 3.3) * 4095; // marromenos 2172

uint16_t senoide_index = 0;

uint16_t senoide[500] = { 1241, 1253, 1264, 1276, 1288, 1299, 1311, 1323, 1334,
		1346, 1358, 1369, 1381, 1392, 1404, 1415, 1427, 1438, 1450, 1461, 1473,
		1484, 1495, 1506, 1518, 1529, 1540, 1551, 1562, 1573, 1584, 1595, 1605,
		1616, 1627, 1637, 1648, 1658, 1669, 1679, 1690, 1700, 1710, 1720, 1730,
		1740, 1750, 1759, 1769, 1779, 1788, 1798, 1807, 1816, 1825, 1834, 1843,
		1852, 1861, 1870, 1878, 1887, 1895, 1903, 1912, 1920, 1928, 1935, 1943,
		1951, 1958, 1966, 1973, 1980, 1987, 1994, 2001, 2008, 2014, 2021, 2027,
		2033, 2039, 2045, 2051, 2057, 2062, 2068, 2073, 2078, 2083, 2088, 2093,
		2098, 2102, 2107, 2111, 2115, 2119, 2123, 2126, 2130, 2133, 2137, 2140,
		2143, 2146, 2148, 2151, 2153, 2156, 2158, 2160, 2161, 2163, 2165, 2166,
		2167, 2168, 2169, 2170, 2171, 2171, 2172, 2172, 2172, 2172, 2172, 2171,
		2171, 2170, 2169, 2168, 2167, 2166, 2165, 2163, 2161, 2160, 2158, 2156,
		2153, 2151, 2148, 2146, 2143, 2140, 2137, 2133, 2130, 2126, 2123, 2119,
		2115, 2111, 2107, 2102, 2098, 2093, 2088, 2083, 2078, 2073, 2068, 2062,
		2057, 2051, 2045, 2039, 2033, 2027, 2021, 2014, 2008, 2001, 1994, 1987,
		1980, 1973, 1966, 1958, 1951, 1943, 1935, 1928, 1920, 1912, 1903, 1895,
		1887, 1878, 1870, 1861, 1852, 1843, 1834, 1825, 1816, 1807, 1798, 1788,
		1779, 1769, 1759, 1750, 1740, 1730, 1720, 1710, 1700, 1690, 1679, 1669,
		1658, 1648, 1637, 1627, 1616, 1605, 1595, 1584, 1573, 1562, 1551, 1540,
		1529, 1518, 1506, 1495, 1484, 1473, 1461, 1450, 1438, 1427, 1415, 1404,
		1392, 1381, 1369, 1358, 1346, 1334, 1323, 1311, 1299, 1288, 1276, 1264,
		1253, 1241, 1229, 1218, 1206, 1194, 1183, 1171, 1159, 1148, 1136, 1124,
		1113, 1101, 1090, 1078, 1067, 1055, 1044, 1032, 1021, 1009, 998, 987,
		976, 964, 953, 942, 931, 920, 909, 898, 887, 877, 866, 855, 845, 834,
		824, 813, 803, 792, 782, 772, 762, 752, 742, 732, 723, 713, 703, 694,
		684, 675, 666, 657, 648, 639, 630, 621, 612, 604, 595, 587, 579, 570,
		562, 554, 547, 539, 531, 524, 516, 509, 502, 495, 488, 481, 474, 468,
		461, 455, 449, 443, 437, 431, 425, 420, 414, 409, 404, 399, 394, 389,
		384, 380, 375, 371, 367, 363, 359, 356, 352, 349, 345, 342, 339, 336,
		334, 331, 329, 326, 324, 322, 321, 319, 317, 316, 315, 314, 313, 312,
		311, 311, 310, 310, 310, 310, 310, 311, 311, 312, 313, 314, 315, 316,
		317, 319, 321, 322, 324, 326, 329, 331, 334, 336, 339, 342, 345, 349,
		352, 356, 359, 363, 367, 371, 375, 380, 384, 389, 394, 399, 404, 409,
		414, 420, 425, 431, 437, 443, 449, 455, 461, 468, 474, 481, 488, 495,
		502, 509, 516, 524, 531, 539, 547, 554, 562, 570, 579, 587, 595, 604,
		612, 621, 630, 639, 648, 657, 666, 675, 684, 694, 703, 713, 723, 732,
		742, 752, 762, 772, 782, 792, 803, 813, 824, 834, 845, 855, 866, 877,
		887, 898, 909, 920, 931, 942, 953, 964, 976, 987, 998, 1009, 1021, 1032,
		1044, 1055, 1067, 1078, 1090, 1101, 1113, 1124, 1136, 1148, 1159, 1171,
		1183, 1194, 1206, 1218, 1229,

};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USART1_UART_Init();
	MX_DAC_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	Utility_Init();
	USART1_Init();

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->PSC = 84-1;
	TIM5->ARR = 5-1;
	TIM5->EGR |= TIM_EGR_UG;
	TIM5->CR1 &= ~TIM_CR1_DIR;

	TIM5->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM5_IRQn);

	TIM5->CR1 |= TIM_CR1_CEN; // habilita a ocntagem

	DAC_Init(DAC_CHANNEL2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void) {

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */

	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

	/*Configure GPIO pins : K1_Pin K0_Pin */
	GPIO_InitStruct.Pin = K1_Pin | K0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TIM5_IRQHandler() {
    TIM5->SR &= ~TIM_SR_UIF;

    DAC_SetValue(DAC_CHANNEL2, senoide[senoide_index], DAC_RES_12BITS);

    senoide_index++;

    if (senoide_index >= 500) {
        senoide_index = 0;
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
