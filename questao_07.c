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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_MAX_VALUE 4095
#define ADC_CENTER 2047      // Valor central do ADC (meio do joystick)
#define DEAD_ZONE 100        // Zona morta ao redor do centro
#define PWM_MAX_DUTY 840     // Valor máximo do duty cycle (ARR)

#define MOTOR_PIN1 PIN_3     // PA3 - Controle direção 1
#define MOTOR_PIN2 PIN_4 	 // PA4 - Controle direção 2

#define LOOP_FREQUENCY 100   // Frequência do loop principal (Hz) - ajuste conforme seu delay
#define MAX_CHANGE_PER_SEC (PWM_MAX_DUTY * 0.25f)  // 25% da velocidade máxima por segundo
#define MAX_CHANGE_PER_LOOP (MAX_CHANGE_PER_SEC / LOOP_FREQUENCY)  // Mudança máxima por iteração

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

float adc_volt = 0.0f;

uint16_t jstck_1 = 0;
uint16_t jstck_2 = 0;

uint16_t duty_1 = 0;
uint16_t duty_2 = 0;

typedef enum {
	MOTOR_STOP = 0, MOTOR_CLOCKWISE,        // Sentido horário
	MOTOR_COUNTERCLOCKWISE  // Sentido anti-horário
} MotorDirection_t;

static float current_duty = 0.0f;           // Duty cycle atual (com sinal)
static MotorDirection_t current_direction = MOTOR_STOP;
static uint32_t last_update_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int abs(int value) {
	return (value < 0) ? -value : value;
}

/* Função para obter valor absoluto de float */
float abs_float(float value) {
	return (value < 0.0f) ? -value : value;
}

uint16_t map_joystick_to_duty(uint16_t adc_value) {
	int16_t centered_value = adc_value - ADC_CENTER;

	// Verifica zona morta
	if (abs(centered_value) < DEAD_ZONE) {
		return 0;
	}

	uint16_t duty;
	if (centered_value > 0) {
		// Lado direito do joystick
		duty = ((centered_value - DEAD_ZONE) * PWM_MAX_DUTY)
				/ (ADC_CENTER - DEAD_ZONE);
	} else {
		// Lado esquerdo do joystick
		duty = ((abs(centered_value) - DEAD_ZONE) * PWM_MAX_DUTY)
				/ (ADC_CENTER - DEAD_ZONE);
	}

	// Limita o duty cycle máximo
	if (duty > PWM_MAX_DUTY) {
		duty = PWM_MAX_DUTY;
	}

	return duty;
}

MotorDirection_t get_motor_direction(uint16_t adc_value) {
	int16_t centered_value = adc_value - ADC_CENTER;

	if (abs(centered_value) < DEAD_ZONE) {
		return MOTOR_STOP;
	} else if (centered_value > 0) {
		return MOTOR_CLOCKWISE;        // Joystick para direita = horário
	} else {
		return MOTOR_COUNTERCLOCKWISE; // Joystick para esquerda = anti-horário
	}
}

/* Função de controle direto do motor (sem rampa) */
void motor_control_direct(uint16_t duty, MotorDirection_t direction) {
	switch (direction) {
	case MOTOR_STOP:
		// Para o motor
		GPIO_Write_Pin(GPIOA, MOTOR_PIN1, LOW);
		GPIO_Write_Pin(GPIOA, MOTOR_PIN2, LOW);
		TIM5->CCR3 = 0;
		break;

	case MOTOR_CLOCKWISE:
		// Sentido horário
		GPIO_Write_Pin(GPIOA, MOTOR_PIN1, 1);
		GPIO_Write_Pin(GPIOA, MOTOR_PIN2, 0);
		TIM5->CCR3 = duty;
		break;

	case MOTOR_COUNTERCLOCKWISE:
		// Sentido anti-horário
		GPIO_Write_Pin(GPIOA, MOTOR_PIN1, 0);
		GPIO_Write_Pin(GPIOA, MOTOR_PIN2, 1);
		TIM5->CCR3 = duty;
		break;
	}
}

void apply_motor_ramp(uint16_t target_duty, MotorDirection_t target_direction) {
	// Converte o target para um valor com sinal
	float target_signed_duty = 0.0f;

	switch (target_direction) {
	case MOTOR_STOP:
		target_signed_duty = 0.0f;
		break;
	case MOTOR_CLOCKWISE:
		target_signed_duty = (float) target_duty;
		break;
	case MOTOR_COUNTERCLOCKWISE:
		target_signed_duty = -(float) target_duty;
		break;
	}

	// Calcula a diferença entre o valor atual e o target
	float difference = target_signed_duty - current_duty;

	// Limita a mudança máxima por iteração
	if (difference > MAX_CHANGE_PER_LOOP) {
		difference = MAX_CHANGE_PER_LOOP;
	} else if (difference < -MAX_CHANGE_PER_LOOP) {
		difference = -MAX_CHANGE_PER_LOOP;
	}

	// Aplica a mudança
	current_duty += difference;

	// Determina a direção atual e o duty cycle absoluto
	uint16_t abs_duty;
	if (current_duty > 1.0f) {
		current_direction = MOTOR_CLOCKWISE;
		abs_duty = (uint16_t) current_duty;
	} else if (current_duty < -1.0f) {
		current_direction = MOTOR_COUNTERCLOCKWISE;
		abs_duty = (uint16_t) (-current_duty);
	} else {
		current_direction = MOTOR_STOP;
		abs_duty = 0;
		current_duty = 0.0f;  // Zera para evitar drift
	}

	// Aplica o controle do motor
	motor_control_direct(abs_duty, current_direction);
}

void motor_control(uint16_t target_duty, MotorDirection_t target_direction) {
	apply_motor_ramp(target_duty, target_direction);
}

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
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */

	Utility_Init();
	USART1_Init();

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // liga o clock do timer 5
	
	TIM5->CR1 &= ~TIM_CR1_DIR; // contador crescente
	TIM5->ARR = PWM_MAX_DUTY - 1; // auto reload 99
	TIM5->PSC = 100 - 1; // presscaler para pulsos a cada 5us (fpwm = 2khz)
	TIM5->CCMR2 |= 0b110 << 4; // seleciona o pwm modo 1
	TIM5->CCMR2 |= TIM_CCMR2_OC3PE; // habilita a saída
	TIM5->CCER |= TIM_CCER_CC3E; // update event para preescrever o valor do preescaler
	TIM5->EGR |= TIM_EGR_UG; // habilita o timer
	TIM5->CR1 |= TIM_CR1_CEN; // habilita a ocntagem

	TIM5->CCR3 = 0;

	ADC_Init(ADC1, SINGLE_CHANNEL, ADC_RES_12BITS);
	ADC_SingleChannel(ADC1, ADC_IN0);

	GPIO_Clock_Enable(GPIOA);
	GPIO_Pin_Mode(GPIOA, PIN_2, ALTERNATE);  // PWM output
	GPIO_Pin_Mode(GPIOA, MOTOR_PIN1, OUTPUT); // Controle motor 1
	GPIO_Pin_Mode(GPIOA, MOTOR_PIN2, OUTPUT); // Controle motor 2

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		jstck_1 = ADC_GetSingleConversion(ADC1);

		MotorDirection_t target_direction = get_motor_direction(jstck_1);
		uint16_t target_duty = map_joystick_to_duty(jstck_1);

		motor_control(target_duty, target_direction);

		// Debug via UART
		printf("ADC: %d, Target: %d/%d, Current: %.1f/%d\n", jstck_1,
				target_duty, target_direction, abs(current_duty),
				current_direction);

		// Pequeno delay para estabilizar
		HAL_Delay(10);

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
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

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

	/** DAC channel OUT2 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);

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

	/*Configure GPIO pins : PA3 PA4 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
