#include "main.h"
#include "Utility.h"
#include "Timers.h"  // Sua biblioteca de timers
#include <stdio.h>

// Variáveis globais para controle dos LEDs
volatile uint32_t led_d2_counter = 0;  // Contador para LED D2 (PA6)
volatile uint32_t led_d3_counter = 0;  // Contador para LED D3 (PA7)
volatile uint8_t led_d2_active = 0;    // Flag se LED D2 está ativo
volatile uint8_t led_d3_active = 0;    // Flag se LED D3 está ativo

#define D2_TIMEOUT_MS 2000  // 2 segundos em ms
#define D3_TIMEOUT_MS 3000  // 2 segundos em ms
#define TIMER_FREQUENCY_HZ 1000  // Timer de 1ms (1kHz)

int main(void) {
	/* Inicializações */
	Utility_Init();
	USART1_Init();

	GPIO_Clock_Enable(GPIOA);
	GPIO_Clock_Enable(GPIOB);
	GPIO_Clock_Enable(GPIOE);

	// Configuração PWM no PA1 (TIM5_CH2) - mantendo sua configuração original
	GPIO_Pin_Mode(GPIOA, PIN_1, ALTERNATE);
	GPIO_Alternate_Function(GPIOA, PIN_1, AF2);

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->CR1 &= ~TIM_CR1_DIR;
	TIM5->PSC = 8399;
	TIM5->ARR = 9999;
	TIM5->CCMR1 &= ~(TIM_CCMR1_OC2M);
	TIM5->CCMR1 |= (0b110 << 12);
	TIM5->CCMR1 |= TIM_CCMR1_OC2PE;
	TIM5->CCER |= TIM_CCER_CC2E;
	TIM5->EGR = TIM_EGR_UG;
	TIM5->CCR2 = 5000;
	TIM5->CR1 |= TIM_CR1_CEN;

	// Configuração do TIM6 para controle dos LEDs (1ms tick)
	// Usando sua biblioteca: Timer de 1ms para controle dos LEDs
	// Com clock de 84MHz, prescaler = 84, period = 1000 -> 1ms
	TIM6_Setup(84, 1000);  // 1ms timer tick

	// Configuração dos botões PB0 e PB1
	GPIO_Pin_Mode(GPIOB, PIN_1, INPUT);
	GPIO_Resistor_Enable(GPIOB, PIN_1, PULL_UP);
	GPIO_Pin_Mode(GPIOB, PIN_0, INPUT);
	GPIO_Resistor_Enable(GPIOB, PIN_0, PULL_UP);

	NVIC_SetPriority(EXTI0_IRQn, 0);
	NVIC_SetPriority(EXTI1_IRQn, 0);
	EXTI_Config(EXTI0, GPIOB, RISING_EDGE);
	NVIC_EnableIRQ(EXTI0_IRQn);
	EXTI_Config(EXTI1, GPIOB, RISING_EDGE);
	NVIC_EnableIRQ(EXTI1_IRQn);

	// Configuração dos botões PE3 e PE4 (K0 e K1)
	GPIO_Pin_Mode(GPIOE, PIN_4, INPUT);
	GPIO_Resistor_Enable(GPIOE, PIN_4, PULL_UP);
	GPIO_Pin_Mode(GPIOE, PIN_3, INPUT);
	GPIO_Resistor_Enable(GPIOE, PIN_3, PULL_UP);

	NVIC_SetPriority(EXTI4_IRQn, 0);
	NVIC_SetPriority(EXTI3_IRQn, 0);
	EXTI_Config(EXTI4, GPIOE, FALLING_EDGE);
	NVIC_EnableIRQ(EXTI4_IRQn);
	EXTI_Config(EXTI3, GPIOE, FALLING_EDGE);
	NVIC_EnableIRQ(EXTI3_IRQn);

	// Configuração dos LEDs como saída
	GPIO_Pin_Mode(GPIOA, PIN_6, OUTPUT);  // D2
	GPIO_Pin_Mode(GPIOA, PIN_7, OUTPUT);  // D3

	// Inicializar LEDs apagados
	GPIO_Write_Pin(GPIOA, PIN_6, HIGH);
	GPIO_Write_Pin(GPIOA, PIN_7, HIGH);

	while (1) {
		// Loop principal vazio - tudo é controlado por interrupções
	}
}

// Handlers de interrupção para PB0 e PB1 (mantidos originais)
void EXTI0_IRQHandler() {
	printf("INTERRUPCAO EXTERNA EM PB0\n");
	EXTI_Clear_Pending(EXTI0);
}

void EXTI1_IRQHandler() {
	printf("INTERRUPCAO EXTERNA EM PB1\n");
	EXTI_Clear_Pending(EXTI1);
}

// Handler para K0 (PE4) - Liga LED D2 por 2s
void EXTI4_IRQHandler() {
	printf("INTERRUPCAO EXTERNA EM K0 - LED D2 LIGADO\n");

	// Liga o LED D2
	GPIO_Write_Pin(GPIOA, PIN_6, LOW);

	// Reinicia/inicia o contador (reseta se já estava contando)
	led_d2_counter = D2_TIMEOUT_MS;
	led_d2_active = 1;

	EXTI_Clear_Pending(EXTI4);
}

// Handler para K1 (PE3) - Liga LED D3 por 2s
void EXTI3_IRQHandler() {
	printf("INTERRUPCAO EXTERNA EM K1\n");

	// Liga o LED D3
	GPIO_Write_Pin(GPIOA, PIN_7, LOW);

	// Reinicia/inicia o contador (reseta se já estava contando)
	led_d3_counter = D3_TIMEOUT_MS;
	led_d3_active = 1;

	EXTI_Clear_Pending(EXTI3);
}

// Handler do TIM5 (mantido original)
void TIM5_IRQHandler() {
	TIM5->SR &= ~TIM_SR_UIF;
}

// Handler do TIM6 - Controla os timeouts dos LEDs
void TIM6_DAC_IRQHandler() {
	// Limpa a flag de interrupção do TIM6
	TIM6->SR &= ~TIM_SR_UIF;

	// Controle do LED D2
	if (led_d2_active) {
		if (led_d2_counter > 0) {
			led_d2_counter--;
		} else {
			// Timeout atingido - desliga o LED D2
			GPIO_Write_Pin(GPIOA, PIN_6, HIGH);
			led_d2_active = 0;
			printf("LED D2 DESLIGADO AUTOMATICAMENTE\n");
		}
	}

	// Controle do LED D3
	if (led_d3_active) {
		if (led_d3_counter > 0) {
			led_d3_counter--;
		} else {
			// Timeout atingido - desliga o LED D3
			GPIO_Write_Pin(GPIOA, PIN_7, HIGH);
			led_d3_active = 0;
			printf("LED D3 DESLIGADO AUTOMATICAMENTE\n");
		}
	}
}
