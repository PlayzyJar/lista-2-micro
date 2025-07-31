#include "main.h"
#include "Utility.h"
#include <stdio.h>

int main(void) {
	/* Inicializações */

	Utility_Init();

	USART1_Init();

	GPIO_Clock_Enable(GPIOA);
	GPIO_Clock_Enable(GPIOB);
	GPIO_Clock_Enable(GPIOE);

	GPIO_Pin_Mode(GPIOA, PIN_1, ALTERNATE);
	GPIO_Alternate_Function(GPIOA, PIN_1, AF2); // PA1 -> TIM5_CH2

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->CR1 &= ~TIM_CR1_DIR;

	TIM5->PSC = 8399;         // Divide 84MHz por 8400 => 10kHz
	TIM5->ARR = 9999;         // Conta 10000 ciclos => 1 segundo

	TIM5->CCMR1 &= ~(TIM_CCMR1_OC2M);
	TIM5->CCMR1 |= (0b110 << 12);        // OC2M = PWM mode 1
	TIM5->CCMR1 |= TIM_CCMR1_OC2PE;      // Enable preload

	TIM5->CCER |= TIM_CCER_CC2E;         // Enable output on CH2
	TIM5->EGR = TIM_EGR_UG;              // Update registers
	TIM5->CCR2 = 5000;                   // 500ms pulse width (50%)
	TIM5->CR1 |= TIM_CR1_CEN;            // Enable timer

	GPIO_Pin_Mode(GPIOB, PIN_1, INPUT);
	GPIO_Resistor_Enable(GPIOB, PIN_1, PULL_UP);

	GPIO_Pin_Mode(GPIOB, PIN_0, INPUT);
	GPIO_Resistor_Enable(GPIOB, PIN_0, PULL_UP);

	GPIO_Pin_Mode(GPIOE, PIN_4, INPUT);
	GPIO_Resistor_Enable(GPIOE, PIN_4, PULL_DOWN);

	GPIO_Pin_Mode(GPIOE, PIN_3, INPUT);
	GPIO_Resistor_Enable(GPIOE, PIN_3, PULL_DOWN);

	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_SetPriority(EXTI1_IRQn, 0);

	EXTI_Config(EXTI0, GPIOB, RISING_EDGE);
	NVIC_EnableIRQ(EXTI0_IRQn);

	EXTI_Config(EXTI1, GPIOB, RISING_EDGE);
	NVIC_EnableIRQ(EXTI1_IRQn);

	while (1) {
		if (GPIO_Read_Pin(GPIOE, PIN_4))
			printf("botao K0");

		if (GPIO_Read_Pin(GPIOE, PIN_3))
			printf("botao K1");

		Delay_ms(200);
	}

}

void EXTI0_IRQHandler() {
	printf("INTERRUPCAO EXTERNA EM PB0\n\n");
	EXTI_Clear_Pending(EXTI0);
}

void EXTI1_IRQHandler() {
	printf("INTERRUPCAO EXTERNA EM PB1\n");
	EXTI_Clear_Pending(EXTI1);
}
