#include "main.h"

#include "Utility.h"

#include <stdio.h>



int main(void)

{

    /* Inicializações */

    Utility_Init();



    // GPIO setup

    GPIO_Clock_Enable(GPIOA);

    GPIO_Pin_Mode(GPIOA, PIN_1, ALTERNATE);

    GPIO_Alternate_Function(GPIOA, PIN_1, AF2); // PA1 -> TIM5_CH2



    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    TIM5->CR1 &= ~TIM_CR1_DIR;



    // 84 MHz / (PSC + 1) / (ARR + 1) = 1 Hz

    // Exemplo: PSC = 8399, ARR = 9999 => 84MHz / 8400 / 10000 = 1 Hz

    TIM5->PSC = 8399;         // Divide 84MHz por 8400 => 10kHz

    TIM5->ARR = 9999;         // Conta 10000 ciclos => 1 segundo



    TIM5->CCMR1 &= ~(TIM_CCMR1_OC2M);

    TIM5->CCMR1 |= (0b110 << 12);        // OC2M = PWM mode 1

    TIM5->CCMR1 |= TIM_CCMR1_OC2PE;      // Enable preload



    TIM5->CCER |= TIM_CCER_CC2E;         // Enable output on CH2

    TIM5->EGR = TIM_EGR_UG;              // Update registers

    TIM5->CCR2 = 5000;                   // 500ms pulse width (50%)

    TIM5->CR1 |= TIM_CR1_CEN;            // Enable timer



    /* Loop principal */

    while (1)

    {

    }

}