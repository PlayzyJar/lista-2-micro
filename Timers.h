#ifndef TIMERS_H_
#define TIMERS_H_


//Defines dos modos possíveis para os Timers
#define PWM_1 								0b110
#define PWM_2								0b111
#define TOGGLE								0b011
#define ACTIVE     							0b001
#define INACTIVE							0b010
#define ALWAYS_ACTIVE							0b101
#define ALWAYS_INACTIVE							0b100

//Canais dos Timers
enum {
	TIM_CHANNEL_1 = 1,
	TIM_CHANNEL_2,
	TIM_CHANNEL_3,
	TIM_CHANNEL_4
};

//Definições dos cabeçalhos das Funções
void TIM_Setup(TIM_TypeDef* TIMx, uint16_t prescaler, uint16_t period);
void TIM_Compare_Setup(TIM_TypeDef* TIMx, uint8_t channel, uint8_t mode);
void TIM_Update_Compare_Value(TIM_TypeDef* TIMx, uint16_t value, uint8_t channel);

//Estrutura para mapear Timer -> Clock Enable e IRQ
typedef struct {
    TIM_TypeDef* timer;
    uint32_t rcc_apb1_bit;
    uint32_t rcc_apb2_bit;
    IRQn_Type irq_type;
} timer_config_t;

//Tabela de configuração dos timers
static const timer_config_t timer_configs[] = {
    {TIM1,  0,                      RCC_APB2ENR_TIM1EN,  TIM1_UP_TIM10_IRQn},
    {TIM3,  RCC_APB1ENR_TIM3EN,     0,                   TIM3_IRQn},
    {TIM4,  RCC_APB1ENR_TIM4EN,     0,                   TIM4_IRQn},
    {TIM5,  RCC_APB1ENR_TIM5EN,     0,                   TIM5_IRQn},
    {TIM6,  RCC_APB1ENR_TIM6EN,     0,                   TIM6_DAC_IRQn},
    {TIM7,  RCC_APB1ENR_TIM7EN,     0,                   TIM7_IRQn},
    {TIM8,  0,                      RCC_APB2ENR_TIM8EN,  TIM8_UP_TIM13_IRQn},
    {TIM9,  0,                      RCC_APB2ENR_TIM9EN,  TIM1_BRK_TIM9_IRQn},
    {TIM10, 0,                      RCC_APB2ENR_TIM10EN, TIM1_UP_TIM10_IRQn},
    {TIM11, 0,                      RCC_APB2ENR_TIM11EN, TIM1_TRG_COM_TIM11_IRQn},
    {TIM12, RCC_APB1ENR_TIM12EN,    0,                   TIM8_BRK_TIM12_IRQn},
    {TIM13, RCC_APB1ENR_TIM13EN,    0,                   TIM8_UP_TIM13_IRQn},
    {TIM14, RCC_APB1ENR_TIM14EN,    0,                   TIM8_TRG_COM_TIM14_IRQn}
};

//Implementação da função genérica
void TIM_Setup(TIM_TypeDef* TIMx, uint16_t prescaler, uint16_t period) {
    // Encontrar a configuração do timer na tabela
    const timer_config_t* config = NULL;
    for (int i = 0; i < sizeof(timer_configs) / sizeof(timer_configs[0]); i++) {
        if (timer_configs[i].timer == TIMx) {
            config = &timer_configs[i];
            break;
        }
    }

    if (config == NULL) return; // Timer não encontrado

    // Ativar o clock apropriado
    if (config->rcc_apb1_bit != 0) {
        RCC->APB1ENR |= config->rcc_apb1_bit;
    }
    if (config->rcc_apb2_bit != 0) {
        RCC->APB2ENR |= config->rcc_apb2_bit;
    }

    // Configurar o timer
    TIMx->PSC = prescaler - 1;  // Ajusta o registrador do prescaler
    TIMx->ARR = period - 1;     // Ajusta o registrador do auto-reload

    TIMx->EGR = TIM_EGR_UG;     // Atualizar Geração
    TIMx->DIER = TIM_DIER_UIE;  // Habilitar a Interrupção

    NVIC_EnableIRQ(config->irq_type); // Habilitar a Interrupção no NVIC
    TIMx->CR1 |= TIM_CR1_CEN;          // Começar a Contagem
}

void TIM_Compare_Setup(TIM_TypeDef* TIMx, uint8_t channel, uint8_t mode) {
	switch (channel) {
		case TIM_CHANNEL_1:
			switch (mode) {
				case PWM_1:
					TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
					break;
				case PWM_2:
					TIMx->CCMR1 |= TIM_CCMR1_OC1M;
					break;
				case ACTIVE:
					TIMx->CCMR1 |= TIM_CCMR1_OC1M_0;
					break;
				case INACTIVE:
					TIMx->CCMR1 |= TIM_CCMR1_OC1M_1;
					break;
				case ALWAYS_ACTIVE:
					TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0;
					break;
				case ALWAYS_INACTIVE:
					TIMx->CCMR1 |= TIM_CCMR1_OC1M_2;
					break;
				case TOGGLE:
					TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
					break;
			}

			TIMx->CCMR1 |= TIM_CCMR1_OC1PE;
			TIMx->CCER  |= TIM_CCER_CC1E;
			break;

		case TIM_CHANNEL_2:
			switch (mode) {
				case PWM_1:
					TIMx->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
					break;
				case PWM_2:
					TIMx->CCMR1 |= TIM_CCMR1_OC2M;
					break;
				case ACTIVE:
					TIMx->CCMR1 |= TIM_CCMR1_OC2M_0;
					break;
				case INACTIVE:
					TIMx->CCMR1 |= TIM_CCMR1_OC2M_1;
					break;
				case ALWAYS_ACTIVE:
					TIMx->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0;
					break;
				case ALWAYS_INACTIVE:
					TIMx->CCMR1 |= TIM_CCMR1_OC2M_2;
					break;
				case TOGGLE:
					TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
					break;
			}

			TIMx->CCMR1 |= TIM_CCMR1_OC2PE;
			TIMx->CCER  |= TIM_CCER_CC2E;
			break;

		case TIM_CHANNEL_3:
			switch (mode) {
				case PWM_1:
					TIMx->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
					break;
				case PWM_2:
					TIMx->CCMR2 |= TIM_CCMR2_OC3M;
					break;
				case ACTIVE:
					TIMx->CCMR2 |= TIM_CCMR2_OC3M_0;
					break;
				case INACTIVE:
					TIMx->CCMR2 |= TIM_CCMR2_OC3M_1;
					break;
				case ALWAYS_ACTIVE:
					TIMx->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0;
					break;
				case ALWAYS_INACTIVE:
					TIMx->CCMR2 |= TIM_CCMR2_OC3M_2;
					break;
				case TOGGLE:
					TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;
					break;
			}

			TIMx->CCMR2 |= TIM_CCMR2_OC3PE;
			TIMx->CCER  |= TIM_CCER_CC3E;
			break;

		case TIM_CHANNEL_4:
			switch (mode) {
				case PWM_1:
					TIMx->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
					break;
				case PWM_2:
					TIMx->CCMR2 |= TIM_CCMR2_OC4M;
					break;
				case ACTIVE:
					TIMx->CCMR2 |= TIM_CCMR2_OC4M_0;
					break;
				case INACTIVE:
					TIMx->CCMR2 |= TIM_CCMR2_OC4M_1;
					break;
				case ALWAYS_ACTIVE:
					TIMx->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0;
					break;
				case ALWAYS_INACTIVE:
					TIMx->CCMR2 |= TIM_CCMR2_OC4M_2;
					break;
				case TOGGLE:
					TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
					break;
			}

			TIMx->CCMR2 |= TIM_CCMR2_OC4PE;
			TIMx->CCER  |= TIM_CCER_CC4E;
			break;
	}
}

void TIM_Update_Compare_Value(TIM_TypeDef* TIMx, uint16_t value, uint8_t channel) {
	switch (channel) {
		case TIM_CHANNEL_1:
			TIMx->CCR1 = value;
			break;
		case TIM_CHANNEL_2:
			TIMx->CCR2 = value;
			break;
		case TIM_CHANNEL_3:
			TIMx->CCR3 = value;
			break;
		case TIM_CHANNEL_4:
			TIMx->CCR4 = value;
			break;
	}
}

//Macros para facilitar o uso
#define TIM1_Setup(prescaler, period)  TIM_Setup(TIM1, prescaler, period)
#define TIM3_Setup(prescaler, period)  TIM_Setup(TIM3, prescaler, period)
#define TIM4_Setup(prescaler, period)  TIM_Setup(TIM4, prescaler, period)
#define TIM5_Setup(prescaler, period)  TIM_Setup(TIM5, prescaler, period)
#define TIM6_Setup(prescaler, period)  TIM_Setup(TIM6, prescaler, period)
#define TIM7_Setup(prescaler, period)  TIM_Setup(TIM7, prescaler, period)
#define TIM8_Setup(prescaler, period)  TIM_Setup(TIM8, prescaler, period)
#define TIM9_Setup(prescaler, period)  TIM_Setup(TIM9, prescaler, period)
#define TIM10_Setup(prescaler, period) TIM_Setup(TIM10, prescaler, period)
#define TIM11_Setup(prescaler, period) TIM_Setup(TIM11, prescaler, period)
#define TIM12_Setup(prescaler, period) TIM_Setup(TIM12, prescaler, period)
#define TIM13_Setup(prescaler, period) TIM_Setup(TIM13, prescaler, period)
#define TIM14_Setup(prescaler, period) TIM_Setup(TIM14, prescaler, period)

#endif /* TIMERS_H_ */
