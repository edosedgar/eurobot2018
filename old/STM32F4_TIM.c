#include "STM32F4_TIM.h"

//-------------------------------------- Base mode --------------------------------------//

// Base mode initialization
void timInitBase(TIM_TypeDef* TIMx, TIM_Init_Typedef* TIM_Init)
{
	// Initialize clocking for timer
	timInitClock(TIMx);
	
	// Load period value to ARR register
	TIMx->ARR = TIM_Init->TIM_Period;
	
	// Load prescaler value to ARR register
	TIMx->PSC = (TIM_Init->TIM_Prescaler - 1);
	
	// Set upcounting or downcounting mode in CR1 register
	TIMx->CR1 |= TIM_Init->TIM_Dir << 4;
	
	// Upload all registers (by trnsfering data frim shadow registers) by generation of update event
	TIMx->EGR |= TIM_EGR_UG;
	
	return;
}

// Set one pulse mode
void timSetOnePulseMode(TIM_TypeDef* TIMx)
{
	TIMx->CR1 |= TIM_CR1_OPM;
	return;
}

//-------------------------------------- PWM mode --------------------------------------//

// PWM mode 1
void timInitPwm(TIM_TypeDef* TIMx, TIM_Init_Typedef* TIM_Init, float dutyCycle[4], uint8_t channels[4])
{
	// Timer Base initialization
	TIM_Init->TIM_Dir = TIM_DIR_UPCOUNTER;
	timInitBase(TIMx, TIM_Init);
	
	// First channel
	if (channels[0])
	{
		// PWM 1 mode
		TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
		// Fast mode
		TIMx->CCMR1 |= TIM_CCMR1_OC1FE;
		// Preload for channel 1
		TIMx->CCMR1 |= TIM_CCMR1_OC1PE;
		// Capture compare enable
		TIMx->CCER  |= TIM_CCER_CC1E;
		//
		TIMx->CCR1 = (TIMx->ARR) * dutyCycle[0];
	}
	if (channels[1])
	{
		// PWM 1 mode
		TIMx->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
		TIMx->CCMR1 |= TIM_CCMR1_OC2FE;
		TIMx->CCMR1 |= TIM_CCMR1_OC2PE;
		TIMx->CCER  |= TIM_CCER_CC2E;
		TIMx->CCR2 = (TIMx->ARR) * dutyCycle[1];
	}
	if (channels[2])
	{
		TIMx->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
		TIMx->CCMR2 |= TIM_CCMR2_OC3FE;
		TIMx->CCMR2 |= TIM_CCMR2_OC3PE;
		TIMx->CCER  |= TIM_CCER_CC3E;
		TIMx->CCR3 = (TIMx->ARR) * dutyCycle[2];
	}
	if (channels[3])
	{
		TIMx->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
		TIMx->CCMR2 |= TIM_CCMR2_OC4FE;
		TIMx->CCMR2 |= TIM_CCMR2_OC4PE;
		TIMx->CCER  |= TIM_CCER_CC4E;
		//
		TIMx->CCR4 = (TIMx->ARR) * dutyCycle[3];
	}
	
	// Generate update event to transfer all values from shadow registers to real ones
	TIMx->EGR  = TIM_EGR_UG;
	
	return;
}

// Change PWM duty cycle "on fly"
void timPwmChangeDutyCycle(TIM_TypeDef* TIMx, uint8_t channel, float dutyCycle)
{
	if (channel == TIM_PWM_CHANNEL_1)
	{
		TIMx->CCR1 = (TIMx->ARR) * (dutyCycle);
		return;
	}
	
	if (channel == TIM_PWM_CHANNEL_2)
	{
		TIMx->CCR2 = (TIMx->ARR) * (dutyCycle);
		return;
	}
	
	if (channel == TIM_PWM_CHANNEL_3)
	{
		TIMx->CCR3 = (TIMx->ARR) * (dutyCycle);
		return;
	}
	
	if (channel == TIM_PWM_CHANNEL_4)
	{
		TIMx->CCR4 = (TIMx->ARR) * (dutyCycle);
		return;
	}
	// Generate update event to transfer all values from shadow registers to real ones
	TIMx->EGR  = TIM_EGR_UG;
	return;
}

//------------------------------------ Encoder mode ------------------------------------//

// Encoder mode
void timInitEncoder(TIM_TypeDef* TIMx)
{
	// Init clock
	timInitClock(TIMx);
	
	// Select inputs (TI1 and TI2) in capture control mode register
	TIMx->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	
	// Timer auto reload register
	TIMx->ARR |= 0xFFFF;
	
	// Set input polarity (non inverted mode rising edge)
	TIMx->CCER &= ~TIM_CCER_CC1NP | ~TIM_CCER_CC1P;
	
	// Encoder mode 3 (SMS = 0x03), counter counts on both rising and falling edge on both TI1FP2 and TI2FP2
	TIMx->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	
	return;
}


//------------------------- Common functions for different modes -----------------------//

// Get default struct for initialization
TIM_Init_Typedef timGetInitStruct(void)
{
	TIM_Init_Typedef settings;
	settings.TIM_Dir = TIM_DIR_UPCOUNTER;
	settings.TIM_Period = 0xFFFF;
	settings.TIM_Prescaler = 1;
	return settings;
}

// Initialization of clocking
void timInitClock(TIM_TypeDef* TIMx)
{
	if (TIMx == TIM1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
		return;
	}
	else if (TIMx == TIM2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		return;
	}
	else if (TIMx == TIM3)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		return;
	}
	else if (TIMx == TIM4)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		return;
	}
	else if (TIMx == TIM5)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
		return;
	}
	else if (TIMx == TIM5)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
		return;
	}
	else if (TIMx == TIM6)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
		return;
	}
	else if (TIMx == TIM7)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
		return;
	}
	else if (TIMx == TIM8)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	}
	else if (TIMx == TIM9)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
		return;
	}
	else if (TIMx == TIM10)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
		return;
	}
	else if (TIMx == TIM11)
	{
		RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
		return;
	}
	else if (TIMx == TIM12)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
		return;
	}
	else if (TIMx == TIM13)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
		return;
	}
	else if (TIMx == TIM14)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
		return;
	}
	return;
}

// Timer deinitialization
void timDeinit(TIM_TypeDef* TIMx)
{
	TIMx->CR1 = 0;
	TIMx->ARR = 0;
	TIMx->BDTR = 0;
	TIMx->CCER = 0;
	TIMx->CCMR1 = 0;
	TIMx->CCMR2 = 0;
	TIMx->CCR1 = 0;
	TIMx->CCR2 = 0;
	TIMx->CCR3 = 0;
	TIMx->CCR4 = 0;
	TIMx->CNT = 0;
	TIMx->CR2 = 0;
	TIMx->DCR = 0;
	TIMx->DIER = 0;
	TIMx->DMAR = 0;
	TIMx->EGR = 0;
	TIMx->OR = 0;
	TIMx->PSC = 0;
	TIMx->RCR = 0;
	TIMx->SMCR = 0;
	TIMx->SR = 0;
	return;
}

// Timer Enable function
void timEnable(TIM_TypeDef* TIMx)
{
	timClearStatusRegister(TIMx);
	if (!(TIMx->CR1 & TIM_CR1_CEN))
	{
		TIMx->CR1 |= TIM_CR1_CEN;
	}
	return;
}

// Timer Disable function
void timDisable(TIM_TypeDef* TIMx)
{
	TIMx->CR1 &= ~TIM_CR1_CEN;
	return;
}

// Enable interrupts for counter
void timInterruptEnable(TIM_TypeDef* TIMx, uint16_t interruptMacro)
{
	TIMx->DIER |= interruptMacro;
	return;
}

// Disable interrupt
void timInterruptDisable(TIM_TypeDef* TIMx, uint16_t interruptMacro)
{
	TIM1->DIER &= ~interruptMacro;
	return;
}

// Get current value of CNT register
uint32_t timGetCounterValue(TIM_TypeDef* TIMx)
{
	return TIMx->CNT;
}

// Clear status register
void timClearStatusRegister(TIM_TypeDef* TIMx)
{
	TIMx->SR = 0;
	return;
}

// Clear one flag from status register
void timClearStatusRegisterFlag(TIM_TypeDef* TIMx, uint16_t statusFlag)
{
	TIMx->SR &= ~statusFlag;
	return;
}
