#include "STM32F4_RCC.h"

// Get value of APB1 prescaler
uint8_t rccGetApb1Prescaler()
{
	uint8_t i;
	uint8_t prescaler = 1;
	// Get PPRE1 
	uint8_t ppre1Value = (RCC->CFGR & RCC_CFGR_PPRE1_Msk ) >> RCC_CFGR_PPRE1_Pos;
	if (ppre1Value < 4)
	{
		return prescaler;
	}
	else
	{
		prescaler = 2;
		for (i = 0; i < (ppre1Value - 4); i++)
		{
			prescaler *= 2;
		}
	}
	return prescaler;
}

// Get value of APB2 prescaler
uint8_t rccGetApb2Prescaler()
{
	uint8_t i;
	uint8_t prescaler = 1;
	// Get PPRE2 
	uint8_t ppre2Value = (RCC->CFGR & RCC_CFGR_PPRE2_Msk ) >> RCC_CFGR_PPRE2_Pos;
	if (ppre2Value < 4)
	{
		return prescaler;
	}
	else
	{
		prescaler = 2;
		for (i = 0; i < (ppre2Value - 4); i++)
		{
			prescaler *= 2;
		}
	}
	return prescaler;
}

// Get value of AHB prescaler
uint16_t rccGetAhbPrescaler()
{
	uint8_t i;
	uint16_t prescaler = 0x1;
	// Get HPRE 
	uint8_t hpreValue = (RCC->CFGR & RCC_CFGR_HPRE_Msk ) >> RCC_CFGR_HPRE_Pos;
	if (hpreValue < 8)
	{
		return prescaler;
	}
	else
	{
		prescaler = 0x2;
		for (i = 0; i < (hpreValue - 8); i++)
		{
			prescaler *= 0x2;
		}
		if (hpreValue > 11)
		{
			prescaler *= 0x2;
		}
	}
	return prescaler;
}

// Set maximum clock speed that equals 168 MHz 
void SystemClockInitHse168Mhz()
{
		// Turn on external oscillator
	RCC->CR |= RCC_CR_HSEON;
	
	// Waiting for HSE to be ready
	while (!(RCC->CR & RCC_CR_HSERDY));
	
	// Clocking of power control
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	PWR->CR |= PWR_CR_VOS;
	
	// Prescalers of buses
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	
	// PLL setup
	RCC->PLLCFGR = 8 + (336 << 6) + (((2 >> 1) -1) << 16) + RCC_PLLCFGR_PLLSRC_HSE + (7 << 24);
	
	// Turn PLL on
	RCC->CR |= RCC_CR_PLLON;
	
	// Wait for PLL to be ready
	while (!(RCC->CR & RCC_CR_PLLRDY))

	// Setup of  instruction cache, data cache and wait state
	FLASH->ACR = FLASH_ACR_ICEN + FLASH_ACR_DCEN + FLASH_ACR_LATENCY_5WS + FLASH_ACR_PRFTEN;
	
	// Choose PLL as a clocking source
	// Clear SW bit
	RCC->CFGR &= ~RCC_CFGR_SW;

	// Set SWS_PLL bit
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait for transition processes
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
}
