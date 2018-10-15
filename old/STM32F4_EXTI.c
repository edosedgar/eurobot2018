#include "STM32F4_EXTI.h"

// Init external interrupt on particular pin
void extiInit(EXTI_Source_Port_Typedef sourcePort,  GPIO_Pin_Number_Typedef  gpioPin, EXTI_Interrupt_Mode_Typedef mode)
{
	// Enable clocking for SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	// External interrupt configuration register
	if (gpioPin <= 0x03)
	{
		SYSCFG->EXTICR[0] |= sourcePort << (0x04 * gpioPin);
	}
	else if (gpioPin <= 0x07  )
	{
		SYSCFG->EXTICR[1] |= sourcePort << (0x04 * (gpioPin - 0x04));
	}
	else if (gpioPin <= 0x0B  )
	{
		SYSCFG->EXTICR[2] |= sourcePort << (0x04 * (gpioPin - 0x08));
	}
	else if (gpioPin <= 0x0F  )
	{
		SYSCFG->EXTICR[3] |= sourcePort << (0x04 * (gpioPin - 0x0C));
	}
	
	// Mode selection
	switch(mode)
	{
		case EXTI_INTERRUPT_MODE_FALLING:
			// Falling trigger selection register
			EXTI->FTSR |= 0x01 << gpioPin;
			break;
		case EXTI_INTERRUPT_MODE_RISING:
			// Rising trigger selection register
			EXTI->RTSR |= 0x01 << gpioPin;
			break;
		case EXTI_INTERRUPT_MODE_RISING_AND_FALLING:
			// Falling trigger selection register
			EXTI->FTSR |= 0x01 << gpioPin;
			// Rising trigger selection register
			EXTI->RTSR |= 0x01 << gpioPin;
			break;
	}
	
	// Turn on interrupt in Interrupt mask register
	EXTI->IMR |= 0x01 << gpioPin;
}

// Turn off interrupt
void extiClearInterrupt(GPIO_Pin_Number_Typedef  gpioPin)
{
	// Turn off line in Interrupt mask register
	EXTI->IMR &= ~(0x01 << gpioPin);
	return;
}
