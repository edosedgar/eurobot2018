#include "STM32F4_GPIO.h"

// Frequently used settings

// Clock initialization
void gpioInitClock(GPIO_TypeDef* GPIOx)
{
	if (GPIOx == GPIOA)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		return;
	}
	if (GPIOx == GPIOB)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
		return;
	}
	if (GPIOx == GPIOC)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		return;
	}
	if (GPIOx == GPIOD)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		return;
	}
	if (GPIOx == GPIOE)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
		return;
	}
	if (GPIOx == GPIOF)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
		return;
	}
	if (GPIOx == GPIOG)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
		return;
	}
	if (GPIOx == GPIOH)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
		return;
	}
	if (GPIOx == GPIOI)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
		return;
	}
	return;
}

// Initialization of pin
void gpioInitPin(GPIO_TypeDef* GPIOx,  GPIO_Pin_Number_Typedef  GPIO_Pin, GPIO_Mode_TypeDef GPIO_Mode, GPIO_Output_Mode_TypeDef GPIO_Output_Mode, GPIO_PuPd_TypeDef GPIO_PuPd_Mode)
{
	// Initialization of a clock
	gpioInitClock(GPIOx);
	
	// Initialization of pin's mode
	
	// Delete all necessary bits of MODERx and Write appropriate bits to MODERx to set mode
	GPIOx->MODER = (GPIOx->MODER & ~(GPIO_MODER_MODE0_Msk << (GPIO_Pin * 2))) | (GPIO_Mode << (GPIO_Pin * 2));
	
	// Pull up and pull down resistors
	GPIOx->PUPDR = (GPIOx->PUPDR & ~(GPIO_PUPDR_PUPDR0 << (GPIO_Pin * 2))) | (GPIO_PuPd_Mode << (GPIO_Pin * 2));
	
	// Initialization of output mode
	if ((GPIO_Mode == GPIO_MODE_OUT) || (GPIO_Mode == GPIO_MODE_AF))
	{
		// Clear bit and set bit
		GPIOx->OTYPER = (GPIOx->OTYPER & ~(1 << GPIO_Pin)) | ((GPIO_Output_Mode) << (GPIO_Pin));
	}
	return;
}


// Initialization of pin in alternative function mode
void gpioInitPinAf(GPIO_TypeDef* GPIOx,  GPIO_Pin_Number_Typedef  GPIO_Pin, uint8_t GPIO_AF_Number)
{
	// Initialization of a clock
	gpioInitClock(GPIOx);
	
	// Clear and Write appropriate bits to MODERx
	GPIOx->MODER = (GPIOx->MODER & ~(GPIO_MODER_MODE0_Msk << (GPIO_Pin * 2))) | (GPIO_MODE_AF << (GPIO_Pin * 2)); 
	
	// Selection of Alternative function
	if (GPIO_Pin <= 7 )
	{
		// Clear bits for AF and write appropriate bits
		GPIOx->AFR[0] = (GPIOx->AFR[0] & ~(15 << (GPIO_Pin * 4))) | (GPIO_AF_Number << (GPIO_Pin * 4));
	}
	else if (GPIO_Pin > 7 )
	{
		// Clear bits for AF and write appropriate bits
		GPIOx->AFR[1] = (GPIOx->AFR[1] & ~(15 << ((GPIO_Pin - 8) * 4))) | GPIO_AF_Number << ((GPIO_Pin - 8) * 4);
	}
	return;
}

// Set HIGH level or LOW level on selected pin
void gpioPinSetLevel(GPIO_TypeDef* GPIOx, GPIO_Pin_Number_Typedef GPIO_Pin,  GPIO_Level_TypeDef  GPIO_Output_Level)
{
	if (GPIO_Output_Level == GPIO_LEVEL_LOW)
	{
		// Reset pin to zero by writing to BSRR register
		GPIOx->BSRR |= 1 << (GPIO_Pin + 16);
		return;
	}
	else if (GPIO_Output_Level == GPIO_LEVEL_HIGH)
	{
		// Set pin to high level by writing to BSRR register
		GPIOx->BSRR |= 1 << (GPIO_Pin);
	}
}

// Set HIGH level or LOW level on selected pin
void gpioPinChangeLevel(GPIO_TypeDef* GPIOx, GPIO_Pin_Number_Typedef GPIO_Pin)
{
	GPIOx->ODR ^= 1 << GPIO_Pin;
	return;
}

// Read current pin's logic level (input mode)
GPIO_Level_TypeDef gpioPinReadInput(GPIO_TypeDef* GPIOx, GPIO_Pin_Number_Typedef GPIO_Pin)
{
	if (GPIOx->IDR & (1 << GPIO_Pin))
		return GPIO_LEVEL_HIGH;
	else
		return GPIO_LEVEL_LOW;
}

// Read current pin's logic level (output mode)
GPIO_Level_TypeDef gpioPinReadOutput(GPIO_TypeDef* GPIOx, GPIO_Pin_Number_Typedef GPIO_Pin)
{
	if (GPIOx->ODR & (1 << GPIO_Pin))
		return GPIO_LEVEL_HIGH;
	else
		return GPIO_LEVEL_LOW;
}
