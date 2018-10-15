#ifndef STM32F4_EXTI
 #define STM32F4_EXTI
 
#include "stm32f4xx.h"
#include "STM32F4_GPIO.h"

typedef enum
{
	EXTI_INTERRUPT_MODE_RISING,
	EXTI_INTERRUPT_MODE_FALLING,
	EXTI_INTERRUPT_MODE_RISING_AND_FALLING,
}EXTI_Interrupt_Mode_Typedef;

typedef enum
{
	EXTI_GPIOA_SOURCE_PORT,
	EXTI_GPIOB_SOURCE_PORT,
	EXTI_GPIOC_SOURCE_PORT,
	EXTI_GPIOD_SOURCE_PORT,
	EXTI_GPIOE_SOURCE_PORT,
	EXTI_GPIOF_SOURCE_PORT,
	EXTI_GPIOG_SOURCE_PORT,
	EXTI_GPIOH_SOURCE_PORT,
	EXTI_GPIOI_SOURCE_PORT,
	
} EXTI_Source_Port_Typedef;


//--------------------------------------------- FUNCTIONS ------------------------------------------------------//

// Init external interrupt on particular pin
void extiInit(EXTI_Source_Port_Typedef sourcePort,  GPIO_Pin_Number_Typedef  gpioPin, EXTI_Interrupt_Mode_Typedef mode);

// Turn off interrupt
void extiClearInterrupt(GPIO_Pin_Number_Typedef  gpioPin);
#endif
