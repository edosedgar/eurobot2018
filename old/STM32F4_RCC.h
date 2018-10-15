#ifndef STM32F4_RCC
#define STM32F4_RCC

#include "stm32f4xx.h"
#define GLOBAL_CORE_FREQUENCY    168 // MHz

// Get value of APB1 prescaler
uint8_t rccGetApb1Prescaler(void);

// Get value of APB2 prescaler
uint8_t rccGetApb2Prescaler(void);

// Get value of AHB prescaler
uint16_t rccGetAhbPrescaler(void);

// Set maximum clock speed that equals 168 MHz 
void SystemClockInitHse168Mhz(void);

#endif
