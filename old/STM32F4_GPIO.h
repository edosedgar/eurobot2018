#ifndef STM32F4_GPIO
#define STM32F4_GPIO

#include "stm32f4xx.h"

// Pins
typedef enum
{
	GPIO_Pin_0 = 0x00, // Pin 0 selected  
	GPIO_Pin_1,        // Pin 1 selected  
	GPIO_Pin_2,        // Pin 2 selected  
	GPIO_Pin_3,        // Pin 3 selected  
	GPIO_Pin_4,        // Pin 4 selected  
	GPIO_Pin_5,        // Pin 5 selected  
	GPIO_Pin_6,        // Pin 6 selected  
	GPIO_Pin_7,        // Pin 7 selected  
	GPIO_Pin_8,        // Pin 8 selected  
	GPIO_Pin_9,        // Pin 9 selected  
	GPIO_Pin_10,       // Pin 10 selected  
	GPIO_Pin_11,       // Pin 11 selected  
	GPIO_Pin_12,       // Pin 12 selected  
	GPIO_Pin_13,       // Pin 13 selected  
	GPIO_Pin_14,       // Pin 14 selected  
	GPIO_Pin_15        // Pin 15 selected  
}GPIO_Pin_Number_Typedef;

// GPIO mode
typedef enum
{
	GPIO_MODE_IN  = 0x00, // GPIO Input Mode
	GPIO_MODE_OUT = 0x01, // GPIO Output Mode
	GPIO_MODE_AF  = 0x02, // GPIO Alternate function Mode
	GPIO_MODE_AN  = 0x03  // GPIO Analog Mode
}GPIO_Mode_TypeDef;

// GPIO output mode type
typedef enum
{ 
	// Push - pull mode
	GPIO_OUTPUT_MODE_PP = 0x00,
	// Open drain mode
	GPIO_OUTPUT_MODE_OD = 0x01  
}GPIO_Output_Mode_TypeDef;

// GPIO pull up, pull down mode
typedef enum
{ 
	GPIO_PUPD_NOPULL = 0x00,
	GPIO_PUPD_UP     = 0x01,
	GPIO_PUPD_DOWN   = 0x02
}GPIO_PuPd_TypeDef;

// GPIO output level 
typedef enum
{ 
	GPIO_LEVEL_LOW  = 0x00,
	GPIO_LEVEL_HIGH = 0x01
}GPIO_Level_TypeDef;

// Struct for GPIO pin initialization
typedef struct
{             
	GPIO_Mode_TypeDef        GPIO_Mode;      
	GPIO_Output_Mode_TypeDef GPIO_Output_Mode;
	GPIO_PuPd_TypeDef        GPIO_PuPd_Mode;
}GPIO_Init_TypeDef;

// Struct for GPIO pin
typedef struct
{             
	GPIO_TypeDef*           port;
	GPIO_Pin_Number_Typedef number;
}GPIO_Pin_TypeDef;

// Macros for alternative functions
// AF 1 selection     
#define GPIO_AF_TIM1          ((uint8_t)0x01)  // TIM1 alternative function  
#define GPIO_AF_TIM2          ((uint8_t)0x01)  // TIM2 alternative function  

// AF 2 selection   
#define GPIO_AF_TIM3          ((uint8_t)0x02)  // TIM3 alternative function  
#define GPIO_AF_TIM4          ((uint8_t)0x02)  // TIM4 alternative function  
#define GPIO_AF_TIM5          ((uint8_t)0x02)  // TIM5 alternative function  

// AF 3 selection   
#define GPIO_AF_TIM8          ((uint8_t)0x03)  // TIM8 alternative function  
#define GPIO_AF_TIM9          ((uint8_t)0x03)  // TIM9 alternative function  
#define GPIO_AF_TIM10         ((uint8_t)0x03)  // TIM10 alternative function  
#define GPIO_AF_TIM11         ((uint8_t)0x03)  // TIM11 alternative function  

// AF 4 selection   
#define GPIO_AF_I2C1          ((uint8_t)0x04)  // I2C1 alternative function  
#define GPIO_AF_I2C2          ((uint8_t)0x04)  // I2C2 alternative function  
#define GPIO_AF_I2C3          ((uint8_t)0x04)  // I2C3 alternative function   

// AF 7 selection    
#define GPIO_AF_USART1        ((uint8_t)0x07)  // USART1 alternative function  
#define GPIO_AF_USART2        ((uint8_t)0x07)  // USART2 alternative function  
#define GPIO_AF_USART3        ((uint8_t)0x07)  // USART3 alternative function  

// AF 8 selection      
#define GPIO_AF_UART4         ((uint8_t)0x08)  // UART4 alternative function  
#define GPIO_AF_UART5         ((uint8_t)0x08)  // UART5 alternative function  
#define GPIO_AF_USART6        ((uint8_t)0x08)  // USART6 alternative function  


// AF 9 selection 
#define GPIO_AF_TIM12         ((uint8_t)0x09)  // TIM12 alternative function  
#define GPIO_AF_TIM13         ((uint8_t)0x09)  // TIM13 alternative function  
#define GPIO_AF_TIM14         ((uint8_t)0x09)  // TIM14 alternative function  




// Initialization of clocking
void gpioInitClock(GPIO_TypeDef* GPIOx);

// Basic initialization of pin
void gpioInitPin(GPIO_TypeDef* GPIOx,  GPIO_Pin_Number_Typedef  GPIO_Pin, GPIO_Mode_TypeDef GPIO_Mode, GPIO_Output_Mode_TypeDef GPIO_Output_Mode, GPIO_PuPd_TypeDef GPIO_PuPd_Mode);

// Initialization of pin in alternative function mode
void gpioInitPinAf(GPIO_TypeDef* GPIOx,  GPIO_Pin_Number_Typedef  GPIO_Pin, uint8_t GPIO_AF_Number);

// Set HIGH level or LOW level on selected pin
void gpioPinSetLevel(GPIO_TypeDef* GPIOx, GPIO_Pin_Number_Typedef GPIO_Pin, GPIO_Level_TypeDef  GPIO_Level);

// Set HIGH level or LOW level on selected pin
void gpioPinChangeLevel(GPIO_TypeDef* GPIOx, GPIO_Pin_Number_Typedef GPIO_Pin);

// Read current pin's logic level (input mode)
GPIO_Level_TypeDef gpioPinReadInput(GPIO_TypeDef* GPIOx, GPIO_Pin_Number_Typedef GPIO_Pin);

// Read current pin's logic level (output mode)
GPIO_Level_TypeDef gpioPinReadOutput(GPIO_TypeDef* GPIOx, GPIO_Pin_Number_Typedef GPIO_Pin);

#endif
