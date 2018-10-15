#ifndef STM32F4_TIM
#define STM32F4_TIM
#include "stm32f407xx.h"

// Type definition for counter direction
typedef enum
{
	TIM_DIR_UPCOUNTER = 0,
	TIM_DIR_DOWNCOUNTER
}TIM_Dir_Typedef;

// Struct for basic initialization
typedef struct
{
	uint16_t TIM_Prescaler;
	uint32_t TIM_Period;
	TIM_Dir_Typedef TIM_Dir;
}TIM_Init_Typedef;

// Struct for channel PWM control
typedef struct
{
	TIM_TypeDef* timModule;
	uint8_t channel;
}TIM_PWM_Typedef;

// Timer's channels enumerator
enum
{
	TIM_PWM_CHANNEL_1 = 0x01,
	TIM_PWM_CHANNEL_2,
	TIM_PWM_CHANNEL_3,
	TIM_PWM_CHANNEL_4
};

//-------------------------------------- Base mode --------------------------------------//

// Timer initialization in base mode 
void timInitBase(TIM_TypeDef* TIMx, TIM_Init_Typedef* TIM_Init);

// Set one pulse mode
void timSetOnePulseMode(TIM_TypeDef* TIMx);

//-------------------------------------- PWM mode --------------------------------------//

// PWM 1 mode initialization (duty cycles is stored in dutyCycle[i], selected  channels is stored in channels[i], i = 0..3)
void timInitPwm(TIM_TypeDef* TIMx, TIM_Init_Typedef* TIM_Init, float dutyCycle[4], uint8_t channels[4]);

// Change PWM duty cycle "on fly"
void timPwmChangeDutyCycle(TIM_TypeDef* TIMx, uint8_t channel, float dutyCycle);

//------------------------------------ Encoder mode ------------------------------------//

// Uses channel 1 and channel 2 of specific timer for quadrature encoder connection
void timInitEncoder(TIM_TypeDef* TIMx);

//------------------------- Common functions for different modes -----------------------//

// Get default struct for initialization
TIM_Init_Typedef timGetInitStruct(void);

// Initialization of clocking
void timInitClock(TIM_TypeDef* TIMx);

// Timer deinitialization
void timDeinit(TIM_TypeDef* TIMx);

// Timer Enable function
void timEnable(TIM_TypeDef* TIMx);

// Timer Disable function
void timDisable(TIM_TypeDef* TIMx);

// Enable interrupts for counter
// As input parameter interruptMacros use macroinstructions from CMSIS (for instance TIM_DIER_UIE)
void timInterruptEnable(TIM_TypeDef* TIMx, uint16_t interruptMacro);

// Disable interrupt the same way as it is described above
// As input parameter interruptMacros use macroinstructions from CMSIS (for instance TIM_DIER_UIE)
void timInterruptDisable(TIM_TypeDef* TIMx, uint16_t interruptMacro);

// Get current value of CNT register
uint32_t timGetCounterValue(TIM_TypeDef* TIMx);

// Clear status register
void timClearStatusRegister(TIM_TypeDef* TIMx);

// Clear one flag from status register
// As input parameter statusFlag use macroinstructions from CMSIS (for instance TIM_SR_UIF)
void timClearStatusRegisterFlag(TIM_TypeDef* TIMx, uint16_t statusFlag);

#endif
