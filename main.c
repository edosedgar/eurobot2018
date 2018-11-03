#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "io_syscall.h"
#include "terminal.h"
#include "motor_kinematics.h"
#include "collision_avoidance.h"

#include "Board.h"
#include "Communication.h"
#include "Manipulators.h"
#include "Collision_avoidance_old.h"

// Struct with Robot's status
extern RobotStatus Robot;

// I2C module for rangefinders
extern I2C_Module_With_State_Typedef I2CModule;

// Time of Robot start
extern uint32_t timeOfStart;

// Communication errors
uint32_t numberOfReceivedPackages;
uint32_t numberOfChecksumErrors;
uint32_t numberOfSmallLengthErrors;

/**
  *   System Clock Configuration
  *   The system Clock is configured as follow :
  *   System Clock source            = PLL (HSE)
  *   SYSCLK(Hz)                     = 168000000
  *   HCLK(Hz)                       = 168000000
  *   AHB Prescaler                  = 1
  *   APB1 Prescaler                 = 4
  *   APB2 Prescaler                 = 2
  *   HSE Frequency(Hz)              = 8000000
  *   PLL_M                          = 8
  *   PLL_N                          = 336
  *   PLL_P                          = 2
  *   VDD(V)                         = 3.3
  *   Main regulator output voltage  = Scale1 mode
  *   Flash Latency(WS)              = 5
  */
static void rcc_config()
{
        /* Enable HSE oscillator */
        LL_RCC_HSE_Enable();
        while(LL_RCC_HSE_IsReady() != 1);

        /* Set FLASH latency */
        LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

        /* Main PLL configuration and activation */
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8,
                                    336, LL_RCC_PLLP_DIV_2);
        LL_RCC_PLL_Enable();
        while(LL_RCC_PLL_IsReady() != 1);

        /* Sysclk activation on the main PLL */
        LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
        LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
        while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

        /* Set APB1 & APB2 prescaler */
        LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
        LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

        /* Set systick to 1ms */
        SysTick_Config(168000000/1000);

        /* Update CMSIS variable (which can be updated also
         * through SystemCoreClockUpdate function) */
        SystemCoreClock = 168000000;
}

static void gpio_config(void)
{
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
        LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
        return;
}

int main() {
        rcc_config();
        gpio_config();
        NVIC_SetPriorityGrouping(0);

        xTaskCreateStatic(terminal_manager, "TERM_MAN", TERM_MAN_STACK_DEPTH,
                          NULL, 2, terminal_manager_ts, &terminal_manager_tb);
        xTaskCreateStatic(motor_kinematics, "MOTOR_KIN", MOTOR_KIN_STACK_DEPTH,
                          NULL, 2, motor_kinematics_ts, &motor_kinematics_tb);
        xTaskCreateStatic(collision_avoidance, "COL_AVOID",
                          COL_AVOID_STACK_DEPTH, NULL, 2, collision_avoidance_ts,
                          &collision_avoidance_tb);

        vTaskStartScheduler();
        return 0;
}

int maina()
{
        // Init everything
    boardInitAll();
        // Upload angle values to manipulators
        initManipulators();
        // Turn on Forward kinematics calculations and Collision avoidance
        Robot.forwardKinCalcStatusFlag = 0x01;
        Robot.collisionAvoidanceStatusFlag = 0x01;
        while (1)
        {
                switch(getPackage())
                {
                        case SMALL_LENGTH:
                                numberOfSmallLengthErrors++;
                                break;
                        case WRONG_CHECKSUM:
                                numberOfChecksumErrors++;
                                break;
                        case SUCCESFULL_PACKAGE:
                                numberOfReceivedPackages++;
                        default:
                                break;
                };
                checkCommandAndExecute();
                if (Robot.startupStatusFlag)
                {
                        // If time is up
                        if (checkTimeout(timeOfStart, ROBOT_TIME_OF_MATCH_TENTH_OF_MS))
                        {
                                turnEverythingOff();
                                Robot.startupStatusFlag = 0x00;
                        }
                }
        }
}
