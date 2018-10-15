#ifndef STM32F4_BOARD
#define STM32F4_BOARD

#include "STM32F4_GPIO.h"
#include "STM32F4_TIM.h"
#include "STM32F4_UART.h"
#include "STM32F4_RCC.h"
#include "STM32F4_I2C.h"
#include "STM32F4_EXTI.h"
#include "Dynamixel_control.h"
#include "Interrupts.h"

//--------------------------------------------- I2C module ---------------------------------------------------//
#define I2C_MODULE                       I2C2
#define I2C_MODULE_EVENT_IRQN            I2C2_EV_IRQn
#define I2C_MODULE_ERROR_IRQN            I2C2_ER_IRQn
#define I2C_MODULE_PIN_AF                GPIO_AF_I2C2
#define I2C_MODULE_SDA_PIN_PORT          GPIOB
#define I2C_MODULE_SDA_PIN_NUMBER        GPIO_Pin_11
#define I2C_MODULE_SCL_PIN_PORT          GPIOB
#define I2C_MODULE_SCL_PIN_NUMBER        GPIO_Pin_10

//--------------------------------------------- USART modules ------------------------------------------------//

#define DYNAMIXEL_USART_MODULE           UART4
#define DYNAMIXEL_USART_IRQN             UART4_IRQn
#define DYNAMIXEL_USART_BAUDRATE         1000000
#define DYNAMIXEL_USART_PIN_AF           GPIO_AF_UART4
#define DYNAMIXEL_USART_TX_PIN_PORT      GPIOC
#define DYNAMIXEL_USART_TX_PIN_NUMBER    GPIO_Pin_10
#define DYNAMIXEL_USART_RX_PIN_PORT      GPIOC
#define DYNAMIXEL_USART_RX_PIN_NUMBER    GPIO_Pin_11

#define COM_USART_MODULE                 USART1
#define COM_USART_IRQN                   USART1_IRQn
#define COM_USART_BAUDRATE               250000
#define COM_USART_PIN_AF                 GPIO_AF_USART1
#define COM_USART_TX_PIN_PORT            GPIOB
#define COM_USART_TX_PIN_NUMBER          GPIO_Pin_6
#define COM_USART_RX_PIN_PORT            GPIOB
#define COM_USART_RX_PIN_NUMBER          GPIO_Pin_7

//--------------------------------------------- Encoders -----------------------------------------------------//
// Initial value for TIM->CNT register (30000)
#define ENCODER_CNT_INITIAL_VALUE        0x7530

#define ENCODER_4_TIM_MODULE             TIM8
#define ENCODER_4_CHA_PORT               GPIOC
#define ENCODER_4_CHA_PIN                GPIO_Pin_7
#define ENCODER_4_CHB_PORT               GPIOC
#define ENCODER_4_CHB_PIN                GPIO_Pin_6
#define ENCODER_4_PIN_AF                 GPIO_AF_TIM8
#define ENCODER_4_CNT                    ((uint16_t *)&(ENCODER_4_TIM_MODULE->CNT))

#define ENCODER_3_TIM_MODULE             TIM1
#define ENCODER_3_CHA_PORT               GPIOE
#define ENCODER_3_CHA_PIN                GPIO_Pin_11
#define ENCODER_3_CHB_PORT               GPIOE
#define ENCODER_3_CHB_PIN                GPIO_Pin_9
#define ENCODER_3_PIN_AF                 GPIO_AF_TIM1
#define ENCODER_3_CNT                    ((uint16_t *)&(ENCODER_3_TIM_MODULE->CNT))

#define ENCODER_2_TIM_MODULE             TIM3
#define ENCODER_2_CHA_PORT               GPIOA
#define ENCODER_2_CHA_PIN                GPIO_Pin_6
#define ENCODER_2_CHB_PORT               GPIOA
#define ENCODER_2_CHB_PIN                GPIO_Pin_7
#define ENCODER_2_PIN_AF                 GPIO_AF_TIM3
#define ENCODER_2_CNT                    ((uint16_t *)&(ENCODER_2_TIM_MODULE->CNT))

#define ENCODER_1_TIM_MODULE             TIM2
#define ENCODER_1_CHA_PORT               GPIOB
#define ENCODER_1_CHA_PIN                GPIO_Pin_3
#define ENCODER_1_CHB_PORT               GPIOA
#define ENCODER_1_CHB_PIN                GPIO_Pin_15
#define ENCODER_1_PIN_AF                 GPIO_AF_TIM2
#define ENCODER_1_CNT                    ((uint16_t *)&(ENCODER_1_TIM_MODULE->CNT))

//--------------------------------------------- Motor PWM control  --------------------------------------------------//

// ARR = 42000, PSC = 2, fapb1 = 42 MHZ, PWM frequency = 1000 Hz
#define MOTOR_PWM_TIM_MODULE             TIM4
#define MOTOR_PWM_TIM_PSC                0x02
#define MOTOR_PWM_TIM_ARR                0xA410
#define MOTOR_PWM_PIN_AF                 GPIO_AF_TIM4

#define MOTOR_CH1_NUMBER                 0x01
#define MOTOR_CH2_NUMBER                 0x02
#define MOTOR_CH3_NUMBER                 0x03
#define MOTOR_CH4_NUMBER                 0x04

#define MOTOR_CH_PWM_PORT                GPIOD
#define MOTOR_CH1_PWM_PIN                GPIO_Pin_12
#define MOTOR_CH2_PWM_PIN                GPIO_Pin_13
#define MOTOR_CH3_PWM_PIN                GPIO_Pin_14
#define MOTOR_CH4_PWM_PIN                GPIO_Pin_15

#define MOTOR_CH1_DIR_PORT               GPIOC
#define MOTOR_CH1_DIR_PIN                GPIO_Pin_14
#define MOTOR_CH2_DIR_PORT               GPIOC
#define MOTOR_CH2_DIR_PIN                GPIO_Pin_15
#define MOTOR_CH3_DIR_PORT               GPIOE
#define MOTOR_CH3_DIR_PIN                GPIO_Pin_10
#define MOTOR_CH4_DIR_PORT               GPIOE
#define MOTOR_CH4_DIR_PIN                GPIO_Pin_14

#define MOTOR_CH1_EN_PORT                GPIOC
#define MOTOR_CH1_EN_PIN                 GPIO_Pin_14
#define MOTOR_CH2_EN_PORT                GPIOC
#define MOTOR_CH2_EN_PIN                 GPIO_Pin_15
#define MOTOR_CH3_EN_PORT                GPIOE
#define MOTOR_CH3_EN_PIN                 GPIO_Pin_10
#define MOTOR_CH4_EN_PORT                GPIOE
#define MOTOR_CH4_EN_PIN                 GPIO_Pin_14

//--------------------------------------------- Timer for motor control (100 Hz) ------------------------------------------------//

// ARR = 42000, PSC = 20, fapb1 = 42 MHZ, Frequency = 100 Hz
#define MOTOR_CONTROL_TIM_MODULE         TIM6
#define MOTOR_CONTROL_TIM_PSC            0x14
#define MOTOR_CONTROL_TIM_ARR            0xA410
#define MOTOR_CONTROL_IRQN               TIM6_DAC_IRQn
#define MOTOR_CONTROL_PERIOD             0.01f 
#define MOTOR_CONTROL_PERIOD_MILLISEC    (uint32_t)(MOTOR_CONTROL_PERIOD * 1000)
#define MOTOR_CONTROL_CALC_COEF          MOTOR_CONTROL_PERIOD/MOTOR_CONTROL_TIM_ARR

//--------------------------------------------- Timer for manipulators control (100 Hz) ----------------------------------------//

// ARR = 42000, PSC = 20, fapb1 = 42 MHZ, Frequency = 100 Hz
#define SERVO_CHECKER_TIM_MODULE         TIM5
#define SERVO_CHECKER_TIM_PSC            0x14
#define SERVO_CHECKER_TIM_ARR            0xA410
#define SERVO_CHECKER_IRQN               TIM5_IRQn
#define SERVO_CHECKER_PERIOD             0.01f
#define SERVO_CHECKER_TICKS_TO_SEC       0.00000238f

//--------------------------------------------- GPIO for servo reboot ----------------------------------------------------------//
#define SERVO_REBOOT_PIN                 GPIO_Pin_6
#define SERVO_REBOOT_PORT                GPIOD

//--------------------------------------------- Timer for collision avoidance (50 Hz) ------------------------------------------//

// ARR = 42000, PSC = 40, fapb1 = 42 MHZ, Frequency = 50 Hz
#define COLL_AVOID_TIM_MODULE            TIM14
#define COLL_AVOID_TIM_PSC               0x28
#define COLL_AVOID_TIM_ARR               0xA410
#define COLL_AVOID_IRQN                  TIM8_TRG_COM_TIM14_IRQn
#define COLL_AVOID_PERIOD                0.02f
#define COLL_AVOID_PERIOD_MILLISEC       (uint32_t)(COLL_AVOID_PERIOD  * 1000)

//--------------------------------------------- GPIO for collision avoidance LED -----------------------------------------------//
#define COLL_AVOID_LED_PIN               GPIO_Pin_2
#define COLL_AVOID_LED_PORT              GPIOD

//--------------------------------------------- Timer for local time in one tenth of a ms (10000 Hz) ---------------------------//
// ARR = 4200, PSC = 2, fapb1 = 42 MHZ, Frequency = 10000 Hz
#define LOCAL_TIME_TIM_MODULE            TIM7
#define LOCAL_TIME_TIM_PSC               0x02
#define LOCAL_TIME_TIM_ARR               0x1068
#define LOCAL_TIME_IRQN                  TIM7_IRQn
#define LOCAL_TIME_PERIOD                0.0001f

//--------------------------------------------- External interrupts ------------------------------------------------------------//

#define EXTI_STARTUP_IRQN                EXTI1_IRQn
#define EXTI_STARTUP_PORT                GPIOD
#define EXTI_STARTUP_SOURCE_PORT         EXTI_GPIOD_SOURCE_PORT
#define EXTI_STARTUP_PIN                 GPIO_Pin_1

//--------------------------------------------- Enable pin for communication with servos ---------------------------------------//

// Pin turns microchip for communication with servos (0 - transmitting data, 1 - receiving data)
#define DYNAMIXEL_SIGNAL_EN_PIN          GPIO_Pin_12
#define DYNAMIXEL_SIGNAL_EN_PORT         GPIOB

//--------------------------------------------- Pin for expander reset ---------------------------------------------------------//

#define EXPANDER_RESET_PIN               GPIO_Pin_0
#define EXPANDER_RESET_PORT              GPIOD

//--------------------------------------------- Pin for expander power reset ---------------------------------------------------//

#define I2C_POWER_RESET_PIN              GPIO_Pin_3
#define I2C_POWER_RESET_PORT             GPIOD


// Initialize all necessary peripheral devices
void boardInitAll(void);

// Indicate error
void showError(void);

// Indicate error
void showNoError(void);

#endif
