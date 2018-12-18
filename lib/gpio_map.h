#ifndef _H_GPIO_MAP_
#define _H_GPIO_MAP_

#include "stm32f407xx.h"
#include "stm32f4xx_ll_gpio.h"

/*
 * Terminal UART pinout
 */
#define TERM_USART_TX_PORT                  GPIOB
#define TERM_USART_TX_PIN                   LL_GPIO_PIN_6
#define TERM_USART_RX_PORT                  GPIOB
#define TERM_USART_RX_PIN                   LL_GPIO_PIN_7
#define TERM_USART_PIN_AF                   LL_GPIO_AF_7
//TODO changed on LL_GPIO_OUTPUT_OPENDRAIN with Odroid communication
#define TERM_USART_OUTPUT_TYPE              LL_GPIO_OUTPUT_PUSHPULL

/*
 * Motor kinematics pinout
 * PWM pins
 */
#define MOTOR_CH_PWM_PORT                   GPIOD
#define MOTOR_CH1_PWM_PIN                   LL_GPIO_PIN_12
#define MOTOR_CH2_PWM_PIN                   LL_GPIO_PIN_13
#define MOTOR_CH3_PWM_PIN                   LL_GPIO_PIN_14
#define MOTOR_CH4_PWM_PIN                   LL_GPIO_PIN_15
#define MOTOR_CH_PWM_PIN_AF                 LL_GPIO_AF_2

/* Direction pins */
#define MOTOR_CH1_DIR_PORT                  GPIOC
#define MOTOR_CH1_DIR_PIN                   LL_GPIO_PIN_14
#define MOTOR_CH2_DIR_PORT                  GPIOC
#define MOTOR_CH2_DIR_PIN                   LL_GPIO_PIN_15
#define MOTOR_CH3_DIR_PORT                  GPIOE
#define MOTOR_CH3_DIR_PIN                   LL_GPIO_PIN_10
#define MOTOR_CH4_DIR_PORT                  GPIOE
#define MOTOR_CH4_DIR_PIN                   LL_GPIO_PIN_14

/*
 * Odometry pinout
 * Encoder pins
 */
#define ENCODER_1_CHA_PORT                  GPIOB
#define ENCODER_1_CHA_PIN                   LL_GPIO_PIN_3
#define ENCODER_1_CHB_PORT                  GPIOA
#define ENCODER_1_CHB_PIN                   LL_GPIO_PIN_15
#define ENCODER_1_PIN_AF                    LL_GPIO_AF_1

#define ENCODER_2_CHA_PORT                  GPIOA
#define ENCODER_2_CHA_PIN                   LL_GPIO_PIN_6
#define ENCODER_2_CHB_PORT                  GPIOA
#define ENCODER_2_CHB_PIN                   LL_GPIO_PIN_7
#define ENCODER_2_PIN_AF                    LL_GPIO_AF_2

#define ENCODER_3_CHA_PORT                  GPIOE
#define ENCODER_3_CHA_PIN                   LL_GPIO_PIN_11
#define ENCODER_3_CHB_PORT                  GPIOE
#define ENCODER_3_CHB_PIN                   LL_GPIO_PIN_9
#define ENCODER_3_PIN_AF                    LL_GPIO_AF_1


/*
 * Stepper pinout 
*/

#define STEP_COIL1_PORT                     GPIOD
#define STEP_COIL1_PIN      	            LL_GPIO_PIN_11
#define STEP_COIL2_PORT 	                GPIOD
#define STEP_COIL2_PIN	                    LL_GPIO_PIN_10
#define STEP_COIL3_PORT                     GPIOD
#define STEP_COIL3_PIN      	            LL_GPIO_PIN_9
#define STEP_COIL4_PORT 	                GPIOD
#define STEP_COIL4_PIN	                    LL_GPIO_PIN_8

#endif


