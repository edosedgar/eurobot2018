#include "steper.h"
#include "peripheral.h"
#include "gpio_map.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


static step_ctrl_t *step_ctrl = NULL;


static void step_hw_config(step_ctrl_t *step_ctrl)
{
        /* Init motor_kinematics pins */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

        /* Config coils pins */
        LL_GPIO_SetPinMode(STEP_COIL1_PORT, STEP_COIL1_PIN,
                           LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(STEP_COIL1_PORT, STEP_COIL1_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(STEP_COIL1_PORT, STEP_COIL1_PIN,
                           LL_GPIO_PULL_NO);

        LL_GPIO_SetPinMode(STEP_COIL2_PORT, STEP_COIL2_PIN,
                           LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(STEP_COIL2_PORT, STEP_COIL2_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(STEP_COIL2_PORT, STEP_COIL2_PIN,
                           LL_GPIO_PULL_NO);

        LL_GPIO_SetPinMode(STEP_COIL3_PORT, STEP_COIL3_PIN,
                           LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(STEP_COIL3_PORT, STEP_COIL3_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(STEP_COIL3_PORT, STEP_COIL3_PIN,
                           LL_GPIO_PULL_NO);

        LL_GPIO_SetPinMode(STEP_COIL4_PORT, STEP_COIL4_PIN,
                           LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(STEP_COIL4_PORT, STEP_COIL4_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(STEP_COIL4_PORT, STEP_COIL4_PIN,
                           LL_GPIO_PULL_NO);
        

        /*
         * Step timer initialization
         */
        LL_TIM_SetAutoReload(STEP_TIM_MODULE, STEP_TIM_ARR);
        LL_TIM_SetPrescaler(STEP_TIM_MODULE, STEP_TIM_PSC);
        LL_TIM_SetCounterMode(STEP_TIM_MODULE, LL_TIM_COUNTERMODE_UP);
        LL_TIM_EnableIT_UPDATE(STEP_TIM_MODULE);
        NVIC_SetPriority(STEP_IRQN, STEP_IRQN_PRIORITY);
        NVIC_EnableIRQ(STEP_IRQN);

        /*
         * Enable timers
         */
        LL_TIM_EnableCounter(STEP_TIM_MODULE);
        
        return;
}

static void step_next_step (step_ctrl_t *step_ctrl)
{
	if ( step_ctrl->position < step_ctrl->goal_position )
		step_ctrl->step_position ++;
	else if ( step_ctrl->position > step_ctrl->goal_position )
		step_ctrl->step_position --;

	if (step_ctrl->step_position == 4) step_ctrl->step_position = 0;
	if (step_ctrl->step_position == -1) step_ctrl->step_position = 3;

}


static void step_movement (step_ctrl_t *step_ctrl)
{
	switch ( step_ctrl->position){

		case 0:	
			LL_GPIO_SetOutputPin(STEP_COIL1_PORT,STEP_COIL1_PIN);
			LL_GPIO_SetOutputPin(STEP_COIL2_PORT,STEP_COIL2_PIN);
			LL_GPIO_ResetOutputPin(STEP_COIL3_PORT,STEP_COIL3_PIN);
			LL_GPIO_ResetOutputPin(STEP_COIL4_PORT,STEP_COIL4_PIN);
		break;
		case 1:
		    LL_GPIO_ReetOutputPin(STEP_COIL1_PORT,STEP_COIL1_PIN);
            LL_GPIO_SetOutputPin(STEP_COIL2_PORT,STEP_COIL2_PIN);
            LL_GPIO_SetOutputPin(STEP_COIL3_PORT,STEP_COIL3_PIN);
            LL_GPIO_ResetOutputPin(STEP_COIL4_PORT,STEP_COIL4_PIN);
        break;
		case 2:
		    LL_GPIO_ResetOutputPin(STEP_COIL1_PORT,STEP_COIL1_PIN);
            LL_GPIO_ResetOutputPin(STEP_COIL2_PORT,STEP_COIL2_PIN);
            LL_GPIO_SetOutputPin(STEP_COIL3_PORT,STEP_COIL3_PIN);
            LL_GPIO_SetOutputPin(STEP_COIL4_PORT,STEP_COIL4_PIN);
        break;
		case 3:
		    LL_GPIO_SetOutputPin(STEP_COIL1_PORT,STEP_COIL1_PIN);
            LL_GPIO_ResetOutputPin(STEP_COIL2_PORT,STEP_COIL2_PIN);
            LL_GPIO_ResetOutputPin(STEP_COIL3_PORT,STEP_COIL3_PIN);
            LL_GPIO_SetOutputPin(STEP_COIL4_PORT,STEP_COIL4_PIN);
        break;

	}

}


void step(void *arg)
{
        (void) arg;

        /*
         * Initialization of main motor control structure
         */
        step_ctrl_t step_ctrl_st = {
        		.step_position = 0,
                .position = 0,
                .goal_position = 0
        };

        step_ctrl_st.step_notify = xTaskGetCurrentTaskHandle();
        step_ctrl = &step_ctrl_st;
        step_hw_config(&step_ctrl_st);

        while (1) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                /*
                 * Calculate next coil stcep
                 */
                step_next_step (&step_ctrl_st);
                /*
                 * Send the coil position  
                 */
                step_movement (&step_ctrl_st);

        }
        return;
}

/*
 * Hardware interrupt handler
 */
void TIM7_DAC_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (LL_TIM_IsActiveFlag_UPDATE(ODOMETRY_TIM_MODULE)) {
                LL_TIM_ClearFlag_UPDATE(ODOMETRY_TIM_MODULE);
                /*
                 * Notify task
                 */
                vTaskNotifyGiveFromISR(step_ctrl->step_notify,
                                       &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


