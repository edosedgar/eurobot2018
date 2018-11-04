#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_bus.h"
#include "collision_avoidance.h"
#include "peripheral.h"
#include "task.h"

/*
 * Private task notifier
 */
static TaskHandle_t xTaskToNotify;

/*
 * Private variable for switching on/off collision avoidance
 */
static col_avoid_switch col_av_sw;

/*
 * Set of private helper functions
 */
 static void col_av_hw_config()
 {
        /*
         * Timer initialization in one pulse mode
         */
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
        LL_TIM_EnableUpdateEvent(COL_AVOID_TIM);
        LL_TIM_SetCounterMode(COL_AVOID_TIM, LL_TIM_COUNTERMODE_UP);
        LL_TIM_SetAutoReload(COL_AVOID_TIM, COL_AVOID_TIM_ARR);
        LL_TIM_SetPrescaler(COL_AVOID_TIM, COL_AVOID_TIM_PSC);
        LL_TIM_SetOnePulseMode(COL_AVOID_TIM, LL_TIM_ONEPULSEMODE_SINGLE);
        LL_TIM_EnableIT_UPDATE(COL_AVOID_TIM);

        LL_TIM_GenerateEvent_UPDATE(COL_AVOID_TIM);
        LL_TIM_EnableCounter(COL_AVOID_TIM);

        /*
         *Enable global TIM14 interrupts
         */
        NVIC_SetPriority(COL_AVOID_TIM_IRQN, COL_AVOID_TIM_IRQN_PRIORITY);
        NVIC_EnableIRQ(COL_AVOID_TIM_IRQN);
 }

/*
 * Main motor kinematics task running by FreeRTOS
 */
void collision_avoidance(void *arg)
{
        (void) arg;

        xTaskToNotify = xTaskGetCurrentTaskHandle();
        col_av_sw = COL_AVOID_ON;
        col_av_hw_config();

        while (1) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                /*
                 * Get and process data from range finders
                 */

                /*
                 * Enable timer in one pulse mode after processing, if collision
                 * avoidance is turned on
                 */
                if (col_av_sw == COL_AVOID_ON)
                        LL_TIM_EnableCounter(COL_AVOID_TIM);
        }
        return;
}

/*
 * Timer global interrupt
 */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (LL_TIM_IsActiveFlag_UPDATE(COL_AVOID_TIM)) {
                LL_TIM_ClearFlag_UPDATE(COL_AVOID_TIM);
                vTaskNotifyGiveFromISR(xTaskToNotify,
                                       &xHigherPriorityTaskWoken);
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * Set of collision avoidance related handlers for terminal
 */

/*
 * Collision avoidance switch command
 * Input: 0 - switch off / other - switch on
 * Output: OK
 */
int cmd_collision_avoidance_switch(void *args)
{
        uint8_t com_arg = *(uint8_t *)args;

        if (com_arg) {
                col_av_sw = COL_AVOID_ON;
                xTaskNotifyGive(xTaskToNotify);
        }
        else
                col_av_sw = COL_AVOID_OFF;
        memcpy(args, "OK", 3);
        return 3;
}
