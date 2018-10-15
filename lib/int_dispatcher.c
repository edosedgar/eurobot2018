#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"

#include <string.h>
#include "int_dispatcher.h"
#include "FreeRTOS.h"
#include "task.h"

void interrupt_manager(void *arg)
{
        (void) arg;
        event_arg_t *event_arg;

        event_queue = xQueueCreateStatic(MAX_EVENTS_NUM, sizeof(void *),
                                         static_queue_storage,
                                         &static_queue_struct);

        while (1) {
                xQueueReceive(event_queue, &event_arg, 0);
                if (!IS_INT_VALID(event_arg->int_num))
                        continue;
                if (!int_handlers[event_arg->int_num])
                        continue;
                int_handlers[event_arg->int_num](event_arg->args);
        }
        return;
}

int int_new_register(int int_code, void (*int_handler)(void *))
{
        if (int_code < 0 || int_code >= MAX_INT_NUM)
                return -1;

        int_handlers[int_code] = int_handler;
        return 0;
}

void USART1_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        static USART_TypeDef usart1_copy;
        static event_arg_t event_args;

        memcpy(&usart1_copy, USART1, sizeof(USART_TypeDef));
        event_args.int_num = USART1_IRQn;
        event_args.args = &usart1_copy;
        xQueueSendFromISR(event_queue, &event_args, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART2_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        static USART_TypeDef usart2_copy;
        static event_arg_t event_args;

        memcpy(&usart2_copy, USART2, sizeof(USART_TypeDef));
        event_args.int_num = USART2_IRQn;
        event_args.args = &usart2_copy;
        xQueueSendFromISR(event_queue, &event_args, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART3_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        static USART_TypeDef usart3_copy;
        static event_arg_t event_args;

        memcpy(&usart3_copy, USART3, sizeof(USART_TypeDef));
        event_args.int_num = USART3_IRQn;
        event_args.args = &usart3_copy;
        xQueueSendFromISR(event_queue, &event_args, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USART4_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        static USART_TypeDef usart4_copy;
        static event_arg_t event_args;

        memcpy(&usart4_copy, UART4, sizeof(USART_TypeDef));
        event_args.int_num = UART4_IRQn;
        event_args.args = &usart4_copy;
        xQueueSendFromISR(event_queue, &event_args, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
