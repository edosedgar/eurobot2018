#include "stm32f4xx_ll_usart.h"
#include "stm32f407xx.h"
#include "terminal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "int_dispatcher.h"
#include "command_list.h"

void terminal_suppl_int(void *args)
{
        (void) args;
        //static USART_TypeDef *usart_p;

        return;
}

void terminal_manager(void *arg)
{
        terminal_task_t term_t;
        int command_code = 0;

        memcpy(&term_t, arg, sizeof(terminal_task_t));
        term_t.buffer = malloc(CHANNEL_BUF_SIZE);
        term_t.com_args = malloc(ARGS_BUF_SIZE);
        term_t.com_resp = term_t.com_args;
        int_new_register(term_t.int_line, terminal_suppl_int);
        term_t.uart2dma_init(term_t.buffer);

        while (1) {
                command_code = term_request(&term_t);
                if (!IS_COMMAND_VALID(command_code))
                        continue;
                commands_handlers[command_code](term_t.com_args);
                term_response(&term_t);
        }
        return;
}

int term_request(terminal_task_t *term_t)
{
        (void) term_t;
        return 0;
}

void term_response(terminal_task_t *term_t)
{
        (void) term_t;
        return;
}
