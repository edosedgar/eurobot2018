#ifndef _H_TERMINAL_
#define _H_TERMINAL_

#include "stm32f4xx_ll_usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "command_list.h"

/*
 * Import commands_handlers table from command_handler.h
 */
extern void (* const commands_handlers[UPPER_BOUND_COMMANDS])(char *);

/*
 * The main structure for terminal operating
 */
typedef struct {
        USART_TypeDef *dev;
        int int_line;
        void (*uart_dma_init)(char *);
        char *buffer;
        char *com_args;
        char *com_resp;
} terminal_task_t;

#define CHANNEL_BUF_SIZE        256
#define ARGS_BUF_SIZE           128
#define TERM_MAN_STACK_DEPTH    1024
#define IS_COMMAND_VALID(com) (((com) > LOWER_BOUND_COMMANDS) && \
                               ((com) < UPPER_BOUND_COMMANDS))

/*
 * Memory for terminal task
 */
StackType_t terminal_manager_ts[TERM_MAN_STACK_DEPTH];
StaticTask_t terminal_manager_tb;

/*
 * Main manager for processing incoming commands
 */
void terminal_manager(void *arg);
/*
 * Handle received request
 */
int term_request(terminal_task_t *term_t);
/*
 * Send response to sender
 */
void term_response(terminal_task_t *term_t);

#endif
