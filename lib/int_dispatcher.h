#ifndef _H_INT_DISPATCHER_
#define _H_INT_DISPATCHER_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define MAX_INT_NUM 81
#define MAX_EVENTS_NUM 10
#define INT_MAN_STACK_DEPTH 512
#define IS_INT_VALID(INT_NUM) ((INT_NUM) >= 0 && (INT_NUM) < MAX_INT_NUM)

StackType_t int_dispatcher_ts[INT_MAN_STACK_DEPTH];
StaticTask_t int_dispatcher_tb;

static void (*int_handlers[MAX_INT_NUM + 1])(void *);

/*
 * The type to be used for event queue
 */
typedef struct {
        int int_num;
        void *args;
} event_arg_t;

uint8_t static_queue_storage[MAX_EVENTS_NUM * sizeof(void *)];
static StaticQueue_t static_queue_struct;
static QueueHandle_t event_queue;

/*
 * The main interrupt manager which is responsible for calling
 * a corresponding registered handler for awoken interrupt
 */
void interrupt_manager(void *arg);

/*
 * To register new handler for a particular interrupt call this
 * function with a pointer to a function and interrupt number
 */
int int_new_register(int int_code, void (*int_handler)(void *));

#endif
