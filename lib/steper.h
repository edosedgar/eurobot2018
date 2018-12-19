#ifndef _STEPPER_H_
#define _STEPPER_H_

#include "FreeRTOS.h"
#include "task.h"

/*
 * Stepper control structure
 */
typedef struct {
        int8_t step_position;
        uint16_t position;
        uint16_t goal_position;
        float speed;
        TaskHandle_t step_notify;
} step_ctrl_t;

#define STEP_STACK_DEPTH    1024
StackType_t step_ts[STEP_STACK_DEPTH];
StaticTask_t step_tb;

void step(void *arg);

#endif
