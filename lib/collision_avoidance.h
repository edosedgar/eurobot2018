#ifndef _COLLISION_AVOIDANCE_H
#define _COLLISION_AVOIDANCE_H

#include "FreeRTOS.h"

/*
 * Collision avoidance switch
 */
typedef enum {
        COL_AVOID_OFF,
        COL_AVOID_ON
} col_avoid_switch;

/*
 * Memory for collision avoidance task
 */
#define COL_AVOID_STACK_DEPTH    1024
StackType_t collision_avoidance_ts[COL_AVOID_STACK_DEPTH];
StaticTask_t collision_avoidance_tb;

/*
 * Main freertos task
 */
void collision_avoidance(void *arg);

#endif
