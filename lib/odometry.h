#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "FreeRTOS.h"
#include "task.h"

/*
 * Encoder ticks to radians coefficient calculation
 */
#define PI_NUMBER                  3.14159265358f
#define MAXON_MOTOR_ENC_TICKS      4096
#define MAXON_MOTOR_GR             28.0f
#define TICKS_TO_RAD_COEF          2 * PI_NUMBER / \
                                   (MAXON_MOTOR_ENC_TICKS * MAXON_MOTOR_GR)

/*
 * Set coordinates command structure
 */
typedef struct {
        float x;
        float y;
        float alpha;
} __attribute__((packed)) cmd_set_coord_t;

/*
 * Odometry control structure
 */
typedef struct {
        float curr_time;
        float prev_time;
        float coordinate[3];
        float inst_global_speed[3];
        float inst_local_speed[3];
        float wheel_speed[3];
        uint16_t *p_enc_ticks[3];
        int16_t delta_enc_ticks[3];
        TaskHandle_t odom_notify;
} odometry_ctrl_t;

#define ODOMETRY_STACK_DEPTH    1024
StackType_t odometry_ts[ODOMETRY_STACK_DEPTH];
StaticTask_t odometry_tb;

/*
 * Inverse kinematics matrix
 */
#define ODOM_INV_KINEMATICS \
        0.016289945f,       0.0f,         -0.016289945f, \
       -0.0094054665f,     0.018810933f,  -0.0094054665f, \
       -0.09326100256f,    -0.09326100256f, -0.09326100256f

/*
 * Main freertos task
 */
void odometry(void *arg);

#endif
