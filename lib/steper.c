#include "odometry.h"
#include "peripheral.h"
#include "gpio_map.h"

#include "string.h"
#include "math.h"
#include "arm_math.h"



static void steper_hw_config()

void odometry(void *arg)
{
        (void) arg;

        /*
         * Initialization of main motor control structure
         */
        odometry_ctrl_t odom_ctrl_st = {
                .curr_time = 0.0f,
                .prev_time = 0.0f,
                .coordinate = {0.0f, 0.0f, 0.0f},
                .inst_global_speed = {0.0f, 0.0f, 0.0f},
                .inst_local_speed = {0.0f, 0.0f, 0.0f},
                .wheel_speed = {0.0f, 0.0f, 0.0f},
                .p_enc_ticks = {ENCODER_1_CNT, ENCODER_2_CNT, ENCODER_3_CNT},
                .delta_enc_ticks = {0.0f, 0.0f, 0.0f}
        };

        step_ctrl_st.step_notify = xTaskGetCurrentTaskHandle();
        step_ctrl = &step_ctrl_st;
        odom_hw_config(&step_ctrl_st);

        while (1) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                /*
                 * Calculate wheel speeds
                 */
                odom_calc_wheels_speeds(&odom_ctrl_st);
                /*
                 * Calculate robot instant local speed
                 */
                odom_calc_robot_speed(&odom_ctrl_st);
                /*
                 * Calculate robot instant global speed and coordinate
                 */
                odom_calc_glob_params(&odom_ctrl_st);
        }
        return;
}
