#include "odometry.h"
#include "peripheral.h"
#include "gpio_map.h"

#include "string.h"
#include "math.h"
#include "arm_math.h"

/*
 * Main motor kinematics control structure
 */
static odometry_ctrl_t *odom_ctrl = NULL;

/*
 * Set of private helper functions
 */

/*
 * Inline function for angle normalization
 */
static inline float normalize_angle(float angle)
{
        if (angle > 2 * PI_NUMBER)
                return angle - 2 * PI_NUMBER;
        else if (angle < -2 * PI_NUMBER)
                return angle + 2 * PI_NUMBER;
        else
                return angle;
}
/*
 * Hardware initialization
 */
static void odom_hw_config(odometry_ctrl_t *odom_ctrl)
{
        /*
         * Enable gpio and timers clocking
         */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

        /*
         * Initialization encoders gpio
         * Initialization first encoder gpios
         */
        LL_GPIO_SetPinMode(ENCODER_1_CHA_PORT, ENCODER_1_CHA_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_0_7(ENCODER_1_CHA_PORT, ENCODER_1_CHA_PIN,
                             ENCODER_1_PIN_AF);
        LL_GPIO_SetPinOutputType(ENCODER_1_CHA_PORT, ENCODER_1_CHA_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinMode(ENCODER_1_CHB_PORT, ENCODER_1_CHB_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(ENCODER_1_CHB_PORT, ENCODER_1_CHB_PIN,
                              ENCODER_1_PIN_AF);
        LL_GPIO_SetPinOutputType(ENCODER_1_CHB_PORT, ENCODER_1_CHB_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);

        /*
         * Initialization second encoder gpios
         */
        LL_GPIO_SetPinMode(ENCODER_2_CHA_PORT, ENCODER_2_CHA_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_0_7(ENCODER_2_CHA_PORT, ENCODER_2_CHA_PIN,
                             ENCODER_2_PIN_AF);
        LL_GPIO_SetPinOutputType(ENCODER_2_CHA_PORT, ENCODER_2_CHA_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinMode(ENCODER_2_CHB_PORT, ENCODER_2_CHB_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_0_7(ENCODER_2_CHB_PORT, ENCODER_2_CHB_PIN,
                             ENCODER_2_PIN_AF);
        LL_GPIO_SetPinOutputType(ENCODER_2_CHB_PORT, ENCODER_2_CHB_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);

        /*
         * Initialization third encoder gpios
         */
        LL_GPIO_SetPinMode(ENCODER_3_CHA_PORT, ENCODER_3_CHA_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(ENCODER_3_CHA_PORT, ENCODER_3_CHA_PIN,
                              ENCODER_3_PIN_AF);
        LL_GPIO_SetPinOutputType(ENCODER_3_CHA_PORT, ENCODER_3_CHA_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinMode(ENCODER_3_CHB_PORT, ENCODER_3_CHB_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(ENCODER_3_CHB_PORT, ENCODER_3_CHB_PIN,
                              ENCODER_3_PIN_AF);
        LL_GPIO_SetPinOutputType(ENCODER_3_CHB_PORT, ENCODER_3_CHB_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);

        /*
         * Initialization encoder timers
         * First encoder mode timer
         */
        LL_TIM_CC_EnableChannel(ENCODER_1_TIM_MODULE,
                                LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
        LL_TIM_IC_Config(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH1,
                         LL_TIM_ACTIVEINPUT_DIRECTTI);
        LL_TIM_IC_Config(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH2,
                         LL_TIM_ACTIVEINPUT_DIRECTTI |
                         LL_TIM_IC_POLARITY_RISING);
        LL_TIM_SetAutoReload(ENCODER_1_TIM_MODULE, ENCODER_TIM_ARR);
        LL_TIM_SetEncoderMode(ENCODER_1_TIM_MODULE, LL_TIM_ENCODERMODE_X4_TI12);
        *(odom_ctrl->p_enc_ticks[0]) = ENCODER_TIM_CNT_INITIAL_VALUE;

        /*
         * Second encoder mode timer
         */
        LL_TIM_CC_EnableChannel(ENCODER_2_TIM_MODULE,
                                LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
        LL_TIM_IC_Config(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH1,
                         LL_TIM_ACTIVEINPUT_DIRECTTI);
        LL_TIM_IC_Config(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH2,
                         LL_TIM_ACTIVEINPUT_DIRECTTI |
                         LL_TIM_IC_POLARITY_RISING);
        LL_TIM_SetAutoReload(ENCODER_2_TIM_MODULE, ENCODER_TIM_ARR);
        LL_TIM_SetEncoderMode(ENCODER_2_TIM_MODULE, LL_TIM_ENCODERMODE_X4_TI12);
        *(odom_ctrl->p_enc_ticks[1]) = ENCODER_TIM_CNT_INITIAL_VALUE;

        /*
         * Third encoder mode timer
         */
        LL_TIM_CC_EnableChannel(ENCODER_3_TIM_MODULE,
                                LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
        LL_TIM_IC_Config(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH1,
                         LL_TIM_ACTIVEINPUT_DIRECTTI);
        LL_TIM_IC_Config(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH2,
                         LL_TIM_ACTIVEINPUT_DIRECTTI |
                         LL_TIM_IC_POLARITY_RISING);
        LL_TIM_SetAutoReload(ENCODER_3_TIM_MODULE, ENCODER_TIM_ARR);
        LL_TIM_SetEncoderMode(ENCODER_3_TIM_MODULE, LL_TIM_ENCODERMODE_X4_TI12);
        *(odom_ctrl->p_enc_ticks[2]) = ENCODER_TIM_CNT_INITIAL_VALUE;

        /*
         * Odometry timer initialization
         */
        LL_TIM_SetAutoReload(ODOMETRY_TIM_MODULE, ODOMETRY_TIM_ARR);
        LL_TIM_SetPrescaler(ODOMETRY_TIM_MODULE, ODOMETRY_TIM_PSC);
        LL_TIM_SetCounterMode(ODOMETRY_TIM_MODULE, LL_TIM_COUNTERMODE_UP);
        LL_TIM_EnableIT_UPDATE(ODOMETRY_TIM_MODULE);
        NVIC_SetPriority(ODOMETRY_IRQN, ODOMETRY_IRQN_PRIORITY);
        NVIC_EnableIRQ(ODOMETRY_IRQN);

        /*
         * Enable timers
         */
        LL_TIM_EnableCounter(ENCODER_1_TIM_MODULE);
        LL_TIM_EnableCounter(ENCODER_2_TIM_MODULE);
        LL_TIM_EnableCounter(ENCODER_3_TIM_MODULE);
        LL_TIM_EnableCounter(ODOMETRY_TIM_MODULE);
        return;
}

/*
 * Read data from encoders and calculate wheels speed
 */
static void odom_calc_wheels_speeds(odometry_ctrl_t *odom_ctrl)
{
        int i = 0;

        for (i = 0; i < 3; i++) {
                odom_ctrl->delta_enc_ticks[i] = (int16_t)
                                                *(odom_ctrl->p_enc_ticks[i]) -
                                                ENCODER_TIM_CNT_INITIAL_VALUE;
                *(odom_ctrl->p_enc_ticks[i]) = ENCODER_TIM_CNT_INITIAL_VALUE;
                odom_ctrl->wheel_speed[i] =
                ((float) odom_ctrl->delta_enc_ticks[i] * TICKS_TO_RAD_COEF);
        }
        return;
}

/*
 * Calculate robot speed from wheel speeds using inversed kinematics matrix
 */
static void odom_calc_robot_speed(odometry_ctrl_t *odom_ctrl)
{
        /*
         * Inverse kinematic matrix for calculation robot speed using wheels
         * speeds
         */
        static arm_matrix_instance_f32 m_inv_kin;
        static float inv_kin[9] = {ODOM_INV_KINEMATICS};
        /*
         * Input wheels speeds in robot's coordinate system
         */
        static arm_matrix_instance_f32 m_wheel_sp;
        static float wheel_sp[3] = {0.0f};
        wheel_sp[0] = odom_ctrl->wheel_speed[0];
        wheel_sp[1] = odom_ctrl->wheel_speed[1];
        wheel_sp[2] = odom_ctrl->wheel_speed[2];
        /*
         * Calculated robot speed
         */
        static arm_matrix_instance_f32 m_inst_speed;

        /*
         * Init arm_math matrice data structures
         */
        arm_mat_init_f32(&m_inv_kin, 3, 3, inv_kin);
        arm_mat_init_f32(&m_wheel_sp, 3, 1, wheel_sp);
        arm_mat_init_f32(&m_inst_speed, 3, 1, odom_ctrl->inst_local_speed);
        /*
         * Calculate instant robot speed
         */
        arm_mat_mult_f32(&m_inv_kin, &m_wheel_sp, &m_inst_speed);
        return;
}

/*
 * Calculate robot global coordinates
 */
static void odom_calc_glob_params(odometry_ctrl_t *odom_ctrl)
{
        /*
         * Rotation angle since last calculations
         */
        float rot_angle = odom_ctrl->coordinate[2] + odom_ctrl->inst_local_speed[2];
        rot_angle = normalize_angle(rot_angle);
        /*
         * Rotation matrix for ransformation of speed in robot coordinate system
         * to global one
         */
        static arm_matrix_instance_f32 m_rot_matrix;
        float rot_matrix[4] = {
                cosf(rot_angle), -sinf(rot_angle),
                sinf(rot_angle),  cosf(rot_angle)
        };
        /*
         * Robot instant global speed
         */
        static arm_matrix_instance_f32 m_inst_global_speed;
        /*
         * Robot instant local speed
         */
        static arm_matrix_instance_f32 m_inst_local_speed;

        /*
         * Init arm_math matrice data structures
         */
        arm_mat_init_f32(&m_rot_matrix, 2, 2, rot_matrix);
        arm_mat_init_f32(&m_inst_global_speed, 2, 1,
                         odom_ctrl->inst_global_speed);
        arm_mat_init_f32(&m_inst_local_speed, 2, 1,
                         odom_ctrl->inst_local_speed);
        /*
         * Robot global speed calculation
         */
        arm_mat_mult_f32(&m_rot_matrix, &m_inst_local_speed,
                         &m_inst_global_speed);
        odom_ctrl->inst_global_speed[2] = odom_ctrl->inst_local_speed[2];
        /*
         * Robot global coordinates calculation
         */
        odom_ctrl->coordinate[0] += odom_ctrl->inst_global_speed[0] ;
        odom_ctrl->coordinate[1] += odom_ctrl->inst_global_speed[1] ;
        odom_ctrl->coordinate[2] = rot_angle;
        odom_ctrl->coordinate[2] = normalize_angle(odom_ctrl->coordinate[2]);
        return;
}

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
        odom_ctrl_st.odom_notify = xTaskGetCurrentTaskHandle();
        odom_ctrl = &odom_ctrl_st;
        odom_hw_config(&odom_ctrl_st);

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

/*
 * Set of motor related handlers for terminal
 */

/*
 * Get instant robot local speed command
 * Input: -
 * Output: 3 float robot local speed
 */
int cmd_get_speed(void *args)
{
        /*
         * Check whether odometry is ready or not
         */
        if (!odom_ctrl)
                goto error_get_speed;
        /*
         * Give robot instant local speed values
         */
        memcpy(args, odom_ctrl->inst_local_speed, 12);
        return 12;
error_get_speed:
        memcpy(args, "ERROR", 6);
        return 6;
}

/*
 * Set robot global coordinates
 * Input: float x, y and alpha positions
 * Output: OK or ERROR status
 */
int cmd_set_coord(void *args)
{
        cmd_set_coord_t *cmd_args = (cmd_set_coord_t *)args;

        /*
         * Check whether odometry is ready or not
         */
        if (!odom_ctrl)
                goto error_set_coord;
        /*
         * Update odometry control structure
         */
        odom_ctrl->coordinate[0] = cmd_args->x;
        odom_ctrl->coordinate[1] = cmd_args->y;
        odom_ctrl->coordinate[2] = cmd_args->alpha;
        memcpy(args, "OK", 3);
        return 3;
error_set_coord:
        memcpy(args, "ERROR", 6);
        return 6;
}

/*
 * Get robot global coordinates
 * Input: -
 * Output: 3 float coordinates
 */
int cmd_get_coord(void *args)
{
        /*
         * Check whether odometry is ready or not
         */
        if (!odom_ctrl)
                goto error_get_coord;
        /*
         * Update odometry control structure
         */
        memcpy(args, odom_ctrl->coordinate, 12);
        return 12;
error_get_coord:
        memcpy(args, "ERROR", 6);
        return 6;
}

/*
 * Get robot global coordinates
 * Input: -
 * Output: 3 float coordinates
 */
int cmd_get_wheel_speed(void *args)
{
        /*
         * Check whether odometry is ready or not
         */
        if (!odom_ctrl)
                goto error_get_wheels_speed;
        /*
         * Update odometry control structure
         */
        memcpy(args, odom_ctrl->wheel_speed, 12);
        return 12;
error_get_wheels_speed:
        memcpy(args, "ERROR", 6);
        return 6;
}

/*
 * Hardware interrupt handler
 */
void TIM6_DAC_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (LL_TIM_IsActiveFlag_UPDATE(ODOMETRY_TIM_MODULE)) {
                LL_TIM_ClearFlag_UPDATE(ODOMETRY_TIM_MODULE);
                /*
                 * Increment time in milliseconds
                 */
                odom_ctrl->prev_time = odom_ctrl->curr_time;
                odom_ctrl->curr_time += 0.01f;
                /*
                 * Notify task
                 */
                vTaskNotifyGiveFromISR(odom_ctrl->odom_notify,
                                       &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
