#include "motor_kinematics.h"
#include "peripheral.h"
#include "gpio_map.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "string.h"
#include "math.h"
#include "arm_math.h"

/*
 * Main motor kinematics control structure
 */
static motors_ctrl_t *mk_ctrl = NULL;

/*
 * Set of private helper functions
 */
static void mk_speed2pwm(motors_ctrl_t *mk_ctrl)
{
        /*
         * Matrices for calculation of wheel speed
         * 1st - linear components
         * 2nd - rotational components
         */
        static arm_matrix_instance_f32 m_fk_lin;
        static float fk_linear[12] = {MK_LIN_KIN_MATRIX};
        static arm_matrix_instance_f32 m_fk_rot;
        static float fk_rotational[12] = {MK_ROT_KIN_MATRIX};
        /*
         * For calculated rotational speeds (in rad/s)
         * Note: both terms
         */
        static arm_matrix_instance_f32 m_lin_sp;
        static float lin_sp[4] = {0.0f};
        static arm_matrix_instance_f32 m_rot_sp;
        static float rot_sp[4] = {0.0f};
        /*
         * Final speed (casted and normalized)
         */
        static arm_matrix_instance_f32 m_speed;
        static float speed[4] = {0.0f};
        /*
         * Input linear speed in robot's coordinate system
         */
        static arm_matrix_instance_f32 m_input_sp;
        static float input_speed[3] = {0.0f};
        input_speed[0] = mk_ctrl->vel_x;
        input_speed[1] = mk_ctrl->vel_y;
        input_speed[2] = mk_ctrl->wz;
        /*
         * max and min value in resulting matrix
         */
        static float max_sp, min_sp;
        /*
         * Calibration parameters to convert angular speed to pwm
         * values
         */
        static float pwm_cal_a[4] = {MK_SPEED2PWM_A};
        static float pwm_cal_b[4] = {MK_SPEED2PWM_B};

        /*
         * Init arm_math matrice data structures
         */
        arm_mat_init_f32(&m_fk_lin, 4, 3, fk_linear);
        arm_mat_init_f32(&m_fk_rot, 4, 3, fk_rotational);
        arm_mat_init_f32(&m_lin_sp, 4, 1, lin_sp);
        arm_mat_init_f32(&m_rot_sp, 4, 1, rot_sp);
        arm_mat_init_f32(&m_speed,  4, 1, speed);
        arm_mat_init_f32(&m_input_sp, 3, 1, input_speed);
        /*
         * Calculate linear components
         */
        arm_mat_mult_f32(&m_fk_lin, &m_input_sp, &m_lin_sp);
        /*
         * Calculate rotational part
         */
        arm_mat_mult_f32(&m_fk_rot, &m_input_sp, &m_rot_sp);
        /*
         * Sum both components
         */
        arm_mat_add_f32(&m_lin_sp, &m_rot_sp, &m_speed);
        /*
         * Find max and min values and fit speeds to the absolute max one
         */
        arm_max_f32(speed, 4, &max_sp, NULL);
        arm_min_f32(speed, 4, &min_sp, NULL);
        if (max_sp < fabsf(min_sp))
                max_sp = -min_sp;
        if (max_sp > MK_MAX_ROT_SPEED)
                arm_mat_scale_f32(&m_speed, MK_MAX_ROT_SPEED/max_sp, &m_speed);
        /*
         * Convert rot/s to pwm normalized values
         */
        arm_mult_f32(speed, pwm_cal_a, speed, 4);
        arm_add_f32(speed, pwm_cal_b, mk_ctrl->pwm_motors, 4);
        return;
}

static void mk_set_pwm(float *pwm_values)
{
        if (pwm_values[0] > 0.0f)
                LL_GPIO_SetOutputPin(MOTOR_CH1_DIR_PORT,MOTOR_CH1_DIR_PIN);
        else
                LL_GPIO_ResetOutputPin(MOTOR_CH1_DIR_PORT, MOTOR_CH1_DIR_PIN);
        LL_TIM_OC_SetCompareCH1(MOTOR_TIM, (uint32_t)(fabsf(pwm_values[0]) *
                                MOTOR_PWM_TIM_ARR));

        if (pwm_values[1] > 0.0f)
                LL_GPIO_SetOutputPin(MOTOR_CH2_DIR_PORT,MOTOR_CH2_DIR_PIN);
        else
                LL_GPIO_ResetOutputPin(MOTOR_CH2_DIR_PORT, MOTOR_CH2_DIR_PIN);
        LL_TIM_OC_SetCompareCH2(MOTOR_TIM, (uint32_t)(fabsf(pwm_values[1]) *
                                MOTOR_PWM_TIM_ARR));

        if (pwm_values[2] > 0.0f)
                LL_GPIO_SetOutputPin(MOTOR_CH3_DIR_PORT,MOTOR_CH3_DIR_PIN);
        else
                LL_GPIO_ResetOutputPin(MOTOR_CH3_DIR_PORT, MOTOR_CH3_DIR_PIN);
        LL_TIM_OC_SetCompareCH3(MOTOR_TIM, (uint32_t)(fabsf(pwm_values[2]) *
                                MOTOR_PWM_TIM_ARR));

        if (pwm_values[3] > 0.0f)
                LL_GPIO_SetOutputPin(MOTOR_CH4_DIR_PORT,MOTOR_CH4_DIR_PIN);
        else
                LL_GPIO_ResetOutputPin(MOTOR_CH4_DIR_PORT, MOTOR_CH4_DIR_PIN);
        LL_TIM_OC_SetCompareCH4(MOTOR_TIM, (uint32_t)(fabsf(pwm_values[3]) *
                                MOTOR_PWM_TIM_ARR));
        return;
}

static void mk_hw_config()
{
        /* Init motor_kinematics pins */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

        /* Config direction pins */
        LL_GPIO_SetPinMode(MOTOR_CH1_DIR_PORT, MOTOR_CH1_DIR_PIN,
                           LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(MOTOR_CH1_DIR_PORT, MOTOR_CH1_DIR_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(MOTOR_CH1_DIR_PORT, MOTOR_CH1_DIR_PIN,
                           LL_GPIO_PULL_NO);

        LL_GPIO_SetPinMode(MOTOR_CH2_DIR_PORT, MOTOR_CH2_DIR_PIN,
                           LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(MOTOR_CH2_DIR_PORT, MOTOR_CH2_DIR_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(MOTOR_CH2_DIR_PORT, MOTOR_CH2_DIR_PIN,
                           LL_GPIO_PULL_NO);

        LL_GPIO_SetPinMode(MOTOR_CH3_DIR_PORT, MOTOR_CH3_DIR_PIN,
                           LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(MOTOR_CH3_DIR_PORT, MOTOR_CH3_DIR_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(MOTOR_CH3_DIR_PORT, MOTOR_CH3_DIR_PIN,
                           LL_GPIO_PULL_NO);

        LL_GPIO_SetPinMode(MOTOR_CH4_DIR_PORT, MOTOR_CH4_DIR_PIN,
                           LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(MOTOR_CH4_DIR_PORT, MOTOR_CH4_DIR_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(MOTOR_CH4_DIR_PORT, MOTOR_CH4_DIR_PIN,
                           LL_GPIO_PULL_NO);

        /* Config PWM pins */
        LL_GPIO_SetPinMode(MOTOR_CH_PWM_PORT, MOTOR_CH1_PWM_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(MOTOR_CH_PWM_PORT, MOTOR_CH1_PWM_PIN,
                              MOTOR_CH_PWM_PIN_AF);
        LL_GPIO_SetPinOutputType(MOTOR_CH_PWM_PORT, MOTOR_CH1_PWM_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);

        LL_GPIO_SetPinMode(MOTOR_CH_PWM_PORT, MOTOR_CH2_PWM_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(MOTOR_CH_PWM_PORT, MOTOR_CH2_PWM_PIN,
                              MOTOR_CH_PWM_PIN_AF);
        LL_GPIO_SetPinOutputType(MOTOR_CH_PWM_PORT, MOTOR_CH2_PWM_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);

        LL_GPIO_SetPinMode(MOTOR_CH_PWM_PORT, MOTOR_CH3_PWM_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(MOTOR_CH_PWM_PORT, MOTOR_CH3_PWM_PIN,
                              MOTOR_CH_PWM_PIN_AF);
        LL_GPIO_SetPinOutputType(MOTOR_CH_PWM_PORT, MOTOR_CH3_PWM_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);

        LL_GPIO_SetPinMode(MOTOR_CH_PWM_PORT, MOTOR_CH4_PWM_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(MOTOR_CH_PWM_PORT, MOTOR_CH4_PWM_PIN,
                              MOTOR_CH_PWM_PIN_AF);
        LL_GPIO_SetPinOutputType(MOTOR_CH_PWM_PORT, MOTOR_CH4_PWM_PIN,
                                 LL_GPIO_OUTPUT_PUSHPULL);

        /* Init timer in PWM mode */
        LL_TIM_EnableUpdateEvent(MOTOR_TIM);
        LL_TIM_SetClockDivision(MOTOR_TIM, LL_TIM_CLOCKDIVISION_DIV4);
        LL_TIM_SetCounterMode(MOTOR_TIM, LL_TIM_COUNTERMODE_UP);
        LL_TIM_SetAutoReload(MOTOR_TIM, MOTOR_PWM_TIM_ARR);
        LL_TIM_SetUpdateSource(MOTOR_TIM, LL_TIM_UPDATESOURCE_REGULAR);

        /* Enable capture mode */
        LL_TIM_CC_EnableChannel(MOTOR_TIM, LL_TIM_CHANNEL_CH1 |
                                LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 |
                                LL_TIM_CHANNEL_CH4);

        /* Set PWM mode */
        LL_TIM_OC_SetMode(MOTOR_TIM, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
        LL_TIM_OC_SetMode(MOTOR_TIM, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
        LL_TIM_OC_SetMode(MOTOR_TIM, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
        LL_TIM_OC_SetMode(MOTOR_TIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);

        /* Enable fast mode */
        LL_TIM_OC_EnableFast(MOTOR_TIM, LL_TIM_CHANNEL_CH1);
        LL_TIM_OC_EnableFast(MOTOR_TIM, LL_TIM_CHANNEL_CH2);
        LL_TIM_OC_EnableFast(MOTOR_TIM, LL_TIM_CHANNEL_CH3);
        LL_TIM_OC_EnableFast(MOTOR_TIM, LL_TIM_CHANNEL_CH4);

        /* Enable preload */
        LL_TIM_OC_EnablePreload(MOTOR_TIM, LL_TIM_CHANNEL_CH1);
        LL_TIM_OC_EnablePreload(MOTOR_TIM, LL_TIM_CHANNEL_CH2);
        LL_TIM_OC_EnablePreload(MOTOR_TIM, LL_TIM_CHANNEL_CH3);
        LL_TIM_OC_EnablePreload(MOTOR_TIM, LL_TIM_CHANNEL_CH4);
        LL_TIM_EnableARRPreload(MOTOR_TIM);

        /* Set initial value */
        LL_TIM_OC_SetCompareCH1(MOTOR_TIM, MOTOR_PWM_TIM_CCR_INIT);
        LL_TIM_OC_SetCompareCH2(MOTOR_TIM, MOTOR_PWM_TIM_CCR_INIT);
        LL_TIM_OC_SetCompareCH3(MOTOR_TIM, MOTOR_PWM_TIM_CCR_INIT);
        LL_TIM_OC_SetCompareCH4(MOTOR_TIM, MOTOR_PWM_TIM_CCR_INIT);

        /* Enable timer */
        LL_TIM_GenerateEvent_UPDATE(MOTOR_TIM);
        LL_TIM_EnableCounter(MOTOR_TIM);
        return;
}

static void mk_set_pwm_ctrl(motors_ctrl_t *mk_ctrl)
{
        mk_ctrl->status |= MK_PWM_CONTROL_BIT;
        mk_ctrl->status &= ~MK_SPEED_CONTROL_BIT;
        return;
}

static void mk_set_speed_ctrl(motors_ctrl_t *mk_ctrl)
{
        mk_ctrl->status |= MK_SPEED_CONTROL_BIT;
        mk_ctrl->status &= ~MK_PWM_CONTROL_BIT;
        return;
}
/*
 * End of section with helper functions
 */

/*
 * Main motor kinematics task running by FreeRTOS
 */
void motor_kinematics(void *arg)
{
        (void) arg;

        /*
         * Proper initialization of main motor
         * control structure
         * All pwm values are set to be 10% of max to avoid reset
         * of Maxon motors (his majesty does not like to reseted
         * as he is pricey)
         */
        motors_ctrl_t mk_ctrl_st = {
                .status = 0x00,
                .vel_x = 0.0f,
                .vel_y = 0.0f,
                .wz = 0.0f,
                .pwm_motors = {0.1f, 0.1f, 0.1f, 0.1f}
        };

        mk_hw_config();
        mk_ctrl_st.lock = xSemaphoreCreateMutexStatic(&mutex_buffer);
        mk_ctrl_st.mk_notify = xTaskGetCurrentTaskHandle();
        mk_ctrl = &mk_ctrl_st;

        while (1) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                xSemaphoreTake(mk_ctrl->lock, portMAX_DELAY);
                /*
                 * If one stopped motors immediately reset all pwm values
                 */
                if (mk_ctrl->status & MK_STOP_MOTORS_BIT) {
                        mk_ctrl->pwm_motors[0] = 0.1f;
                        mk_ctrl->pwm_motors[1] = 0.1f;
                        mk_ctrl->pwm_motors[2] = 0.1f;
                        mk_ctrl->pwm_motors[3] = 0.1f;
                }
                /*
                 * If motors are allowed to be running and control
                 * was switched to speed mode call mk_speed2pwm to calculate
                 * matrix kinematics
                 */
                if (mk_ctrl->status & MK_SPEED_CONTROL_BIT &&
                    !(mk_ctrl->status & MK_STOP_MOTORS_BIT)) {
                        mk_speed2pwm(mk_ctrl);
                }
                /*
                 * In the end all pwm values shall be updated and lock
                 * released.
                 */
                mk_set_pwm(mk_ctrl->pwm_motors);
                xSemaphoreGive(mk_ctrl->lock);
        }
        return;
}

/*
 * Set of motor related handlers for terminal
 */

/*
 * Set motors pwm command
 * Input: values for each pwm channel
 * Output: OK or ERROR status
 */
int cmd_set_pwm(void *args)
{
        cmd_set_pwm_t *cmd_args = (cmd_set_pwm_t *)args;

        /*
         * Check whether kinematics ready or not
         * and check parameters
         */
        if (!mk_ctrl)
                goto error_set_pwm;
        if (SET_PWM_ARGS_ERR(cmd_args))
                goto error_set_pwm;
        /*
         * Update control structure
         */
        xSemaphoreTake(mk_ctrl->lock, portMAX_DELAY);
        mk_set_pwm_ctrl(mk_ctrl);
        mk_ctrl->pwm_motors[cmd_args->channel - 1] = cmd_args->pwm_value;
        xSemaphoreGive(mk_ctrl->lock);
        /*
         * Wake up kinematics task
         */
        xTaskNotifyGive(mk_ctrl->mk_notify);
        memcpy(args, "OK", 3);
        return 3;
error_set_pwm:
        memcpy(args, "ERROR", 6);
        return 6;
}

/*
 * Set motors speed command
 * Input: linear and rotational speed
 * Output: OK or ERROR status
 */
int cmd_set_speed(void *args)
{
        cmd_set_speed_t *cmd_args = (cmd_set_speed_t *)args;
        /*
         * Check whether kinematics ready or not
         */
        if (!mk_ctrl)
                goto error_set_speed;
        /*
         * Update control structure
         */
        xSemaphoreTake(mk_ctrl->lock, portMAX_DELAY);
        mk_set_speed_ctrl(mk_ctrl);
        memcpy(&(mk_ctrl->vel_x), &(cmd_args->vx), 12);
        xSemaphoreGive(mk_ctrl->lock);
        /*
         * Wake up kinematics task
         */
        xTaskNotifyGive(mk_ctrl->mk_notify);
        memcpy(args, "OK", 3);
        return 3;
error_set_speed:
        memcpy(args, "ERROR", 6);
        return 6;
}
