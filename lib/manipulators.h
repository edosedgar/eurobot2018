#ifndef _MANIPULATORS_H_
#define _MANIPULATORS_H_

#include "stm32f4xx_ll_usart.h"

#include "FreeRTOS.h"
#include "task.h"

#define STM_DRIVER_BUF_SIZE             256
#define STM_DRIVER_STACK_DEPTH          1024

#define MAX_COMMANDS                    8

/*
 * Set dynamixel angle command
 */
#define DYN_SET_ANGLE(manip_ctrl, num, id, angle, speed, delay) \
        (manip_ctrl)->dyn_cmd[(num)].cmd_buff[0] = (0x01); \
        (manip_ctrl)->dyn_cmd[(num)].cmd_buff[1] = (id); \
        (manip_ctrl)->dyn_cmd[(num)].cmd_buff[2] = (uint8_t) ((angle) & 0xff); \
        (manip_ctrl)->dyn_cmd[(num)].cmd_buff[3] = (uint8_t) (((angle) >> 8) \
                                                             & 0xff); \
        (manip_ctrl)->dyn_cmd[(num)].cmd_buff[4] = (uint8_t) ((speed) & 0xff); \
        (manip_ctrl)->dyn_cmd[(num)].cmd_buff[5] = (uint8_t) (((speed) >> 8) \
                                                             & 0xff); \
        (manip_ctrl)->dyn_cmd[(num)].delay_ms = delay

/*
 * Memory for terminal task
 */
StackType_t manipulators_ts[STM_DRIVER_STACK_DEPTH];
StaticTask_t manipulators_tb;

/*
 * Flags for dynamixel
 */
#define DYN_BUSY_POS                    (0U)
#define DYN_BUSY                        (0x01 << DYN_BUSY_POS)
#define BLOCK_PUMP_POS                  (1U)
#define BLOCK_PUMP                      (0x01 << BLOCK_PUMP_POS)
#define BLOCK_DYN_POS                   (2U)
#define BLOCK_DYN                       (0x01 << BLOCK_DYN_POS)
#define BLOCK_STEPPER_POS               (3U)
#define BLOCK_STEPPER                   (0x01 << BLOCK_STEPPER_POS)

#define is_manip_flag_set(manip_ctrl, bit) \
        (manip_ctrl->flags & bit)

#define manip_set_flag(manip_ctrl, bit) \
        manip_ctrl->flags |= bit

#define manip_clr_flag(manip_ctrl, bit) \
        manip_ctrl->flags &= (~bit)

/*
 * Dynamixel command structure
 */
typedef struct {
        uint8_t cmd_buff[10];
        uint32_t delay_ms;
} dyn_cmd_t;

/*
 * Manipulators control structure defenition
 */
typedef struct {
        dyn_cmd_t dyn_cmd[MAX_COMMANDS];
        uint8_t cmd_len;
        uint8_t flags;
        TaskHandle_t manip_notify;
} manip_ctrl_t;

/*
 * Public function for blocking all manipulators
 */
void manipulators_block(void);
/*
 * Main manager for processing incoming commands
 */
void manipulators_manager(void *arg);

#endif
