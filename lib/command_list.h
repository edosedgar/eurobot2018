#ifndef _H_COMMAND_LIST_
#define _H_COMMAND_LIST_

enum {
        LOWER_BOUND_COMMANDS,
        COM_ECHO                         = 0x01,

        COM_SET_PWM                      = 0x03,
        COM_SET_DIR_BIT                  = 0x04,
        COM_CLEAR_DIR_BIT                = 0x05,

        COM_SET_ALL_MOTOR_SPEEDS         = 0x06,
        COM_GET_ALL_WHEELS_SPEEDS        = 0x07,

        COM_SET_SPEED_ROBOT_CS1          = 0x08,
        COM_GET_SPEED_ROBOT_CS1          = 0x09,
        COM_GET_COORD_ROBOT_CS1          = 0x0A,

        COM_SET_ANGLE_DNMX               = 0x0B,
        COM_GET_ANGLE_DNMX               = 0x0C,

        COM_SET_COORD_ROBOT_CS_GLOBAL    = 0x0E,
        COM_GET_COORD_ROBOT_CS_GLOBAL    = 0x0F,

        COM_TURN_FORW_KIN_ON_OFF         = 0x0D,

        COM_GET_ODOMETRY_MOVEMENT_STATUS = 0xA0,
        COM_GET_MANIPULATOR_STATUS       = 0xA1,
        COM_GET_STARTUP_STATUS           = 0xA3,
        COM_FORCED_START                 = 0xA4,

        COM_TAKE_CUBE                    = 0xB0,
        COM_UNLOAD_TOWER                 = 0xB1,
        COM_OPEN_CLOSE_DOOR              = 0xB2,
        COM_LIFT_MANIPULATOR_TO_INTERM   = 0xB3,
        COM_RELEASE_MAGIC_CUBE           = 0xB4,
        COM_TAKE_LAST_CUBE               = 0xB5,
        COM_MAKE_FUNNY_ACTION            = 0xB6,
        COM_FORM_CUBE                    = 0xB7,

        COM_ODOMETRY_MOVEMENT            = 0xA2,

        COM_GET_DATA_AND_STATUS_FROM_RF  = 0xD0,

        COM_TURN_COLL_AVOID_ON_OFF       = 0xE0,
        COM_REBOOT_SERVOS                = 0XE1,
        UPPER_BOUND_COMMANDS,
};

#endif
