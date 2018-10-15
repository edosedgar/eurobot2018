#ifndef ROBOT_COMMANDS
#define ROBOT_COMMANDS

#include "Communication.h"
#include "Robot.h"

extern float wheelsSpeed[ROBOT_NUMBER_OF_MOTORS];
extern float robotTargetSpeedCs1[3];
extern float robotSpeedCs1[3];
extern float robotCoordCs1[3];
extern float robotCoordCsGlobal[3];
extern float accelerationMax[3];
extern uint32_t timeOfStart;
extern Cube_Manipulator_Typedef cubeManipulators[NUMBER_OF_MANIPULATORS];
extern Range_Finders_Struct_Typedef rangeFinders;

enum
{
	ECHO                          = 0x01,
	
	SET_PWM                       = 0x03,
	SET_DIR_BIT                   = 0x04,
	CLEAR_DIR_BIT                 = 0x05,
	
	SET_ALL_MOTOR_SPEEDS          = 0x06,
	GET_ALL_WHEELS_SPEEDS         = 0x07,
	
	SET_SPEED_ROBOT_CS1           = 0x08,
	GET_SPEED_ROBOT_CS1           = 0x09,
	GET_COORD_ROBOT_CS1           = 0x0A,
	
	SET_ANGLE_DNMX                = 0x0B,
	GET_ANGLE_DNMX                = 0x0C,
	
	SET_COORD_ROBOT_CS_GLOBAL     = 0x0E,
	GET_COORD_ROBOT_CS_GLOBAL     = 0x0F,
	
	TURN_FORW_KIN_ON_OFF          = 0x0D,
	
	GET_ODOMETRY_MOVEMENT_STATUS  = 0xA0,
	GET_MANIPULATOR_STATUS        = 0xA1,
	GET_STARTUP_STATUS            = 0xA3,
	FORCED_START                  = 0xA4,
	
	TAKE_CUBE                     = 0xB0,
	UNLOAD_TOWER                  = 0xB1,
	OPEN_CLOSE_DOOR               = 0xB2,
	LIFT_MANIPULATOR_TO_INTERM    = 0xB3,
	RELEASE_MAGIC_CUBE            = 0xB4,
	TAKE_LAST_CUBE                = 0xB5,
	MAKE_FUNNY_ACTION             = 0xB6,
	FORM_CUBE                     = 0xB7,
	
	ODOMETRY_MOVEMENT             = 0xA2,
	
	GET_DATA_AND_STATUS_FROM_RF   = 0xD0,
	
	TURN_COLL_AVOID_ON_OFF        = 0xE0,
	REBOOT_SERVOS                 = 0XE1,
};

void checkCommandAndExecute(void);
#endif
