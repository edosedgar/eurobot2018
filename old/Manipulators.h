#ifndef MANIPULATORS
#define MANIPULATORS
#include "Robot.h"
#include "Dynamixel_control.h"
#include "stm32f4xx.h"

#define NUMBER_OF_MANIPULATORS               0x03
#define MANIPULATOR_INIT_TIMEOUT_TENTH_OF_MS 0xEA60  // equals 6000  ms

//--------------------------------------------- Typedefs and enums for tasks executor ---------------------------//

#define TASKS_EXECUTOR_MAX_SEND_RETRIES      0x0A

// Tasks executor's status
typedef enum
{
	TASKS_EXECUTOR_SUCCESFUL_EXECUTION,
	TASKS_EXECUTOR_ACTIVE_MODE,
	TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED,
	TASKS_EXECUTOR_ERROR_WRONG_POSITION,
	TASKS_EXECUTOR_ERROR_MAX_LOAD_EXCEEDED,
	TASKS_EXECUTOR_ERROR_TERMINATOR_REACHED,
} Tasks_Executor_Status;

// Enumeration for possible subtasks
typedef enum
{
	SUBTASK_OPEN_MANIPULATOR,
	SUBTASK_CLOSE_MANIPULATOR,
	SUBTASK_LOWER_MANIPULATOR,
	SUBTASK_LOWER_MANIPULATOR_LOWER_INTERM,
	SUBTASK_LIFT_MANIPULATOR,
	SUBTASK_LIFT_MANIPULATOR_INTERM,
	SUBTASK_LIFT_MANIPULATOR_UPPER_INTERM,
	SUBTASK_OPEN_DOOR,
	SUBTASK_OPEN_DOOR_SLIGHTLY,
	SUBTASK_CLOSE_DOOR,
	SUBTASK_PUSH_MAGIC_CUBE,
	SUBTASK_RETURN_MAGIC_CUBE,
	SUBTASK_TERMINATOR
} Manipulator_Subtasks_Typedef;

//--------------------------------------------- Typedefs and enums for servoChecker timer ----------------------//

#define SERVO_CHECKER_ANGLE_EPS              10.0f
#define SERVO_CHECKER_MAX_READ_REQUESTS      0x0A
// Maximum timeout in ms
#define SERVO_CHECKER_TIMEOUT_TENTH_OF_MS    0x4650  // equals 1800 ms

// Checker status
typedef enum
{
	SERVO_CHECKER_WAITING_MODE,
	SERVO_CHECKER_ACTIVE_MODE,
	SERVO_CHECKER_SUCCESFUL_CONFIRMATION,
	SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED,
	SERVO_CHECKER_ERROR_WRONG_POSITION,
	SERVO_CHECKER_ERROR_MAXIMUM_LOAD_EXCEEDED,
} Checker_Status_Typedef;

// Checker structure
typedef struct
{
	uint8_t                 servoId;
	uint32_t                startTime;
	float                   targetPos;
	float                   previousPos;
	Checker_Status_Typedef  statusFlag;
}Servo_Checker_Typedef;

//--------------------------------------------- Typedefs for manipulator control --------------------------------//

// Struct for gripper's parameters
typedef struct
{
	uint8_t id;
	uint16_t openedAngle;
	uint16_t closedAngle;
} Gripper_Typedef;

// Struct for magic cube's parameters
typedef struct
{
	uint8_t id;
	uint16_t initialPosition;
	uint16_t finalPosition;
} Magic_Cube_Typedef;

// Struct for sliders's parameters
typedef struct
{
	uint8_t id;
	uint16_t topPos;
	uint16_t botPos;
	uint16_t intermPos;
	uint16_t uppperIntermPos;
	uint16_t lowerIntermPos;
} Slider_Typedef;

// Struct for door's parameters
typedef struct
{
	uint8_t id;
	uint16_t openedAngle;
	uint16_t openedSlightlyAngle;
	uint16_t closedAngle;
} Door_Typedef;

// Struct for cube manipulators's parameters
typedef struct
{
	Gripper_Typedef                gripper;
	Slider_Typedef                 slider;
	Door_Typedef                   door;
	Magic_Cube_Typedef             magicCube;
	Manipulator_Subtasks_Typedef*  tasksSequencePtr;
	Tasks_Executor_Status          subtasksExecutorStatusFlag;
	
}Cube_Manipulator_Typedef;

// Manipulator's commands
typedef enum
{
	TAKE_CUBE_COMMAND,
	TAKE_LAST_CUBE_COMMAND,
	UNLOAD_TOWER_COMMAND,
	LIFT_TO_INTERMEDIATE_POS_COMMAND,
	OPEN_DOOR_COMMAND,
	OPEN_DOOR_SLIGHTLY_COMMAND,
	CLOSE_DOOR_COMMAND,
	RELEASE_MAGIC_CUBE_COMMAND,
	FORM_CUBE_COMMAND,
	INIT_MANIPULATOR_COMMAND,
} Manipulator_Command_Typedef;

//--------------------------------------------- Macros for servos ------------------------------------------------//
// P.S Left, right and central manipulators when we are in front of robot
// ID of servo motors
#define MANIP_RIGHT_SERVO_SLIDER_ID                 0x01  // id = 1
#define MANIP_RIGHT_SERVO_GRIPPER_ID                0x12  // id = 18
#define MANIP_RIGHT_SERVO_MAGIC_CUBE_ID             0x14  // id = 20
#define MANIP_RIGHT_SERVO_DOOR_ID                   0x0A  // id = 10

#define MANIP_LEFT_SERVO_SLIDER_ID                  0x02  // id = 2
#define MANIP_LEFT_SERVO_GRIPPER_ID                 0x10  // id = 16
#define MANIP_LEFT_SERVO_MAGIC_CUBE_ID              0x15  // id = 21
#define MANIP_LEFT_SERVO_DOOR_ID                    0x0B  // id = 11

#define MANIP_CENTRAL_SERVO_SLIDER_ID               0x03  // id = 3 
#define MANIP_CENTRAL_SERVO_GRIPPER_ID              0x11  // id = 17
#define MANIP_CENTRAL_SERVO_FUNNY_ID                0x64  // id = 100

// Boundary angles
#define MANIP_RIGHT_SERVO_SLIDER_TOP_POS            0x14  // 20°
#define MANIP_RIGHT_SERVO_SLIDER_BOT_POS            0x124 // 292°
#define MANIP_RIGHT_SERVO_SLIDER_INTERM_POS         0xF5  // 245°
#define MANIP_RIGHT_SERVO_SLIDER_UPPER_INTERM_POS   0x41  // 65°
#define MANIP_RIGHT_SERVO_SLIDER_LOWER_INTERM_POS   0xCD  // 205°
#define MANIP_RIGHT_SERVO_GRIPPER_OPENED_POS        0x46  // 70°
#define MANIP_RIGHT_SERVO_GRIPPER_CLOSED_POS        0x7F  // 127°
#define MANIP_RIGHT_SERVO_MAGIC_CUBE_INITIAL_POS    0xF5  // 245°
#define MANIP_RIGHT_SERVO_MAGIC_CUBE_FINAL_POS      0x12C // 300°
#define MANIP_RIGHT_SERVO_DOOR_OPENED_POS           0xE6  // 230°
#define MANIP_RIGHT_SERVO_DOOR_OPENED_SLIGHTLY_POS  0x9C  // 156°
#define MANIP_RIGHT_SERVO_DOOR_CLOSED_POS           0x97  // 151°

#define MANIP_LEFT_SERVO_SLIDER_TOP_POS             0x122 // 290°
#define MANIP_LEFT_SERVO_SLIDER_BOT_POS             0x14  // 20°
#define MANIP_LEFT_SERVO_SLIDER_INTERM_POS          0x3C  // 60° TBD
#define MANIP_LEFT_SERVO_SLIDER_UPPER_INTERM_POS    0xFA  // 250°
#define MANIP_LEFT_SERVO_SLIDER_LOWER_INTERM_POS    0x64  // 100°
#define MANIP_LEFT_SERVO_GRIPPER_OPENED_POS         0xAA  // 170°
#define MANIP_LEFT_SERVO_GRIPPER_CLOSED_POS         0x6E  // 110°
#define MANIP_LEFT_SERVO_MAGIC_CUBE_INITIAL_POS     0x46  // 70°
#define MANIP_LEFT_SERVO_MAGIC_CUBE_FINAL_POS       0x00  // 0°
#define MANIP_LEFT_SERVO_DOOR_OPENED_POS            0x46  // 70°
#define MANIP_LEFT_SERVO_DOOR_OPENED_SLIGHTLY_POS   0x8E  // 142°
#define MANIP_LEFT_SERVO_DOOR_CLOSED_POS            0x93  // 147°

#define MANIP_CENTRAL_SERVO_SLIDER_TOP_POS          0x0C  // 12°  
#define MANIP_CENTRAL_SERVO_SLIDER_BOT_POS          0x11F // 287° 
#define MANIP_CENTRAL_SERVO_SLIDER_INTERM_POS       0xFA  // 250° TBD
#define MANIP_CENTRAL_SERVO_SLIDER_UPPER_INTERM_POS 0x32  // 50°
#define MANIP_CENTRAL_SERVO_SLIDER_LOWER_INTERM_POS 0xC3  // 195°
#define MANIP_CENTRAL_SERVO_GRIPPER_OPENED_POS      0x50  // 80°
#define MANIP_CENTRAL_SERVO_GRIPPER_CLOSED_POS      0x8C  // 140°
#define MANIP_CENTRAL_SERVO_FUNNY_BUTTON_POS        0x38  // 56° 
#define MANIP_CENTRAL_SERVO_FUNNY_BEE_POS           0x8F  // 143° 
#define MANIP_CENTRAL_SERVO_FUNNY_CLOSED_POS        0xE7  // 231°

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//
void initManipulators(void);
static ErrorStatus setServoAngleWithRetries(const uint8_t servoId, const uint16_t servoAngle);
static ErrorStatus getServoAngleWithRetries(const uint8_t servoId, float* servoAngle);
//static ErrorStatus getServoLoadWithRetries(const uint8_t servoId, uint16_t* servoLoad);

//--------------------------------------------- ServoChecker functions -----------------------------------------//
void checkPosServo(Servo_Checker_Typedef* servoChecker);
void resetChecker(Servo_Checker_Typedef* servoChecker);

//--------------------------------------------- Tasks executor's functions -------------------------------------//
void execManipTasks(uint8_t numberOfmanipulator, Cube_Manipulator_Typedef* manipulators);
void execManipSubtasks(uint8_t numberOfManipulator, Cube_Manipulator_Typedef* manipulators);

//--------------------------------------------- High-level command assignment ----------------------------------//
void setManipHighLevelCommand(Manipulator_Command_Typedef command, uint8_t numberOfManipulator, Cube_Manipulator_Typedef* manipulator);
#endif
