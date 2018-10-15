#include "Manipulators.h"
#include "Interrupts.h"

Cube_Manipulator_Typedef cubeManipulators[NUMBER_OF_MANIPULATORS];
Servo_Checker_Typedef servoChecker[NUMBER_OF_MANIPULATORS];

// Task sequence for taking a cube
Manipulator_Subtasks_Typedef takeCubeTaskSeq[] = {SUBTASK_OPEN_MANIPULATOR, SUBTASK_LOWER_MANIPULATOR_LOWER_INTERM,
                                                  SUBTASK_CLOSE_MANIPULATOR, SUBTASK_OPEN_MANIPULATOR,
                                                  SUBTASK_LOWER_MANIPULATOR, SUBTASK_CLOSE_MANIPULATOR,
                                                  SUBTASK_LIFT_MANIPULATOR, SUBTASK_TERMINATOR};
// Task sequence for taking last cube
Manipulator_Subtasks_Typedef takeLastCubeTaskSeq[] = {SUBTASK_OPEN_MANIPULATOR, SUBTASK_LOWER_MANIPULATOR_LOWER_INTERM,
	                                                  SUBTASK_CLOSE_MANIPULATOR, SUBTASK_OPEN_MANIPULATOR,
	                                                  SUBTASK_LOWER_MANIPULATOR, SUBTASK_CLOSE_MANIPULATOR,
                                                      SUBTASK_LIFT_MANIPULATOR_INTERM, SUBTASK_TERMINATOR};
// Task sequence for unloading tower
Manipulator_Subtasks_Typedef unloadTowerTaskSeq[] = {SUBTASK_LOWER_MANIPULATOR, SUBTASK_OPEN_MANIPULATOR,
                                                     SUBTASK_TERMINATOR};
// Task sequence for lifting to intermediate position
Manipulator_Subtasks_Typedef liftToIntermPosTaskSeq[] = {SUBTASK_LIFT_MANIPULATOR_INTERM, SUBTASK_TERMINATOR};

// Task sequence for lifting to upper intermediate position
Manipulator_Subtasks_Typedef formCubeTaskSeq[] = {SUBTASK_OPEN_MANIPULATOR, SUBTASK_LOWER_MANIPULATOR_LOWER_INTERM,
                                                  SUBTASK_CLOSE_MANIPULATOR, SUBTASK_OPEN_MANIPULATOR,
                                                  SUBTASK_LOWER_MANIPULATOR, SUBTASK_LIFT_MANIPULATOR_UPPER_INTERM,
	                                              SUBTASK_CLOSE_MANIPULATOR, SUBTASK_LIFT_MANIPULATOR,
                                                  SUBTASK_TERMINATOR};
// Task sequence to open the door
Manipulator_Subtasks_Typedef openDoorTaskSeq[] = {SUBTASK_OPEN_DOOR, SUBTASK_TERMINATOR};

// Task sequence to close the door
Manipulator_Subtasks_Typedef closeDoorTaskSeq[] = {SUBTASK_CLOSE_DOOR, SUBTASK_TERMINATOR};

// Task sequence to open the door a little bit
Manipulator_Subtasks_Typedef openDoorSlightlyTaskSeq[] = {SUBTASK_OPEN_DOOR_SLIGHTLY, SUBTASK_TERMINATOR};

// Task sequence to release magic cube
Manipulator_Subtasks_Typedef releaseMagicCubeTaskSeq[] = {SUBTASK_PUSH_MAGIC_CUBE, SUBTASK_RETURN_MAGIC_CUBE, SUBTASK_TERMINATOR};

// Task sequence to init manipulator
Manipulator_Subtasks_Typedef initManipulatorTaskSeq[] = {SUBTASK_CLOSE_MANIPULATOR, SUBTASK_LIFT_MANIPULATOR, SUBTASK_CLOSE_DOOR, SUBTASK_TERMINATOR};

Manipulator_Subtasks_Typedef taskTerminator = SUBTASK_TERMINATOR;
													 
// Timer interrupt handler for servoChecker
void TIM5_IRQHandler(void)
{
	if (SERVO_CHECKER_TIM_MODULE->SR & TIM_SR_UIF)
	{	  
		timClearStatusRegisterFlag(SERVO_CHECKER_TIM_MODULE, TIM_SR_UIF);
		// Check manipulators
		execManipTasks(0x00, &cubeManipulators[0]);
		execManipTasks(0x01, &cubeManipulators[1]);
		execManipTasks(0x02, &cubeManipulators[2]);
		checkPosServo(&servoChecker[0]);
		checkPosServo(&servoChecker[1]);
		checkPosServo(&servoChecker[2]);
	}
	return;
}

//--------------------------------------------- FUNCTIONS ------------------------------------------------------//
void initManipulators(void)
{
	cubeManipulators[0].gripper.id = MANIP_LEFT_SERVO_GRIPPER_ID;
	cubeManipulators[0].gripper.closedAngle = MANIP_LEFT_SERVO_GRIPPER_CLOSED_POS;
	cubeManipulators[0].gripper.openedAngle = MANIP_LEFT_SERVO_GRIPPER_OPENED_POS;
	cubeManipulators[0].slider.id = MANIP_LEFT_SERVO_SLIDER_ID;
	cubeManipulators[0].slider.botPos = MANIP_LEFT_SERVO_SLIDER_BOT_POS;
	cubeManipulators[0].slider.topPos = MANIP_LEFT_SERVO_SLIDER_TOP_POS;
	cubeManipulators[0].slider.intermPos = MANIP_LEFT_SERVO_SLIDER_INTERM_POS;
	cubeManipulators[0].slider.uppperIntermPos = MANIP_LEFT_SERVO_SLIDER_UPPER_INTERM_POS;
	cubeManipulators[0].slider.lowerIntermPos = MANIP_LEFT_SERVO_SLIDER_LOWER_INTERM_POS;
	cubeManipulators[0].door.id = MANIP_LEFT_SERVO_DOOR_ID;
	cubeManipulators[0].door.closedAngle = MANIP_LEFT_SERVO_DOOR_CLOSED_POS;
	cubeManipulators[0].door.openedAngle  = MANIP_LEFT_SERVO_DOOR_OPENED_POS;
	cubeManipulators[0].door.openedSlightlyAngle  = MANIP_LEFT_SERVO_DOOR_OPENED_SLIGHTLY_POS;
	cubeManipulators[0].magicCube.id = MANIP_LEFT_SERVO_MAGIC_CUBE_ID;
	cubeManipulators[0].magicCube.initialPosition = MANIP_LEFT_SERVO_MAGIC_CUBE_INITIAL_POS;
	cubeManipulators[0].magicCube.finalPosition = MANIP_LEFT_SERVO_MAGIC_CUBE_FINAL_POS;
	cubeManipulators[0].tasksSequencePtr = &taskTerminator;
	
	cubeManipulators[1].gripper.id = MANIP_CENTRAL_SERVO_GRIPPER_ID;
	cubeManipulators[1].gripper.closedAngle = MANIP_CENTRAL_SERVO_GRIPPER_CLOSED_POS;
	cubeManipulators[1].gripper.openedAngle = MANIP_CENTRAL_SERVO_GRIPPER_OPENED_POS;
	cubeManipulators[1].slider.id = MANIP_CENTRAL_SERVO_SLIDER_ID;
	cubeManipulators[1].slider.botPos = MANIP_CENTRAL_SERVO_SLIDER_BOT_POS;
	cubeManipulators[1].slider.topPos = MANIP_CENTRAL_SERVO_SLIDER_TOP_POS;
	cubeManipulators[1].slider.intermPos = MANIP_CENTRAL_SERVO_SLIDER_INTERM_POS;
	cubeManipulators[1].slider.uppperIntermPos = MANIP_CENTRAL_SERVO_SLIDER_UPPER_INTERM_POS;
	cubeManipulators[1].slider.lowerIntermPos = MANIP_CENTRAL_SERVO_SLIDER_LOWER_INTERM_POS;
	cubeManipulators[1].door.id = MANIP_CENTRAL_SERVO_FUNNY_ID;
	cubeManipulators[1].door.closedAngle = MANIP_CENTRAL_SERVO_FUNNY_CLOSED_POS;
	cubeManipulators[1].door.openedAngle  = MANIP_CENTRAL_SERVO_FUNNY_BEE_POS;
	cubeManipulators[1].door.openedSlightlyAngle  = MANIP_CENTRAL_SERVO_FUNNY_BUTTON_POS;
	cubeManipulators[1].tasksSequencePtr = &taskTerminator;
	
	cubeManipulators[2].gripper.id = MANIP_RIGHT_SERVO_GRIPPER_ID;
	cubeManipulators[2].gripper.closedAngle = MANIP_RIGHT_SERVO_GRIPPER_CLOSED_POS;
	cubeManipulators[2].gripper.openedAngle = MANIP_RIGHT_SERVO_GRIPPER_OPENED_POS;
	cubeManipulators[2].slider.id = MANIP_RIGHT_SERVO_SLIDER_ID;
	cubeManipulators[2].slider.botPos = MANIP_RIGHT_SERVO_SLIDER_BOT_POS;
	cubeManipulators[2].slider.topPos = MANIP_RIGHT_SERVO_SLIDER_TOP_POS;
	cubeManipulators[2].slider.intermPos = MANIP_RIGHT_SERVO_SLIDER_INTERM_POS;
	cubeManipulators[2].slider.uppperIntermPos = MANIP_RIGHT_SERVO_SLIDER_UPPER_INTERM_POS;
	cubeManipulators[2].slider.lowerIntermPos = MANIP_RIGHT_SERVO_SLIDER_LOWER_INTERM_POS;
	cubeManipulators[2].door.id = MANIP_RIGHT_SERVO_DOOR_ID;
	cubeManipulators[2].door.closedAngle = MANIP_RIGHT_SERVO_DOOR_CLOSED_POS;
	cubeManipulators[2].door.openedAngle  = MANIP_RIGHT_SERVO_DOOR_OPENED_POS;
	cubeManipulators[2].door.openedSlightlyAngle  = MANIP_RIGHT_SERVO_DOOR_OPENED_SLIGHTLY_POS;
	cubeManipulators[2].magicCube.id = MANIP_RIGHT_SERVO_MAGIC_CUBE_ID;
	cubeManipulators[2].magicCube.initialPosition = MANIP_RIGHT_SERVO_MAGIC_CUBE_INITIAL_POS;
	cubeManipulators[2].magicCube.finalPosition = MANIP_RIGHT_SERVO_MAGIC_CUBE_FINAL_POS;
	cubeManipulators[2].tasksSequencePtr = &taskTerminator;
	
	uint8_t i;
	
	// Turn all actuators into the start position
	for (i = 0x00; i < NUMBER_OF_MANIPULATORS; i++)
	{
		setManipHighLevelCommand(INIT_MANIPULATOR_COMMAND, i, &cubeManipulators[i]);
	}
	
	uint32_t startTime = getLocalTime();
	
	while(!checkTimeout(startTime, MANIPULATOR_INIT_TIMEOUT_TENTH_OF_MS))
	{
		// Check status
		for (i = 0x00; i < NUMBER_OF_MANIPULATORS; i++)
		{
			if ((cubeManipulators[i].subtasksExecutorStatusFlag != TASKS_EXECUTOR_SUCCESFUL_EXECUTION)
				&& (cubeManipulators[i].subtasksExecutorStatusFlag != TASKS_EXECUTOR_ACTIVE_MODE))
			{
				// Show error
				showError();
				return;
			}
		}
	}
	// No error
	showNoError();
	return;
}

static ErrorStatus setServoAngleWithRetries(const uint8_t servoId, const uint16_t servoAngle)
{
	uint8_t i;
	for (i = 0x00; i < TASKS_EXECUTOR_MAX_SEND_RETRIES; i++)
	{
		if (setServoAngle(servoId, servoAngle))
		{
			return SUCCESS;
		}
	}
	return ERROR;
}

static ErrorStatus getServoAngleWithRetries(const uint8_t servoId, float* servoAngle)
{
	uint8_t i;
	for (i = 0x00; i < TASKS_EXECUTOR_MAX_SEND_RETRIES; i++)
	{
		if (getServoAngle(servoId, servoAngle))
		{
			return SUCCESS;
		}
	}
	return ERROR;
}
//--------------------------------------------- servoChecker functions ---------------------------------------//
void checkPosServo(Servo_Checker_Typedef* servoChecker)
{
	if (servoChecker->statusFlag == SERVO_CHECKER_ACTIVE_MODE)
	{	
		// buffer for current angle and torgue
		float angle;

		// Read current angle of the servo
		if (!getServoAngleWithRetries(servoChecker->servoId, &angle))
		{
			servoChecker->statusFlag = SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED;
			return;
		}
		
		// Check if servo reached target position
		if (fabs(servoChecker->targetPos - angle) < SERVO_CHECKER_ANGLE_EPS)
		{
			servoChecker->statusFlag = SERVO_CHECKER_SUCCESFUL_CONFIRMATION;
			return;
		}
//		else if (servoChecker->numberOfTimerPeriods >= SERVO_CHECKER_MAX_TIMEOUT)
//		{
//			servoChecker->statusFlag = SERVO_CHECKER_ERROR_WRONG_POSITION;
//			return;
//		}
		else if (checkTimeout(servoChecker->startTime, SERVO_CHECKER_TIMEOUT_TENTH_OF_MS))
		{
			servoChecker->statusFlag = SERVO_CHECKER_ERROR_WRONG_POSITION;
			return;
		}
	}
	return;
}

void resetChecker(Servo_Checker_Typedef* servoChecker)
{
	servoChecker->startTime = 0x00;
	servoChecker->servoId = 0x00;
	servoChecker->targetPos = 0.0;
	servoChecker->statusFlag = SERVO_CHECKER_WAITING_MODE;
	return;
}
//--------------------------------------------- Tasks executor's functions -------------------------------------//
void execManipTasks(uint8_t number, Cube_Manipulator_Typedef* manipulator)
{	
	// Check if task executor is active
	if (manipulator->subtasksExecutorStatusFlag == TASKS_EXECUTOR_ACTIVE_MODE)
	{
		Checker_Status_Typedef checkerStatusFlag = servoChecker[number].statusFlag;
		switch (checkerStatusFlag)
		{
			case SERVO_CHECKER_WAITING_MODE:
				// First command
				execManipSubtasks(number, manipulator);
				// Extend timer
				timEnable(SERVO_CHECKER_TIM_MODULE);
				break;
			case SERVO_CHECKER_ACTIVE_MODE:
				// Wait for servo checker
				// Extend timer
				timEnable(SERVO_CHECKER_TIM_MODULE);
				break;
			case SERVO_CHECKER_SUCCESFUL_CONFIRMATION:
				// Check if all subtasks were executed
				manipulator->tasksSequencePtr++;
				if(manipulator->tasksSequencePtr[0] == SUBTASK_TERMINATOR)
				{
					manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_SUCCESFUL_EXECUTION;
					// Reset servo checker
					resetChecker(&servoChecker[number]);
				}
				else
				{
					// Next subtask
					execManipSubtasks(number, manipulator);
				}
				break;
			case SERVO_CHECKER_ERROR_MAXIMUM_RETRIES_EXCEEDED:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
				// Reset servo checker
				resetChecker(&servoChecker[number]);
				break;
			case SERVO_CHECKER_ERROR_WRONG_POSITION:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_WRONG_POSITION;
				// Reset servo checker
				resetChecker(&servoChecker[number]);
				break;
			case SERVO_CHECKER_ERROR_MAXIMUM_LOAD_EXCEEDED:
				manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_LOAD_EXCEEDED;
				// Reset servo checker
				resetChecker(&servoChecker[number]);
				break;
		}
	}
	return;
}
void execManipSubtasks(uint8_t number, Cube_Manipulator_Typedef* manipulator)
{
	// Extract current subtask
	Manipulator_Subtasks_Typedef subtask = *(manipulator->tasksSequencePtr);
	
	// Buffers for servo id, target position and current position
	uint16_t servoTargetPos;
	float servoCurrentPos;
	uint8_t servoId;
	
	// Unload values that corresponds to partivular subtask
	switch (subtask)
	{
		case SUBTASK_OPEN_MANIPULATOR:
			servoId  = manipulator->gripper.id;
			servoTargetPos = manipulator->gripper.openedAngle;
			break;
		case SUBTASK_CLOSE_MANIPULATOR:
			servoId = manipulator->gripper.id;
			servoTargetPos = manipulator->gripper.closedAngle;
			break;
		case SUBTASK_LOWER_MANIPULATOR:
			servoId = manipulator->slider.id;
			servoTargetPos = manipulator->slider.botPos;
			break;
		case SUBTASK_LOWER_MANIPULATOR_LOWER_INTERM:
			servoId = manipulator->slider.id;
			servoTargetPos = manipulator->slider.lowerIntermPos;
			break;
		case SUBTASK_LIFT_MANIPULATOR:
			servoId = manipulator->slider.id;
			servoTargetPos = manipulator->slider.topPos;
			break;
		case SUBTASK_LIFT_MANIPULATOR_INTERM:
			servoId = manipulator->slider.id;
			servoTargetPos = manipulator->slider.intermPos;
			break;
		case SUBTASK_LIFT_MANIPULATOR_UPPER_INTERM:
			servoId = manipulator->slider.id;
			servoTargetPos = manipulator->slider.uppperIntermPos;
			break;
		case SUBTASK_OPEN_DOOR:
			servoId = manipulator->door.id;
			servoTargetPos = manipulator->door.openedAngle;
			break;
		case SUBTASK_OPEN_DOOR_SLIGHTLY:
			servoId = manipulator->door.id;
			servoTargetPos = manipulator->door.openedSlightlyAngle;	
			break;		
		case SUBTASK_CLOSE_DOOR:
			servoId = manipulator->door.id;
			servoTargetPos = manipulator->door.closedAngle;
			break;
		case SUBTASK_PUSH_MAGIC_CUBE:
			servoId = manipulator->magicCube.id;
			servoTargetPos = manipulator->magicCube.finalPosition;
			break;
		case SUBTASK_RETURN_MAGIC_CUBE:
			servoId  = manipulator->magicCube.id;
			servoTargetPos = manipulator->magicCube.initialPosition;
			break;
		case SUBTASK_TERMINATOR:
			manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_TERMINATOR_REACHED;
			return;
	}
	// Read current angle of the servo
	if (!getServoAngleWithRetries(servoId, &servoCurrentPos))
	{
		manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
		// Reset servo checker
		resetChecker(&servoChecker[number]);
		return;
	}
	// Send set angle command to servo
	if (!setServoAngleWithRetries(servoId, servoTargetPos))
	{
		manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ERROR_MAX_RETRIES_EXCEEDED;
		// Reset servo checker
		resetChecker(&servoChecker[number]);
		return;
	}
	// Reset servo checker
	resetChecker(&servoChecker[number]);
	// Load angle, id, current time into servoChecker and turn it on
	servoChecker[number].servoId = servoId;
	servoChecker[number].targetPos = servoTargetPos;
	servoChecker[number].previousPos = servoCurrentPos;
	servoChecker[number].statusFlag = SERVO_CHECKER_ACTIVE_MODE;
	servoChecker[number].startTime = getLocalTime();
	// Extend timer
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}

//--------------------------------------------- High-level command assignment ----------------------------------//
void setManipHighLevelCommand(Manipulator_Command_Typedef command, uint8_t number, Cube_Manipulator_Typedef* manipulator)
{
	if (number > NUMBER_OF_MANIPULATORS)
	{
		// Wrong manipulator's id
		return;
	}
	//Check if manipulator is still working
	if (manipulator->subtasksExecutorStatusFlag == TASKS_EXECUTOR_ACTIVE_MODE)
	{
		return;
	}
	// Set Command
	switch(command)
	{
		case TAKE_CUBE_COMMAND:
			manipulator->tasksSequencePtr = takeCubeTaskSeq;
			break;
		case TAKE_LAST_CUBE_COMMAND:
			manipulator->tasksSequencePtr = takeLastCubeTaskSeq;
			break;
		case UNLOAD_TOWER_COMMAND:
			manipulator->tasksSequencePtr = unloadTowerTaskSeq;
			break;
		case LIFT_TO_INTERMEDIATE_POS_COMMAND:
			manipulator->tasksSequencePtr = liftToIntermPosTaskSeq;
			break;
		case FORM_CUBE_COMMAND:
			manipulator->tasksSequencePtr = formCubeTaskSeq;
			break;
		case OPEN_DOOR_COMMAND:
			manipulator->tasksSequencePtr = openDoorTaskSeq;
			break;
		case OPEN_DOOR_SLIGHTLY_COMMAND:
			manipulator->tasksSequencePtr = openDoorSlightlyTaskSeq;
			break;
		case CLOSE_DOOR_COMMAND:
			manipulator->tasksSequencePtr = closeDoorTaskSeq;
			break;
		case RELEASE_MAGIC_CUBE_COMMAND:
			manipulator->tasksSequencePtr = releaseMagicCubeTaskSeq;
			break;
		case INIT_MANIPULATOR_COMMAND:
			manipulator->tasksSequencePtr = initManipulatorTaskSeq;
			break;
	}
	// Set flag for task sequence execution
	manipulator->subtasksExecutorStatusFlag = TASKS_EXECUTOR_ACTIVE_MODE;
	// Turn timer on
	timEnable(SERVO_CHECKER_TIM_MODULE);
	return;
}
