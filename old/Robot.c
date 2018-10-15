#include "Robot.h"

// Struct for current robot status
RobotStatus Robot;

// Struct for odometry movement
OdometryMovementStruct OdometryMovement;

// Values, calculated by using encoder's data
// Speeds (rad/s) of particular wheel
float wheelsSpeed[ROBOT_NUMBER_OF_MOTORS];

// Instantaneous speeds and coordinates of robot in global coordinate system
float robotSpeedCsGlobal[3];
float robotCoordCsGlobal[3];

// Instantaneous speeds and coordinates of robot in robot coordinate system
float robotSpeedCs1[3];
float robotCoordCs1[3];

// Value's that are calculated by STM or Raspberry pi and used for output
// Target speeds and coordinates of robot in robot's coordinate system
float robotTargetSpeedCs1[3];
float robotTargetMotorSpeedCs1[ROBOT_NUMBER_OF_MOTORS];

// Accuracy of movement and acceleration for odometry movement
float const accuracyOfMovement[3] = {MOVEMENT_XY_ACCURACY, MOVEMENT_XY_ACCURACY, MOVEMENT_ANGULAR_ACCURACY};
float const accelerationMax[3] = {ODOMETRY_MOVEMENT_ACCELERATION_X, ODOMETRY_MOVEMENT_ACCELERATION_Y, ODOMETRY_MOVEMENT_ACCELERATION_W};

// Time of last odometry data acquisition (tenth of millisec)
uint32_t timeofLastOdometryDataAcquisition;
// Last time interval (sec)
float lastTimeIntervalSec;

// Normalize input angle to range [0, 2*pi)
static void normalizeAngle(float* angle)
{
	if (*angle >= 2*PI_NUMBER)
	{
		*angle = *angle - 2 * PI_NUMBER;
	}
	else if (*angle < 0)
	{
		*angle = *angle + 2 * PI_NUMBER;
	}
	return;
}

// Maximum value of array
static void maxValue(float *a,uint8_t rows,float *b)
{
	uint8_t i;
	*b = fabs(*a);
	for (i = 0x01; (i <= rows - 0x01); i++)
	{
		if ((*b) < fabs(*(a+i)))
		*b = fabs(*(a + i));
	}
	return;
}

// Time of Robot start
uint32_t timeOfStart;

void turnEverythingOff()
{
	// Global disable of interrupts

	__disable_irq();

	// Turn off dynamixels
	gpioPinSetLevel(SERVO_REBOOT_PORT, SERVO_REBOOT_PIN, GPIO_LEVEL_HIGH);

	// Turn off maxons
	uint8_t i;
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		// Change PWM
		timPwmChangeDutyCycle(motorPwmCh[i].timModule, motorPwmCh[i].channel, 0.0f);
	}
	while (1)
	{

	}
}

//--------------------------------------------- Functions for acquiring odometry and navigation-----------------//

// Calculate robot speed and coordinates in global coordinate system by using speeds in robot's coordinate system
void calcGlobSpeedAndCoord(void)
{
	float alphaRobot = robotCoordCsGlobal[2];
	// Calculation of matrix that transforms speed from robot coordinate system to global one
	float transfMatrix [2][2] = { {cosf(alphaRobot), -sinf(alphaRobot)},
	                              {sinf(alphaRobot), cosf(alphaRobot)}};
	// Transformation
	// Linear speeds
	matrixMultiplyM2M(&transfMatrix[0][0], 2, 2, &robotSpeedCs1[0], 2, 1, &robotSpeedCsGlobal[0]);
	// Rotational speed
	robotSpeedCsGlobal[2] = robotSpeedCs1[2];

	// Coordinates' increment calculation
	robotCoordCsGlobal[0] += robotSpeedCsGlobal[0] * lastTimeIntervalSec;
	robotCoordCsGlobal[1] += robotSpeedCsGlobal[1] * lastTimeIntervalSec;
	robotCoordCsGlobal[2] += robotSpeedCsGlobal[2] * lastTimeIntervalSec;
	// Normalize angle of rotation
	normalizeAngle(&robotCoordCsGlobal[2]);
	return;
}

// Read data from encoders, calculate coordinates and speeds of wheel
void readEnc(void)
{
	// Counter
	uint8_t i;
	// Buffer for encoders' ticks
	int16_t encTicksBuf[ROBOT_NUMBER_OF_MOTORS];

	#ifdef ENCODER_IMITATION
	// Imitation of movement
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		// Actually now it is only motor's speed, but not wheel speed
		wheelsSpeed[i] = robotTargetMotorSpeedCs1[i];
	}
	#else
	// Get data from encoders
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		encTicksBuf[i] = *encCnt[i] - ENCODER_CNT_INITIAL_VALUE;
		*encCnt[i] = ENCODER_CNT_INITIAL_VALUE;
	}

	// Calculate time interval between current and last data acquisition
	lastTimeIntervalSec = (float)(getTimeDifference(timeofLastOdometryDataAcquisition)) / 10000;
	timeofLastOdometryDataAcquisition = getLocalTime();

	// Check for appropriate values that can possibly cause Nan errors
	if (lastTimeIntervalSec == 0.0f)
	{
		return;
	}

	// Calculate speeds
	for (i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		// Actually now it is only motor's speed, but not wheel speed
		wheelsSpeed[i] = (float)(encTicksBuf[i]) * TICKS_TO_RAD_COEF_SHORT / (lastTimeIntervalSec);
		// Now one motor (1st and 4rd) is especial (20.02.2018)
		if ((i == 0) || (i == 3))
		{
			wheelsSpeed[i] = (float)(encTicksBuf[i]) * TICKS_TO_RAD_COEF_LONG / (lastTimeIntervalSec);
		}
	}
	#endif

	// Inverse motors 2 and 3 because of cylindrical transmission (Positive direction is CCW)
	wheelsSpeed[1] = - wheelsSpeed[1];
	wheelsSpeed[2] = - wheelsSpeed[2];

	// Calculate inverse kinematics (wheelsSpeed -> robotSpeedCs1)
	calcInverseKin();

	// Calculate instanteneous coordinates in robot coordinate system
	for (i = 0x00; i < 0x03; i++)
	{
		robotCoordCs1[i] += robotSpeedCs1[i] * lastTimeIntervalSec;
	}
	normalizeAngle(&robotCoordCs1[2]);
	return;
}

//--------------------------------------------- Functions for calculation of kinematics ------------------------//

// Calculate forward kinematics (from speeds in robot's coordinate system to motors' speed)
void calcForwardKin(void)
{
	float vLine[ROBOT_NUMBER_OF_MOTORS], vRot[ROBOT_NUMBER_OF_MOTORS], vSum[ROBOT_NUMBER_OF_MOTORS];
	float maxMotorSpeed;
	float correctionCoef;

	// Note : matrix of forward kinematics already takes into account 2 cylindrical transmissions (wheels 2 and 4)
	// Calculate linear speed
	matrixMultiplyM2M(&MLineSpeed[0][0], ROBOT_NUMBER_OF_MOTORS, 3, &robotTargetSpeedCs1[0], 3, 1, &vLine[0]);

	// Calculate rotational speed
	matrixMultiplyM2M(&MRotSpeed[0][0], ROBOT_NUMBER_OF_MOTORS, 3, &robotTargetSpeedCs1[0], 3, 1, &vRot[0]);

	// Sum speeds
	matrixPlusMinus(&vLine[0], &vRot[0], ROBOT_NUMBER_OF_MOTORS, 1, 1, &vSum[0]);

	// Find max value all motor's speeds
	maxValue(&vSum[0], ROBOT_NUMBER_OF_MOTORS, &maxMotorSpeed);

	// Check if we exceed maximum available motor speed
	if (fabs(maxMotorSpeed) > MAX_ROT_SPEED)
	{
		correctionCoef = fabs(MAX_ROT_SPEED / maxMotorSpeed);
	}
	else
	{
		correctionCoef = 1.0;
	}
	matrixMultiplyS2M(&vSum[0], ROBOT_NUMBER_OF_MOTORS, 1, correctionCoef, &robotTargetMotorSpeedCs1[0]);
	return;
}

// Check saturation of motors and change speeds if it is needed (speeds in robot's coordinate system)
void checkSaturation( float* targetSpeed)
{
	float vLine[ROBOT_NUMBER_OF_MOTORS], vRot[ROBOT_NUMBER_OF_MOTORS], vSum[ROBOT_NUMBER_OF_MOTORS];
	float maxMotorSpeed;
	float correctionCoef;

	// Note : matrix of forward kinematics already takes into account 2 cylindrical transmissions (wheels 2 and 4)
	// Calculate linear speed
	matrixMultiplyM2M(&MLineSpeed[0][0], ROBOT_NUMBER_OF_MOTORS, 3, targetSpeed, 3, 1, vLine);

	// Calculate rotational speed
	matrixMultiplyM2M(&MRotSpeed[0][0], ROBOT_NUMBER_OF_MOTORS, 3, targetSpeed, 3, 1, vRot);

	// Sum speeds
	matrixPlusMinus(vLine, vRot, ROBOT_NUMBER_OF_MOTORS, 1, 1, vSum);

	// Find max value all motor's speeds
	maxValue(vSum, ROBOT_NUMBER_OF_MOTORS, &maxMotorSpeed);

	// Check if we exceed maximum available motor speed
	if (fabs(maxMotorSpeed) > MAX_ROT_SPEED)
	{
		correctionCoef = fabs(MAX_ROT_SPEED / maxMotorSpeed);
	}
	else
	{
		correctionCoef = 1.0;
	}
	matrixMultiplyS2M(targetSpeed, 0x03, 0x01, correctionCoef, targetSpeed);
	return;
}

// Calculate inverse kinematics (from encoder's wheels' speeds to speeds in robot's coordinate system)
void calcInverseKin(void)
{
	// Multiply inverse kinematics matrix by wheels' speeds
	matrixMultiplyM2M(&InverseKinematics[0][0], 3, ROBOT_NUMBER_OF_MOTORS, &wheelsSpeed[0], ROBOT_NUMBER_OF_MOTORS, 1, &robotSpeedCs1[0]);
}

//--------------------------------------------- Functions for motor control ------------------------------------//

// Set speed of particular motor
void setMotorSpeed(uint8_t motorNumber, float speed)
{
	if (motorNumber > ROBOT_NUMBER_OF_MOTORS)
	{
		return;
	}
	float duty;

	// Shift of motor numeration
	motorNumber = motorNumber - 0x01;

	// Set direction
	if (speed >= 0)
	{
		gpioPinSetLevel(motorDir[motorNumber].port, motorDir[motorNumber].number, GPIO_LEVEL_HIGH);
	}
	else
	{
		gpioPinSetLevel(motorDir[motorNumber].port, motorDir[motorNumber].number, GPIO_LEVEL_LOW);
		speed = - speed;
	}

	if (speed >= MAX_ROT_SPEED)
	{
		speed = MAX_ROT_SPEED;
	}
	else if (speed < MIN_ROT_SPEED)
	{
		speed = MIN_ROT_SPEED;
	}
	// Calculate duty cycle
	duty = speed*(float)ESCON_CALIBR_COEF_A + (float)ESCON_CALIBR_COEF_B;

	// Change PWM
	timPwmChangeDutyCycle(motorPwmCh[motorNumber].timModule, motorPwmCh[motorNumber].channel, duty);
	return;
}

// Set speeds for all motors by using target motors' speed array
void setMotorSpeeds(void)
{
	uint8_t i;
	for(i = 0; i < ROBOT_NUMBER_OF_MOTORS; i++)
	{
		setMotorSpeed(i + 0x01, robotTargetMotorSpeedCs1[i]);
	}
	return;
}

//--------------------------------------------- Functions for movement -----------------------------------------//

// Start circular rotation with defined radius, length of arc and absolute linear speed
void startCircularRotation( float radius, float arcLength, float linearSpeedAbs)
{
	if ((arcLength != 0.0f) && (linearSpeedAbs > 0.0f))
	{
		float distance[3] = {0.0f, 0.0f, 0.0f};
		float speedAbs[3] = {0.0f, 0.0f, 0.0f};
		float accelerationAbs[3] = {0.0f, 0.0f, 0.0f};

		// Set maximum possible x acceleration
		accelerationAbs[1] = ODOMETRY_MOVEMENT_ACCELERATION_X;
		// Set x stable speed
		speedAbs[1] = linearSpeedAbs;
		// Set x distance to be passed
		distance[1] = arcLength;

		// Set maximum possible w acceleration
		accelerationAbs[2] = accelerationAbs[1] / radius;
		// Check if angular aceleration exceeds bounds
		if (accelerationAbs[2] > ODOMETRY_MOVEMENT_ACCELERATION_W)
		{
			return;
		}
		// Set w stable speed
		speedAbs[2] = linearSpeedAbs / radius;
		// Set w distance to be passed
		distance[2] = arcLength / radius;

		startMovementRobotCs1(distance, speedAbs, accelerationAbs);
	}
	return;
}

// Start predefined movement to particular distance in robot's coordinate system
// Input accelerations are corrected inside functions in order to syncronize acceleration and decceleration
void startMovementRobotCs1(float* distance, float* speedAbs, float accelerationAbs[3])
{
	// Counter
	uint8_t i;

	// -1 Normalize speeds
	checkSaturation(speedAbs);

	// 0 Normalize accelerations
	syncAccelerations(distance, speedAbs, accelerationAbs);

	// Fill Odometry movement struct with appropriate values
	for (i = 0x00; i < 0x03; i++)
	{
		// 1 Set current coordinate in own system to 0
		robotCoordCs1[i] = 0.0f;

		// 2 Check if distance is non equal to 0 and speed is more than 0
		if ((distance[i] != 0.0f) && (speedAbs[i] > 0.0f))
		{
			if (distance[i] > 0)
			{
				OdometryMovement.stableSpeed[i] = speedAbs[i];
				OdometryMovement.acceleration[i] = accelerationAbs[i];
				OdometryMovement.direction[i] = 1;
			}
			else
			{

				OdometryMovement.stableSpeed[i] = -speedAbs[i];
				OdometryMovement.acceleration[i] = -accelerationAbs[i];
				OdometryMovement.direction[i] = -1;
			}

			// 3 Copy distance to be passed
			OdometryMovement.robotTargetDistanceCs1[i] = distance[i];

			// 4 Optimize direction of robots rotation and angle
			if (i == 0x02)
			{
				// Check if angle exceeds bounds
				if (fabs(distance[i]) > 2 * PI_NUMBER)
				{
					return;
				}

				// Normalize angle
				normalizeAngle(&distance[i]);

				if (distance[i] > PI_NUMBER)
				{
					OdometryMovement.robotTargetDistanceCs1[i] = distance[i] -  2 * PI_NUMBER;
					OdometryMovement.acceleration[i] = -accelerationAbs[i];
					OdometryMovement.stableSpeed[i] = -speedAbs[i];
					OdometryMovement.direction[i] = -1;
				}
				else
				{
					OdometryMovement.robotTargetDistanceCs1[i] = distance[i];
					OdometryMovement.acceleration[i] = accelerationAbs[i];
					OdometryMovement.stableSpeed[i] = speedAbs[i];
					OdometryMovement.direction[i] = 1;
				}
			}

			// 5 Set current speeds
			OdometryMovement.initialSpeed[i] = robotSpeedCs1[i];

			// 6 Calculate speed increment after all direction are defined
			OdometryMovement.speedIncrement[i] = OdometryMovement.acceleration[i] * MOTOR_CONTROL_PERIOD;

			// 7 Calculation of essential parameters of the trajectory
			calculateTrajectParameters(i, accelerationAbs);

			// 8 Remember start time of movement and set first stage (acceleration)
			OdometryMovement.startTime[i] = getLocalTime();
			OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_ACCELERATION;
		}
	}
	// 9 Set flag to make odometry movement
	Robot.odometryMovingStatusFlag = 0x01;
	return;
}

// Inner function for calculation of points of accelleration and decceleration
void calculateTrajectParameters(uint8_t i, float* accelerationAbs)
{
	float deltaCoordAcc = OdometryMovement.direction[i] * (OdometryMovement.stableSpeed[i] * OdometryMovement.stableSpeed[i] - OdometryMovement.initialSpeed[i] * OdometryMovement.initialSpeed[i])
	/ (0x02 * accelerationAbs[i]);

	float deltaCoordDecc = OdometryMovement.direction[i] * (OdometryMovement.stableSpeed[i] * OdometryMovement.stableSpeed[i] - OdometryMovement.finalSpeed[i] * OdometryMovement.finalSpeed[i])
	/ (0x02 * accelerationAbs[i]);

	// Point of trajectory to stop acceleration
	OdometryMovement.stopAccCoordinate[i] = deltaCoordAcc;

	// Point of trajectory to start decceleration
	OdometryMovement.startDeccCoordinate[i] = OdometryMovement.robotTargetDistanceCs1[i] - deltaCoordDecc;

	// Calculate time duration of movement in tenth of milliseconds
	OdometryMovement.durationOfMovement[i] = (uint32_t)((fabs(OdometryMovement.stableSpeed[i] - OdometryMovement.initialSpeed[i]) / accelerationAbs[i] +
	                                         fabs(OdometryMovement.stableSpeed[i] - OdometryMovement.finalSpeed[i]) / accelerationAbs[i] +
	                                         (fabs((OdometryMovement.startDeccCoordinate[i] - OdometryMovement.stopAccCoordinate[i]) / OdometryMovement.stableSpeed[i]))) * 10000);


	// If distance is too short, then correct stable speed
	if (fabs(OdometryMovement.startDeccCoordinate[i]) < fabs(OdometryMovement.stopAccCoordinate[i]))
	{
		// TBD
		OdometryMovement.stableSpeed[i] = OdometryMovement.direction[i] * sqrtf(OdometryMovement.acceleration[i] * OdometryMovement.robotTargetDistanceCs1[i]);
		OdometryMovement.stopAccCoordinate[i] = OdometryMovement.robotTargetDistanceCs1[i] / 2;
		OdometryMovement.startDeccCoordinate[i] = OdometryMovement.stopAccCoordinate[i];
	}
	return;
}

// Inner function for syncronizing accelerations
void syncAccelerations(float* distance, float* speedAbs, float* accelerationAbs)
{
	uint8_t i;
	float accelTimes[3];
	float maxAccelTime = 0;

	// Normalize accelerations
	for (i = 0x00; i < 0x03; i++)
	{
		if ((distance[i] != 0.0f) && (speedAbs[i] > 0.0f))
		{
			// Calculatuion of time duration of acceleration process for each coordinate
			accelTimes[i] = speedAbs[i] / accelerationAbs[i];
			if (accelTimes[i] > maxAccelTime)
			{
				// Find maximum value of accelTimes[i] array to determine the "slowest" coordinate
				maxAccelTime = accelTimes[i];
			}
		}
	}

	// Calculate accelerations for each coordinate in order to obtain equal time of acceleration
	for (i = 0x00; i < 0x03; i++)
	{
		accelerationAbs[i] = speedAbs[i] / maxAccelTime;
	}
	return;
}

// Speed recalculation based on current speed and coordinate in robot's coordinate system
void speedRecalculation(void)
{
	uint8_t i;
	for (i = 0x00; i < 0x03; i++)
	{
		switch(OdometryMovement.odometryMovementStatusFlag[i])
		{
			case ODOMETRY_MOVEMENT_NO_MOVEMENT:
				break;

			case ODOMETRY_MOVEMENT_ACCELERATION:
				robotTargetSpeedCs1[i] = robotTargetSpeedCs1[i] + OdometryMovement.acceleration[i] * lastTimeIntervalSec;
				// Constrain increase in speed if coordinate is still less than stopAccCoord[i]
				if (fabs(robotTargetSpeedCs1[i]) > fabs(OdometryMovement.stableSpeed[i]))
				{
					robotTargetSpeedCs1[i] = OdometryMovement.stableSpeed[i];
				}
				break;

			case ODOMETRY_MOVEMENT_STABLE_SPEED:

				// Keep constant speed
				robotTargetSpeedCs1[i] = OdometryMovement.stableSpeed[i];
				break;

			case ODOMETRY_MOVEMENT_DECCELERATION:
				// Check if speed is less than speed increment
				if (fabs(robotSpeedCs1[i]) < fabs(OdometryMovement.speedIncrement[i]))
				{
					robotTargetSpeedCs1[i] = OdometryMovement.finalSpeed[i];
					OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_NO_MOVEMENT;
				}
				else
				{
					// Decrease speed
					robotTargetSpeedCs1[i] = robotTargetSpeedCs1[i] - OdometryMovement.acceleration[i] * lastTimeIntervalSec;
					// Check if we decreased speed too much
					if (robotTargetSpeedCs1[i] * OdometryMovement.direction[i] < 0.0f)
					{
						robotTargetSpeedCs1[i] = OdometryMovement.finalSpeed[i];
					}
					break;
				}
		}
	}
	return;
}
// Function checks if we reached desired position or not and changes mode of movement
// when we reach transition points
void checkIfPositionIsReached(void)
{
	uint8_t i;
	float robotCoordBuf[3];

	// Copy coordinates from robotCoordCs1[i] to robotCoordBuf[i]
	for (i = 0x00; i < 0x03; i++)
	{
		robotCoordBuf[i] = robotCoordCs1[i];
	}
	// Change representation of angle if it is needed
	if (robotCoordBuf[2] > PI_NUMBER)
	{
		robotCoordBuf[2] = robotCoordBuf[2] - 2 * PI_NUMBER;
	}

	for (i = 0x00; i < 0x03; i++)
	{
		// Check if each particular coordinate is reached and if it is needed to change mode
		checkIfPositionIsReachedCoord(i, robotCoordBuf);
	}

	// Change status flag if we finished odometry movement
	if ((OdometryMovement.odometryMovementStatusFlag[0] == ODOMETRY_MOVEMENT_NO_MOVEMENT) &&
		(OdometryMovement.odometryMovementStatusFlag[1] == ODOMETRY_MOVEMENT_NO_MOVEMENT) &&
		(OdometryMovement.odometryMovementStatusFlag[2] == ODOMETRY_MOVEMENT_NO_MOVEMENT))
	{
		Robot.odometryMovingStatusFlag = 0x00;
	}
	else
	{
		Robot.odometryMovingStatusFlag = 0x01;
	}
	return;
}

// Subfunction for checkIfPositionIsReached() function
void checkIfPositionIsReachedCoord(uint8_t i, float* robotCoordBuf)
{
	if (OdometryMovement.odometryMovementStatusFlag[i] != ODOMETRY_MOVEMENT_NO_MOVEMENT)
		{
			// Check timeout for movement
			if (checkTimeout(OdometryMovement.startTime[i], OdometryMovement.durationOfMovement[i]))
			{
				// Stop motors
				robotTargetSpeedCs1[i] = 0.0;
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_NO_MOVEMENT;
				return;
			}
			// Check if we reached final position or not
			if (fabs(OdometryMovement.robotTargetDistanceCs1[i] - robotCoordBuf[i]) < accuracyOfMovement[i])
			{
				// Stop motors
				robotTargetSpeedCs1[i] = 0.0;
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_NO_MOVEMENT;
				return;
			}

			// Check if mode should be changed
			if ((fabs(robotCoordBuf[i]) < fabs(OdometryMovement.stopAccCoordinate[i])) && (fabs(robotSpeedCs1[i]) < fabs(OdometryMovement.stableSpeed[i])))
			{
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_ACCELERATION;
				return;
			}
			else if (fabs(robotCoordBuf[i]) < fabs(OdometryMovement.startDeccCoordinate[i]))
			{
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_STABLE_SPEED;
				return;
			}
			else
			{
				OdometryMovement.odometryMovementStatusFlag[i] = ODOMETRY_MOVEMENT_DECCELERATION;
				return;
			}
		}
	return;
}
//--------------------------------------------- Other functions ------------------------------------------------//

// Check status
void updateRobotStatus(void)
{
	uint8_t i;
	for ( i = 0x00; i < ROBOT_NUMBER_OF_MOTORS; i++ )
	{
		if (wheelsSpeed[i] > EPS_OF_ROT_SPEED)
		{
			Robot.movingStatusFlag = 0x01;
			return;
		}
	}
	Robot.movingStatusFlag = 0x00;
	return;
}
