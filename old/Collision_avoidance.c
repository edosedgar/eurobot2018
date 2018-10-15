#include "Collision_avoidance.h"

extern I2C_Module_With_State_Typedef I2CModule;
extern float robotTargetSpeedCs1[3];
extern RobotStatus Robot;


Range_Finders_Struct_Typedef rangeFinders;


// Levels of measurements for calibration
uint8_t lowLevelValueToCompare[NUMBER_OF_RANGE_FINDERS_FOR_CALIBR] = {0x00, 0x32, 0x00, 0x32, 0x00, 0x3C};
uint8_t highLevelValueToCompare[NUMBER_OF_RANGE_FINDERS_FOR_CALIBR] = {0x3C, 0x78, 0x3C, 0x78, 0x3C, 0x82};
//                                                                     0-60,50-120,0-60,50-120,0-60,60-130

// Coordinate of sensor's vectors (x and y)
float sensorsCoordinateCollAvoid[NUMBER_OF_RANGE_FINDERS_FOR_COLL_AVOID][3] = {{0.0f, 1.0f, -1.0f}, {-MAGIC_VALUE, MAGIC_VALUE, 0.0f}, {-1.0f, 0.0f, 0.0f}, {-MAGIC_VALUE, -MAGIC_VALUE, 0.0f}, {0.0f, -1.0f, 1.0f},
                                                                               {0.0f, -1.0f, -1.0f}, {MAGIC_VALUE, -MAGIC_VALUE, 0.0f}, {1.0f, 0.0f, 0.0f}, {MAGIC_VALUE, MAGIC_VALUE, 0.0f}, {0.0f, 1.0f, 1.0f}};
//--------------------------------------------- High level functions -------------------------------------------//
// Global initialization
ErrorStatus initRangeFindersGlobally(void)
{
	if (initExpanderOutputMode(EXPANDER_RESET_I2C_ADDRESS) != SUCCESS)
	{
		// Error arised
		rangeFinders.outputExpanderErrorFlag = EXPANDER_ERROR;
		// Clear low level error flag
		I2CModule.status = I2C_ACTIVE_MODE;
		// Indicate error
		showError();
		return ERROR;
	}
//	if (initExpanderInterruptMode(EXPANDER_INTERRUPT_I2C_ADDRESS) != SUCCESS)
//	{
//		// Error arised
//		rangeFinders.interruptExpander = EXPANDER_ERROR;
//		// Clear low level error flag
//		I2CModule.status = I2C_ACTIVE_MODE;
//		// Indicate error
//		showError();
//		return ERROR;
//	}
	// Initialize rangefinders
	uint8_t i;
	for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
	{
		if (initRangeFinder(i) != SUCCESS)
		{
			// Some error arised
			rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_INIT;
			// Error was recoreded into high level flag, so clear low level
			I2CModule.status = I2C_ACTIVE_MODE;
			// Indicate error
			showError();
		}
	}
	for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
	{
		if (rangeFinderStartContiniousMeasurements(RANGE_FINDER_INITIAL_ADDR_TO_SETUP + i) != SUCCESS)
		{  
			// Some error arised
			rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_START_MEASUREMENTS;
			// Error was recoreded into high level flag, so clear low level
			I2CModule.status = I2C_ACTIVE_MODE;
			// Indicate error
			showError();
		}
	}

	// Indicate No error
	showNoError();
	return SUCCESS;
}

// Read all values of rangefinders
ErrorStatus readRangesGlobally(void)
{
	uint8_t i;
	// Check timeout for last I2C bus restart
	if (!checkTimeout(I2CModule.timeOfLastI2CResetMillis, I2C_TIMEOUT_AFTER_RESET_TENTH_OF_MS))
	{
		return SUCCESS;
	}
	for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
	{
		if (readRange(i) != SUCCESS)
		{
			return ERROR;
		}
	}
	return SUCCESS;
}

// Read one value from rangefinder
ErrorStatus readRange(uint8_t i)
{
	uint8_t interruptStatus;
	
	// If error occured before, do not acquire measurements
	if (rangeFinders.reinitFlags[i] == RANGE_FINDER_NO_NEED_TO_REINIT)
	{
		// Get status of interrupt of particular rangefinder
		if (rangeFinderCheckInterruptStatusOfSensor(RANGE_FINDER_INITIAL_ADDR_TO_SETUP + i, &interruptStatus, RANGEFINDER_INTERRUPT_NEW_SAMPLE_READY) != SUCCESS)
		{
			rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_OPERATION;
			rangeFinders.reinitFlags[i] = RANGE_FINDER_REINIT_IS_NECESSARY;
			// Error was recoreded into high level flag, so clear low level
			I2CModule.status = I2C_ACTIVE_MODE;
			// Indicate error
			showError();
			return ERROR;
		}
		// Status was received. No errors occured
		rangeFinders.errorFlags[i] = RANGE_FINDER_NO_ERROR;
		
		// If measurement is ready
		if (interruptStatus)
		{
			// Read measurements
			if(rangeFinderReadMeasuredRange(RANGE_FINDER_INITIAL_ADDR_TO_SETUP + i, &rangeFinders.rangeValues[i]) != SUCCESS)
			{
				rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_OPERATION;
				rangeFinders.reinitFlags[i] = RANGE_FINDER_REINIT_IS_NECESSARY;
				// Error was recoreded into high level flag, so clear low level
				I2CModule.status = I2C_ACTIVE_MODE;
				// Indicate error
				showError();
				return ERROR;
			}
		}
		else
		{
			uint8_t status = 0x00;
			if(rangeFinderReadRangeReadyStatus(RANGE_FINDER_INITIAL_ADDR_TO_SETUP + i, &status) != SUCCESS)
			{
				rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_OPERATION;
				rangeFinders.reinitFlags[i] = RANGE_FINDER_REINIT_IS_NECESSARY;
				// Error was recoreded into high level flag, so clear low level
				I2CModule.status = I2C_ACTIVE_MODE;
			}
			
			rangeFinders.reinitFlags[i] = RANGE_FINDER_REINIT_IS_NECESSARY;
			// Indicate error
			showError();
		}
	}
	return SUCCESS;
}

void checkRangeFindersReinitFlags(void)
{
	uint8_t i;
	uint8_t sum = 0x00;
	
	// Check if power is under reset or not
	if (checkTimeout(I2CModule.timeOfLastI2CResetMillis, EXPANDER_POWER_RESET_DELAY)
		&& rangeFinders.powerResetStatusFlag)
	{
		powerTurnOnExpander();
		rangeFinders.powerResetStatusFlag = 0x00;
		return;
	}
	
	// Check timeout after I2C reset
	if (checkTimeout(I2CModule.timeOfLastI2CResetMillis, I2C_TIMEOUT_AFTER_RESET_TENTH_OF_MS))
	{
		// Check if all rangefinders + expander should be reinitialized at the same time
		for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
		{
			sum += rangeFinders.reinitFlags[i];
		}
		sum += rangeFinders.outputExpanderErrorFlag;
		// If all rangefinders + expander should be reinitialized then there is an error with bus
		if (sum == NUMBER_OF_RANGE_FINDERS + 0x01)
		{
			// Reset I2C bus
			I2CReset(&I2CModule);
			
			// Turn off power of expander and raise a flag
			//powerTurnOffExpander();
			//rangeFinders.powerResetStatusFlag = 0x01;
			
			// Clear Reinit flags
			for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
			{
				rangeFinders.reinitFlags[i] = RANGE_FINDER_NO_NEED_TO_REINIT;
			}
			rangeFinders.outputExpanderErrorFlag = EXPANDER_NO_ERROR;
			
			// Indicate no error
			showNoError();
		}
		else
		{	
			// Check if Expander was under reset or not
			rangeFinders.outputExpanderErrorFlag = checkExpander();
			
			// Reinit expander if it is necessary
			if (rangeFinders.outputExpanderErrorFlag == EXPANDER_ERROR)
			{
				if (initExpanderOutputMode(EXPANDER_RESET_I2C_ADDRESS) != SUCCESS)
				{
					// Error arised
					rangeFinders.outputExpanderErrorFlag = EXPANDER_ERROR;
					I2CModule.status = I2C_ACTIVE_MODE;
					// Indicate error
					showError();
				}
				else
				{
					rangeFinders.outputExpanderErrorFlag = EXPANDER_NO_ERROR;
					// Indicate no error
					showNoError();
				}
				return;
				
			}
			// Initialize "stopped" rangefinders
			for(i = 0x00; i < NUMBER_OF_RANGE_FINDERS; i++)
			{
				if (rangeFinders.reinitFlags[i] == RANGE_FINDER_REINIT_IS_NECESSARY)
				{
					// Init rangefinder
					if (initRangeFinder(i) != SUCCESS)
					{
						rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_REINIT;
						I2CModule.status = I2C_ACTIVE_MODE;
						showError();
						return;
					}
					// Turn on measurements
					if (rangeFinderStartContiniousMeasurements(RANGE_FINDER_INITIAL_ADDR_TO_SETUP + i) != SUCCESS)
					{  
						// Some error arised
						rangeFinders.errorFlags[i] = RANGE_FINDER_ERROR_WHILE_START_MEASUREMENTS;
						// Error was recoreded into high level flag, so clear low level
						I2CModule.status = I2C_ACTIVE_MODE;
						showError();
						return;
					}
					rangeFinders.reinitFlags[i] = RANGE_FINDER_NO_NEED_TO_REINIT;
					// Indicate no error
					showNoError();
					return;
				}
			}
		}
	}
	return;
}

// Check collision avoidance
void checkCollisionAvoidance()
{
	uint8_t i;
	float scalarProduct;
	for (i = 0x00; i < NUMBER_OF_RANGE_FINDERS_FOR_COLL_AVOID; i++)
	{
		if (rangeFinders.rangeValues[i] < THRESHOLD_FOR_COLLISION_AVOIDANCE_MM)
		{
			if ((Robot.odometryMovingStatusFlag) || (Robot.forwardKinCalcStatusFlag))
			{
				// Check x
				scalarProduct = sensorsCoordinateCollAvoid[i][0]*robotTargetSpeedCs1[0];
				if (scalarProduct > 0.0f)
				{
					robotTargetSpeedCs1[0] = 0.0f;
				}
				// Check y
				scalarProduct = sensorsCoordinateCollAvoid[i][1]*robotTargetSpeedCs1[1];
				if (scalarProduct > 0.0f)
				{
					robotTargetSpeedCs1[1] = 0.0f;
				}
			}
			else
			{
				// Calculate scalar product and check linear speeds x and y 
				scalarProduct = sensorsCoordinateCollAvoid[i][0]*robotTargetSpeedCs1[0] + sensorsCoordinateCollAvoid[i][1]*robotTargetSpeedCs1[1];
				if (scalarProduct > 0.0f)
				{
					// Substract projections
					robotTargetSpeedCs1[0] -= sensorsCoordinateCollAvoid[i][0] * scalarProduct;
					robotTargetSpeedCs1[1] -= sensorsCoordinateCollAvoid[i][1] * scalarProduct;
				}
			}
			
			// Calculate scalar product and change rotation
			scalarProduct = sensorsCoordinateCollAvoid[i][2]*robotTargetSpeedCs1[2];
			if (scalarProduct > 0.0f)
			{
				// Change rotation
				robotTargetSpeedCs1[2] -= sensorsCoordinateCollAvoid[i][2] * scalarProduct;
			}
		}
	}
	return;
}

// Make postprocessing of data from rangefinders for calibration
void postprocessDataForCalibration(void)
{
	uint8_t i;
	for ( i = 0x00; i < NUMBER_OF_RANGE_FINDERS_FOR_CALIBR; i++)
	{
		if ( (rangeFinders.rangeValues[RANGE_FINDER_NUMBER_OF_LAST_COL_AV_SENSOR + 0x01 + i] > lowLevelValueToCompare[i])
			 && (rangeFinders.rangeValues[RANGE_FINDER_NUMBER_OF_LAST_COL_AV_SENSOR + 0x01 + i] < highLevelValueToCompare[i]))
		{
			rangeFinders.dataForCalibration[i] = 0x01;
		}
		else
		{
			rangeFinders.dataForCalibration[i] = 0x00;
		}
	}
	return;
}

// Reset particular rangefinder
ErrorStatus resetRangeFinder(uint8_t numberOfSensor)
{
	// Set low level voltage on reset pin
	rangeFinders.outputVoltageOfExpander &= ~(1 << numberOfSensor);
	if (setExpanderVoltage(rangeFinders.outputVoltageOfExpander, EXPANDER_RESET_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	delayInTenthOfMs(RANGE_FINDER_RESET_DELAY_TENTH_OF_MS);
	// Set high level voltage on reset pin
	rangeFinders.outputVoltageOfExpander |= (1 << numberOfSensor);
	if (setExpanderVoltage(rangeFinders.outputVoltageOfExpander, EXPANDER_RESET_I2C_ADDRESS) != SUCCESS)
	{
		return ERROR;
	}
	delayInTenthOfMs(RANGE_FINDER_MCU_BOOT_DELAY_TENTH_OF_MS);
	return SUCCESS;
}
// Init particular rangefinder
ErrorStatus initRangeFinder(uint8_t numberOfSensor)
{
	// Reset sensor
	if (resetRangeFinder(numberOfSensor) != SUCCESS)
	{
		// Error of output Expander arised
		rangeFinders.outputExpanderErrorFlag = EXPANDER_ERROR;
		return ERROR;
	}
	// Initialization
	if ( numberOfSensor > RANGE_FINDER_NUMBER_OF_LAST_COL_AV_SENSOR)
	{
		// Initialize rangefinders as optical switches
		if(rangeFinderInitContiniousInterruptNewSampleOpticalSwitchMode(RANGEFINDER_DEFAULT_ADDR)  != SUCCESS)
		{
			return ERROR;
		}
	}
	else
	{
		// Initialize rangefinders for collision avoidance 
		if(rangeFinderInitContiniousInterruptNewSampleColAvoidMode(RANGEFINDER_DEFAULT_ADDR)  != SUCCESS)
		{
			return ERROR;
		}
	}
	// Change address
	uint8_t newAddr = RANGE_FINDER_INITIAL_ADDR_TO_SETUP + numberOfSensor;
	if(rangeFinderChangeAddress(RANGEFINDER_DEFAULT_ADDR, newAddr)  != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

//--------------------------------------------- Middle level functions -----------------------------------------//

// Reset expander
void resetExpander()
{
	// Reset expander (Low logic level)
	gpioPinSetLevel(EXPANDER_RESET_PORT, EXPANDER_RESET_PIN, GPIO_LEVEL_LOW);
	delayInTenthOfMs(EXPANDER_RESET_DELAY_TENTH_OF_MS);
	// Turn on expander (High logic level)
	gpioPinSetLevel(EXPANDER_RESET_PORT, EXPANDER_RESET_PIN, GPIO_LEVEL_HIGH);
	delayInTenthOfMs(EXPANDER_BOOT_DELAY_TENTH_OF_MS);
	return;
}

// I2C Power turn off
void powerTurnOffExpander()
{
	gpioPinSetLevel(I2C_POWER_RESET_PORT, I2C_POWER_RESET_PIN, GPIO_LEVEL_HIGH);
	return;
}

// I2C Power turn on
void powerTurnOnExpander()
{
	gpioPinSetLevel(I2C_POWER_RESET_PORT, I2C_POWER_RESET_PIN, GPIO_LEVEL_LOW);
	return;
}

// Check expander (if it was under reset or not)
Expander_Errors_Typedef checkExpander(void)
{
	uint8_t expanderConfigRegValue;
	// Read value
	if (expanderReadReg(EXPANDER_CONFIG_REG, &expanderConfigRegValue, EXPANDER_RESET_I2C_ADDRESS) != SUCCESS)
	{
		return EXPANDER_CONNECTION_ERROR;
	}
	if (expanderConfigRegValue != EXPANDER_CONFIG_REG_CHECK_VALUE)
	{
		return EXPANDER_ERROR;
	}
	// Everything is OK
	return EXPANDER_NO_ERROR;
}

// Initialize expander in output mode
ErrorStatus initExpanderOutputMode(uint8_t expanderAddr)
{
	resetExpander();
	// Setup config register for right mapping of ports
	if (expanderWriteReg(EXPANDER_CONFIG_REG_DEFAULT, EXPANDER_CONFIG_REG_CHECK_VALUE, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init direction of A bank (0 - output, 1 - input) A - output
	if (expanderWriteReg(EXPANDER_REG_DIR_A, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init direction of B bank (0 - output, 1 - input) B - output
	if (expanderWriteReg(EXPANDER_REG_DIR_B, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Output value of A bank (initial voltages - low)
	if (expanderWriteReg(EXPANDER_REG_VALUE_A, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Output value of B bank (initial voltages - low)
	if (expanderWriteReg(EXPANDER_REG_VALUE_B, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Initialize expander in input (interrupt) mode
ErrorStatus initExpanderInterruptMode(uint8_t expanderAddr)
{
	resetExpander();
	// Setup config register for right mapping of ports
	if (expanderWriteReg(EXPANDER_CONFIG_REG_DEFAULT, 0xA2, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init direction of A bank (0 - output, 1 - input) A - input
	if (expanderWriteReg(EXPANDER_REG_DIR_A, 0xFF, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init direction of B bank (0 - output, 1 - input) B - input
	if (expanderWriteReg(EXPANDER_REG_DIR_B, 0xFF, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Set value to compare for A bank
	if (expanderWriteReg(EXPANDER_REG_COMPARE_VALUE_FOR_INTER_A, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Set value to compare for B bank
	if (expanderWriteReg(EXPANDER_REG_COMPARE_VALUE_FOR_INTER_B, 0x00, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init interrupt mode that describes how the associated pin value 
	// is compared for the interrupt-on-change feature (it is compared against REG_COMPARE_VALUE_FOR_INTER_A)
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_CONTROL_A, 0xff, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Init interrupt mode that describes how the associated pin value 
	// is compared for the interrupt-on-change feature (it is compared against REG_COMPARE_VALUE_FOR_INTER_B)
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_CONTROL_B, 0xff, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Enable interrupt on change (all A pins)
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_ON_A, 0xFF, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Enable interrupt on change (all B pins)
	if (expanderWriteReg(EXPANDER_REG_INTERRUPT_ON_B, 0xFF, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	uint16_t buf;
	// Read interrupt 
	if (expanderReadInterrupt(expanderAddr, &buf) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read interrupt of expander
ErrorStatus expanderReadInterrupt(uint8_t expanderAddr, uint16_t* interruptStatus)
{
	uint8_t interruptStatusA;
	uint8_t interruptStatusB;
	
	// Interrupt captured value for port A
	if (expanderReadReg(EXPANDER_REG_INT_CAP_VAL_A, &interruptStatusA, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Interrupt captured value for port B
	if (expanderReadReg(EXPANDER_REG_INT_CAP_VAL_B, &interruptStatusB, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	*interruptStatus = (interruptStatusB << 8) + interruptStatusA;
	return SUCCESS;
}

// Set voltage on expander's pins
ErrorStatus setExpanderVoltage(uint16_t voltage, uint8_t expanderAddr)
{
	uint8_t voltageA = (uint8_t)(voltage & 0xFF);
	uint8_t voltageB = (uint8_t)(voltage >> 8);
	// Output value of A bank
	if (expanderWriteReg(EXPANDER_REG_VALUE_A, voltageA, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	// Output value of B bank
	if (expanderWriteReg(EXPANDER_REG_VALUE_B, voltageB, expanderAddr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

//--------------------------------------------- Low level functions to access registers ------------------------//

// Write an 8-bit value to expander's register
ErrorStatus expanderWriteReg(uint8_t reg, uint8_t value, uint8_t addr)
{
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;
	
	if (I2CSendBytes(&I2CModule, buf, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read a 8-bit value from  register
ErrorStatus expanderReadReg(uint8_t reg, uint8_t* value, uint8_t addr)
{
	if (I2CSendBytes(&I2CModule, &reg, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (I2CReadBytes(&I2CModule, value, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
