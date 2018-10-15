 #include "VL6180x.h"
 #include "STM32F4_I2C.h"
 #include "math.h"
 #include "Interrupts.h"
 
extern I2C_Module_With_State_Typedef I2CModule;

//--------------------------------------------- High level functions -------------------------------------------//
// Initialization sensors for optical switch mode (new sample event interrupt without physical interrupt)
ErrorStatus rangeFinderInitContiniousInterruptNewSampleOpticalSwitchMode(uint8_t addr)
{
	// Check if system is under reset or not
 	uint8_t buf;
	if (rangeFinderReadReg(SYSTEM_FRESH_OUT_OF_RESET, &buf, addr) != SUCCESS)
	{
	  return ERROR;
	}
	// If system is not under reset
	if (buf == 0x01)
	{
		// Mandatory private registers
		if (rangeFinderInitMandatoryRegs(addr) != SUCCESS)
		{
			return ERROR;
		}

		// Recommended settings
		// The light and dark gain
		if (rangeFinderWriteReg(SYSALS_ANALOGUE_GAIN, 0x46, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Sets the # of range measurements after which auto calibtration of system is performed
		if (rangeFinderWriteReg(SYSRANGE_VHV_REPEAT_RATE, 0xFF, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Performs  a single temperature calibration of  the ranging sensor
		if(rangeFinderWriteReg(SYSRANGE_VHV_RECALIBRATE, 0x01, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Optional: public registers
		// Sets default ranging inter measurement period to 20 ms
		if (rangeFinderWriteReg(SYSRANGE_INTERMEASUREMENT_PERIOD, 0x01, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Sets max converge time (13 ms)
		if (rangeFinderWriteReg(SYSRANGE_MAX_CONVERGENCE_TIME, 0x0D, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Turn  SNR check
		if (rangeFinderWriteReg(SYSRANGE_RANGE_CHECK_ENABLES, 0x10, addr) != SUCCESS)
		{
			return ERROR;
		}
//		// Write early convergence estimate value
//		if (rangeFinderWriteReg16(SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, EARLY_CONVEREGENCE_ESTIMATE_VALUE, addr) != SUCCESS)
//		{
//			return ERROR;
//		}
		// Indicate that data will be updated (Flag set over I2C to indicate that data is being updated)
		if (rangeFinderWriteReg(SYSTEM_GROUPED_PARAMETER_HOLD, 0x01, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Configure interrupt on New Sample event mode
		if (rangeFinderWriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Indicate that data was updated
		if (rangeFinderWriteReg(SYSTEM_GROUPED_PARAMETER_HOLD, 0x00, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Max SNR 0.5 % 
		if (rangeFinderWriteReg(SYSRANGE_MAX_AMBIENT_LEVEL_MULT, 0x08, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Finish setup
		if (rangeFinderWriteReg(SYSTEM_FRESH_OUT_OF_RESET, 0x00, addr) != SUCCESS)
		{
			return ERROR;
		}		
		return SUCCESS;
  }
  else
  {
    // Sensor has already been initialized (but we use reset button, so it is undesired situation)
	  return ERROR;
  }
	//return SUCCESS;
}

// Initialization sensor for collision avoidance mode
ErrorStatus rangeFinderInitContiniousInterruptNewSampleColAvoidMode(uint8_t addr)
{
	// Check if system is under reset or not
 	uint8_t buf;
	if (rangeFinderReadReg(SYSTEM_FRESH_OUT_OF_RESET, &buf, addr) != SUCCESS)
	{
	  return ERROR;
	}
	// If system is not under reset
	if (buf == 0x01)
	{
		// Mandatory private registers
		if (rangeFinderInitMandatoryRegs(addr) != SUCCESS)
		{
			return ERROR;
		}

		// Recommended settings
		// The light and dark gain
		if (rangeFinderWriteReg(SYSALS_ANALOGUE_GAIN, 0x46, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Sets the # of range measurements after which auto calibtration of system is performed
		if (rangeFinderWriteReg(SYSRANGE_VHV_REPEAT_RATE, 0xFF, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Performs  a single temperature calibration of  the ranging sensor
		if(rangeFinderWriteReg(SYSRANGE_VHV_RECALIBRATE, 0x01, addr) != SUCCESS)
		{
			return ERROR;
		}
		
		// Optional: public registers
		// Sets default ranging inter measurement period to 20 ms
		if (rangeFinderWriteReg(SYSRANGE_INTERMEASUREMENT_PERIOD, 0x01, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Sets max converge time (13 ms)
		if (rangeFinderWriteReg(SYSRANGE_MAX_CONVERGENCE_TIME, 0x0D, addr) != SUCCESS)
		{
			return ERROR;
		}	
		// Indicate that data will be updated (Flag set over I2C to indicate that data is being updated)
		if (rangeFinderWriteReg(SYSTEM_GROUPED_PARAMETER_HOLD, 0x01, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Turn on SNR check
		if (rangeFinderWriteReg(SYSRANGE_RANGE_CHECK_ENABLES, 0x10, addr) != SUCCESS)
		{
			return ERROR;
		}
//		// Write early convergence estimate value
//		if (rangeFinderWriteReg16(SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, EARLY_CONVEREGENCE_ESTIMATE_VALUE, addr) != SUCCESS)
//		{
//			return ERROR;
//		}
		// Configure interrupt on New Sample event mode
		if (rangeFinderWriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Indicate that data was updated
		if (rangeFinderWriteReg(SYSTEM_GROUPED_PARAMETER_HOLD, 0x00, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Max SNR 5 % 
		if (rangeFinderWriteReg(SYSRANGE_MAX_AMBIENT_LEVEL_MULT, 0x50, addr) != SUCCESS)
		{
			return ERROR;
		}
		// Finish setup
		if (rangeFinderWriteReg(SYSTEM_FRESH_OUT_OF_RESET, 0x00, addr) != SUCCESS)
		{
			return ERROR;
		}
		return SUCCESS;
  }
  else
  {
    // Sensor has already been initialized (but we use reset button, so it is undesired situation)
	  return ERROR;
  }
	//return SUCCESS;
}

// Change address
ErrorStatus rangeFinderChangeAddress(uint8_t addr, uint8_t newAddress)
{
	if (rangeFinderWriteReg(I2C_SLAVE_DEVICE_ADDRESS, newAddress, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Return interrupt status Status
ErrorStatus rangeFinderCheckInterruptStatusOfSensor(uint8_t addr, uint8_t* answer, Rangefinder_Interrupt_Status_Typedef referenceStatus)
{
	uint8_t buf;
	if(rangeFinderReadReg(RESULT_INTERRUPT_STATUS_GPIO, &buf, addr)!= SUCCESS)
	{
		return ERROR;
	}
	// Separate first 3 bits
	buf = buf & 0x07;
	if (buf == referenceStatus)
	{
		*answer = 0x01;
	}
	else
	{
		*answer = 0x00;
	}
	// If interrupt was then clear it
	if (*answer == 0x01)
	{
		// Clear interrupt
		if(rangeFinderWriteReg(SYSTEM_INTERRUPT_CLEAR, 0x07, addr) != SUCCESS)
		{
			return ERROR;
		}
	}
	return SUCCESS;
}

// Single shot measurement
ErrorStatus rangeFinderSingleShotMeasurement(uint8_t addr, uint8_t* value)
{
	uint32_t startTime = getLocalTime();
	//uint8_t status;

	// Start a range measurements
	if(rangeFinderWriteReg(SYSRANGE_START, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	uint8_t buf = 0x00;
	// Wait for range measurement to be completed
	while(!(buf & BIT2_MASK))
	{
		if(rangeFinderReadReg(RESULT_INTERRUPT_STATUS_GPIO, &buf, addr)!= SUCCESS || (checkTimeout(startTime, MEASUREMENT_TIMEOUT)))
		{
			return ERROR;
		}
	}
	// Read value of range
	if(rangeFinderReadMeasuredRange(addr,value) != SUCCESS)
	{
		return ERROR;
	};
	return SUCCESS;
}



// Start continious measurements
ErrorStatus rangeFinderStartContiniousMeasurements(uint8_t addr)
{
	// Start a continious range measurements
	if(rangeFinderWriteReg(SYSRANGE_START, 0x03, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read value after measurement
ErrorStatus rangeFinderReadMeasuredRange(uint8_t addr, uint8_t* value)
{
	// Read value of range
 	if(rangeFinderReadReg(RESULT_RANGE_VAL, value, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Read device ready status byte
ErrorStatus rangeFinderReadRangeReadyStatus(uint8_t addr, uint8_t* value)
{
	// Read value of range device ready status byte
 	if(rangeFinderReadReg(RESULT_RANGE_STATUS, value, addr) != SUCCESS)
	{
		return ERROR;
	}
	// Apply mask
	//*value &= 0x01;
	return SUCCESS;
}

// Read raw value (without crosstalk calibration)
ErrorStatus rangeFinderReadMeasuredRangeRaw(uint8_t addr, uint8_t* value)
{
	// Read value of range
	if(rangeFinderReadReg(RESULT_RANGE_RAW, value, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

// Initialize mandatory registers
static ErrorStatus rangeFinderInitMandatoryRegs(uint8_t addr)
{
	// Mandatory private registers
	if (rangeFinderWriteReg(0x207, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x208, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x096, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x097, 0xFD, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E3, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E4, 0x04, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E5, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E6, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0E7, 0x03, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0F5, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0D9, 0x05, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0DB, 0xCE, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0DC, 0x03, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0DD, 0xF8, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x09F, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0A3, 0x3C, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0B7, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0BB, 0x3C, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0B2, 0x09, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0CA, 0x09, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x198, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x1B0, 0x17, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x1AD, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x0FF, 0x05, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x100, 0x05, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x199, 0x05, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x1A6, 0x1B, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x1AC, 0x3E, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (rangeFinderWriteReg(0x030, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}

//--------------------------------------------- Low level functions to access registers ------------------------//

 // Write an 8-bit value to register
static ErrorStatus rangeFinderWriteReg(uint16_t reg, uint8_t value, uint8_t addr)
{
	uint8_t buf[3];
	buf[0] = (reg >> 8) & MASK_8_BITS;
	buf[1] = reg & MASK_8_BITS;
	buf[2] = value;
	
	if (I2CSendBytes(&I2CModule, buf, 0x03, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
 
// // Write a 16-bit value to register
static ErrorStatus rangeFinderWriteReg16(uint16_t reg, uint16_t value, uint8_t addr)
{
	uint8_t buf[4];
	buf[0] = (reg >> 8) & MASK_8_BITS;
	buf[1] = reg & MASK_8_BITS;
	buf[2] = (value >> 8) & MASK_8_BITS;
	buf[3] = value & MASK_8_BITS;
	
	if (I2CSendBytes(&I2CModule, buf, 0x04, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
 
// // Write a 32-bit value to register
//static ErrorStatus rangeFinderWriteReg32(uint16_t reg, uint32_t value, uint8_t addr)
//{
//	uint8_t buf[6];
//	buf[0] = (reg >> 8) & MASK_8_BITS;
//	buf[1] = reg & MASK_8_BITS;
//	buf[2] = (value >> 24) & MASK_8_BITS;
//	buf[3] = (value >> 16) & MASK_8_BITS;
//	buf[4] = (value >> 8) & MASK_8_BITS;
//	buf[5] = value & MASK_8_BITS;
//	
//	if (I2CSendBytes(&I2CModule, buf, 0x06, addr) != SUCCESS)
//	{
//		return ERROR;
//	}
//	return SUCCESS;
//}
 
 // Read a 8-bit value from  register
static ErrorStatus rangeFinderReadReg(uint16_t reg, uint8_t* value, uint8_t addr)
{
	uint8_t bufReg [2];
	bufReg[0] = (reg >> 8) & MASK_8_BITS;
	bufReg[1] = reg & MASK_8_BITS;
	if (I2CSendBytes(&I2CModule, bufReg, 0x00, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (I2CSendBytes(&I2CModule, bufReg, 0x02, addr) != SUCCESS)
	{
		return ERROR;
	}
	if (I2CReadBytes(&I2CModule, value, 0x01, addr) != SUCCESS)
	{
		return ERROR;
	}
	return SUCCESS;
}
 
// // Read a 16-bit value from  register
//static ErrorStatus rangeFinderReadReg16(uint16_t reg, uint16_t* value, uint8_t addr)
//{
//	uint8_t bufReg [2];
//	bufReg[0] = (reg >> 8) & MASK_8_BITS;
//	bufReg[1] = reg & MASK_8_BITS;
//	
//	uint8_t bufValue [2];
//	
//	if (I2CSendBytes(&I2CModule, bufReg, 0x02, addr) != SUCCESS)
//	{
//		return ERROR;
//	}
//	
//	if (I2CReadBytes(&I2CModule, bufValue, 0x02, addr) != SUCCESS)
//	{
//		return ERROR;
//	}
//	
//	*value = (bufValue[0] << 8) + bufReg[1];
//	return SUCCESS;
//}
// 
// // Read a 32-bit value from  register
//static ErrorStatus rangeFinderReadReg32(uint16_t reg, uint32_t* value, uint8_t addr)
//{
//	uint8_t bufReg [2];
//	bufReg[0] = (reg >> 8) & MASK_8_BITS;
//	bufReg[1] = reg & MASK_8_BITS;
//	
//	uint8_t bufValue [4];
//	
//	if (I2CSendBytes(&I2CModule, bufReg, 0x02, addr) != SUCCESS)
//	{
//		return ERROR;
//	}
//	
//	if (I2CReadBytes(&I2CModule, bufValue, 0x04, addr) != SUCCESS)
//	{
//		return ERROR;
//	}
//	
//	*value = (bufValue[0] << 24) + (bufValue[1] << 16) + (bufValue[2] << 8) + bufValue[3];
//	return SUCCESS;
//}
