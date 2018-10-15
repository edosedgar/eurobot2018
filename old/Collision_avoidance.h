#ifndef COLLISION_AVOIDANCE
#define COLLISION_AVOIDANCE

#include "STM32F4_I2C.h"
#include "VL6180x.h"
#include "Interrupts.h"
#include "Robot.h"

//--------------------------------------------- Macros for proximity sensor ------------------------------------//
#define RANGE_FINDER_RESET_DELAY_TENTH_OF_MS         0x03   //  equals 0.3 ms
#define RANGE_FINDER_MCU_BOOT_DELAY_TENTH_OF_MS      0x10   //  equals 1.6 ms
#define RANGE_FINDERS_FIRST_READ_DELAY_TENTH_OF_MS   0x4E20 // equals  2000 ms

#define RANGE_FINDER_INITIAL_ADDR_TO_SETUP           RANGEFINDER_DEFAULT_ADDR + 0x01

#define RANGE_FINDER_NUMBER_OF_LAST_COL_AV_SENSOR    0x09 // Number of last sensor for collision avoidance

#define NUMBER_OF_RANGE_FINDERS                      0x10
#define NUMBER_OF_RANGE_FINDERS_FOR_CALIBR           0x06
#define NUMBER_OF_RANGE_FINDERS_FOR_COLL_AVOID       0x0A

#define THRESHOLD_FOR_COLLISION_AVOIDANCE_MM         0x96 // 150

// 1/sqrt(2)
#define MAGIC_VALUE                                  0.70710678f
//--------------------------------------------- Macros for expander --------------------------------------------//
#define EXPANDER_RESET_DELAY_TENTH_OF_MS             0x02   //  equals 0.2 ms
#define EXPANDER_BOOT_DELAY_TENTH_OF_MS              0x02   //  equals 0.2 ms
#define EXPANDER_POWER_RESET_DELAY                   0x6A4  //  equals 170 ms

#define EXPANDER_INTERRUPT_I2C_ADDRESS               0x20
#define EXPANDER_RESET_I2C_ADDRESS                   0x21

#define EXPANDER_REG_DIR_A                           0x00
#define EXPANDER_REG_DIR_B                           0x10
#define EXPANDER_REG_VALUE_A                         0x0A
#define EXPANDER_REG_VALUE_B                         0x1A
#define EXPANDER_REG_INTERRUPT_ON_A                  0x02
#define EXPANDER_REG_INTERRUPT_ON_B                  0x12
#define EXPANDER_REG_COMPARE_VALUE_FOR_INTER_A       0x03
#define EXPANDER_REG_COMPARE_VALUE_FOR_INTER_B       0x13
#define EXPANDER_REG_INTERRUPT_CONTROL_A             0x04
#define EXPANDER_REG_INTERRUPT_CONTROL_B             0x14
#define EXPANDER_CONFIG_REG_DEFAULT                  0x0A
#define EXPANDER_CONFIG_REG                          0x05

#define EXPANDER_CONFIG_REG_CHECK_VALUE              0xA2

//INTERRUPT CAPTURED VALUE FOR PORT REGISTER
#define EXPANDER_REG_INT_CAP_VAL_A                   0x08
#define EXPANDER_REG_INT_CAP_VAL_B                   0x18 

// Typedef for rangefinder's errors
typedef enum
{
	RANGE_FINDER_NO_ERROR,
	RANGE_FINDER_ERROR_WHILE_INIT,
	RANGE_FINDER_ERROR_WHILE_START_MEASUREMENTS,
	RANGE_FINDER_ERROR_WHILE_REINIT,
	RANGE_FINDER_ERROR_WHILE_OPERATION,
} Range_Finder_Errors_Typedef;

// Typedef for expander's errors
typedef enum
{
	EXPANDER_NO_ERROR,
	EXPANDER_ERROR,
	EXPANDER_CONNECTION_ERROR,
} Expander_Errors_Typedef;

typedef enum
{
	RANGE_FINDER_NO_NEED_TO_REINIT,
	RANGE_FINDER_REINIT_IS_NECESSARY,
} Range_Finders_Reinit_flag_Typedef;

// Typedef for common struc for all rangefinders that contain the whole information
typedef struct
{
	uint8_t rangeValues[NUMBER_OF_RANGE_FINDERS];
	uint8_t dataForCalibration[NUMBER_OF_RANGE_FINDERS_FOR_CALIBR];
	Range_Finder_Errors_Typedef errorFlags[NUMBER_OF_RANGE_FINDERS];
	Range_Finders_Reinit_flag_Typedef reinitFlags[NUMBER_OF_RANGE_FINDERS];
	Expander_Errors_Typedef outputExpanderErrorFlag;
	Expander_Errors_Typedef interruptExpanderErrorFlag;
	uint16_t outputVoltageOfExpander;
	uint8_t powerResetStatusFlag;
} Range_Finders_Struct_Typedef;

//--------------------------------------------- High level functions -------------------------------------------//

// Global initialization
ErrorStatus initRangeFindersGlobally(void);

// Read all values of rangefinders
ErrorStatus readRangesGlobally(void);

// Read one value from rangefinder
ErrorStatus readRange(uint8_t numberOfRangeFinder);

// Check if rangefinders should be reinitialized or not
void checkRangeFindersReinitFlags(void);

// Check collision avoidance
void checkCollisionAvoidance(void);

// Make postprocessing of data from rangefinders for calibration
void postprocessDataForCalibration(void);

// Reset particular rangefinder
ErrorStatus resetRangeFinder(uint8_t numberOfSensor);

// Init particular rangefinder
ErrorStatus initRangeFinder(uint8_t numberOfSensor);

//--------------------------------------------- Middle level functions -----------------------------------------//
// Reset expander
void resetExpander(void);

// Expander Power reset off
void powerTurnOffExpander(void);

// Expander Power reset on
void powerTurnOnExpander(void);

// Check expander (if it was under reset or not)
Expander_Errors_Typedef checkExpander(void);

// Initialize expander in output mode
ErrorStatus initExpanderOutputMode(uint8_t expanderAddr);

// Initialize expander in input (interrupt) mode
ErrorStatus initExpanderInterruptMode(uint8_t expanderAddr);

// Read interrupt of expander
ErrorStatus expanderReadInterrupt(uint8_t expanderAddr, uint16_t* interruptStatus);

// Set voltage on expander's pins
ErrorStatus setExpanderVoltage(uint16_t voltage, uint8_t expanderAddr);

//--------------------------------------------- Low level functions to access registers ------------------------//

// Write an 8-bit value to expander's register
ErrorStatus expanderWriteReg(uint8_t reg, uint8_t value, uint8_t addr);

// Read a 8-bit value from  register
ErrorStatus expanderReadReg(uint8_t reg, uint8_t* value, uint8_t addr);

#endif
