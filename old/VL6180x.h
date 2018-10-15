#ifndef VL6180x
 #define VL6180x
 
#include "stm32f4xx.h"

#define RANGEFINDER_DEFAULT_ADDR           0x29

#define MASK_8_BITS                        0xFF
#define BIT2_MASK                          0x04  // P.S. BIT 0 exists

#define MEASUREMENT_TIMEOUT                0x1F4 // equals 50 milliseconds

#define LEVEL_LOW_RANGE_INTERRUPT_VALUE    0xC8  // 200 mm 
#define EARLY_CONVEREGENCE_ESTIMATE_VALUE  0x3E8  // 1000

typedef enum
{
	RANGEFINDER_INTERRUPT_EMPTY,
	RANGEFINDER_INTERRUPT_LEVEL_LOW,
	RANGEFINDER_INTERRUPT_LEVEL_HIGH,
	RANGEFINDER_INTERRUPT_OUT_OF_WINDOW,
	RANGEFINDER_INTERRUPT_NEW_SAMPLE_READY
} Rangefinder_Interrupt_Status_Typedef;

enum regAddr
{
  IDENTIFICATION__MODEL_ID              = 0x000,
  IDENTIFICATION__MODEL_REV_MAJOR       = 0x001,
  IDENTIFICATION__MODEL_REV_MINOR       = 0x002,
  IDENTIFICATION__MODULE_REV_MAJOR      = 0x003,
  IDENTIFICATION__MODULE_REV_MINOR      = 0x004,
  IDENTIFICATION__DATE_HI               = 0x006,
  IDENTIFICATION__DATE_LO               = 0x007,
  IDENTIFICATION__TIME                  = 0x008, // 16-bit

  SYSTEM_MODE_GPIO0                    = 0x010,
  SYSTEM_MODE_GPIO1                    = 0x011,
  SYSTEM_HISTORY_CTRL                  = 0x012,
  SYSTEM_INTERRUPT_CONFIG_GPIO         = 0x014,
  SYSTEM_INTERRUPT_CLEAR               = 0x015,
  SYSTEM_FRESH_OUT_OF_RESET            = 0x016,
  SYSTEM_GROUPED_PARAMETER_HOLD        = 0x017,

  SYSRANGE_START                       = 0x018,
  SYSRANGE_THRESH_HIGH                 = 0x019,
  SYSRANGE_THRESH_LOW                  = 0x01A,
  SYSRANGE_INTERMEASUREMENT_PERIOD     = 0x01B,
  SYSRANGE_MAX_CONVERGENCE_TIME        = 0x01C,
  SYSRANGE_CROSSTALK_COMPENSATION_RATE = 0x01E, // 16-bit
  SYSRANGE_CROSSTALK_VALID_HEIGHT      = 0x021,
  SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  = 0x022, // 16-bit
  SYSRANGE_PART_TO_PART_RANGE_OFFSET   = 0x024,
  SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   = 0x025,
  SYSRANGE_RANGE_IGNORE_THRESHOLD      = 0x026, // 16-bit
  SYSRANGE_MAX_AMBIENT_LEVEL_MULT      = 0x02C,
  SYSRANGE_RANGE_CHECK_ENABLES         = 0x02D,
  SYSRANGE_VHV_RECALIBRATE             = 0x02E,
  SYSRANGE_VHV_REPEAT_RATE             = 0x031,

  SYSALS_START                         = 0x038,
  SYSALS_THRESH_HIGH                   = 0x03A,
  SYSALS_THRESH_LOW                    = 0x03C,
  SYSALS_INTERMEASUREMENT_PERIOD       = 0x03E,
  SYSALS_ANALOGUE_GAIN                 = 0x03F,
  SYSALS_INTEGRATION_PERIOD            = 0x040,

  RESULT_RANGE_STATUS                  = 0x04D,
  RESULT_ALS_STATUS                    = 0x04E,
  RESULT_INTERRUPT_STATUS_GPIO         = 0x04F,
  RESULT_ALS_VAL                       = 0x050, // 16-bit
  RESULT_HISTORY_BUFFER_0              = 0x052, // 16-bit
  RESULT_HISTORY_BUFFER_1              = 0x054, // 16-bit
  RESULT_HISTORY_BUFFER_2              = 0x056, // 16-bit
  RESULT_HISTORY_BUFFER_3              = 0x058, // 16-bit
  RESULT_HISTORY_BUFFER_4              = 0x05A, // 16-bit
  RESULT_HISTORY_BUFFER_5              = 0x05C, // 16-bit
  RESULT_HISTORY_BUFFER_6              = 0x05E, // 16-bit
  RESULT_HISTORY_BUFFER_7              = 0x060, // 16-bit
  RESULT_RANGE_VAL                     = 0x062,
  RESULT_RANGE_RAW                     = 0x064,
  RESULT_RANGE_RETURN_RATE             = 0x066, // 16-bit
  RESULT_RANGE_REFERENCE_RATE          = 0x068, // 16-bit
  RESULT_RANGE_RETURN_SIGNAL_COUNT     = 0x06C, // 32-bit
  RESULT_RANGE_REFERENCE_SIGNAL_COUNT  = 0x070, // 32-bit
  RESULT_RANGE_RETURN_AMB_COUNT        = 0x074, // 32-bit
  RESULT_RANGE_REFERENCE_AMB_COUNT     = 0x078, // 32-bit
  RESULT_RANGE_RETURN_CONV_TIME        = 0x07C, // 32-bit
  RESULT_RANGE_REFERENCE_CONV_TIME     = 0x080, // 32-bit

  RANGE_SCALER                         = 0x096, // 16-bit 

  READOUT_AVERAGING_SAMPLE_PERIOD      = 0x10A,
  FIRMWARE_BOOTUP                      = 0x119,
  FIRMWARE_RESULT_SCALER               = 0x120,
  I2C_SLAVE_DEVICE_ADDRESS             = 0x212,
  INTERLEAVED_MODE__ENABLE             = 0x2A3,
};

//--------------------------------------------- High level functions -------------------------------------------//

// Initialization of sensor for optical switch mode (new sample event interrupt without physical interrupt)
ErrorStatus rangeFinderInitContiniousInterruptNewSampleOpticalSwitchMode(uint8_t addr);

// Initialization of sensor for collision avoidance mode (new sample normal operation mode)
ErrorStatus rangeFinderInitContiniousInterruptNewSampleColAvoidMode(uint8_t addr);

// Change address
ErrorStatus rangeFinderChangeAddress(uint8_t addr, uint8_t newAddress);

// Return interrupt status 
ErrorStatus rangeFinderCheckInterruptStatusOfSensor(uint8_t addr, uint8_t* answer, Rangefinder_Interrupt_Status_Typedef referenceStatus);

// Single shot measurement
ErrorStatus rangeFinderSingleShotMeasurement(uint8_t addr, uint8_t* value);

// Start continious measurements
ErrorStatus rangeFinderStartContiniousMeasurements(uint8_t addr);

// Read value after measurement
ErrorStatus rangeFinderReadMeasuredRange(uint8_t addr, uint8_t* value);

// Read device ready status byte
ErrorStatus rangeFinderReadRangeReadyStatus(uint8_t addr, uint8_t* value);

// Read raw value (without crosstalk calibration)
ErrorStatus rangeFinderReadMeasuredRangeRaw(uint8_t addr, uint8_t* value);

// Initialize mandatory registers
static ErrorStatus rangeFinderInitMandatoryRegs(uint8_t addr);

//--------------------------------------------- Low level functions to access registers ------------------------//

 // Write an 8-bit value to register
static ErrorStatus rangeFinderWriteReg(uint16_t reg, uint8_t value, uint8_t addr);
 
 // Write a 16-bit value to register
static ErrorStatus rangeFinderWriteReg16(uint16_t reg, uint16_t value, uint8_t addr);
 
 // Write a 32-bit value to register
static ErrorStatus rangeFinderWriteReg32(uint16_t reg, uint32_t value, uint8_t addr);
 
 // Read a 8-bit value from  register
static ErrorStatus rangeFinderReadReg(uint16_t reg, uint8_t* value, uint8_t addr);
 
 // Read a 16-bit value from  register
static ErrorStatus rangeFinderReadReg16(uint16_t reg, uint16_t* value, uint8_t addr);
 
 // Read a 32-bit value from  register
static ErrorStatus rangeFinderReadReg32(uint16_t reg, uint32_t* value, uint8_t addr);
 
#endif
