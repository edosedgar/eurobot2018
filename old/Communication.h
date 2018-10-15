
#ifndef STM32F4_COMMUNICATION
#define STM32F4_COMMUNICATION

#include "stm32f4xx.h"
#include "Board.h"
#include "Commands.h"

#ifndef __packed
   #define __packed    __attribute__ ((__packed__))
 #endif

// Buffer settings
#define COM_REC_BUFFER_LEN     200

// Protocol settings
#define COM_SYNC_BYTE          0xFA
#define COM_ROBOT_ADR_BYTE     0xAF
#define COM_RASPB_ADR_BYTE     0xFA

// Header + adress + length + command + 1 parameter + checksum 
#define COM_MIN_PACKAGE_LEN    5
// Maximum number of parameters = 30
#define COM_MAX_PARAM_SIZE     0x1E

// Struct for  command
typedef struct 
{
	uint8_t command;
	uint8_t params[COM_MAX_PARAM_SIZE];
	uint8_t numberOfreceivedParams;
	uint8_t status;
} Command_Struct;

// Typedef for errors in getPackage() functions 
typedef enum 
{
	SMALL_LENGTH,
	WRONG_CHECKSUM,
	SUCCESFULL_PACKAGE,
	WAIT_MODE
} Typedef_Protocol_Error;

// Get number of available bytes in buffer
static uint32_t getBytesAvailable(const uint8_t* receiveBufferStart, const uint8_t* receiveBufferEnd);

// Increment pointer of buffer
static uint8_t* incrementPtr(const uint8_t* ptr, const uint32_t deltaPos);

// Check if there is a package in buffer
Typedef_Protocol_Error getPackage(void);

// Send answer in specified form
void sendAnswer(uint8_t command, uint8_t* params, uint8_t numberOfParams);

#endif
