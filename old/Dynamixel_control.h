/*

Code to control multiple Dynamixel AX-12 servo motors over USART
on an STM32F4 chip.

*/

#ifndef Dynamixels_h
#define Dynamixels_h

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#define SERVO_USART_MODULE        DYNAMIXEL_USART_MODULE

#define REC_BUFFER_LEN            32
#define SERVO_MAX_PARAMS          (REC_BUFFER_LEN - 5)

#define REC_WAIT_START_US         75
#define REC_WAIT_PARAMS_US        (SERVO_MAX_PARAMS * 5)
#define REC_WAIT_MAX_RETRIES      20000

#define SERVO_INSTRUCTION_ERROR   (1 << 6)
#define SERVO_OVERLOAD_ERROR      (1 << 5)
#define SERVO_CHECKSUM_ERROR      (1 << 4)
#define SERVO_RANGE_ERROR         (1 << 3)
#define SERVO_OVERHEAT_ERROR      (1 << 2)
#define SERVO_ANGLE_LIMIT_ERROR   (1 << 1)
#define SERVO_INPUT_VOLTAGE_ERROR (1)

#define CW  0x0400
#define CCW 0x0000

#define RETURN_DELAY        0x05
#define BLINK_CONDITIONS    0x11
#define SHUTDOWN_CONDITIONS 0x12
#define TORQUE              0x22
#define CURRENT_LOAD        0x28
#define MOVING_SPEED        0x20
#define CURRENT_SPEED       0x26
#define GOAL_ANGLE          0x1e
#define CURRENT_ANGLE       0x24
#define CW_ANGLE_LIMIT      0x06
#define CCW_ANGLE_LIMIT     0x08

typedef enum ServoCommand
{
    PING = 1,
    READ = 2,
    WRITE = 3
} ServoCommand;



extern uint8_t servoErrorCode;



void initServoUSART (void);


//------------------------------------------------------------------------------
// All functions return true on success and false on failure

// ping a servo, returns true if we get back the expected values
bool pingServo (const uint8_t servoId);

// set the number of microseconds the servo waits before returning a response
// servo factory default value is 500, but we probably want it to be 0
// max value: 510
bool setServoReturnDelayMicros (const uint8_t servoId,
                                const uint16_t micros);

// set the errors that will cause the servo to blink its LED
bool setServoBlinkConditions (const uint8_t servoId,
                              const uint8_t errorFlags);

// set the errors that will cause the servo to shut off torque
bool setServoShutdownConditions (const uint8_t servoId,
                                 const uint8_t errorFlags);


// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque (const uint8_t servoId,
                     const uint16_t torqueValue);

//get maximum allowed torque value
bool getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue);

// speed values go from 1 (incredibly slow) to 1023 CCW
// speed values go from 1024 (incredibly slow) to 2047 CW
//  0 < speedValue < 1023
// direction CCW = 0x0000, CW = 0x0400
// a value of zero will disable velocity control
bool setServoMovingSpeed (const uint8_t servoId,
                       const uint16_t speedValue, const uint16_t direction);

// get maximum allowed speed
bool getServoMovingSpeed (const uint8_t servoId,
                       uint16_t *speedValue);

// get current speed
bool getServoCurrentSpeed (const uint8_t servoId,
                              int16_t *speedValue);

// get current load on the shaft
bool getCurrentLoad (const uint8_t servoId,
                     uint16_t *loadValue);

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle (const uint8_t servoId,
                    const uint16_t angle);

// get current servo angle
bool getServoAngle (const uint8_t servoId,
                    float *angle);

// set CW angle  limit
// if both CW and CCW angle limits set to 0, endless turn is activated
bool setServoCWAngleLimit (const uint8_t servoId,
                     const uint16_t limitValue);

// set CCW angle  limit
// if both CW and CCW angle limits set to 0, endless turn is activated
bool setServoCCWAngleLimit (const uint8_t servoId,
                     const uint16_t limitValue);

//------------------------------------------------------------------------------
// these shouldn't need to be called externally:

typedef struct ServoResponse
{
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t checksum;
} ServoResponse;

void sendServoByte (const uint8_t byte);

void clearServoReceiveBuffer (void);

size_t  getServoBytesAvailable (void);
uint8_t getServoByte (void);

void sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params);
bool getServoResponse (void);

#endif /* Dynamixels_h */
