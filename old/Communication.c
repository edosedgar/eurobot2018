#include "Communication.h"
// Internal FIFO buffer
uint8_t comReceiveBuffer[COM_REC_BUFFER_LEN];
uint8_t* comReceiveBufferStart = comReceiveBuffer;
uint8_t* comReceiveBufferEnd = comReceiveBuffer;
uint8_t comBufLength = COM_REC_BUFFER_LEN;

// Structure for input command
Command_Struct inputCommand;

static uint32_t getBytesAvailable(const uint8_t* receiveBufferStart,const uint8_t* receiveBufferEnd)
{
        uint8_t* start = (uint8_t*) receiveBufferStart;
        uint8_t* end = (uint8_t*) receiveBufferEnd;

        if (end >= start)
                return (end - start);
        else
                return (comBufLength - (start - end));
}

// Check if there is a package in buffer
Typedef_Protocol_Error getPackage()
{
        uint8_t i;
        uint8_t* ptr = comReceiveBufferStart;
        uint8_t* tempPtr;
        uint8_t* packagelengthPtr;
        uint8_t checksum = (uint8_t)(COM_SYNC_BYTE + COM_ROBOT_ADR_BYTE);
        // Check if minimum number of bytes is available
        while (getBytesAvailable(ptr, comReceiveBufferEnd) >= COM_MIN_PACKAGE_LEN)
        {
                // Check sync bit and adress bit
                if ((*ptr == COM_SYNC_BYTE) && (*(incrementPtr(ptr, 0x01)) == COM_ROBOT_ADR_BYTE))
                {
                        packagelengthPtr = incrementPtr(ptr, 0x02);
                        if (*packagelengthPtr < COM_MIN_PACKAGE_LEN)
                        {
                                // Too small length
                                packagelengthPtr = incrementPtr(ptr, 0x01);
                                comReceiveBufferStart = packagelengthPtr;
                                return SMALL_LENGTH;
                        }
                        if (*packagelengthPtr <= getBytesAvailable(ptr, comReceiveBufferEnd))
                        {
                                // grab length of package
                                checksum += *packagelengthPtr;
                                // Increment pointer
                                tempPtr = incrementPtr(packagelengthPtr, 0x01);
                                // grab command
                                inputCommand.command = *tempPtr;
                                checksum += *tempPtr;
                                // Increment pointer
                                tempPtr = incrementPtr(tempPtr, 0x01);
                                // grab parameters if any
                                inputCommand.numberOfreceivedParams = 0;
                                for (i = 0x00; i < *packagelengthPtr - 0x05; i++)
                                {
                                        inputCommand.params[i] = *tempPtr;
                                        checksum += *tempPtr;
                                        inputCommand.numberOfreceivedParams++;
                                        // Increment pointer
                                        tempPtr = incrementPtr(tempPtr, 0x01);
                                }
                                // Check checksum value
                                if (*tempPtr == checksum)
                                {
                                        // Checksum is OK
                                        // Command status - need to be executed (0x01)
                                        inputCommand.status = 0x01;
                                        // Increment pointer
                                        tempPtr = incrementPtr(tempPtr, 0x01);
                                        comReceiveBufferStart = tempPtr;
                                        return SUCCESFULL_PACKAGE;
                                }
                                else
                                {
                                        // Wrong checksum
                                        tempPtr = incrementPtr(tempPtr, 0x01);
                                        // Command status - does not need to be executed (0x00)
                                        inputCommand.status = 0x00;
                                        comReceiveBufferStart = tempPtr;
                                        return WRONG_CHECKSUM;
                                }

                        }
                        else
                        {
                                // Not all bytes have been received
                                return WAIT_MODE;
                        }
                }
                else
                {
                        ptr = incrementPtr(ptr, 0x01);
                }
        }
        comReceiveBufferStart = ptr;
        return WAIT_MODE;
}

// Increment pointer of buffer
static uint8_t* incrementPtr(const uint8_t* ptr, const uint32_t deltaPos)
{
        uint32_t i;
        uint8_t* tempPtr = (uint8_t*) ptr;
        for ( i = 0; i < deltaPos; i++)
        {
                tempPtr++;
                if (tempPtr >= comReceiveBuffer + comBufLength)
                {
                        tempPtr = (uint8_t*) comReceiveBuffer;
                }
        }
        return tempPtr;
}

// Send answer in specified form
void sendAnswer(uint8_t command, uint8_t* params, uint8_t numberOfParams)
{
        uint8_t i;
        // Length of package (Sync bit + adress + length bit + command + checksum + parameters)
        uint8_t length = 0x05 + numberOfParams;
        uint8_t checksum = (uint8_t)(COM_SYNC_BYTE + COM_RASPB_ADR_BYTE) + length + command;
        usartPutC(COM_USART_MODULE, COM_SYNC_BYTE);
        usartPutC(COM_USART_MODULE, COM_RASPB_ADR_BYTE);
        usartPutC(COM_USART_MODULE, length);
        usartPutC(COM_USART_MODULE, command);
        for (i = 0; i < numberOfParams; i++ )
        {
                usartPutC(COM_USART_MODULE, params[i]);
                checksum += params[i];
        }
        usartPutC(COM_USART_MODULE, checksum);
        return;
}

// Interrupt handler for receiving data from Raspberry Pi (COM_USART_MODULE = USART1)
__attribute__((weak)) void USART1_IRQHandler ()
{
        if (READ_BIT(COM_USART_MODULE->SR, USART_SR_RXNE))
        {
                CLEAR_BIT(COM_USART_MODULE->SR, USART_SR_RXNE);
                const uint8_t byte = COM_USART_MODULE->DR;
                *comReceiveBufferEnd = byte;
                comReceiveBufferEnd = incrementPtr(comReceiveBufferEnd, 0x01);
                return;
        }
}
