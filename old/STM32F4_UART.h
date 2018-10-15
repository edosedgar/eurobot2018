#ifndef STM32F4_UART
#define STM32F4_UART

#include "stm32f4xx.h"

#define NumberOfTicks 2000000000

// Parity typedef
typedef enum 
{
	USART_PARITY_NONE = 0,
	USART_PARITY_ENABLE
}USART_Parity_Typedef;

// Half duplex mode typedef
typedef enum 
{
	USART_HALF_DUPLEX_MODE_DISABLE = 0,
	USART_HALF_DUPLEX_MODE_ENABLE
}USART_Half_Duplex_Mode_Typedef;

// Stop bits typedef
typedef enum 
{
	USART_STOP_BITS_1 = 0,
	USART_STOP_BITS_2 = 2
}USART_Stop_Bits_Typedef;

// Struct for USART module initialization
typedef struct
{
	uint32_t USART_Baudrate;
	USART_Half_Duplex_Mode_Typedef USART_Half_Duplex_Mode;
	USART_Parity_Typedef USART_Parity;
	USART_Stop_Bits_Typedef USART_Stop_Bits;
}USART_Init_Typedef;

// Initialize UART0 (module itself)
void usartInit(USART_TypeDef * USARTx, USART_Init_Typedef* USART_Init);

// Initialize clock for USART module
void usartInitClock(USART_TypeDef * USARTx);

// Initialize gpio for USART module (if USART is in half duplex mode only TX pin will be initialized)
void usartInitGpio(USART_TypeDef * USARTx, USART_Half_Duplex_Mode_Typedef USART_Half_Duplex_Mode);

// Enable USARTx
void usartEnable(USART_TypeDef * USARTx);

// Deinitialize UART
void usartDeinit(USART_TypeDef * USARTx);

// Disable USART
void usartDisable(USART_TypeDef * USARTx);

// Get default structure for initialization
USART_Init_Typedef usartGetInitStruct(void);

// Read one char from USART
ErrorStatus usartGetC(USART_TypeDef * USARTx, char* c);

// Write one char to USART
ErrorStatus usartPutC(USART_TypeDef * USARTx, unsigned char c);

// Write one char to USART without echo (usefull when TX and RX are connected)
ErrorStatus usartPutCharNoEcho(USART_TypeDef * USARTx, unsigned char c);
	
// Write string to the USART
ErrorStatus usartPutStr(USART_TypeDef * USARTx, const char *str);

// Send Int
ErrorStatus usartPutInt(USART_TypeDef * USARTx, const uint16_t value);

// Print number in ASCII
ErrorStatus usartPutNumberInASCII(USART_TypeDef * USARTx, uint16_t inputValue);

#endif
