#include "STM32F4_UART.h"
#include "STM32F4_GPIO.h"
#include "STM32F4_RCC.h"

// Initialize USART
void usartInit(USART_TypeDef * USARTx, USART_Init_Typedef* USART_Init)
{
        // Current frequency
        double frequency;
        SystemCoreClockUpdate();
        frequency = SystemCoreClock;

        // Consider AHB prescaler
        frequency = frequency / rccGetAhbPrescaler();

        // Consider APBx prescalers
        if ((USARTx == USART1) || (USARTx == USART6))
        {
                // For modules of APB2 bus
                frequency = frequency / rccGetApb2Prescaler();
        }
        else
        {
                // For modules of APB1 bus
                frequency = frequency / rccGetApb1Prescaler();
        }

        // Enable clocking for USART module
        usartInitClock(USARTx);

        // Set stop bits
        USARTx->CR2 |= USART_Init->USART_Stop_Bits << 12;

        // Set parity
        USARTx->CR1 |= USART_Init->USART_Parity << 9;

        // Enable Half duplex mode if we need
        USARTx->CR3 |= USART_Init->USART_Half_Duplex_Mode << 3;

        // Set baudrate
        int divMantissa;
        int divFraction;
        divMantissa = (int)(frequency / (16 * USART_Init->USART_Baudrate));
        divFraction = (int)(((frequency / (16 * USART_Init->USART_Baudrate)) - divMantissa) * 16 + 0.5);
        if (divFraction == 16)
        {
                divMantissa++;
                divFraction = 0;
        }
        USARTx->BRR = divFraction + (divMantissa << 4);

        //Enable receiver, tranceiver and receive interrupt
        USARTx->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
}

// Initialize clock for USART module
void usartInitClock(USART_TypeDef * USARTx)
{
        if (USARTx == USART1)
        {
                // Enable clock signal for  USART1
                RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
                return;
        }
        else if (USARTx == USART2)
        {
                //Enable clock signal for  USART2
                RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
                return;
        }
        else if (USARTx == USART3)
        {
                //Enable clock signal for  USART3
                RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
                return;
        }
        else if (USARTx == UART4)
        {
                //Enable clock signal for  UART4
                RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
                return;
        }
        else if (USARTx == UART5)
        {
                //Enable clock signal for  UART5
                RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
                return;
        }
        else if (USARTx == USART6)
        {
                //Enable clock signal for  USART6
                RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
                return;
        }
        return;
}

// Initialize gpio for USART module (if USART is in half duplex mode only TX pin will be initialized)
void usartInitGpio(USART_TypeDef * USARTx, USART_Half_Duplex_Mode_Typedef USART_Half_Duplex_Mode)
{
        if (USARTx == USART1)
        {
                // There are two possibilities for TX (B6 and A9) and for RX (B7 and A10)
                // Uncomment necessary  strings

                // Turn on GPIOB and enable Alternative functions on B6 (TX) and B7 (RX)
                gpioInitPinAf(GPIOB, GPIO_Pin_6, GPIO_AF_USART1);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOB, GPIO_Pin_7, GPIO_AF_USART1);

                // Turn on GPIOA and enable Alternative functions on A9 (TX) and A10 (RX)
                /*
                gpioInitPinAf(GPIOA, GPIO_Pin_9, GPIO_AF_USART1);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOA, GPIO_Pin_10, GPIO_AF_USART1);
                */
                return;
        }
        else if (USARTx == USART2)
        {
                // There are two possibilities for TX (A2 and D5) and for RX (A3 and D6)
                // Uncomment necessary  strings

                // Turn on GPIOA and enable Alternative functions on A2 (TX) and A3 (RX)
                gpioInitPinAf(GPIOA, GPIO_Pin_2, GPIO_AF_USART2);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOA, GPIO_Pin_3, GPIO_AF_USART2);

                // Turn on GPIOD and enable Alternative functions on D5 (TX) and D6 (RX)
                /*
                gpioInitPinAf(GPIOD, GPIO_Pin_5, GPIO_AF_USART2);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOD, GPIO_Pin_6, GPIO_AF_USART2);
                */
                return;
        }
        else if (USARTx == USART3)
        {
                // There are three possibilities for TX (B10, C10 and D8) and for RX (B11, C11 and D9)
                // Uncomment necessary  strings

                // Turn on GPIOB and enable Alternative functions on B10 (TX) and B11 (RX)
                gpioInitPinAf(GPIOB, GPIO_Pin_10, GPIO_AF_USART3);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOB, GPIO_Pin_11, GPIO_AF_USART3);

                // Turn on GPIOC and enable Alternative functions on C10 (TX) and C11 (RX)
                /*
                gpioInitPinAf(GPIOC, GPIO_Pin_10, GPIO_AF_USART3);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOC, GPIO_Pin_11, GPIO_AF_USART3);
                */

                // Turn on GPIOD and enable Alternative functions on D8 (TX) and D9 (RX)
                /*
                gpioInitPinAf(GPIOD, GPIO_Pin_8, GPIO_AF_USART3);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOD, GPIO_Pin_9, GPIO_AF_USART3);
                */
                return;
        }
        else if (USARTx == UART4)
        {
                // There are two possibilities for TX (A0 and C10) and for RX (A1 and C11)
                // Uncomment necessary  strings

                // Turn on GPIOA and enable Alternative functions on A0 (TX) and A1 (RX)
                /*
                gpioInitPinAf(GPIOA, GPIO_Pin_0, GPIO_AF_UART4);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOA, GPIO_Pin_1, GPIO_AF_UART4);
                */

                // Turn on GPIOC and enable Alternative functions on C10 (TX) and C11 (RX)
                gpioInitPinAf(GPIOC, GPIO_Pin_10, GPIO_AF_UART4);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOC, GPIO_Pin_11, GPIO_AF_UART4);
                return;
        }
        else if (USARTx == UART5)
        {
                // Turn on GPIOC and GPIOD and enable Alternative functions on C12 (TX) and D2 (RX)
                gpioInitPinAf(GPIOC, GPIO_Pin_12, GPIO_AF_UART5);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOD, GPIO_Pin_2, GPIO_AF_UART5);
                return;
        }
        else if (USARTx == USART6)
        {
                // Turn on GPIOA and enable Alternative functions on C6 (TX) and C7 (RX)
                gpioInitPinAf(GPIOC, GPIO_Pin_6, GPIO_AF_USART6);
                if (!(USART_Half_Duplex_Mode))
                gpioInitPinAf(GPIOC, GPIO_Pin_7, GPIO_AF_USART6);
                return;
        }
}

// Enable USARTx
void usartEnable(USART_TypeDef * USARTx)
{
        USARTx->CR1 |= USART_CR1_UE;
        return;
}

// Deinitialize UART
void usartDeinit(USART_TypeDef * USARTx)
{
        USARTx->BRR = 0;
        USARTx->CR1 = 0;
        USARTx->CR2 = 0;
        USARTx->CR3 = 0;
        USARTx->GTPR = 0;
}
// Disable USARTx
void usartDisable(USART_TypeDef * USARTx)
{
        USARTx->CR1 &= ~USART_CR1_UE;
}

// Get default structure for initialization
USART_Init_Typedef usartGetInitStruct()
{
        USART_Init_Typedef settings;
        settings.USART_Baudrate = 9600;
        settings.USART_Parity = USART_PARITY_NONE;
        settings.USART_Stop_Bits = USART_STOP_BITS_1;
        settings.USART_Half_Duplex_Mode = USART_HALF_DUPLEX_MODE_DISABLE;
        return settings;
}

// Read one char from USART
ErrorStatus usartGetC(USART_TypeDef * USARTx, char* c)
{
        int ticks = NumberOfTicks;
        // Wait for receiver's flag
        while(!(USARTx->SR & USART_SR_RXNE))
                        {
                                ticks--;
                                if (ticks == 0)
                                {
                                        return ERROR;
                                }
                        }
        *c = USARTx->DR;
        // Clear flag
        USARTx->SR&=~USART_SR_RXNE;
        return SUCCESS;
}
// Write one char to USART
ErrorStatus usartPutC(USART_TypeDef * USARTx, unsigned char c)
{
        int ticks = NumberOfTicks;
        // Clear "transmission complete" flag
        USARTx->SR &= ~USART_SR_TC;
        USARTx->DR = c;
        // Wait for transmitter's flag
                while (!(USARTx->SR & USART_SR_TC) )
                {
                        ticks--;
                        if (ticks == 0)
                        {
                                return ERROR;
                        }
                }
        return SUCCESS;
}

// Write one char to USART without echo (usefull when TX and RX are connected)
ErrorStatus usartPutCharNoEcho(USART_TypeDef * USARTx, unsigned char c)
{
        int ticks = NumberOfTicks;
        USARTx->DR = c;
        USARTx->CR1 &= ~ USART_CR1_RE;
        // Wait for transmitter's flag
                while (!(USARTx->SR & USART_SR_TC) )
                {
                        ticks--;
                        if (ticks == 0)
                        {
                                return ERROR;
                        }
                }
        USARTx->CR1 |= USART_CR1_RE;
        return SUCCESS;
}

// Write string to the USART
ErrorStatus usartPutStr(USART_TypeDef * USARTx, const char *str)
{
        ErrorStatus state;
        while(*str)
        {
                state = usartPutC(USARTx, *str++);
                if (state == ERROR)
                {
                        return ERROR;
                }
        }
        return SUCCESS;
}
// Send Int
ErrorStatus usartPutInt(USART_TypeDef * USARTx, const uint16_t value)
{
        ErrorStatus state;
        //first part
        char byte1 = (value) & (0xff);
        //second part
        char byte2 = (value) >> 8;

        state = usartPutC(USARTx, byte1);
        state = usartPutC(USARTx, byte2);
        return state;
}
// Print number in ASCII
ErrorStatus usartPutNumberInASCII(USART_TypeDef * USARTx, uint16_t inputValue)
{
        ErrorStatus state;
        // A temp array to hold results of conversion
        char value[10];

        // Loop index
        int i = 0;
        do
        {
                // Convert integer to character
                value[i++] = (char)(inputValue % 10) + '0';
                inputValue /= 10;
        } while(inputValue);

        // Send data
        while(i)
        {
                state = usartPutC(USARTx, value[--i]);
        }
        return state;
}
