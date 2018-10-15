#include "STM32F4_I2C.h"
#include "STM32F4_RCC.h"
#include "math.h"

// I2C module for rangefinders
I2C_Module_With_State_Typedef I2CModule;
//--------------------------------------------- Initialization and reset -------------------------------------//

// Initialization of I2C
void I2CInit(I2C_Module_With_State_Typedef* I2Cx)
{
	//Turn on clocking I2C
	if (I2Cx->module == I2C1)
	{
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	}
	else if (I2Cx->module == I2C2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}
	else if (I2Cx->module == I2C3)
	{
		RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
	}
	// Software reset
	//I2Cx->module->CR1|=I2C_CR1_SWRST;
	//I2Cx->module->CR1^=I2C_CR1_SWRST;

	// Writing frequency of APB to CR2 register
	I2Cx->module->CR2 &= ~I2C_CR2_FREQ;
	uint32_t buf = (uint32_t)(GLOBAL_CORE_FREQUENCY / rccGetApb1Prescaler());
	I2Cx->module->CR2 |= (uint32_t)(GLOBAL_CORE_FREQUENCY / rccGetApb1Prescaler());

	//Event and buffer interrupt enable
	//I2Cx->CR2 |= I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN;

	//Error interrupt enable
	I2Cx->module->CR2 |= I2C_CR2_ITERREN;

	// Standard Mode, ~100 kHz
	I2Cx->module->CCR |= I2C_CCR_FS;
	//I2Cx->CCR |= I2C_CCR_DUTY;
	//I2Cx->CCR |= (uint32_t) ceilf(GLOBAL_CORE_FREQUENCY * 1000 / rccGetApb1Prescaler() / 25 / 300);
	I2Cx->module->CCR |= (uint32_t) ceilf(GLOBAL_CORE_FREQUENCY * 1000 / rccGetApb1Prescaler()/ 3 / 300);

	//Speed of signal's rising
	I2Cx->module->TRISE = 15;

	return;
}

// Enable of I2C
void I2CEnable(I2C_TypeDef* I2Cx)
{
	// Enable I2C module
	I2Cx->CR1 |= I2C_CR1_PE;
	return;
}

// Disable I2C
void I2CDisable(I2C_TypeDef* I2Cx)
{
	// Disable I2C module
	I2Cx->CR1 &= ~I2C_CR1_PE;
}

// Reset I2C bus
void I2CReset(I2C_Module_With_State_Typedef* I2Cx)
{
	// 1.0 Disable I2C
	I2CDisable(I2Cx->module);

	// 2.0 Configure the SCL and SDA as general purpose Output open drain (HIGH level)

	// Initialization for SDA and SCL pins as open drain high
	gpioInitPin(I2C_MODULE_SCL_PIN_PORT, I2C_MODULE_SCL_PIN_NUMBER, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	gpioPinSetLevel(I2C_MODULE_SCL_PIN_PORT, I2C_MODULE_SCL_PIN_NUMBER, GPIO_LEVEL_HIGH);
	gpioInitPin(I2C_MODULE_SDA_PIN_PORT, I2C_MODULE_SDA_PIN_NUMBER, GPIO_MODE_OUT, GPIO_OUTPUT_MODE_OD, GPIO_PUPD_NOPULL);
	gpioPinSetLevel(I2C_MODULE_SDA_PIN_PORT, I2C_MODULE_SDA_PIN_NUMBER, GPIO_LEVEL_HIGH);

	// 3.0 Check SCL and SDA High level
	delayInTenthOfMs(0x03);

	// 4.0 Configure the SDA low level
	gpioPinSetLevel(I2C_MODULE_SDA_PIN_PORT, I2C_MODULE_SDA_PIN_NUMBER, GPIO_LEVEL_LOW);

	// 5.0 Check SDA Low level
	delayInTenthOfMs(0x03);

	// 6.0 Configure the SCL low level
	gpioPinSetLevel(I2C_MODULE_SCL_PIN_PORT, I2C_MODULE_SCL_PIN_NUMBER, GPIO_LEVEL_LOW);

	// 7.0 Check SCL Low level
	delayInTenthOfMs(0x03);

	// 8.0 Configure the SCL high level
	gpioPinSetLevel(I2C_MODULE_SCL_PIN_PORT, I2C_MODULE_SCL_PIN_NUMBER, GPIO_LEVEL_HIGH);

	// 9.0 Check SCL high level
	delayInTenthOfMs(0x03);

	// 10.0 Configure the SDA high level
	gpioPinSetLevel(I2C_MODULE_SDA_PIN_PORT, I2C_MODULE_SDA_PIN_NUMBER, GPIO_LEVEL_HIGH);

	// 11.0 Check SDA high level
	delayInTenthOfMs(0x03);

	// 12.0 Configure SCL and SDA alternative function open drain
	gpioInitPinAf(I2C_MODULE_SCL_PIN_PORT, I2C_MODULE_SCL_PIN_NUMBER, I2C_MODULE_PIN_AF);
	gpioInitPinAf(I2C_MODULE_SDA_PIN_PORT, I2C_MODULE_SDA_PIN_NUMBER, I2C_MODULE_PIN_AF);

	// 13.0 Set SWRST and then clear it
	I2Cx->module->CR1|=I2C_CR1_SWRST;
	I2Cx->module->CR1^=I2C_CR1_SWRST;

	// Enable
	I2CEnable(I2Cx->module);

	// Init
	I2CInit(I2Cx);

	// Remeber time of last reset
	I2Cx->timeOfLastI2CResetMillis = getLocalTime();

	// Return active status
	I2Cx->status = I2C_ACTIVE_MODE;

	return;
}

// Check I2C bus
void I2CCheckBus(I2C_Module_With_State_Typedef* I2Cx)
{
//	if (I2Cx->status == I2C_ACKNOWLEDGE_ERROR)
//	{
//		return;
//	}
	if (I2Cx->status != I2C_ACTIVE_MODE)
	{
		// Reset I2C bus
		I2CReset(I2Cx);
	}
	return;
}
//--------------------------------------------- Inner functions ----------------------------------------------//

// Generate stop conditions
static void I2CStop(I2C_TypeDef* I2Cx)
{
	I2Cx->CR1|=I2C_CR1_STOP;
	return;
}

// Set acknowledgement bit
static void I2CSetAck(I2C_TypeDef* I2Cx)
{
	I2Cx->CR1 |= I2C_CR1_ACK;
	return;
}

// Clear acknowledgement bit
static void I2CUnSetAck(I2C_TypeDef* I2Cx)
{
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	return;
}

// Clear "adress was sent" flag
static void I2CClearAddrFlag(I2C_TypeDef* I2Cx)
{
	(void) I2Cx->SR2;
	return;
}

// Set POS flag
static void I2CSetPOS(I2C_TypeDef* I2Cx)
{
	I2Cx->CR1 |= I2C_CR1_POS;
	return;
}

// Clear POS flag
static void I2CUnSetPOS(I2C_TypeDef* I2Cx)
{
	I2Cx->CR1 &= ~I2C_CR1_POS;
	return;
}

// Read data register DR
void I2CReadDR(I2C_TypeDef* I2Cx, uint8_t *byte)
{
	*byte=I2Cx->DR;
	return;
}

//--------------------------------------------- Inner functions that require status --------------------------//

// Generate start conditions
static I2C_Status_Typedef I2CStart(I2C_Module_With_State_Typedef* I2Cx)
{
	uint32_t startTime = getLocalTime();

	// Generate start condition
	I2Cx->module->CR1 |= I2C_CR1_START;

	// Check START status flag and sending address
	while (!((I2Cx->module->SR1) & I2C_SR1_SB))
	{
		if (I2Cx->status != I2C_ACTIVE_MODE)
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			return I2Cx->status;
		}
		if (checkTimeout(startTime, I2C_TIMEOUT_VALUE_TENTH_OF_MS))
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			I2Cx->status = I2C_TIMEOUT_ERROR;
			return I2C_TIMEOUT_ERROR;
		}
	}

	//Clear SB flag (+ followed writing in DR)
	(void) I2Cx->module->SR1;
	return I2C_ACTIVE_MODE;
}

// Send adress byte with read or write mode
static I2C_Status_Typedef I2CSendAddr(I2C_Module_With_State_Typedef* I2Cx, uint8_t Addr, I2C_MODE Mode)
{
	uint32_t startTime = getLocalTime();

	//Send adress + mode
	I2Cx->module->DR = I2C_ADDRESS(Addr,Mode);

	//Waiting for sending adress
	while (!(I2Cx->module->SR1 & I2C_SR1_ADDR))
	{
		if (I2Cx->status != I2C_ACTIVE_MODE)
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			return I2Cx->status;
		}
		if (checkTimeout(startTime, I2C_TIMEOUT_VALUE_TENTH_OF_MS))
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			I2Cx->status = I2C_TIMEOUT_ERROR;
			return I2C_TIMEOUT_ERROR;
		}
	}
	return I2C_ACTIVE_MODE;
}

// Wait for one byte to be received
static I2C_Status_Typedef I2CWaitForByte(I2C_Module_With_State_Typedef* I2Cx)
{
	uint32_t startTime = getLocalTime();

	//Waiting for RXNE flag to be set
	while (!(I2Cx->module->SR1 & I2C_SR1_RXNE))
	{
		if (I2Cx->status != I2C_ACTIVE_MODE)
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			return I2Cx->status;
		}
		if (checkTimeout(startTime, I2C_TIMEOUT_VALUE_TENTH_OF_MS))
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			I2Cx->status = I2C_TIMEOUT_ERROR;
			return I2C_TIMEOUT_ERROR;
		}
	}
	return I2C_ACTIVE_MODE;
}

// Wait for byte transfer finished event
static I2C_Status_Typedef I2CWaitForBTF(I2C_Module_With_State_Typedef* I2Cx)
{
	uint32_t startTime = getLocalTime();

	//Waiting for BTF flag to be set
	while (!(I2Cx->module->SR1 & I2C_SR1_BTF))
	{
		if (I2Cx->status != I2C_ACTIVE_MODE)
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			return I2Cx->status;
		}
		if (checkTimeout(startTime, I2C_TIMEOUT_VALUE_TENTH_OF_MS))
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			I2Cx->status = I2C_TIMEOUT_ERROR;
			return I2C_TIMEOUT_ERROR;
		}
	}
	return I2C_ACTIVE_MODE;
}

// Wait for stop flag to be cleared
static I2C_Status_Typedef I2CWaitForStopToBeCleared(I2C_Module_With_State_Typedef* I2Cx)
{
	uint32_t startTime = getLocalTime();

	// Check stop flag
	while (I2Cx->module->CR1 & I2C_CR1_STOP)
	{
		if (I2Cx->status != I2C_ACTIVE_MODE)
		{
			return I2Cx->status;
		}
		if (checkTimeout(startTime, I2C_TIMEOUT_VALUE_TENTH_OF_MS))
		{
			I2Cx->status = I2C_TIMEOUT_ERROR;
			return I2C_TIMEOUT_ERROR;
		}
	}
	return I2C_ACTIVE_MODE;
}

// Wait for busy line
static I2C_Status_Typedef I2CWaitBusyLine(I2C_Module_With_State_Typedef* I2Cx)
{
	uint32_t startTime = getLocalTime();
	//Waiting for Bus to be free
	while (I2Cx->module->SR2 & I2C_SR2_BUSY)
	{
		if (I2Cx->status != I2C_ACTIVE_MODE)
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			return I2Cx->status;
		}
		if (checkTimeout(startTime, I2C_TIMEOUT_VALUE_TENTH_OF_MS))
		{
			// Generate stop condition
			I2CStop(I2CModule.module);
			I2CWaitForStopToBeCleared(&I2CModule);
			I2Cx->status = I2C_TIMEOUT_ERROR;
			return I2C_TIMEOUT_ERROR;
		}
	}
	return I2C_ACTIVE_MODE;
}

//--------------------------------------------- High level functions -----------------------------------------//

// Send predefined number of bytes
ErrorStatus I2CSendBytes(I2C_Module_With_State_Typedef* I2Cx, uint8_t* bytesArray, uint32_t numberOfBytes, uint8_t SlaveAddr)
{
	uint8_t* bytesPtr = bytesArray;
	if(I2CWaitBusyLine(I2Cx) != I2C_ACTIVE_MODE)
	{
		return ERROR;
	}
	if(I2CStart(I2Cx) != I2C_ACTIVE_MODE)
	{
		return ERROR;
	}
	if(I2CSendAddr(I2Cx, SlaveAddr,I2C_MODE_WRITE)!= I2C_ACTIVE_MODE)
	{
		return ERROR;
	}
	I2CClearAddrFlag(I2Cx->module);
	while (numberOfBytes)
	{
		// Send byte
		I2Cx->module->DR = *bytesPtr;

		//Waiting for byte to be sent
		if (I2CWaitForBTF(I2Cx) != I2C_ACTIVE_MODE)
		{
			return ERROR;
		}

		//Clear BTF flag
		(void) I2Cx->module->SR1;
		bytesPtr++;
		numberOfBytes--;
	}
	// Generate stop event
	I2CStop(I2Cx->module);
	if(I2CWaitForStopToBeCleared(I2Cx) != I2C_ACTIVE_MODE)
	{
		return ERROR;
	}
	return SUCCESS;
}


// Read predefined number of bytes
ErrorStatus I2CReadBytes(I2C_Module_With_State_Typedef* I2Cx, uint8_t* readBuffer, uint32_t numberOfBytesToBeRead, uint8_t SlaveAddr)
{
	uint8_t* buffer = readBuffer;
	if(I2CWaitBusyLine(I2Cx)!= I2C_ACTIVE_MODE)
	{
		return ERROR;
	}
	if (numberOfBytesToBeRead == 0x01)
	{
		if(I2CStart(I2Cx) != I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		if(I2CSendAddr(I2Cx, SlaveAddr,I2C_MODE_READ)!= I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		I2CUnSetAck(I2Cx->module);
		//__disable_irq();
		I2CClearAddrFlag(I2Cx->module);
		I2CStop(I2Cx->module);
		//__enable_irq();
		if(I2CWaitForByte(I2Cx)!= I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		I2CReadDR(I2Cx->module, buffer);
		if(I2CWaitForStopToBeCleared(I2Cx)!= I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		I2CSetAck(I2Cx->module);
		return SUCCESS;
	}
	else if (numberOfBytesToBeRead == 0x02)
	{
		I2CSetAck(I2Cx->module);
		I2CSetPOS(I2Cx->module);
		if(I2CStart(I2Cx) != SUCCESS)
		{
			return ERROR;
		}
		if(I2CSendAddr(I2Cx, SlaveAddr,I2C_MODE_READ)!= I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		//__disable_irq();
		I2CClearAddrFlag(I2Cx->module);
		I2CUnSetAck(I2Cx->module);
		//__enable_irq();
		if(I2CWaitForBTF(I2Cx)!= I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		__disable_irq();
		I2CStop(I2Cx->module);
		I2CReadDR(I2Cx->module, buffer);
		__enable_irq();
		buffer++;
		I2CReadDR(I2Cx->module, buffer);
		if(I2CWaitForStopToBeCleared(I2Cx)!= I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		I2CUnSetPOS(I2Cx->module);
		I2CSetAck(I2Cx->module);
	}
	else
	{
		if(I2CStart(I2Cx) != I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		if(I2CSendAddr(I2Cx, SlaveAddr,I2C_MODE_READ)!= I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		I2CClearAddrFlag(I2Cx->module);
		while (numberOfBytesToBeRead)
		{
			if (numberOfBytesToBeRead != 0x03)
			{
				if(I2CWaitForBTF(I2Cx)!= I2C_ACTIVE_MODE)
				{
					return ERROR;
				}
				I2CReadDR(I2Cx->module, buffer);
				buffer++;
				numberOfBytesToBeRead--;
			}
			if (numberOfBytesToBeRead == 0x03)
			{
				if(I2CWaitForBTF(I2Cx)!= I2C_ACTIVE_MODE)
				{
					return ERROR;
				}
				I2CUnSetAck(I2Cx->module);
				//__disable_irq();
				I2CReadDR(I2Cx->module, buffer);
				buffer++;
				I2CStop(I2Cx->module);
				I2CReadDR(I2Cx->module, buffer);
				//__enable_irq();
				buffer++;
				if(I2CWaitForByte(I2Cx)!= I2C_ACTIVE_MODE)
				{
					return ERROR;
				}
				I2CReadDR(I2Cx->module,buffer);
				numberOfBytesToBeRead = 0;
			}
		}
		if(I2CWaitForStopToBeCleared(I2Cx)!= I2C_ACTIVE_MODE)
		{
			return ERROR;
		}
		I2CSetAck(I2Cx->module);
	}
	return SUCCESS;
}


// Search I2C device on Bus
void I2CSearch(I2C_Module_With_State_Typedef* I2Cx)
{
	uint8_t address = 0x7F;
	uint32_t startTime = getLocalTime();
	while (address!=0)
	{
		startTime = getLocalTime();
		I2CSetAck(I2Cx->module);
		I2CWaitBusyLine(I2Cx);
		I2CStart(I2Cx);
		//Send adress + mode Read
		I2Cx->module->DR = I2C_ADDRESS(address,I2C_MODE_WRITE);
		//Waiting for sending adress
		while ((!(I2Cx->module->SR1 & I2C_SR1_ADDR)) || (!checkTimeout(startTime, 1)))
		{

		}
		I2CStop(I2Cx->module);
		I2CWaitForStopToBeCleared(I2Cx);
		if (checkTimeout(startTime, 1))
		{
		}
		address--;

		// Clear AF failure flag
		I2Cx->module->SR1&=~I2C_SR1_AF;
		// Clear address flag
		(void) I2Cx->module->SR2;
	}

}
