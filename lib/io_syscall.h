#ifndef _H_IO_SYSCALL_
#define _H_IO_SYSCALL_

#include "stm32f407xx.h"

USART_TypeDef *chardev;

int iosysc_setdev(USART_TypeDef *cd);

#endif
