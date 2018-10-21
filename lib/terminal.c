#include "peripheral.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f407xx.h"
#include "terminal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "command_list.h"

static int dma_transm_compl;

void uart1dma_config(char *ch_buf)
{
        /* Enable clocking on USART and DMA */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

        /* UART configuration */
        LL_USART_SetTransferDirection(TERM_USART, TERM_USART_TRANSFER_DIRECTION);
        LL_USART_SetParity(TERM_USART, TERM_USART_PARITY);
        LL_USART_SetDataWidth(TERM_USART, TERM_USART_DATA_WIDTH);
        LL_USART_SetStopBitsLength(TERM_USART, TERM_USART_STOPBITS);
        LL_USART_SetHWFlowCtrl(TERM_USART, TERM_USART_HAWDWARE_FLOAT_CNTRL);
        LL_USART_SetBaudRate(TERM_USART, SystemCoreClock/2, TERM_USART_OVERSAMPL, TERM_USART_BAUDRATE);

        LL_USART_Enable(TERM_USART);
        LL_USART_EnableDMAReq_RX(TERM_USART);
        LL_USART_EnableIT_IDLE(TERM_USART);

        NVIC_SetPriority(TERM_USART_IRQN, TERM_USART_IRQN_PRIORITY);
        NVIC_EnableIRQ(TERM_USART_IRQN);

        /* DMA configuration */
        LL_DMA_SetChannelSelection(TERM_DMA, TERM_DMA_STREAM, TERM_DMA_CHANNEL);
        LL_DMA_ConfigAddresses(TERM_DMA, TERM_DMA_STREAM, TERM_DMA_SRC_ADDR, (uint32_t)ch_buf, TERM_DMA_DIRECTION);
        LL_DMA_SetDataLength(TERM_DMA, TERM_DMA_STREAM, TERM_DMA_BUFFER_SIZE);
        LL_DMA_SetMemoryIncMode(TERM_DMA, TERM_DMA_STREAM, TERM_DMA_MEM_INC_MODE);

        LL_DMA_EnableStream(TERM_DMA, TERM_DMA_STREAM);
        LL_DMA_EnableIT_TC(TERM_DMA, TERM_DMA_STREAM);
        
        /* Enable global DMA stream interrupts */
        NVIC_SetPriority(TERM_DMA_STREAM_IRQN, TERM_DMA_STREAM_IRQN_PRIORITY);
        NVIC_EnableIRQ(TERM_DMA_STREAM_IRQN);

        return;
}

void terminal_manager(void *arg)
{
        terminal_task_t term_t;
        char term_bufer[CHANNEL_BUF_SIZE]; 
        int command_code = 0;

        uart1dma_config(term_bufer);
        dma_transm_compl = 0;

        while (1) {
                command_code = term_request(&term_t);
                if (!IS_COMMAND_VALID(command_code))
                        continue;
                commands_handlers[command_code](term_t.com_args);
                term_response(&term_t);
        }
        return;
}

int term_request(terminal_task_t *term_t)
{
        (void) term_t;

        if(dma_transm_compl)
        {
                dma_transm_compl = 0;
        }
        return 0;
}

void term_response(terminal_task_t *term_t)
{
        (void) term_t;
        return;
}

void USART1_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        LL_USART_ClearFlag_IDLE(TERM_USART);
        LL_DMA_DisableStream(TERM_DMA, TERM_DMA_STREAM);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void DMA2_Stream2_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (LL_DMA_IsActiveFlag_TC2(TERM_DMA)) 
        {
                LL_DMA_ClearFlag_TC2(TERM_DMA);
                LL_DMA_ClearFlag_HT2(TERM_DMA);
                LL_DMA_EnableStream(TERM_DMA, TERM_DMA_STREAM);
                dma_transm_compl = 1;
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}