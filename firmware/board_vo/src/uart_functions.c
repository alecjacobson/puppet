/*
 * uart_functions.c
 *
 *  Created on: Jan 12, 2012
 *      Author: glausero
 */

/* includes */
#include "stm32f10x.h"
#include "platform_config.h"
#include "stdio.h"
#include "node_functions.h"
#include "spi_functions.h"
#include "i2c_functions.h"
#include "uart_queue.h"

#include "configuration.h"

/* variables */
extern uint8_t not_interrupted;
extern uint8_t in_debug;
extern uint8_t ID;
extern NodeType TYPE;
extern uint16_t *CALIB_VALUES;
extern uint8_t initialized;

static Line current_line = {0,{0}};
extern UartQueue uart_input_queue;
extern UartQueue uart_output_queue;

DMA_InitTypeDef DMA_InitStructure;

//__attribute__ ((section("flash_data_sect"))) __IO uint16_t init_val1;

//__IO uint16_t *init_val1 = (__IO uint16_t *)0x0800EA65;

uint8_t init_val1;

/******************************************************************************/
/*            Functions for a uart to work				                      */
/******************************************************************************/

/**
  * @brief  configuration of all uart related things^^
  * @param  None
  * @retval None
  */
void uart_config()
{
	// RCC
	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(USARTz_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable USARTz Clock */
	RCC_APB2PeriphClockCmd(USARTz_CLK, ENABLE);

	/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	// GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USARTz Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = USARTz_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);

	/* Configure USARTz Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = USARTz_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);

	// NVIC
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTz Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USARTz_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	// DMA

	/* USARTy_Tx_DMA_Channel (triggered by USARTy Tx event) Config */
	DMA_DeInit(USARTz_Tx_DMA_Channel);

	DMA_InitStructure.DMA_PeripheralBaseAddr = USARTz_DR_Base;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	//DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)test;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(USARTz_Tx_DMA_Channel, &DMA_InitStructure);

	// UART
	/*USARTz configuration -------------------------------------------*/
	USART_InitTypeDef USART_InitStructure;

	// TODO: maybe change back to 460800 and 230400

	if(is_joint(TYPE))
	{
		USART_InitStructure.USART_BaudRate = 230400; // weird things happening ... baudrate for terminal is 128000 hm
	}
	else
	{
		USART_InitStructure.USART_BaudRate = 115200; // weird things happening ... baudrate for terminal is 128000 hm
	}

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure USARTz */
	USART_Init(USARTz, &USART_InitStructure);

	/* Enable USARTy DMA TX request */
	USART_DMACmd(USARTz, USART_DMAReq_Tx, ENABLE);

	/* Enable USARTz */
	USART_Cmd(USARTz, ENABLE);

	/* Enable the USARTz Receive Interrupt */
	USART_ITConfig(USARTz, USART_IT_RXNE, ENABLE);

	/* Enable the DMA finished Interrupt */
	DMA_ITConfig(USARTz_Tx_DMA_Channel, DMA_IT_TC, ENABLE);

	/* Enable USARTz DMA TX Channel */
	DMA_Cmd(USARTz_Tx_DMA_Channel, ENABLE);
}

/**
  * @brief  makes dma send an array starting at pointer text
  * @param  None
  * @retval None
  */
void send_uart_dma(char* text, int length)
{
	if(in_debug)
	{
		int length_t;

		int time = TIM_GetCounter(TIM3);

		length_t = snprintf(text+length, UART_QUEUE_LINE_LENGTH, "#t,%d\r\n", time);

		length = length + length_t;
	}

	DMA_Cmd(USARTz_Tx_DMA_Channel, DISABLE);

	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)text;

	DMA_InitStructure.DMA_BufferSize = length;

	DMA_Init(USARTz_Tx_DMA_Channel, &DMA_InitStructure);

	DMA_Cmd(USARTz_Tx_DMA_Channel, ENABLE);
}

/**
  * @brief  send a data package via UART to Computer
  * @param  None
  * @retval None
  */
void send_uart(USART_TypeDef* USARTx, char* TxBuffer, uint8_t package_size)
{
	__disable_irq();

	uint8_t TxCounter = 0;

	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);

	while(TxCounter!=package_size)
	{
		USART_SendData(USARTx, TxBuffer[TxCounter++]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	}

	__enable_irq();
}

/**
  * @brief  receive a data package via UART from Computer and analyze it
  * @param  None
  * @retval None
  */
void receive_uart(USART_TypeDef* USARTx)
{
	if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register, otherwise interrupt will be called endless, ++ not forget */
        char ch = USART_ReceiveData(USARTx);
        if (ch == '\n' || ch == '\r') {
            if (current_line.length) { // ignore empty lines
                uart_queue_push_line(&uart_input_queue, &current_line);
                current_line.length = current_line.text[0] = 0;
            }
        } else if ((ch > ' ') && (ch < 0x80)) { // exclude tabs and special characters (\r, \t, ...)
            current_line.text[current_line.length++] = ch;
            // Saturation
            if (current_line.length >= UART_QUEUE_LINE_LENGTH) {
                current_line.length = UART_QUEUE_LINE_LENGTH;
                current_line.text[UART_QUEUE_LINE_LENGTH-1] = 0;
            }
        }
    }
}
