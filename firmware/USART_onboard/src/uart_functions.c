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
#include "i2c_functions.h"

/* variables */
extern uint8_t not_interrupted;
extern uint8_t ID;
extern NodeType TYPE;
extern uint8_t initialized;

static Line current_line = {0,{0}};

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
	RCC_APB1PeriphClockCmd(USARTz_CLK, ENABLE);

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

	// UART
	/*USARTz configuration -------------------------------------------*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 256000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure USARTz */
	USART_Init(USARTz, &USART_InitStructure);

	/* Enable the USARTz Receive Interrupt */
	USART_ITConfig(USARTz, USART_IT_RXNE, ENABLE);

	/* Enable USARTz */
	USART_Cmd(USARTz, ENABLE);
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
        if (ch == '\n') {
            if (current_line.length) { // ignore empty lines
                uart_queue_push_line(&uart_input_queue, &current_line);
                curent_line.length = current_line.text[0] = 0;
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

