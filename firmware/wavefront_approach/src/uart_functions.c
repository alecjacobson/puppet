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
	static uint8_t RxCounter = 0;
	static char RxBuffer[32];
	static char TxBuffer[64];
	static uint8_t package_size;

	int n, module_id, joint_type;
	int rate_r, rate_g, rate_b;

	if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register, otherwise interrupt will be called endless, ++ not forget */
		RxBuffer[RxCounter] = USART_ReceiveData(USARTx);

		if(RxBuffer[RxCounter]=='\n'||RxBuffer[RxCounter]=='\r')
		{
			RxBuffer[RxCounter] = 0;

			switch(RxBuffer[0]){
			case 'b':
				// TODO: not_interrupted = !not_interrupted;
				break;
			case 'l':
				n = sscanf(RxBuffer,"l, %d, %d, %d, %d", &module_id, &rate_r, &rate_g, &rate_b);

				if(module_id!=ID)
				{
					TxBuffer[0] = SET_LED; //led
					TxBuffer[1] = rate_r >> 8;
					TxBuffer[2] = rate_r;
					TxBuffer[3] = rate_g >> 8;
					TxBuffer[4] = rate_g;
					TxBuffer[5] = rate_b >> 8;
					TxBuffer[6] = rate_b;

					send_i2c(I2C1, TxBuffer, module_id, 7);
				}
				else
				{
					set_leds(rate_r, rate_g, rate_b);
				}

				break;
			case 't':
				n = sscanf(RxBuffer,"t,%02x",&joint_type);
				set_type(joint_type);

				package_size = sprintf(TxBuffer, "new type set: 0x%02x\r\n", joint_type);
				send_uart(USARTx, TxBuffer, package_size);
				break;
			case 's':
				//not_interrupted = 0;

				package_size = sprintf(TxBuffer, "my id is: 0x%02x\r\n", ID);
				send_uart(USARTx, TxBuffer, package_size);
				if(TYPE==COLLECTOR)
				{
					package_size = sprintf(TxBuffer, "and COLLECTOR (0x%02x)\r\n", TYPE);
					send_uart(USARTx, TxBuffer, package_size);
				}
				else
				{
					package_size = sprintf(TxBuffer, "and NOT COLLECTOR (0x%02x)\r\n", TYPE);
					send_uart(USARTx, TxBuffer, package_size);
				}
				break;
			case 'h':
				//not_interrupted = 0;

				package_size = sprintf(TxBuffer, "b\t\tbreak\r\n");
				send_uart(USARTx, TxBuffer, package_size);
				package_size = sprintf(TxBuffer, "l,<id>,<r[ms]>,<g[ms]>,<b[ms]>\tleds (id=0 for gc)\r\n");
				send_uart(USARTx, TxBuffer, package_size);
				package_size = sprintf(TxBuffer, "i,<id>\t\tset new id\r\n");
				send_uart(USARTx, TxBuffer, package_size);
				package_size = sprintf(TxBuffer, "s\t\tget summary\r\n");
				send_uart(USARTx, TxBuffer, package_size);
				package_size = sprintf(TxBuffer, "h\t\thelp menu\r\n");
				send_uart(USARTx, TxBuffer, package_size);
				break;
			case 'i':
				//initialized = 0;
				break;
			default:
				package_size = sprintf(TxBuffer, "unknown command: '%c'\r\n", RxBuffer[0]);
				send_uart(USARTx, TxBuffer, package_size);
				break;
			}

			RxCounter = 0;
			RxBuffer[0] = 0;
		}
		else
		{
			RxCounter++;
		}

		// restart buffer
		if(RxCounter == 32)
		{
			RxCounter = 0;
		}
	}
}
