/*
 * uart_functions.h
 *
 *  Created on: Jan 12, 2012
 *      Author: glausero
 */

#ifndef UART_FUNCTIONS_H_
#define UART_FUNCTIONS_H_

void uart_config();
void send_uart(USART_TypeDef* USARTx, char* TxBuffer, uint8_t package_size);
void receive_uart(USART_TypeDef* USARTx);

#endif /* UART_FUNCTIONS_H_ */
