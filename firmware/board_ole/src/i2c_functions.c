/*
 * i2c_functions.c
 *
 *  Created on: Jan 13, 2012
 *      Author: glausero
 */

/* includes */
#include "stm32f10x.h"
#include "platform_config.h"
#include "node_functions.h"
#include "i2c_functions.h"

/* defines */
#define ClockSpeed	100000 //not fix

/* variables */
extern uint8_t ID;
extern uint8_t TYPE;
extern uint8_t i2c_error_code;
I2C_InitTypeDef  I2C_InitStructure;

void i2c_config()
{
	// RCC
	/* Enable GPIOB clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	// GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable I2C1 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* Release I2C1 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    I2C_DeInit(I2C1);

	// I2C
	/* Enable I2C1 */
	I2C_Cmd(I2C1, ENABLE);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = ID;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_GeneralCallCmd(I2C1, ENABLE);

	/* Enable Event IT needed for ADDR and STOPF events ITs */
	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);

	/* Enable Error IT */
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

	/* Enable Buffer IT (TXE and RXNE ITs) */
	I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);

	// NVIC
	/* 1 bit for pre-emption priority, 3 bits for subpriority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_SetPriority(I2C1_EV_IRQn, 0x00);
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	NVIC_SetPriority(I2C1_ER_IRQn, 0x01);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
}

ErrorStatus send_i2c(I2C_TypeDef * I2Cx, uint8_t* TxBuffer, uint8_t slave_address, uint8_t package_size)
{
	static uint16_t Timeout;
	uint16_t i2c_error = 0;
	//static uint16_t TxBuffer_serial[32];
	//static uint16_t package_size_serial;
	uint8_t TxCounter = 0;

	//=============================================================
	/* Disable Event IT needed for ADDR and STOPF events ITs */
	I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);

	/* Enable Error IT */
	I2C_ITConfig(I2C1, I2C_IT_ERR, DISABLE);

	/* Enable Buffer IT (TXE and RXNE ITs) */
	I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
	//=============================================================

	Timeout = 0xFFF;
	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
        if (Timeout-- == 0)
        {
			//package_size_serial = sprintf(TxBuffer_serial, "bus was busy\r\n");
			//send_uart(USARTz, TxBuffer_serial, package_size_serial);
        	i2c_error_code = BUSY;
			i2c_error = 1;
			break;
        }
	}

	if(i2c_error == 0)
	{
		/* Send I2Cx START condition */
		I2C_GenerateSTART(I2Cx, ENABLE);

		Timeout = 0xFFF;
		/* Test on I2Cx EV5 and clear it */
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		{
			if (Timeout-- == 0)
			{
				//package_size_serial = sprintf(TxBuffer_serial, "not able to become master\r\n");
				//send_uart(USARTz, TxBuffer_serial, package_size_serial);
	        	i2c_error_code = NOT_MASTER;
				i2c_error = 1;
				break;
			}
		}
	}

	if(i2c_error == 0)
	{
		/* Send slave Address for write */
		I2C_Send7bitAddress(I2Cx, slave_address, I2C_Direction_Transmitter);

		Timeout = 0xFF;
		/* wait for acknowledge bit */
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			if (Timeout-- == 0)
			{
				//package_size_serial = sprintf(TxBuffer_serial, "no acknowldgement received\r\n");
				//send_uart(USARTz, TxBuffer_serial, package_size_serial);
	        	i2c_error_code = NO_ACKNOWLEDGEMENT;
				i2c_error = 1;
				break;
			}
		}
	}

	if(i2c_error == 0)
	{
		while(TxCounter!=package_size)
		{
			/* Send some data */
			I2C_SendData(I2Cx, TxBuffer[TxCounter++]);

			Timeout = 0xFF;
			while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
				if (Timeout-- == 0)
				{
					//package_size_serial = sprintf(TxBuffer_serial, "couldn't send data\r\n");
					//send_uart(USARTz, TxBuffer_serial, package_size_serial);
		        	i2c_error_code = DATA_NOT_SENT;
					i2c_error = 1;
					break;
				}
			}
		}
	}

	/* Send I2Cx STOP Condition */
	I2C_GenerateSTOP(I2Cx, ENABLE);

	Timeout = 0xFF;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
	{
        if (Timeout-- == 0)
			//package_size_serial = sprintf(TxBuffer_serial, "couldn't send stop\r\n");
			//send_uart(USARTz, TxBuffer_serial, package_size_serial);
        	I2C_GenerateSTOP(I2Cx, ENABLE);
    		i2c_error_code = STOP_NOT_SENT;
        	i2c_error = 1;
			break;
		// really needed?
	}

	//=============================================================
	/* Enable Event IT needed for ADDR and STOPF events ITs */
	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);

	/* Enable Error IT */
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

	/* Enable Buffer IT (TXE and RXNE ITs) */
	I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
	//=============================================================

	if(i2c_error)
	{
		i2c_config();
		return ERROR;
	}
	else
	{
		return SUCCESS;
	}
}

ErrorStatus receive_i2c(I2C_TypeDef * I2Cx, uint8_t * RxBuffer, uint8_t slave_address, uint8_t package_size)
{
	static uint32_t Timeout;
	uint8_t TxCounter = 0;
	uint16_t i2c_error = 0;

	//=============================================================
	/* Enable Event IT needed for ADDR and STOPF events ITs */
	I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);

	/* Enable Error IT */
	I2C_ITConfig(I2C1, I2C_IT_ERR, DISABLE);

	/* Enable Buffer IT (TXE and RXNE ITs) */
	I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
	//=============================================================

	/* While the bus is busy */
	Timeout = 0xFFF;
	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
        if (Timeout-- == 0)
        {
        	i2c_error_code = BUSY;
			i2c_error = 1;
			break;
        }
	}

	if(i2c_error == 0)
	{
		/* Send I2Cx START condition */
		I2C_GenerateSTART(I2Cx, ENABLE);

		Timeout = 0xFFF;
		/* Test on I2Cx EV5 and clear it */
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		{
			if (Timeout-- == 0)
			{
	        	i2c_error_code = NOT_MASTER;
				i2c_error = 1;
				break;
			}
		}
	}

	if(i2c_error == 0)
	{
		/* Send slave Address for write */
		I2C_Send7bitAddress(I2Cx, slave_address, I2C_Direction_Receiver);

		Timeout = 0xFFFF;
		/* wait for acknowledge bit */
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
	        if (Timeout-- == 0)
	        {
	        	i2c_error_code = NO_ACKNOWLEDGEMENT;
				i2c_error = 1;
				break;
	        }
		}
	}

	if(i2c_error == 0)
	{
		while(TxCounter!=package_size-1)
		{
				Timeout = 0xFFFF;

				while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED ))
				{
					if (Timeout-- == 0)
					{
						i2c_error_code = DATA_NOT_RECEIVED;
						i2c_error = 1;
						break;
					}
				}

				/* Receive some data */
				RxBuffer[TxCounter++] = I2C_ReceiveData(I2Cx);
		}
	}

	I2C_AcknowledgeConfig(I2Cx, DISABLE);

	if(i2c_error == 0)
	{
		/* Send I2Cx STOP Condition */
		I2C_GenerateSTOP(I2Cx, ENABLE);

		Timeout = 0xFFFF;

		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED ))
		{
			if (Timeout-- == 0)
			{
	        	i2c_error_code = DATA_NOT_RECEIVED;
				i2c_error = 1;
			}
		}

		/* Receive some data */
		RxBuffer[TxCounter++] = I2C_ReceiveData(I2Cx);
	}

	Timeout = 0xFF;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
	{
        if (Timeout-- == 0)
        	I2C_GenerateSTOP(I2Cx, ENABLE);
    		i2c_error_code = STOP_NOT_SENT;
        	i2c_error = 1;
			break;
		// really needed?
	}

	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	//=============================================================
	/* Enable Event IT needed for ADDR and STOPF events ITs */
	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);

	/* Enable Error IT */
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

	/* Enable Buffer IT (TXE and RXNE ITs) */
	I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
	//=============================================================

	if(i2c_error)
	{
		i2c_config();
		return ERROR;
	}
	else
	{
		return SUCCESS;
	}
}

void clear_busy_i2c()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure I2C1 pins: SCL */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	uint16_t Timeout = 0xFFFF;

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)&&Timeout--!=0)
	{
		GPIOB->ODR ^= GPIO_Pin_6;
		wait(1000000/ClockSpeed);
	}

	I2C_GenerateSTOP(I2C1, ENABLE);

	/* configure back */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
