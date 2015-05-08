/*
 * spi_functions.c
 *
 *  Created on: Jan 12, 2012
 *      Author: glausero
 */

/* includes */
#include "stm32f10x.h"
#include "platform_config.h"
#include "node_functions.h"

/* defines */
#define TxBufferSize 2
#define RxBufferSize 10

#define SPI_SS_LOW(N)	GPIO_ResetBits(SPI_MASTER_GPIO, SPI_SELECT[N])
#define SPI_SS_HIGH(N)	GPIO_SetBits(SPI_MASTER_GPIO, SPI_SELECT[N])

/******************************************************************************/
/*            Functions for a spi to work			                      */
/******************************************************************************/

/**
  * @brief  configuration of all spi related things^^
  * @param  None
  * @retval None
  */
void spi_config()
{
	// RCC
	RCC_PCLK2Config(RCC_HCLK_Div2);
	/* Enable SPI_MASTER clock and GPIO clock for SPI_MASTER */
	RCC_APB2PeriphClockCmd(SPI_MASTER_GPIO_CLK | SPI_MASTER_CLK, ENABLE);

	// GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure SPI_MASTER pins: NSS, SCK, MOSI*/
	GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_SCK | SPI_MASTER_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_SS_1 | SPI_MASTER_PIN_SS_2 | SPI_MASTER_PIN_SS_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);

	// NVIC
	/* no interrupts */
	SPI_InitTypeDef  SPI_InitStructure;

	// SPI
	/* SPI_MASTER configuration ------------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_MASTER, &SPI_InitStructure);

	SPI_BiDirectionalLineConfig(SPI_MASTER, SPI_Direction_Tx);

	/* Enable SPI_MASTER */
	SPI_Cmd(SPI_MASTER, ENABLE);
}

/**
  * @brief  configuration of all spi related things^^
  * @param  0,1 or 2
  * @retval angle
  */
uint16_t read_spi(uint8_t selected_spi)
{
	/* variables */
	uint8_t SPI_MASTER_Buffer_Rx[RxBufferSize];

	static uint16_t SPI_SELECT[]={SPI_MASTER_PIN_SS_3, SPI_MASTER_PIN_SS_2, SPI_MASTER_PIN_SS_1};

	static uint8_t RxIdx = 0;

	static uint16_t sensor_angle;
	static uint16_t sensor_angle_invert;
	static uint16_t angle_to_send;

	/* prepare gpio_init structure for mode changes ----------------------------*/
	static GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* CHIP SELECT ON*/
	SPI_SS_LOW(selected_spi);

	Delay(1);

	/* send data request */
	SPI_I2S_SendData(SPI_MASTER, 0xAA);

	/* wait till data was sent */
	while(SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE)==RESET);
	while(SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_BSY)==SET);

	/* change from transmit to receive mode */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);
	SPI_Cmd(SPI_MASTER, DISABLE);
	Delay(2);
	SPI_BiDirectionalLineConfig(SPI_MASTER, SPI_Direction_Rx);
	Delay(3);

	/* receive data: 10 bytes*/
	RxIdx=0;

	while(RxIdx<RxBufferSize)
	{
		SPI_Cmd(SPI_MASTER, ENABLE);
		Delay(5);
		SPI_Cmd(SPI_MASTER, DISABLE);

		while(SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE)!=RESET)
		SPI_MASTER_Buffer_Rx[RxIdx++] = SPI_I2S_ReceiveData(SPI_MASTER);

		Delay(5);
	}

	/* back to transmit mode */
	SPI_BiDirectionalLineConfig(SPI_MASTER, SPI_Direction_Tx);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);
	SPI_Cmd(SPI_MASTER, ENABLE);

	Delay(5);

	/* CHIP SELECT OFF */
	SPI_SS_HIGH(selected_spi);

	/* data interpretation */
	sensor_angle = (SPI_MASTER_Buffer_Rx[2] << 8) + SPI_MASTER_Buffer_Rx[3];
	sensor_angle_invert = (SPI_MASTER_Buffer_Rx[4] << 8) + SPI_MASTER_Buffer_Rx[5];

	if(!(sensor_angle & 1) || sensor_angle == 0xFFFF || (sensor_angle & sensor_angle_invert) || (sensor_angle | sensor_angle_invert) != 0xFFFF)
	{
		angle_to_send = 0xabcd;
	}
	else
	{
		angle_to_send = sensor_angle;
	}

	Delay(50);

	return angle_to_send;
}
