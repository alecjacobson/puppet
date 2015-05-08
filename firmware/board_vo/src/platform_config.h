/**
  ******************************************************************************
  * @file    USART/DMA_Interrupt/platform_config.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Evaluation board specific configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define USARTz                   	USART1
#define USARTz_GPIO              	GPIOA
#define USARTz_CLK               	RCC_APB2Periph_USART1
#define USARTz_GPIO_CLK          	RCC_APB2Periph_GPIOA
#define USARTz_RxPin             	GPIO_Pin_10
#define USARTz_TxPin             	GPIO_Pin_9
#define USARTz_Tx_DMA_Channel    	DMA1_Channel4 // TODO
#define USARTz_Tx_DMA_FLAG       	DMA1_FLAG_TC4 // TODOW
#define USARTz_IRQn              	USART1_IRQn
#define USARTz_DR_Base				0x40013804

#define SPI_MASTER                  SPI1
#define SPI_MASTER_CLK              RCC_APB2Periph_SPI1
#define SPI_MASTER_GPIO             GPIOA
#define SPI_MASTER_GPIO_CLK         RCC_APB2Periph_GPIOA
#define SPI_MASTER_PIN_SS_1         GPIO_Pin_1
#define SPI_MASTER_PIN_SS_2         GPIO_Pin_15
#define SPI_MASTER_PIN_SS_3         GPIO_Pin_3
#define SPI_MASTER_PIN_SCK          GPIO_Pin_5
#define SPI_MASTER_PIN_MISO         GPIO_Pin_6
#define SPI_MASTER_PIN_MOSI         GPIO_Pin_7
#define SPI_MASTER_IRQn             SPI1_IRQn
#define SPI_N 						3

// topo uses the same pins as the spi, makes use of only one board more possible
#define TOPO_RX						GPIO_Pin_12
#define TOPO_TX_1					GPIO_Pin_11
#define TOPO_TX_2					SPI_MASTER_PIN_SS_1
#define TOPO_TX_3					SPI_MASTER_PIN_SS_3
#define TOPO_TX_4					SPI_MASTER_PIN_SCK
#define TOPO_TX_5 					SPI_MASTER_PIN_MOSI
#define TOPO_RX_EXTI_Line			EXTI_Line12
#define TOPO_RX_IRQn				EXTI15_10_IRQn

#define LED_GPIO_CLK				RCC_APB2Periph_GPIOB
#define LED_GPIO					GPIOB
#define LED_RED						GPIO_Pin_3
#define LED_GREEN					GPIO_Pin_4
#define LED_BLUE					GPIO_Pin_5

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
