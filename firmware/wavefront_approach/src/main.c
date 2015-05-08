/**
  ******************************************************************************
  * @file    USART/DMA_Interrupt/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"
#include "STM32vldiscovery.h"
#include "node_functions.h"
#include "spi_functions.h"
#include "uart_functions.h"
#include "i2c_functions.h"
#include "stdio.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t ID = 0;
NodeMode MODE = NODE_POWERED;
NodeType TYPE = UNDEFINED;

uint16_t last_i2c_event_time;

// i2c variables
/* Buffer of data to be received by I2C1 */
uint8_t Buffer_Rx_I2C[255];
/* Buffer of data to be transmitted by I2C1 */
uint8_t Buffer_Tx_I2C[255];
uint8_t Buffer_Tx_Data[255];

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f10x_xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f10x.c file
	 */

	uint8_t n_sensors = 0;
	uint16_t angle_to_send[] = {-1, -1, -1};


	char TxBuffer[32];
	uint8_t package_size = 0;
	int i;

	// load type from flash :)
	uint32_t type_from_flash = FLASH_GetUserOptionByte();
	TYPE = (type_from_flash >> 8);

	/* configure uart and spi */
	uart_config();

	// configures topology pins (rx and tx)
	topo_config();

	if(is_joint(TYPE))
	{
		spi_config();
		n_sensors = (TYPE >> 4) & 0b0011;
		Buffer_Tx_Data[0] = I2C_DATA;
	}

	if(TYPE==COLLECTOR)
	{
		// set ID to 0x01 for collector
		ID = 0x01;
	}

	i2c_config();

	waiter_config();

	/* SysTick configuration ------------------------------------------------------*/
	while(SysTick_Config(SystemCoreClock / 50000))
	{

	}

	// configure leds
	led_config();

	// endless
	while (1)
	{
		if(TYPE==COLLECTOR)
		{
			MODE = NODE_IDED;

			// reset id_map
			for(i=0; i<256; i++)
			{
				if(i%2==0)
				{
					id_map[i].taken = 1;
				}
				else
				{
					id_map[i].taken = 0;
				}
			}

			// reset some variables
			next_splitter.completed = 0;
			next_splitter.exists = 0;

			// inform user about start init phase
			package_size = sprintf(TxBuffer, "===START===\r\n");
			send_uart(USARTz, TxBuffer, package_size);

			// broadcast collector i2c id, init initialization phase
			TxBuffer[0] = I2C_ID;
			TxBuffer[1] = TYPE;
			TxBuffer[2] = ID;

			send_i2c(I2C1, TxBuffer, 0, 3);

			id_map[1].taken = 1;
			id_map[1].type = TYPE;

			// send collector id to serial
			package_size = sprintf(TxBuffer, "i,%02x,%02x\r\n", TYPE, ID);
			send_uart(USARTz, TxBuffer, package_size);

			MODE = NODE_PROBING;

			// send pulse (token) to first module
			send_topo(TOPO_TX_1);

			last_i2c_event_time = TIM_GetCounter(TIM16);

			uint16_t counter_value = TIM_GetCounter(TIM16) - last_i2c_event_time;

			// wait for a break (no new modules for a certain time) or for the first splitter to finish
			while( ( counter_value < 250 ) || ( ( next_splitter.exists == 1 ) && ( next_splitter.completed == 0 ) ) )
			{
				counter_value = TIM_GetCounter(TIM16) - last_i2c_event_time;
			}

			// send serial 'init done'
			package_size = sprintf(TxBuffer, "===END===\r\n");
			send_uart(USARTz, TxBuffer, package_size);
		}
		else if(is_joint(TYPE))
		{
			// make some readings from SPI (melexis)
			for(i=0; i<n_sensors ; i++)
			{
				angle_to_send[i] = read_spi(i);

				// store in buffer to be sent to collector via i2c
				Buffer_Tx_Data[2+2*i] = angle_to_send[i] >> 8;
				Buffer_Tx_Data[2+2*i+1] = angle_to_send[i];
			}
		}

//			if(TYPE==COLLECTOR)
//			{
//				for(i=1; i<topo_counter; i++)
//				{
//					// check if really joint module or something else (has to be solved different
//					if(is_joint(topology[i] >> 8))
//					{
//						if(receive_i2c(I2C1, Buffer_Rx_I2C, topology[i], 6))
//						{
//							// make angle package
//							angle_to_send[0] = Buffer_Rx_I2C[0] << 8 | Buffer_Rx_I2C[1];
//							angle_to_send[1] = Buffer_Rx_I2C[2] << 8 | Buffer_Rx_I2C[3];
//							angle_to_send[2] = Buffer_Rx_I2C[4] << 8 | Buffer_Rx_I2C[5];
//
//							// spit out via UART to computer, sprintf takes 0.1 ms
//
//							package_size = sprintf(TxBuffer, "m,%02x,%04x,%04x,%04x\r\n", topology[i] & 0xF, angle_to_send[0], angle_to_send[1], angle_to_send[2]);
//							send_uart(USARTz, TxBuffer, package_size);
//						}
//						else
//						{
//							// error message
//							package_size = sprintf(TxBuffer, "error talking to slave %02x\r\n", topology[i]);
//							send_uart(USARTz, TxBuffer, package_size);
//						}
//					}
//				}
//			}

	}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
