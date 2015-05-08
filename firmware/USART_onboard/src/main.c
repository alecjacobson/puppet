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

uint8_t not_interrupted = 1;

uint8_t ID = 0;
NodeType TYPE = UNDEFINED;

uint16_t topology[256];
uint8_t topo_counter = 0;
uint8_t next_splitter_completed = 0;
uint8_t next_splitter_ID = 0;

uint8_t initialized = 0;

// i2c variables
/* Buffer of data to be received by I2C1 */
uint8_t Buffer_Rx2[255];
/* Buffer of data to be transmitted by I2C1 */
uint8_t Buffer_Tx2[255];

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

	uint8_t old_topo_counter = 0;


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
	}

	i2c_config();

	/* SysTick configuration ------------------------------------------------------*/
	while(SysTick_Config(SystemCoreClock / 50000))
	{

	}

	// configure leds
	led_config();

	// endless
	while (1)
	{
		if(!initialized)
		{
			if(TYPE==COLLECTOR)
			{
				// reset some variables
				next_splitter_completed = 0;
				next_splitter_ID = 0;
				topo_counter=0;

				// set ID to 0x01 for collector
				ID = 0x06;

				// inform user about start init phase
				package_size = sprintf(TxBuffer, "start initialization ===================\r\n", topo_counter, next_splitter_completed);
				send_uart(USARTz, TxBuffer, package_size);

				// wait, maybe unnecessary
				Delay(500);

				// broadcast collector i2c id, init initialization phase
				TxBuffer[0] = I2C_ID_BROADCAST;
				TxBuffer[1] = TYPE;
				TxBuffer[2] = ID;

				send_i2c(I2C1, TxBuffer, 0, 3);
				topology[topo_counter++] = ID;

				// send collector id to serial
				package_size = sprintf(TxBuffer, "i,%02x,%02x\r\n", TYPE, ID);
				send_uart(USARTz, TxBuffer, package_size);

				// wait, maybe unnecessary
				Delay(5000);

				// send pulse (token) to first module
				send_topo(TOPO_TX_1);
			}

			while(!initialized)
			{
				if(TYPE == COLLECTOR)
				{
					// wait for a break (no new modules for a certain time) or for the first splitter to finish
					do{
						if(next_splitter_completed==1)
						{
							break;
						}

						old_topo_counter = topo_counter;
						Delay(25000);
					} while(old_topo_counter != topo_counter || next_splitter_ID != 0);

					// initialization is done
					initialized = 1;

					// broadcast 'init done'
					TxBuffer[0] = I2C_COMPLETE;
					send_i2c(I2C1, TxBuffer, 0, 1);

					// send serial 'init done'
					package_size = sprintf(TxBuffer, "initialization completed: %i nodes found, %i ====\r\n", topo_counter, next_splitter_completed);
					send_uart(USARTz, TxBuffer, package_size);
				}
			}

			if(TYPE==COLLECTOR)
			{
				// TODO: figure out configuration and important guys to talk to, done in a sloppy fashion down somewhere

				not_interrupted = 0; // TODO: remove in final version

				// wait for everybody, very long ^^
				Delay(10000);
			}
		}

		// if not interrupted
		if(not_interrupted)
		{
			if(is_joint(TYPE))
			{
				// make some readings from SPI (melexis)
				for(i=0; i<n_sensors ; i++)
				{
					angle_to_send[i] = read_spi(i);

					// store in buffer to be sent to collector via i2c
					Buffer_Tx2[2*i] = angle_to_send[i] >> 8;
					Buffer_Tx2[2*i+1] = angle_to_send[i];
				}
			}

			if(TYPE==COLLECTOR)
			{
				for(i=1; i<topo_counter; i++)
				{
					// check if really joint module or something else (has to be solved different
					if(is_joint(topology[i] >> 8))
					{
						if(receive_i2c(I2C1, Buffer_Rx2, topology[i], 6))
						{
							// make angle package
							angle_to_send[0] = Buffer_Rx2[0] << 8 | Buffer_Rx2[1];
							angle_to_send[1] = Buffer_Rx2[2] << 8 | Buffer_Rx2[3];
							angle_to_send[2] = Buffer_Rx2[4] << 8 | Buffer_Rx2[5];

							// spit out via UART to computer, sprintf takes 0.1 ms

							package_size = sprintf(TxBuffer, "m,%02x,%04x,%04x,%04x\r\n", topology[i] & 0xF, angle_to_send[0], angle_to_send[1], angle_to_send[2]);
							send_uart(USARTz, TxBuffer, package_size);
						}
						else
						{
							// error message
							package_size = sprintf(TxBuffer, "error talking to slave %02x\r\n", topology[i]);
							send_uart(USARTz, TxBuffer, package_size);
						}
					}
				}
			}
		}
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
