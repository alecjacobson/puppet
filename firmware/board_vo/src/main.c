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
#include "node_functions.h"
#include "spi_functions.h"
#include "uart_functions.h"
#include "i2c_functions.h"
#include "stdio.h"

#include "uart_queue.h"
#include "collector.h"
#include "measurements.h"

#include "configuration.h"

// state variables
uint8_t not_interrupted = 1;
uint8_t in_debug = 0;
uint8_t initialized = 0;

// node variables
uint8_t ID = 0x58;
NodeType TYPE = NODE_UNDEFINED;
uint8_t *UID = (uint8_t *)0x1FFFF7E8;
uint16_t *CALIB_VALUES = (uint16_t*)0x0800EA60;
uint8_t n_sensors = 0;

// topology variables
uint16_t topology[256];
uint16_t topology_error[256] = {0};
uint8_t topo_counter = 0;
uint8_t next_splitter_completed = 0;
uint8_t next_splitter_ID = 0;

// i2c variables
uint8_t Buffer_Rx2[255];
uint8_t Buffer_Tx2[255];
I2cError i2c_error_code = NONE;

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

	DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STANDBY | DBGMCU_STOP, ENABLE);

	// load type from flash
	uint32_t type_from_flash = FLASH_GetUserOptionByte();
	TYPE = (type_from_flash >> 8);

	// configure leds
	led_config();

	/* configure uart*/
    uart_queue_initialize(&uart_input_queue);
    uart_queue_initialize(&uart_output_queue);

	uart_config();

	// configures topology pins (rx and tx)
	topo_config();

	if(is_joint(TYPE))
	{
		spi_config();
		n_sensors = (TYPE >> 4) & 0b0011;
	}

	i2c_config();

	waiter_config();

	initialize_measurements();

	// endless
	while (1)
	{
        process_uart_input();

        if (is_joint(TYPE)) {
            collect_joint_measurements(n_sensors);
            if (in_debug) {
                Line l;
                create_joint_message(ID,&l);
                uart_queue_push_line(&uart_output_queue,&l);
            }
            pack_joint_measurements_for_i2c();
        }

        if (TYPE == NODE_COLLECTOR) {
            run_collector_state_machine();
            // non-blocking uart management
            process_uart_output(0);
        } else {
            // apart from the collector, any uart output is debug,
            // so there is no sense in having it non-blocking
            process_uart_output(1);
        }
    }

    // never reached
    return 0;
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
