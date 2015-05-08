/**
  ******************************************************************************
  * @file    USART/DMA_Interrupt/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "stm32f10x_it.h"
#include "platform_config.h"
#include "STM32vldiscovery.h"
#include "uart_functions.h"
#include "node_functions.h"
#include "i2c_functions.h"
#include "stdio.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_DMA_Interrupt
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// for delay function
extern __IO uint32_t TimingDelay;

// for i2c reception and sending
__IO uint8_t Tx_Idx2=0, Rx_Idx2=0;
extern uint8_t Buffer_Rx_I2C[];
extern uint8_t Buffer_Tx_I2C[];
extern uint8_t Buffer_Tx_Data[];

// general information about the node
extern NodeType TYPE;
extern NodeMode MODE;
extern uint8_t ID;

// splitter topo_tx pin map
extern uint16_t topo_txs[];

// for reconfiguration during initialization
extern I2C_InitTypeDef  I2C_InitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;

// led blinking
extern uint16_t CCR1Val;
extern uint16_t CCR2Val;

// test
extern uint16_t last_i2c_event_time;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  if (TimingDelay != 0x00)
  {
	TimingDelay--;
  }
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	receive_uart(USART2);
}

/**
  * @brief  Catches a rising edge on the topology input pin
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	uint8_t BufferTx[3];
	int i, n;
	static uint16_t topo_sent_event_time;

	// disable topo_rx interrupt to ensure stability^^
 	EXTI_ClearITPendingBit(EXTI_Line10);

	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	if(MODE = NODE_SAMPLING_START)
	{
		i = 0;

		while(id_map[i].taken == 1)
		{
			i++;
		}

		ID = i;

		Buffer_Tx_Data[1] = ID;

		BufferTx[0] = I2C_ID;
		BufferTx[1] = TYPE;
		BufferTx[2] = ID;

		I2C_InitStructure.I2C_OwnAddress1 = ID;
		I2C_Init(I2C1, &I2C_InitStructure);

		send_i2c(I2C1, BufferTx, 0, 3);
	}

	// for splitters probe each branch
	if(is_splitter(TYPE))
	{
		n = (TYPE & 0xF0) >> 4;

		for(i=0;i<n;i++)
		{
			// set back variables when probing into new branch
			next_splitter.completed = 0;
			next_splitter.exists = 0;

			// send info to collector which branch is probed
			BufferTx[0] = I2C_SPLITTER;
			BufferTx[1] = i;
			BufferTx[2] = ID;
			send_i2c(I2C1, BufferTx, 0, 3);

			MODE = NODE_PROBING;

			// send pulse to neighbor on branch i
			send_topo(topo_txs[i]);

			// save time :)
			topo_sent_event_time =  TIM_GetCounter(TIM16);

			// the branch can be called finished when either for a certain time no new module is encountered
			// or when the first splitter on the branch broadcasts 'complete'
			while( ( (uint16_t) last_i2c_event_time-topo_sent_event_time < 20 ) || ( ( next_splitter.exists == 1 ) && ( next_splitter.completed == 0 ) ) );

			MODE = NODE_IDED;
		}

		BufferTx[0] = I2C_SPLITTER;
		BufferTx[1] = 0xFF;
		BufferTx[2] = ID;

		// if all branch probing is done, inform everybody (i2c broadcast): 'complete'
		send_i2c(I2C1, BufferTx, 0, 3);
	}
	else
	{
		// data is filled into Buffer in main, in main loop and when new id is received
		send_i2c(I2C1, Buffer_Tx_Data, 0, 8);

		// is called for normal joints, pass on topo pulse (token)
		send_topo(TOPO_TX_1);
	}
}

/**
  * @brief  This function handles I2C2 global interrupt request.
  * @param  None
  * @retval None
  */
void I2C1_EV_IRQHandler(void)
{
	static char TxBuffer[64];
	static uint8_t package_size;

    /* If I2C1 is slave (MSL flag = 0) */
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL)==RESET)
    {
    	last_i2c_event_time =  TIM_GetCounter(TIM16);

        /* If ADDR = 1: EV1 */
    	if(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR)==SET)
        {
            /* Initialize the transmit/receive counters for next transmission/reception
            using Interrupt  */
            Tx_Idx2 = 0;
            Rx_Idx2 = 0;
        }

        /* If TXE = 1: EV3 */
    	else if(I2C_GetITStatus(I2C1, I2C_IT_TXE)==SET)
        {
            /* Write data in data register */
        	I2C_SendData(I2C1,Buffer_Tx_I2C[Tx_Idx2++]);
        }

        /* If RXNE = 1: EV2 */
    	else if(I2C_GetITStatus(I2C1, I2C_IT_RXNE)==SET)
        {
            /* Read data from data register */
            Buffer_Rx_I2C[Rx_Idx2++] = I2C_ReceiveData(I2C1);
        }

        /* If STOPF = 1: EV4 (Slave has detected a STOP condition on the bus */
    	else if (I2C_GetITStatus(I2C1, I2C_IT_STOPF)==SET)
        {
        	switch(Buffer_Rx_I2C[0]){
        	case I2C_DATA:
        		// data is broadcasted
        		if(MODE==NODE_SAMPLING_START)
        		{
        			// mark the ids as taken that are taken
        			id_map[Buffer_Rx_I2C[1]].taken = 1;
        		}

        		if(TYPE == COLLECTOR)
        		{
					// spit out via UART to computer, sprintf takes 0.1 ms
					package_size = sprintf(TxBuffer, "m,%02x,%04x,%04x,%04x\r\n", Buffer_Rx_I2C[1], Buffer_Rx_I2C[2] << 8 | Buffer_Rx_I2C[3], Buffer_Rx_I2C[4] << 8 | Buffer_Rx_I2C[5], Buffer_Rx_I2C[6] << 8 | Buffer_Rx_I2C[7]);
					send_uart(USARTz, TxBuffer, package_size);
        		}
        		break;
        	case I2C_ID:
        		// this case is called when a new i2c id was broadcasted

        		// collectors have to store the id of the first splitter
        		// and to output serial information of joint type and id
        		if(TYPE == COLLECTOR)
        		{
					id_map[Buffer_Rx_I2C[2]].taken = 1;
					id_map[Buffer_Rx_I2C[2]].type = Buffer_Rx_I2C[1];

        			// serial output
        			package_size = sprintf(TxBuffer, "i,%02x,%02x\r\n", Buffer_Rx_I2C[1], Buffer_Rx_I2C[2]);
        			send_uart(USARTz, TxBuffer, package_size);
        		}
        		else
        		{
        			if(Buffer_Rx_I2C[1]==COLLECTOR)
        			{
                		if( MODE==NODE_POWERED )
                		{
                			uint8_t i;

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

        					// put collector into id_map
        					id_map[Buffer_Rx_I2C[2]].taken = 1;
        					id_map[Buffer_Rx_I2C[2]].type = Buffer_Rx_I2C[1];

        					// change to sampling start mode
        					MODE=NODE_SAMPLING_START;
                		}
                		else if( MODE==NODE_SAMPLING_START || MODE==NODE_IDED)
                		{
                			// enable topo_rx interrupt to be ready to receive a pulse
                			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                			NVIC_Init(&NVIC_InitStructure);
                		}
        			}
        		}

        		break;
        	case I2C_SPLITTER:
        		// is called when a special splitter message is on the bus
        		// important for splitters and collector, to know when the probing on the current branch is done
        		if(MODE==NODE_PROBING)
        		{
        			if( ( next_splitter.exists == 1 ) && ( Buffer_Rx_I2C[1] == 0xFF ) && ( Buffer_Rx_I2C[2] == next_splitter.id ) )
        			{
        				next_splitter.completed = 1;
        			}
        			else if( next_splitter.exists == 0 )
        			{
        				next_splitter.exists = 1;
        				next_splitter.id = Buffer_Rx_I2C[2];
        				next_splitter.completed = 0;
        			}
        		}

        		// also get ids of splitters
        		if(MODE==NODE_SAMPLING_START)
        		{
        			// mark the ids as taken that are taken
        			id_map[Buffer_Rx_I2C[2]].taken = 1;
        		}

        		// collector has to keep track of all messages to get the topology
        		// other joints only keep track of the id
        		if(TYPE == COLLECTOR)
        		{
            		// send serial
        			package_size = sprintf(TxBuffer, "s,%02x,%02x\r\n", Buffer_Rx_I2C[1], Buffer_Rx_I2C[2]);
        			send_uart(USARTz, TxBuffer, package_size);
        		}
        		break;
        	case SET_LED:
        		// sets the led
        		set_leds((Buffer_Rx_I2C[1] << 8) | Buffer_Rx_I2C[2], (Buffer_Rx_I2C[3] << 8) | Buffer_Rx_I2C[4], (Buffer_Rx_I2C[5] << 8) | Buffer_Rx_I2C[6]);
				break;
			default:
				break;
			}

        	I2C_Cmd(I2C1, ENABLE);
        }
    } /* End slave mode */
    else
    {
    	(void)(I2C1->SR2);
    }
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  static uint16_t capture = 0;

  if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    capture = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, (capture + CCR1Val)%10000);
  }

  if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    capture = TIM_GetCapture4(TIM3);
    TIM_SetCompare4(TIM3, (capture + CCR2Val)%10000);
  }
}

/**
  * @brief  This function handles I2C2 Error interrupt request.
  * @param  None
  * @retval : None
  */
void I2C1_ER_IRQHandler(void)
{

    __IO uint32_t SR1Register =0;

    /* Read the I2C1 status register */
    SR1Register = I2C1->SR1;
    /* If AF = 1 */
    if ((SR1Register & 0x0400) == 0x0400)
    {
        I2C1->SR1 &= 0xFBFF;
        SR1Register = 0;
    }
    /* If ARLO = 1 */
    if ((SR1Register & 0x0200) == 0x0200)
    {
        I2C1->SR1 &= 0xFBFF;
        SR1Register = 0;
    }
    /* If BERR = 1 */
    if ((SR1Register & 0x0100) == 0x0100)
    {
        I2C1->SR1 &= 0xFEFF;
        SR1Register = 0;
    }

    /* If OVR = 1 */

    if ((SR1Register & 0x0800) == 0x0800)
    {
        I2C1->SR1 &= 0xF7FF;
        SR1Register = 0;
    }
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
