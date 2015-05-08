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
#include "uart_queue.h"
#include "uart_functions.h"
#include "node_functions.h"
#include "i2c_functions.h"
#include "stdio.h"

#include "configuration.h"

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
extern uint8_t Buffer_Rx2[];
extern uint8_t Buffer_Tx2[];

// general information about the node
extern NodeType TYPE;
extern uint8_t ID;
extern uint8_t *UID;

// initialization variables
extern uint16_t topology[256];
extern uint8_t topo_counter;
extern uint8_t initialized;

extern uint8_t next_splitter_completed;
extern uint8_t next_splitter_ID;

uint8_t splitter_is_probing = 0;

// splitter topo_tx pin map
extern uint16_t topo_txs[];

// for reconfiguration during initialization
extern I2C_InitTypeDef  I2C_InitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;

// led blinking
extern uint16_t LedRedRate;
extern uint16_t LedGreenRate;
extern uint16_t LedBlueRate;

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
  * @brief  Is called when DMA Uart sending Line is finished
  * @param  None
  * @retval None
  */
void DMA1_Channel4_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC4);
	uart_output_interrupt();
}

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	receive_uart(USART1);
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
	uint8_t old_topo_counter = topo_counter;
	uint16_t Timeout;

	// assign and broadcast ID to module, maximum + 2
	ID = topology[topo_counter-1]+2;

	if(ID == 0x58)
	{
		ID = ID + 2;
	}

	BufferTx[0] = I2C_ID_BROADCAST;
	BufferTx[1] = ID;
	BufferTx[2] = TYPE;

	for(i=0; i<12; i++)
	{
		BufferTx[3+i] = UID[i];
	}

	I2C_InitStructure.I2C_OwnAddress1 = ID;
	I2C_Init(I2C1, &I2C_InitStructure);

	send_i2c(I2C1, BufferTx, 0, 15);

	topology[topo_counter++] = (TYPE << 8) | ID;

	// disable topo_rx interrupt to ensure stability^^
 	EXTI_ClearITPendingBit(TOPO_RX_EXTI_Line);

	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	// for splitters probe each branch
	if(is_splitter(TYPE))
	{
		n = (TYPE & 0xF0) >> 4;

		for(i=0;i<n;i++)
		{
			// set back variables when probing into new branch
			next_splitter_completed = 0;
			next_splitter_ID = 0;

			// set splitter to probing, important in order to know when to store splitter id from broadcast (i2c interrupt)
			splitter_is_probing = 1;

			// send info to collector which branch is probed
			BufferTx[0] = I2C_SPLITTER;
			BufferTx[1] = ID;
			BufferTx[2] = i;
			send_i2c(I2C1, BufferTx, 0x01, 3);

			// send pulse to neighbor on branch i
			send_topo(topo_txs[i]);

			Timeout = 0xFFFF;

			// the branch can be called finished when either for a certain time no new module is encountered
			// or when the first splitter on the branch broadcasts 'complete'
			while(1)
			{
				// is set in the i2c interrupt
				if(next_splitter_completed == 1)
				{
					break;
				}

				// when no splitter is on the branch, break when for a certain time no new module is encountered
			    if (Timeout-- == 0 && next_splitter_ID == 0)
			    {
			    	if(old_topo_counter != topo_counter)
			    	{
				    	Timeout = 0xFFFF;
						old_topo_counter = topo_counter;
			    	}
			    	else
			    	{
			    		break;
			    	}
			    }
			}

			// maybe pointless
			splitter_is_probing = 0;
		}

		// if all branch probing is done, inform everybody (i2c broadcast): 'complete'
		BufferTx[0] = I2C_SPLITTER;
		BufferTx[1] = ID;
		BufferTx[2] = 0xFF;

		send_i2c(I2C1, BufferTx, 0, 3);
	}
	else
	{
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
    Line lout;

    /* If I2C1 is slave (MSL flag = 0) */
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_MSL)==RESET)
    {
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
        	I2C_SendData(I2C1,Buffer_Tx2[Tx_Idx2++]);
        }

        /* If RXNE = 1: EV2 */
    	else if(I2C_GetITStatus(I2C1, I2C_IT_RXNE)==SET)
        {
            /* Read data from data register */
            Buffer_Rx2[Rx_Idx2++] = I2C_ReceiveData(I2C1);
        }

        /* If STOPF = 1: EV4 (Slave has detected a STOP condition on the bus */
    	else if (I2C_GetITStatus(I2C1, I2C_IT_STOPF)==SET)
        {
        	switch(Buffer_Rx2[0]){
        	case I2C_ID_BROADCAST:
        		// this case is called when a new i2c id was broadcasted

        		// the i2c broadcast of the collector makes everybody else to go into initialization mode
        		if(Buffer_Rx2[1]==0x01&&TYPE!=NODE_COLLECTOR)
        		{
            		// set back of variables, to restart initialization
        			topo_counter = 0;
        			initialized = 0;

        			// enable interrupt on topo_rx pin
        			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        			NVIC_Init(&NVIC_InitStructure);
        		}

        		// keep counting the i2c ids taken and the corresponding joint type
        		topology[topo_counter++] = Buffer_Rx2[2] << 8 | Buffer_Rx2[1];

        		// splitters have to store if there was a splitter on the current (probing) branch
        		// value of the first splitter is store if the branch is connected to the splitter
        		if(is_splitter(TYPE) && splitter_is_probing && next_splitter_ID == 0 && is_splitter(Buffer_Rx2[2]))
        		{
					next_splitter_ID = Buffer_Rx2[1];
        		}

        		// collectors have to store the id of the first splitter
        		// and to output serial information of joint type and id
        		if(TYPE == NODE_COLLECTOR)
        		{
        			// checks if the received i2c_id_broadcast message is from the first splitter if there was no splitter yet
        			if(next_splitter_ID == 0 && is_splitter(Buffer_Rx2[2]))
        			{
        				next_splitter_ID = Buffer_Rx2[1];
        			}

        			// serial output
        			lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "I,%02x,%02x,%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\r\n", Buffer_Rx2[1], Buffer_Rx2[2], Buffer_Rx2[3], Buffer_Rx2[4], Buffer_Rx2[5], Buffer_Rx2[6], Buffer_Rx2[7], Buffer_Rx2[8], Buffer_Rx2[9], Buffer_Rx2[10], Buffer_Rx2[11], Buffer_Rx2[12], Buffer_Rx2[13], Buffer_Rx2[14]);
        			uart_queue_push_line(&uart_output_queue,&lout);
        		}

        		break;
        	case I2C_COMPLETE:
        		// is called when the collector sends the init_complete broadcast
        		// module passes from init to measurement phase
        		if(ID!=0)
        		{
            		initialized = 1;
        		}
        		break;
        	case I2C_SPLITTER:
        		// is called when a special splitter message is on the bus
        		// important for splitters and collector, to know when the probing on the current branch is done
        		if(Buffer_Rx2[2] == 0xFF && Buffer_Rx2[1] == next_splitter_ID)
        		{
        			// used in the topo_rx interrupt to go to the next splitter branch
        			// also used for the master to determine when init is done
        			next_splitter_completed = 1;
        		}

        		// collector has to keep track of all messages to get the topology
        		// other joints only keep track of the id
        		if(TYPE == NODE_COLLECTOR)
        		{
            		//topology[topo_counter++] = Buffer_Rx2[2] << 8 | Buffer_Rx2[1];

            		// send serial
        			lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "S,%02x,%02x\r\n", Buffer_Rx2[1], Buffer_Rx2[2]);
        			uart_queue_push_line(&uart_output_queue,&lout);
        		}
        		break;
        	case SET_LED:
        		// sets the led
        		set_leds((Buffer_Rx2[1] << 8) | Buffer_Rx2[2], (Buffer_Rx2[3] << 8) | Buffer_Rx2[4], (Buffer_Rx2[5] << 8) | Buffer_Rx2[6]);
				break;
        	case CALIB:
        		if(is_joint(TYPE))
        		{
            		calibrate_angles();
        		}
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

  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    capture = TIM_GetCapture1(TIM3);
    LED_GPIO->ODR ^= LED_GREEN;
    TIM_SetCompare1(TIM3, (capture + LedGreenRate)%0xFFFF);
  }

  if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    capture = TIM_GetCapture2(TIM3);
    LED_GPIO->ODR ^= LED_BLUE;
    TIM_SetCompare2(TIM3, (capture + LedBlueRate)%0xFFFF);
  }
}

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  static uint16_t capture = 0;

  if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    capture = TIM_GetCapture2(TIM2);
    LED_GPIO->ODR ^= LED_RED;
    TIM_SetCompare2(TIM2, (capture + LedRedRate)%0xFFFF);
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
