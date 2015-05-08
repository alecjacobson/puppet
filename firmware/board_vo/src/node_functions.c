/*
 * node_functions.c
 *
 *  Created on: Jan 12, 2012
 *      Author: glausero
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "node_functions.h"
#include "spi_functions.h"
#include "platform_config.h"
#include "configuration.h"
//#include "STM32vldiscovery.h"

/* defines */

/* variables -----------------------------------------------------------------*/
uint16_t LedRedRate = 500;
uint16_t LedGreenRate = 150;
uint16_t LedBlueRate = 100;
extern uint8_t ID;
extern uint8_t TYPE;
extern uint8_t not_interrupted;
extern uint8_t *CALIB_VALUES;
__IO uint32_t TimingDelay;
NVIC_InitTypeDef NVIC_InitStructure;

/* variables */
uint16_t topo_txs[]={TOPO_TX_1, TOPO_TX_2, TOPO_TX_3, TOPO_TX_4, TOPO_TX_5};

/******************************************************************************/
/*            Functions for a node to work				                      */
/******************************************************************************/



/**
  * @brief  This function stores the current angles in flash
  * @param  None
  * @retval None
  */
void calibrate_angles()
{
	uint16_t angle, i, j;

	__disable_irq();

	FLASH_Unlock();

	/* Clear All pending flags */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	FLASH_ErasePage(0x0800EA60);

	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 5; j++)
		{
			if((angle = read_spi(i)))
			{
				break;
			}
		}

		FLASH_ProgramHalfWord(0x0800EA60+2*i, angle);
	}

	FLASH_Lock();

	__enable_irq();
}

/**
  * @brief  This function turns on or off leds
  * @param  None
  * @retval None
  */
void set_leds(uint16_t rate_r, uint16_t rate_g, uint16_t rate_b)
{
	static uint16_t counter = 0;

	if(rate_r>1)
	{
	    counter = TIM_GetCounter(TIM2);
		LedRedRate = rate_r;

	    LED_GPIO->BRR = LED_RED;

	    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

	    TIM_SetCompare2(TIM2, (counter + LedRedRate)%0xFFFF);
	}
	else
	{
		TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);

		if(rate_r==0)
		{
			LED_GPIO->BRR = LED_RED;
		}
		else
		{
			LED_GPIO->BSRR = LED_RED;
		}
	}

	if(rate_g>1)
	{
	    counter = TIM_GetCounter(TIM3);
		LedGreenRate = rate_g;

	    LED_GPIO->BRR = LED_GREEN;

	    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

	    TIM_SetCompare1(TIM3, (counter + LedGreenRate)%0xFFFF);
	}
	else
	{
		TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);

		if(rate_g==0)
		{
			LED_GPIO->BRR = LED_GREEN;
		}
		else
		{
			LED_GPIO->BSRR = LED_GREEN;
		}
	}

	if(rate_b>1)
	{
	    counter = TIM_GetCounter(TIM3);
		LedBlueRate = rate_b;

	    LED_GPIO->BRR = LED_BLUE;

	    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

	    TIM_SetCompare2(TIM3, (counter + LedBlueRate)%0xFFFF);
	}
	else
	{
		TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);

		if(rate_b==0)
		{
			LED_GPIO->BRR = LED_BLUE;
		}
		else
		{
			LED_GPIO->BSRR = LED_BLUE;
		}
	}
}

/**
  * @brief  Changes the id of the module
  * @param  None
  * @retval None
  */
void set_type(uint8_t module_type)
{
	// don't really know what I do here ... but it works
	FLASH_Unlock();

	unsigned int WRPR_Value = FLASH_GetWriteProtectionOptionByte();
	unsigned int ProtectedPages = WRPR_Value & (FLASH_WRProt_Pages0to1 | FLASH_WRProt_Pages2to3 |
												FLASH_WRProt_Pages4to5 | FLASH_WRProt_Pages6to7);

	// pages are not write protected
	if(ProtectedPages != 0x00)
	{
		FLASH_EraseOptionBytes();

		//FLASH_ReadOutProtection(ENABLE);

		// reprogram user option bytes
		FLASH_UserOptionByteConfig(OB_IWDG_SW, OB_STOP_NoRST, OB_STDBY_NoRST);

		// program ID to flash options
		FLASH_ProgramOptionByteData(0x1FFFF804, module_type);

		// FLASH_EnableWriteProtection(FLASH_WRProt_Pages0to1 | FLASH_WRProt_Pages2to3 |
		// FLASH_WRProt_Pages4to5 | FLASH_WRProt_Pages6to7);

		// FLASH_ReadOutProtection(DISABLE);
	}

	FLASH_Lock();

	TYPE = module_type;
}

/**
  * @brief  Configures topology pins
  * @param  None
  * @retval None
  */
void topo_config()
{
	int i,n;

	if(is_splitter(TYPE))
	{
		n = (TYPE & 0xF0) >> 4;
	}
	else if(TYPE==NODE_COLLECTOR)
	{
		n = 2;
	}
	else
	{
		n = 1;
	}

	// RCC
	RCC_APB2PeriphClockCmd(SPI_MASTER_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

	// GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure pin output*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	for(i=0;i<n;i++)
	{
		GPIO_InitStructure.GPIO_Pin =  topo_txs[i];
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}

	/* Configure pin 10: input*/
	GPIO_InitStructure.GPIO_Pin =  TOPO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	if(TYPE==NODE_COLLECTOR)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIOA->BSRR = TOPO_RX; //TODO change back to topo_txs[1]
	}
	else
	{
		// EXTI & NVIC
		EXTI_InitTypeDef EXTI_InitStructure;

		/* Connect EXTI Line to GPIO Pin */
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = TOPO_RX_EXTI_Line ;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = TOPO_RX_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;

		NVIC_Init(&NVIC_InitStructure);
	}
}

/**
  * @brief  Sends a pulse to pass on i2c master
  * @param  None
  * @retval None
  */
void send_topo(uint16_t PIN)
{
	static uint16_t Timeout;

	// set on high
	GPIOA->BSRR = PIN;

	// wait
	//Delay(5);

	wait(25);

//	Timeout = 0x00FF;
//	while(Timeout-- != 0){}

	// set on low
	GPIOA->BRR = PIN;
}

/**
  * @brief  Configures the led to work in output compare mode
  * @param  None
  * @retval None
  */
void restart_slaves()
{
	// set on low
	GPIOA->BRR = TOPO_RX; // TODO: change back to topo_rxs[1]

	wait_ms(10); //TODO: find good value

	// set on high
	GPIOA->BSRR = TOPO_RX;// TODO: change back to topo_rxs[1]
}

/**
  * @brief  Configures the led to work in output compare mode
  * @param  None
  * @retval None
  */
void led_config()
{
	RCC_APB2PeriphClockCmd(LED_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE );

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

	// GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  LED_RED |  LED_GREEN | LED_BLUE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;            // Alt Function - Push Pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( LED_GPIO, &GPIO_InitStructure );

	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE);
 	GPIO_PinRemapConfig( GPIO_PartialRemap_TIM3, ENABLE );
	GPIO_PinRemapConfig( GPIO_PartialRemap1_TIM2, ENABLE );

	//NVIC
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM3 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_Init(&NVIC_InitStructure);

	/* Time base configuration */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 24000;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Output Compare Toggle Mode configuration: Channel1 */
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OCInitStructure.TIM_Pulse = LedRedRate;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

	TIM_OCInitStructure.TIM_Pulse = LedGreenRate;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	TIM_OCInitStructure.TIM_Pulse = LedBlueRate;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* TIM enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* TIM enable counter */
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

	set_leds(0, 1, 0);
}

/**
  * @brief  Set up time for wait function
  * @param  blabla
  * @retval None
  */
void waiter_config()
{
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 24;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM4, ENABLE);
}

/**
  * @brief  milliseconds wait function, max 0xFFFF ms
  * @param  time to wait in milliseconds
  * @retval None
  */
void wait_ms(uint16_t msTime)
{
	uint16_t start_value = TIM_GetCounter(TIM3);

	uint16_t counter_value = TIM_GetCounter(TIM3) - start_value;

	while(counter_value < msTime)
	{
		counter_value = TIM_GetCounter(TIM3) - start_value;
	}
}

/**
  * @brief  microsecond wait function, max 0xFFFF us
  * @param  time to wait in microseconds
  * @retval None
  */
void wait(uint16_t usTime)
{
	uint16_t start_value = TIM_GetCounter(TIM4);

	uint16_t counter_value = TIM_GetCounter(TIM4) - start_value;

	while(counter_value < usTime)
	{
		counter_value = TIM_GetCounter(TIM4) - start_value;
	}

//    static uint16_t start_value;
//
//	static uint32_t old_counter;
//
//    static uint16_t n = 0;
//    static uint16_t i = 0;
//    static uint32_t counter;
//
//    start_value = TIM_GetCounter(TIM4);
//
//    if( usTime > 0xFFFF )
//    {
//    	counter = start_value + usTime;
//
//        n = counter/0xFFFF;
//
//        counter = counter-n*0xFFFF;
//
//        old_counter = TIM_GetCounter(TIM4);
//
//        while( i < n || counter > TIM_GetCounter(TIM4) )
//        {
//        	if(old_counter > TIM_GetCounter(TIM4))
//        	{
//        		i++;
//        	}
//
//        	old_counter = TIM_GetCounter(TIM4);
//        }
//    }
//    else
//    {
//    	uint16_t counter_value = TIM_GetCounter(TIM4) - start_value;
//
//    	while(counter_value<usTime)
//    	{
//    		counter_value = TIM_GetCounter(TIM4) - start_value;
//    	}
//    }
}

/**
  * @brief  Tests if its a splitter
  * @param  node_type
  * @retval None
  */
ErrorStatus is_splitter(uint8_t type_test)
{
	if((0x0F & type_test) == 0x08)
	{
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}

/**
  * @brief  Tests if it is a joint
  * @param  node_type
  * @retval None
  */
ErrorStatus is_joint(uint8_t type_test)
{
	if((0x0F & type_test) == 0x04)
	{
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}
