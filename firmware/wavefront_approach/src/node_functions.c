/*
 * node_functions.c
 *
 *  Created on: Jan 12, 2012
 *      Author: glausero
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "node_functions.h"
#include "platform_config.h"
#include "STM32vldiscovery.h"

/* defines */

/* variables -----------------------------------------------------------------*/
uint16_t CCR1Val = 100;
uint16_t CCR2Val = 500;
extern uint8_t ID;
extern uint8_t TYPE;
extern uint8_t not_interrupted;
__IO uint32_t TimingDelay;
NVIC_InitTypeDef NVIC_InitStructure;

/* variables */
uint16_t topo_txs[]={TOPO_TX_1, TOPO_TX_2, TOPO_TX_3, TOPO_TX_4, TOPO_TX_5};

/******************************************************************************/
/*            Functions for a node to work				                      */
/******************************************************************************/

/**
  * @brief  This function turns on or off leds
  * @param  None
  * @retval None
  */
void set_leds(uint16_t rate_r, uint16_t rate_g, uint16_t rate_b)
{
	static uint16_t counter = 0;

	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    counter = TIM_GetCounter(TIM3);

	if(rate_r>1)
	{
		//TODO: when red led is available
	}
	else
	{

	}

	if(rate_g>1)
	{
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;

		CCR2Val = rate_g;

	    TIM_SetCompare4(TIM3, (counter + CCR2Val)%10000);

	    GPIO_Init( GPIOC, &GPIO_InitStructure );
	}
	else
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		if(rate_g==0)
		{
			GPIOC->BRR = GPIO_Pin_9;
		}
		else
		{
			GPIOC->BSRR = GPIO_Pin_9;
		}
	}

	if(rate_b>1)
	{
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;

		CCR1Val = rate_b;

	    TIM_SetCompare3(TIM3, (counter + CCR1Val)%10000);

	    GPIO_Init( GPIOC, &GPIO_InitStructure );
	}
	else
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		if(rate_b==0)
		{
			GPIOC->BRR = GPIO_Pin_8;
		}
		else
		{
			GPIOC->BSRR = GPIO_Pin_8;
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
	else
	{
		n = 1;
	}

	// RCC
	RCC_APB2PeriphClockCmd(SPI_MASTER_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

	// GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure pin 9: output*/
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

/**
  * @brief  Sends a pulse to pass on i2c master
  * @param  None
  * @retval None
  */
void send_topo(uint16_t PIN)
{
	// set on high
	GPIOA->BSRR = PIN;

	// wait
	//Delay(5);
	//Timeout = 0x00FF;
	//while(Timeout-- != 0){}
	wait(20);

	// set on low
	GPIOA->BRR = PIN;
}

/**
  * @brief  Configures the led to work in output compare mode
  * @param  None
  * @retval None
  */
void led_config()
{
	// RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

    // GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // Alt Function - Push Pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );        // Map TIM3_CH3 to GPIOC.Pin8 and GPIOC.Pin9

    // NVIC
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM3 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // TIM
    /* Compute the prescaler value */
    //uint16_t PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;

    /* Time base configuration */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 10000;
    TIM_TimeBaseStructure.TIM_Prescaler = 24000;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* Output Compare Toggle Mode configuration: Channel1 */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR1Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

    /* Output Compare Toggle Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR2Val;

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);

    set_leds(0,0,0);

    /* TIM enable counter */
    TIM_Cmd(TIM3, ENABLE);

    TIM_ITConfig(TIM3, TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	//GPIOC->BRR = GPIO_Pin_8 | GPIO_Pin_9;
}

/**
  * @brief  Set up time for wait function
  * @param  blabla
  * @retval None
  */
void waiter_config()
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM16 | RCC_APB2Periph_AFIO  | RCC_APB2Periph_GPIOB ,  ENABLE );

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 24;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM16, ENABLE);
}

/**
  * @brief  microsecond wait function
  * @param  blabla
  * @retval None
  */
void wait(uint16_t usTime)
{
    static uint16_t start_value;

    start_value = TIM_GetCounter(TIM16);

    if(usTime>0xFFFF)
    {
    	uint32_t counter = start_value + usTime;

        uint16_t old_counter;
        uint16_t n = 0;
        uint16_t i = 0;

        n = counter/0xFFFF;

        counter = counter-n*0xFFFF;

        while((counter>TIM_GetCounter(TIM16))||(i<n))
        {
        	if(old_counter-TIM_GetCounter(TIM16)<0)
        	{
        		i++;
        	}

        	old_counter = TIM_GetCounter(TIM16);
        }
    }
    else
    {
    	uint16_t counter_value = TIM_GetCounter(TIM16) - start_value;

    	while(counter_value<usTime)
    	{
    		counter_value = TIM_GetCounter(TIM16) - start_value;
    	}
    }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
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
