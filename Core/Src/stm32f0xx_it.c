/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

	if (LL_TIM_IsActiveFlag_UPDATE(TIM14)) {
		LL_TIM_ClearFlag_UPDATE(TIM14);
		debug_print(".\n");
	}

//	static int8_t ir_data_bit_cnt = 0;
//	static uint32_t ir_data;
//
//	switch (ir_state)
//	{
//		case IR_IDLE:
//			//default state - waiting for initial signal
//			if(IR_PIN_LOW)
//			{
//				ir_state = IR_BURST;
//			}
//			break;
//		case IR_BURST:
//			if(IR_PIN_HIGH)
//			{
//				//9ms of burst pulses in 38KHz freq
//				if(TCNT1 > COUNT_BURST_MIN && TCNT1 < COUNT_BURST_MAX)
//				{
//					ir_state = IR_GAP;
//				}
//			}
//			break;
//		case IR_GAP:
//			// if GAP 4500us we have data, if GAP 2200us it is a repeat
//			if(IR_PIN_LOW)
//			{
//				//4.5ms space (data)
//				if(TCNT1 > COUNT_GAP_MIN && TCNT1 < COUNT_GAP_MAX)
//				{
//					ir_state = IR_DATA;
////LED_PIN_HIGH;LED_PIN_LOW;
//					ir_data_bit_cnt = 0;
//					ir_data = 0;
//				}
//				// 2.2ms (repeat)
//				else if (TCNT1 > COUNT_GAP_REP_MIN && TCNT1 < COUNT_GAP_REP_MAX)
//				{
//					ir_state = IR_IDLE;
//					ir_data_ready_flag = IR_DATA_READY_FLAG_REPEAT;
//				}
//			}
//			break;
//		case IR_DATA:
//			if(IR_PIN_LOW)
//			{
//				/*
//					we are expecting 32 bit of data
//					8bit address - 8bit inverted address - 8bit data - 8bit inverted data [LSB FIRST]
//					562.5us burst pulse followed by a 562.5us space is counted as a logical 0 (1.125 ms total)
//					562.5us burst pulse followed by a 3x562.5us space is counted as a logical 1 (2.25ms total)
//
//					basika ston diko mou sensora einai to anapodo....
//				*/
//
//				ir_data <<= 1;
//
//				if(TCNT1 > COUNT_LOGICAL0_MIN && TCNT1 < COUNT_LOGICAL0_MAX)
//				{
//
//				}
//				else
//				{
//					//logical 1
//					//ir_data[ir_data_byte_cnt] |= (1<<ir_data_bit_cnt);
//					ir_data |= 0x01;
//				}
//				if (++ir_data_bit_cnt > 31)
//				{
//					//data are ready change state
//					ir_state = IR_FINISH;
//				}
//			}
//			break;
//		case IR_FINISH:
//			//endiamesi katastasi, edw tha kanw jump otan trww timeout h paei kati lathos
//			ir_state = IR_IDLE;
//			ir_rawdata = ir_data;
//			ir_data_ready_flag = IR_DATA_READY_FLAG_DATA;
//			break;
//		default:
//			break;
//	}
//
//	TCNT1 = 0;
  /* USER CODE END TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
