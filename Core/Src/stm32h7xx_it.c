/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#include "stm32h7xx_it.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
/** USER Includes */
#include "bsp_debug_usart.h"
#include "bsp_led.h"
#include "bsp_protocol.h"
#include "bsp_bldc_tim.h"

//extern TIM_HandleTypeDef htim6;


/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

///**
//  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
//  */
//void TIM6_DAC_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
//
//  /* USER CODE END TIM6_DAC_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim6);
//  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
//
//  /* USER CODE END TIM6_DAC_IRQn 1 */
//}

/** USER CODE BEGIN 1 */
/**
* @brief This function handles System tick timer.
*/
extern void xPortSysTickHandler(void);
//systick中断服务函数
void SysTick_Handler(void)
{
    #if (INCLUDE_xTaskGetSchedulerState  == 1 )
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
    #endif  /* INCLUDE_xTaskGetSchedulerState */
        xPortSysTickHandler();
    #if (INCLUDE_xTaskGetSchedulerState  == 1 )
        }
    #endif  /* INCLUDE_xTaskGetSchedulerState */
}
/** USER CODE END 2 */

/** USER CODE BEGIN 2 */
void USART_IRQHandler(void)
{
    uint8_t data[1];

    if(__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE) != RESET)
    {
        data[0] = UartHandle.Instance->RDR;
        PushArr(data_buff,data[0]);
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_IT_RXNE);
    }
    HAL_UART_IRQHandler(&UartHandle);
    __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);
}
/** USER CODE END 2 */



