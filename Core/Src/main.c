/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/*** USER CODE END Header */
/*** Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "bsp_led.h"
#include "bep_key.h"
#include "core_delay.h"
#include "bsp_debug_usart.h"
#include "retarget.h"

/*** FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"

/**************************** 任务句柄 ********************************/
/***
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t LED_Task_Handle = NULL;
static TaskHandle_t KEY_Task_Handle = NULL;

UART_HandleTypeDef huart1;


/***
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void); /*** 用于创建任务 */

static void LED_Task(void* pvParameters); /*** LED_Task任务实现*/
static void KEY_Task(void* pvParameters); /*** KEY_Task任务实现*/

static void BSP_Init(void); /*** 用于初始化板载相关资源*/

void MX_FREERTOS_Init(void);

/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{
    BaseType_t xReturn = pdPASS;

    /** 开发板硬件初始化 */
    BSP_Init();

    RetargetInit(&huart1);



    /** 创建AppTaskCreate任务 */
    xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                          (const char*    )"AppTaskCreate",/* 任务名字 */
                          (uint16_t       )512,  /* 任务栈大小 */
                          (void*          )NULL,/* 任务入口函数参数 */
                          (UBaseType_t    )1, /* 任务的优先级 */
                          (TaskHandle_t*  )&AppTaskCreate_Handle);
    /*** 启动任务调度 */
    if (pdPASS == xReturn)
        vTaskStartScheduler(); /*** 启动任务，开启调度 */
    else
        return -1;

    while (1);    /*** 正常不会执行到这里 */
}

/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
    BaseType_t xReturn = pdPASS; /*** 定义一个创建信息返回值，默认为pdPASS */

    taskENTER_CRITICAL(); /*** 进入临界区 */

    /*** 创建LED_Task任务 */
    xReturn = xTaskCreate((TaskFunction_t)  LED_Task  ,/** 任务入口函数 */
                          (const char*   ) "LED_Task" ,/** 任务名字*/
                          (uint16_t      )  512       ,/** 任务栈大小*/
                          (void*         )  NULL      ,/** 任务入口函数参数*/
                          (UBaseType_t   )  2         ,/** 任务的优先级*/
                          (TaskHandle_t* )  &LED_Task_Handle);/** 任务控制块指针*/
    if(pdPASS == xReturn)

    /*** 创建KEY_Task任务 */
    xReturn =  xTaskCreate((TaskFunction_t ) KEY_Task   ,/** 任务入口函数 */
                           (const char*    ) "KEY_Task" ,/** 任务名字*/
                           (uint16_t       ) 512        ,/** 任务栈大小*/
                           (void*          ) NULL       ,/** 任务入口函数参数*/
                           (UBaseType_t    ) 3          ,/** 任务的优先级*/
                           (TaskHandle_t*  ) &KEY_Task_Handle);   /** 任务控制块指针*/
    if(pdPASS == xReturn)

    vTaskDelete(AppTaskCreate_Handle);

    taskEXIT_CRITICAL();
}

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：
  * @ 返回值  ： 无
  ********************************************************************/
static void LED_Task(void* parameter)
{
    while (1)
    {
        LED1_ON
        printf("LED_Task Running,LED1_ON\\r\\n");
        vTaskDelay(500);   /* 延时500个tick */

        LED1_OFF;
        printf("LED_Task Running,LED1_OFF\r\n");
        vTaskDelay(500);   /* 延时500个tick */
    }
}

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：
  * @ 返回值  ： 无
  ********************************************************************/
static void KEY_Task(void* parameter)
{
    while (1)
    {
        if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON )
        {/* K1 被按下 */
            vTaskSuspend(LED_Task_Handle);/* 挂起LED任务 */
        }
        if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON )
        {/* K2 被按下 */
            vTaskResume(LED_Task_Handle);/* 恢复LED任务！ */
        }
        vTaskDelay(20);/* 延时20个tick */
    }
}

/***********************************************************************
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 板级外设初始化，所有板子上的初始化均可放在这个函数里面
  * @ 参数    ：
  * @ 返回值  ： 无
  *********************************************************************/
static void BSP_Init(void)
{
//    HAL_Init();
    /* 系统时钟初始化成400MHz */
    SystemClock_Config();

    /* 初始化SysTick */
    HAL_SYSTICK_Config( HAL_RCC_GetSysClockFreq() / configTICK_RATE_HZ );

    /* LED 端口初始化 */
    LED_GPIO_Config();

    /* usart 端口初始化 */
    DEBUG_USART_Config();

    /*按键初始化*/
    Key_GPIO_Config();

}

/**
  * @brief  System Clock 配置
  *         system Clock 配置如下:
	*            System Clock source  = PLL (HSE)
	*            SYSCLK(Hz)           = 480000000 (CPU Clock)
	*            HCLK(Hz)             = 240000000 (AXI and AHBs Clock)
	*            AHB Prescaler        = 2
	*            D1 APB3 Prescaler    = 2 (APB3 Clock  120MHz)
	*            D2 APB1 Prescaler    = 2 (APB1 Clock  120MHz)
	*            D2 APB2 Prescaler    = 2 (APB2 Clock  120MHz)
	*            D3 APB4 Prescaler    = 2 (APB4 Clock  120MHz)
	*            HSE Frequency(Hz)    = 25000000
	*            PLL_M                = 5
	*            PLL_N                = 192
	*            PLL_P                = 2
	*            PLL_Q                = 4
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    HAL_StatusTypeDef ret = HAL_OK;
    /*使能供电配置更新 */
    MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
    /* 当器件的时钟频率低于最大系统频率时，电压调节可以优化功耗，
           关于系统频率的电压调节值的更新可以参考产品数据手册。  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    /* 启用HSE振荡器并使用HSE作为源激活PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 160;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if(ret != HAL_OK)
    {
        while(1) { ; }
    }
    /* 选择PLL作为系统时钟源并配置总线时钟分频器 */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK  | \
                                   RCC_CLOCKTYPE_HCLK    | \
                                   RCC_CLOCKTYPE_D1PCLK1 | \
                                   RCC_CLOCKTYPE_PCLK1   | \
                                   RCC_CLOCKTYPE_PCLK2   | \
                                   RCC_CLOCKTYPE_D3PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
    if(ret != HAL_OK)
    {
        while(1) { ; }
    }
}

