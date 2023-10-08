/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   使用串口1，重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  */
#include "bsp_debug_usart.h"

UART_HandleTypeDef UartHandle1;
extern uint8_t ucTemp;
/**
 * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
 * @param  无
 * @retval 无
 */
void DEBUG_USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

    DEBUG_USART_RX_GPIO_CLK_ENABLE();
    DEBUG_USART_TX_GPIO_CLK_ENABLE();

    /* 配置串口1时钟源*/
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
    /* 使能串口1时钟 */
    DEBUG_USART_CLK_ENABLE();

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = DEBUG_USART_TX_AF;
    HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
    GPIO_InitStruct.Alternate = DEBUG_USART_RX_AF;
    HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置串DEBUG_USART 模式 */
    UartHandle1.Instance = DEBUG_USART;
    UartHandle1.Init.BaudRate = 115200;
    UartHandle1.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle1.Init.StopBits = UART_STOPBITS_1;
    UartHandle1.Init.Parity = UART_PARITY_NONE;
    UartHandle1.Init.Mode = UART_MODE_TX_RX;
    UartHandle1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle1.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    UartHandle1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&UartHandle1);

    /*串口1中断初始化 */
    HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 6, 0);
    HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ);
    /*配置串口接收中断 */
    __HAL_UART_ENABLE_IT(&UartHandle1,UART_IT_RXNE);
}


/*****************  发送字符串 **********************/
void Usart_SendString(uint8_t *str)
{
    unsigned int k=0;
    do
    {
        HAL_UART_Transmit( &UartHandle1,(uint8_t *)(str + k) ,1,1000);
        k++;
    } while(*(str + k)!='\0');

}
///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
    /* 发送一个字节数据到串口DEBUG_USART */
    HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 1000);

    return (ch);
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{

    int ch;
    HAL_UART_Receive(&UartHandle1, (uint8_t *)&ch, 1, 1000);
    return (ch);
}
/*********************************************END OF FILE**********************/
