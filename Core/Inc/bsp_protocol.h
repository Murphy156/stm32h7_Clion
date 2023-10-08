#ifndef CPROJECT_BSP_PROTOCOL_H
#define CPROJECT_BSP_PROTOCOL_H

#include "stm32h7xx.h"
#include <stdio.h>

/** 串口波特率 */
#define USART_BAUDRATE					115200

/** 数据头结构体 */
typedef __packed struct
{
    uint16_t head;    	    /** 包头 */
    uint8_t cmd;      	    /** 命令 */
    uint32_t para;			/** 参数 */
    uint8_t parity;			/** 校验和 */
}packet_head_t;

/** 引脚定义 */
/*************************************/
#define USART_SEL									USART1
#define USART_CLK_ENABLE()							__USART1_CLK_ENABLE();

#define USART_RX_GPIO_PORT                			GPIOB
#define USART_RX_GPIO_CLK_ENABLE()        			__HAL_RCC_GPIOB_CLK_ENABLE()
#define USART_RX_PIN                      			GPIO_PIN_6
#define USART_RX_AF                       			GPIO_AF7_USART1

#define USART_TX_GPIO_PORT                			GPIOB
#define USART_TX_GPIO_CLK_ENABLE()       			__HAL_RCC_GPIOB_CLK_ENABLE()
#define USART_TX_PIN                      			GPIO_PIN_7
#define USART_TX_AF                       			GPIO_AF7_USART1

#define USART_IRQHandler                  			USART1_IRQHandler
#define USART_IRQ                 		    		USART1_IRQn

/** 数据接收缓冲区大小 */
#define		min_protocol_length	 8
#define		max_protocol_length	 16
#define	 max_length		(min_protocol_length * max_protocol_length) /** 乘法一定要加括号，16*8 = 128 */

/** 帧头 */
#define PACK_HEADER						0x55AA

/** 指令 （上位机 -> 下位机）*/
#define EnablePump1_CMD				0x01			/** 使能泵1的指令,步进1 */
#define DisablePump1_CMD			0x02			/** 失能泵1的指令,步进1 */
#define EnablePump2_CMD				0x03			/** 使能泵2的指令,步进2 */
#define DisablePump2_CMD			0x04			/** 失能泵2的指令,步进2 */
#define EnablePump3_CMD				0x05			/** 使能泵3的指令,步进3 */
#define DisablePump3_CMD			0x06			/** 失能泵3的指令,步进3 */
#define EnablePump4_CMD				0x07			/** 使能泵4的指令,步进4 */
#define DisablePump4_CMD			0x08			/** 失能泵4的指令,步进4 */
#define EnablePump5_CMD				0x09			/** 使能泵5的指令,BLDC */
#define DisablePump5_CMD			0x0A			/** 失能泵5的指令,BLDC */

#define ChangePump1Speed_CMD  0x11			        /** 改变泵1转速的指令,步进1 */
#define ChangePump2Speed_CMD  0x12			        /** 改变泵2转速的指令,步进2 */
#define ChangePump3Speed_CMD  0x13			        /** 改变泵3转速的指令,步进3 */
#define ChangePump4Speed_CMD  0x14			        /** 改变泵4转速的指令,步进4 */
#define ChangePump5Speed_CMD  0x15			        /** 改变泵5转速的指令,BLDC */

/* 空指令 */
#define CMD_NONE             	0xFF     // 空指令

void USART_Config(void);
uint8_t PushArr(uint8_t *arr,uint8_t data);
uint8_t PopArr(uint8_t *arr ,uint8_t *data);
extern uint8_t	data_buff[max_length];
extern UART_HandleTypeDef UartHandle;
uint8_t calculateChecksum(unsigned char *data);


int8_t receiving_process(void);
uint16_t calculateCRC16(uint8_t arr[7]);
static uint8_t recvBuff_Find_Header(void);
uint16_t mergeBytesToUint16(uint8_t data[2]);
uint32_t mergeParametersToUint32(void);
static uint8_t protocol_Check_header_CRC(void);
void parseData(uint32_t input,uint8_t motor_chose);
void parseBLDC(uint32_t input);


#endif //CPROJECT_BSP_PROTOCOL_H
