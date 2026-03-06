#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"

/* 配置宏定义 - 可根据需要修改 */
#define UART_TX_BUFFER_SIZE        256     // 串口缓冲区大小
#define UART_RX_BUFFER_SIZE        5     // 串口接受缓冲区大小

#define BLOCK_WAITING_TIME      1000

typedef enum
{
	UART_DMA_RX = 0, UART_DMA_ToIdle_RX, UART_IT_RX, UART_IT_ToIdle_RX, UART_Block_RX
	
}	UART_RX_MODE;

typedef enum
{
	UART_DMA_TX = 0, UART_IT_TX, UART_Block_TX
	
}	UART_TX_MODE;

extern void uart_init(UART_HandleTypeDef *huart, uint8_t uart_mode);
extern int my_uart_printf(UART_HandleTypeDef *huart, uint8_t send_mode, const char *format, ...);



#endif
