/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   此文件包含了 usart.c 文件中函数的原型声明以及相关变量的外部声明。
  ******************************************************************************
  * @attention
  *
  * 版权所有 (c) 2024 STMicroelectronics。
  * 保留所有权利。
  *
  * 本软件根据 LICENSE 文件中的条款许可使用。
  * 如果没有 LICENSE 文件，本软件按“原样”提供。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含 ---------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/* 在此处添加其他需要的头文件 */
/* USER CODE END Includes */

/* 私有定义 -----------------------------------------------------------------*/
/* 定义缓冲区大小，如果已经在其他地方定义，可以注释掉以下定义 */
#ifndef USART_REC_LEN
#define USART_REC_LEN  256  // 根据需要调整大小
#endif

#ifndef RXBUFFERSIZE
#define RXBUFFERSIZE  128  // 根据需要调整大小
#endif
/* USER CODE BEGIN Private defines */
/* 在此处添加其他私有定义 */
/* USER CODE END Private defines */

/* 外部变量声明 ------------------------------------------------------------*/

/* UART 句柄 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/* DMA 句柄 */
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN EXTERN_Variables */

/* USART1 相关变量 */
extern uint8_t USART1_RX_BUF[USART_REC_LEN]; // USART1 接收缓冲区
extern uint16_t USART1_RX_STA;               // USART1 接收状态
extern uint8_t aRxBuffer1[RXBUFFERSIZE];     // USART1 HAL 接收缓冲区
extern UART_HandleTypeDef UART1_Handler;     // USART1 UART句柄

/* USART2 相关变量 */
extern uint8_t USART2_RX_BUF[USART_REC_LEN]; // USART2 接收缓冲区
extern uint16_t USART2_RX_STA;               // USART2 接收状态
extern uint8_t aRxBuffer2[RXBUFFERSIZE];     // USART2 HAL 接收缓冲区
extern UART_HandleTypeDef UART2_Handler;     // USART2 UART句柄

/* USART3 相关变量 */
extern uint8_t USART3_RX_BUF[USART_REC_LEN]; // USART3 接收缓冲区
extern uint16_t USART3_RX_STA;               // USART3 接收状态
extern uint8_t aRxBuffer3[RXBUFFERSIZE];     // USART3 HAL 接收缓冲区
extern UART_HandleTypeDef UART3_Handler;     // USART3 UART句柄

/* USER CODE END EXTERN_Variables */

/* 函数原型声明 -------------------------------------------------------------*/
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle);
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

int Uart3SendChar(uint8_t ch);

/* USER CODE BEGIN Prototypes */
/* 在此处添加其他函数原型 */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
