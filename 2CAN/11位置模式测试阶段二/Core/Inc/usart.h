/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define USART_REC_LEN  256   // 接收缓冲区大�??
#define RXBUFFERSIZE   1     // HAL UART 接收缓冲区大�??
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

// USART 接收状�?�标�???
  extern uint8_t USART1_RX_BUF[USART_REC_LEN]; // USART1 接收缓冲�?
  extern uint8_t USART2_RX_BUF[USART_REC_LEN]; // USART2 接收缓冲�?
  extern uint8_t USART3_RX_BUF[USART_REC_LEN]; // USART3 接收缓冲�?

  extern uint16_t USART1_RX_STA; // USART1 接收状�??
  extern uint16_t USART2_RX_STA; // USART2 接收状�??
  extern uint16_t USART3_RX_STA; // USART3 接收状�??

  extern uint8_t aRxBuffer1[RXBUFFERSIZE]; // USART1 HAL 接收缓冲�?
  extern uint8_t aRxBuffer2[RXBUFFERSIZE]; // USART2 HAL 接收缓冲�?
  extern uint8_t aRxBuffer3[RXBUFFERSIZE]; // USART3 HAL 接收缓冲�?

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

// USART3 发�?�单个字符函�???
int Uart3SendChar(uint8_t ch);

// UART 接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// 中断处理函数声明
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

// 数据解析函数 (�??? c 文件中定�???)
void analyseData(uint8_t *data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

