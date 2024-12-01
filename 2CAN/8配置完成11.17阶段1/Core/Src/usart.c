/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "main.h"

/*
int Uart3SendChar(u8 ch)
{
  while((USART3->ISR&0X40)==0);//循环发�??,直到发�?�完�?
  USART3->TDR=(u8)ch;
  return ch;
}
*/

u8 USART1_RX_BUF[USART_REC_LEN]; //接收缓冲,�?大USART_REC_LEN个字�?.
//接收状�??
//bit15�?	接收完成标志
//bit14�?	接收�?0x0d
//bit13~0�?	接收到的有效字节数目
u16 USART1_RX_STA = 0; //接收状�?�标�?
u8 aRxBuffer1[RXBUFFERSIZE];		  //HAL库使用的串口接收缓冲
UART_HandleTypeDef UART1_Handler; //UART句柄

//串口2
u8 USART2_RX_BUF[USART_REC_LEN];     //接收缓冲,�?大USART_REC_LEN个字�?.
u16 USART2_RX_STA=0;       //接收状�?�标�?
u8 aRxBuffer2[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
UART_HandleTypeDef UART2_Handler; //UART句柄

//串口3
u8  USART3_RX_BUF[USART_REC_LEN]; //接收缓冲,�?大USART_REC_LEN个字�?.末字节为换行�?
u16 USART3_RX_STA;         		//接收状�?�标�?
UART_HandleTypeDef UART3_Handler; //UART句柄
u8 aRxBuffer3[RXBUFFERSIZE];//HAL库USART接收Buffer
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int i;
	if (huart->Instance == USART1) //如果是串�?1
	{

	}

	if(huart->Instance==USART2)//如果是串�?2
	{

	}

	while(huart->Instance==USART3)//如果是串�?3
	{
		USART3_RX_BUF[USART3_RX_STA] = aRxBuffer3[0];
		if (USART3_RX_STA == 0 && USART3_RX_BUF[USART3_RX_STA] != 0xAA) break; //帧头不对，丢�?
		if (USART3_RX_STA == 1 && USART3_RX_BUF[USART3_RX_STA] != 0x55) break; //帧头不对，丢�?
		USART3_RX_STA++;
		if (USART3_RX_STA > USART_REC_LEN) USART3_RX_STA = 0;  ///接收数据错误,重新�?始接�?

		if (USART3_RX_STA == 25)	//接受完一帧数�?
		{
			analyseData(&USART3_RX_BUF[0]);
			for (i = 0; i<25; i++)
			{
				USART3_RX_BUF[i] = 0;
			}
			USART3_RX_STA = 0;

		}
		break;
	}
}

/*
//串口1中断服务程序
void USART1_IRQHandler(void)
{
	u32 timeout = 0;
	u32 maxDelay = 0x1FFFF;
#if SYSTEM_SUPPORT_OS //使用OS
	OSIntEnter();
#endif

	HAL_UART_IRQHandler(&UART1_Handler); //调用HAL库中断处理公用函�?

	timeout = 0;
	while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY) //等待就绪
	{
		timeout++; ////超时处理
		if (timeout > maxDelay)
			break;
	}

	timeout = 0;
	while (HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer1, RXBUFFERSIZE) != HAL_OK) //�?次处理完成之后，重新�?启中断并设置RxXferCount�?1
	{
		timeout++; //超时处理
		if (timeout > maxDelay)
			break;
	}
#if SYSTEM_SUPPORT_OS //使用OS
	OSIntExit();
#endif
}


//串口2中断服务程序
void USART2_IRQHandler(void)
{
	u32 timeout=0;
    u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();
#endif

	HAL_UART_IRQHandler(&UART2_Handler);	//调用HAL库中断处理公用函�?

	timeout=0;
    while (HAL_UART_GetState(&UART2_Handler)!=HAL_UART_STATE_READY)//等待就绪
	{
        timeout++;////超时处理
        if(timeout>maxDelay) break;
	}

	timeout=0;
	while(HAL_UART_Receive_IT(&UART2_Handler,(u8 *)aRxBuffer2, RXBUFFERSIZE)!=HAL_OK)//�?次处理完成之后，重新�?启中断并设置RxXferCount�?1
	{
        timeout++; //超时处理
        if(timeout>maxDelay) break;
	}
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();
#endif
}

//串口3中断服务程序
void USART3_IRQHandler(void)
{
	u32 timeout=0;
    u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();
#endif

	HAL_UART_IRQHandler(&UART3_Handler);	//调用HAL库中断处理公用函�?

	timeout=0;
    while (HAL_UART_GetState(&UART3_Handler)!=HAL_UART_STATE_READY)//等待就绪
	{
        timeout++;////超时处理
        if(timeout>maxDelay) break;
	}

	timeout=0;
	while(HAL_UART_Receive_IT(&UART3_Handler,(u8 *)aRxBuffer3, RXBUFFERSIZE)!=HAL_OK)//�?次处理完成之后，重新�?启中断并设置RxXferCount�?1
	{
        timeout++; //超时处理
        if(timeout>maxDelay) break;
	}
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();
#endif
}
*/
int Uart3SendChar(u8 ch)
{
	while ((USART3->SR & 0x40) == 0); // 循环发�?�，直到发�?�完�?
	USART3->DR = (u8)ch;
	return ch;
}


/* USER CODE END 1 */
