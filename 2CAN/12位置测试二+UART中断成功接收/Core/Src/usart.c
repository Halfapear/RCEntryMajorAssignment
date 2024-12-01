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
  while((USART3->ISR&0X40)==0);//寰幆鍙戦??,鐩村埌鍙戦?佸畬姣?
  USART3->TDR=(u8)ch;
  return ch;
}
*/

u8 USART1_RX_BUF[USART_REC_LEN]; //鎺ユ敹缂撳啿,鏈?澶SART_REC_LEN涓瓧鑺?.
//鎺ユ敹鐘舵??
//bit15锛?	鎺ユ敹瀹屾垚鏍囧織
//bit14锛?	鎺ユ敹鍒?0x0d
//bit13~0锛?	鎺ユ敹鍒扮殑鏈夋晥瀛楄妭鏁扮洰
u16 USART1_RX_STA = 0; //鎺ユ敹鐘舵?佹爣璁?
u8 aRxBuffer1[RXBUFFERSIZE];		  //HAL搴撲娇鐢ㄧ殑涓插彛鎺ユ敹缂撳啿
UART_HandleTypeDef UART1_Handler; //UART鍙ユ焺

//涓插彛2
u8 USART2_RX_BUF[USART_REC_LEN];     //鎺ユ敹缂撳啿,鏈?澶SART_REC_LEN涓瓧鑺?.
u16 USART2_RX_STA=0;       //鎺ユ敹鐘舵?佹爣璁?
u8 aRxBuffer2[RXBUFFERSIZE];//HAL搴撲娇鐢ㄧ殑涓插彛鎺ユ敹缂撳啿
UART_HandleTypeDef UART2_Handler; //UART鍙ユ焺

//涓插彛3
u8  USART3_RX_BUF[USART_REC_LEN]; // 接收缓冲区，最大长度为 USART_REC_LEN
u16 USART3_RX_STA;                // 接收状态标志
UART_HandleTypeDef UART3_Handler; // UART 句柄
u8 aRxBuffer3[RXBUFFERSIZE];      // HAL USART 接收缓冲区
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

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
	if (huart->Instance == USART1) //濡傛灉鏄覆鍙?1
	{
		uint8_t test_data[] = "UART1 Test\r\n";
		HAL_UART_Transmit(&huart1, test_data, sizeof(test_data) - 1, HAL_MAX_DELAY);
		//printf("UART1 IR success");
		/*
		printf("UART1 Received: 0x%02X\r\n", UART1_RX_Buffer[0]);
		// 清空接收缓冲区（可选）
		UART1_RX_Buffer[0] = 0;

		// 重新启动中断接收
		if (HAL_UART_Receive_IT(&huart1, UART1_RX_Buffer, UART1_RX_BUFFER_SIZE) != HAL_OK)
		{
			// 接收重新启动失败处理
			Error_Handler();
		}
		*/
	}

	if(huart->Instance==USART2)//濡傛灉鏄覆鍙?2
	{

	}

	while(huart->Instance==USART3)//濡傛灉鏄覆鍙?3
	{
		USART3_RX_BUF[USART3_RX_STA] = aRxBuffer3[0];
		if (USART3_RX_STA == 0 && USART3_RX_BUF[USART3_RX_STA] != 0xAA)
		{
			printf("AA fail \r\n");
			break; //甯уご涓嶅锛屼涪鎺?
		}
		if (USART3_RX_STA == 1 && USART3_RX_BUF[USART3_RX_STA] != 0x55)
		{
			printf("55 fail \r\n");
			break; //甯уご涓嶅锛屼涪鎺?
		}
		USART3_RX_STA++;
		if (USART3_RX_STA > USART_REC_LEN) USART3_RX_STA = 0;  ///鎺ユ敹鏁版嵁閿欒,閲嶆柊寮?濮嬫帴鏀?

		if (USART3_RX_STA == 25)	//鎺ュ彈瀹屼竴甯ф暟鎹?
		{
			analyseData(&USART3_RX_BUF[0]);
			for (i = 0; i<25; i++)
			{
				USART3_RX_BUF[i] = 0;
			}
			printf("UARTcallbackSuccess \r\n");
			USART3_RX_STA = 0;

		}
		break;
	}
}

/*
//涓插彛1涓柇鏈嶅姟绋嬪簭
void USART1_IRQHandler(void)
{
	u32 timeout = 0;
	u32 maxDelay = 0x1FFFF;
#if SYSTEM_SUPPORT_OS //浣跨敤OS
	OSIntEnter();
#endif

	HAL_UART_IRQHandler(&UART1_Handler); //璋冪敤HAL搴撲腑鏂鐞嗗叕鐢ㄥ嚱鏁?

	timeout = 0;
	while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY) //绛夊緟灏辩华
	{
		timeout++; ////瓒呮椂澶勭悊
		if (timeout > maxDelay)
			break;
	}

	timeout = 0;
	while (HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer1, RXBUFFERSIZE) != HAL_OK) //涓?娆″鐞嗗畬鎴愪箣鍚庯紝閲嶆柊寮?鍚腑鏂苟璁剧疆RxXferCount涓?1
	{
		timeout++; //瓒呮椂澶勭悊
		if (timeout > maxDelay)
			break;
	}
#if SYSTEM_SUPPORT_OS //浣跨敤OS
	OSIntExit();
#endif
}


//涓插彛2涓柇鏈嶅姟绋嬪簭
void USART2_IRQHandler(void)
{
	u32 timeout=0;
    u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//浣跨敤OS
	OSIntEnter();
#endif

	HAL_UART_IRQHandler(&UART2_Handler);	//璋冪敤HAL搴撲腑鏂鐞嗗叕鐢ㄥ嚱鏁?

	timeout=0;
    while (HAL_UART_GetState(&UART2_Handler)!=HAL_UART_STATE_READY)//绛夊緟灏辩华
	{
        timeout++;////瓒呮椂澶勭悊
        if(timeout>maxDelay) break;
	}

	timeout=0;
	while(HAL_UART_Receive_IT(&UART2_Handler,(u8 *)aRxBuffer2, RXBUFFERSIZE)!=HAL_OK)//涓?娆″鐞嗗畬鎴愪箣鍚庯紝閲嶆柊寮?鍚腑鏂苟璁剧疆RxXferCount涓?1
	{
        timeout++; //瓒呮椂澶勭悊
        if(timeout>maxDelay) break;
	}
#if SYSTEM_SUPPORT_OS	 	//浣跨敤OS
	OSIntExit();
#endif
}

//涓插彛3涓柇鏈嶅姟绋嬪簭
void USART3_IRQHandler(void)
{
	u32 timeout=0;
    u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//浣跨敤OS
	OSIntEnter();
#endif

	HAL_UART_IRQHandler(&UART3_Handler);	//璋冪敤HAL搴撲腑鏂鐞嗗叕鐢ㄥ嚱鏁?

	timeout=0;
    while (HAL_UART_GetState(&UART3_Handler)!=HAL_UART_STATE_READY)//绛夊緟灏辩华
	{
        timeout++;////瓒呮椂澶勭悊
        if(timeout>maxDelay) break;
	}

	timeout=0;
	while(HAL_UART_Receive_IT(&UART3_Handler,(u8 *)aRxBuffer3, RXBUFFERSIZE)!=HAL_OK)//涓?娆″鐞嗗畬鎴愪箣鍚庯紝閲嶆柊寮?鍚腑鏂苟璁剧疆RxXferCount涓?1
	{
        timeout++; //瓒呮椂澶勭悊
        if(timeout>maxDelay) break;
	}
#if SYSTEM_SUPPORT_OS	 	//浣跨敤OS
	OSIntExit();
#endif
}
*/
int Uart3SendChar(u8 ch)
{
	while ((USART3->SR & 0x40) == 0); // 寰幆鍙戦?侊紝鐩村埌鍙戦?佸畬姣?
	USART3->DR = (u8)ch;
	return ch;
}


/* USER CODE END 1 */
