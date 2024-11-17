/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"

CAN_HandleTypeDef   CAN1_Handler;   //CAN1句柄
CanTxMsgTypeDef     TxMessage;      //发送消息
CanRxMsgTypeDef     RxMessage;      //接收消息
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//CAN中断服务函数
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&CAN1_Handler);//此函数会调用CAN_Receive_IT()接收数据
}

//CAN中断处理过程
//此函数会被CAN_Receive_IT()调用
//hcan:CAN句柄
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  int i = 0;
  if (hcan->Instance == CAN1)
  {
    //CAN_Receive_IT()函数会关闭FIFO0消息挂号中断，因此我们需要重新打开
    __HAL_CAN_ENABLE_IT(&CAN1_Handler, CAN_IT_FMP0); //重新开启FIF00消息挂号中断
    i = hcan->pRxMsg->StdId - 0x201;

    switch(hcan->pRxMsg->StdId){
    case 0x201:
    case 0x202:
    case 0x203:
    case 0x204:
      if (moto_chassis[i].msg_cnt < 50)
        get_moto_offset(&moto_chassis[i], hcan);		// 开始的时候读出的是偏置值
      if (moto_chassis[i].msg_cnt == 50)
      {
        get_moto_measure(&moto_chassis[i], hcan);
        moto_chassis[i].total_angle -= moto_chassis[i].offset_angle;	// 实际的位置 = 读到的位置 - 偏置值
      }
      else
        get_moto_measure(&moto_chassis[i], hcan);
      moto_chassis[i].msg_cnt++;
      break;
    default:break;
    }
  }

}


//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
//len:数据长度(最大为8)
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{
  u16 i=0;
  CAN1_Handler.pTxMsg->StdId=0X12;        //标准标识符
  CAN1_Handler.pTxMsg->ExtId=0x12;        //扩展标识符(29位)
  CAN1_Handler.pTxMsg->IDE=CAN_ID_STD;    //使用标准帧
  CAN1_Handler.pTxMsg->RTR=CAN_RTR_DATA;  //数据帧
  CAN1_Handler.pTxMsg->DLC=len;
  for(i=0;i<len;i++)
    CAN1_Handler.pTxMsg->Data[i]=msg[i];
  if(HAL_CAN_Transmit(&CAN1_Handler,10)!=HAL_OK) return 1;     //发送
  return 0;
}

//can口接收数据查询
//buf:数据缓存区;
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{
  u32 i;
  if(HAL_CAN_Receive(&CAN1_Handler,CAN_FIFO0,0)!=HAL_OK) return 0;//接收数据
  for(i=0;i<CAN1_Handler.pRxMsg->DLC;i++)
    buf[i]=CAN1_Handler.pRxMsg->Data[i];
  return CAN1_Handler.pRxMsg->DLC;
}

/* USER CODE END 1 */
