/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN 配置文件
  ******************************************************************************
  * @attention
  *
  * 版权�?�? (c) 2024 STMicroelectronics�?
  * 保留�?有权利�??
  *
  * 本软件根据根目录中的 LICENSE 文件中的条款进行许可�?
  * 如果软件未随�? LICENSE 文件，则�? "AS IS" 提供�?
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"

CAN_HandleTypeDef hcan1;   // CAN1 句柄

/* 定义接收和发送的消息头结构体 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t CAN_TxData[8];
uint8_t CAN_RxData[8];

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
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
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
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*
// CAN1 中断服务函数
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1); // 调用 HAL 库的中断处理函数
}
*/

// CAN 接收完成回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  int i = 0;
  if (hcan->Instance == CAN1)
  {
    // �? FIFO0 中获取消�?
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_RxData) != HAL_OK)
    {
      // 接收错误处理
      Error_Handler();
    }

    i = RxHeader.StdId - 0x201;

    switch(RxHeader.StdId){
    case 0x201:
    case 0x202:
    case 0x203:
    case 0x204:
      if (moto_chassis[i].msg_cnt < 50)
        get_moto_offset(&moto_chassis[i], hcan);		// �?始的时�?�读出的是偏置�??
      printf("Motor %d initializing...\n", i);
      if (moto_chassis[i].msg_cnt == 50)
      {
        get_moto_measure(&moto_chassis[i], hcan);
        moto_chassis[i].total_angle -= moto_chassis[i].offset_angle;	// 实际的位�? = 读到的位�? - 偏置�?
      }
      else
        get_moto_measure(&moto_chassis[i], hcan);
      moto_chassis[i].msg_cnt++;
      printf("Motor %d running...\n", i);
      break;
    default:break;
    }
  }

}

// CAN 发�?�一组数�?(固定格式:ID�?0X12,标准�?,数据�?)
// len:数据长度(�?大为8)
// msg:数据指针,�?大为8个字�?.
// 返回�?:0,成功;
//       其他,失败;
uint8_t CAN1_Send_Msg(uint8_t* msg, uint8_t len)
{
  uint32_t TxMailbox;

  TxHeader.StdId = 0x12;        // 标准标识�?
  TxHeader.IDE = CAN_ID_STD;    // 使用标准�?
  TxHeader.RTR = CAN_RTR_DATA;  // 数据�?
  TxHeader.DLC = len;
  TxHeader.TransmitGlobalTime = DISABLE;

  for (uint8_t i = 0; i < len; i++)
    CAN_TxData[i] = msg[i];

  if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_TxData, &TxMailbox) != HAL_OK)
    return 1;     // 发�?�失�?

  return 0;       // 发�?�成�?
}

// CAN 接收数据查询
// buf:数据缓存�?;
// 返回�?:0,无数据被收到;
//       其他,接收的数据长�?;
uint8_t CAN1_Receive_Msg(uint8_t *buf)
{
  if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0)
    return 0; // 无数�?

  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, CAN_RxData) != HAL_OK)
    return 0; // 接收失败

  for(uint8_t i = 0; i < RxHeader.DLC; i++)
    buf[i] = CAN_RxData[i];

  return RxHeader.DLC;
}

void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
  //can1 &can2 use same filter config
  CAN_FilterTypeDef		CAN_FilterConfigStructure;

  //	CAN_FilterConfigStructure.FilterNumber = 0;
  CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
  CAN_FilterConfigStructure.FilterIdLow = 0x0000;
  CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
  CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
  CAN_FilterConfigStructure.FilterBank = 14;//can1(0-13)和can2(14-27)分别得到�?半的filter
  CAN_FilterConfigStructure.FilterActivation = ENABLE;

  if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
  {
    //		while(1); //show error!
  }
}
/* USER CODE END 1 */
