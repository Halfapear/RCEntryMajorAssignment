/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN 配置文件
  ******************************************************************************
  * @attention
  *
  * 版权所有 (c) 2024 STMicroelectronics。
  * 保留所有权利。
  *
  * 本软件根据根目录中的 LICENSE 文件中的条款进行许可。
  * 如果软件未随附 LICENSE 文件，则按 "AS IS" 提供。
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

/* CAN1 初始化函数 */
void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;  // 修改为 ENABLE，自动重传
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 配置 CAN 过滤器，接收所有消息 */
  CAN_FilterTypeDef CAN_FilterConfig;
  CAN_FilterConfig.FilterBank = 0;
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfig.FilterIdHigh = 0x0000;
  CAN_FilterConfig.FilterIdLow = 0x0000;
  CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfig.FilterMaskIdLow = 0x0000;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_FilterConfig.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* 启动 CAN */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 激活接收中断 */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
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
    // 从 FIFO0 中获取消息
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

// CAN 发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
// len:数据长度(最大为8)
// msg:数据指针,最大为8个字节.
// 返回值:0,成功;
//       其他,失败;
uint8_t CAN1_Send_Msg(uint8_t* msg, uint8_t len)
{
  uint32_t TxMailbox;

  TxHeader.StdId = 0x12;        // 标准标识符
  TxHeader.IDE = CAN_ID_STD;    // 使用标准帧
  TxHeader.RTR = CAN_RTR_DATA;  // 数据帧
  TxHeader.DLC = len;
  TxHeader.TransmitGlobalTime = DISABLE;

  for (uint8_t i = 0; i < len; i++)
    CAN_TxData[i] = msg[i];

  if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_TxData, &TxMailbox) != HAL_OK)
    return 1;     // 发送失败

  return 0;       // 发送成功
}

// CAN 接收数据查询
// buf:数据缓存区;
// 返回值:0,无数据被收到;
//       其他,接收的数据长度;
uint8_t CAN1_Receive_Msg(uint8_t *buf)
{
  if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0)
    return 0; // 无数据

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
  CAN_FilterConfigStructure.FilterBank = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
  CAN_FilterConfigStructure.FilterActivation = ENABLE;

  if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
  {
    //		while(1); //show error!
  }
}
/* USER CODE END 1 */
