/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * 本文件中添加了CAN发送函数的错误处理，并修正了发送函数中的变量使用问题。
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#define BSP_CAN_OK  0  // 表示成功状态的宏定义

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
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = ENABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
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

/**
  * @brief  配置CAN1过滤器
  * @retval uint8_t: 返回状态
  */
uint8_t bsp_can1_filter_config(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterActivation = ENABLE;                   // 激活过滤器
    filter.FilterMode = CAN_FILTERMODE_IDMASK;          // 过滤器模式：标识符屏蔽模式
    filter.FilterScale = CAN_FILTERSCALE_32BIT;         // 过滤器位宽：32位
    filter.FilterBank = 0;                              // 过滤器组编号
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;     // 分配到FIFO0
    filter.FilterIdHigh = 0x0000;                       // 标识符高位
    filter.FilterIdLow = 0x0000;                        // 标识符低位
    filter.FilterMaskIdHigh = 0x0000;                   // 屏蔽标识符高位
    filter.FilterMaskIdLow = 0x0000;                    // 屏蔽标识符低位

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        /* 配置过滤器失败，返回错误状态 */
        return 1;
    }
    else
    {
        /* 配置成功，返回成功状态 */
        return BSP_CAN_OK;
    }
}

/**
  * @brief  发送CAN消息
  * @param  id_type: 标识符类型（标准ID或扩展ID）
  * @param  basic_id: 标准ID
  * @param  ex_id: 扩展ID
  * @param  data: 要发送的数据
  * @param  data_len: 数据长度
  * @retval uint8_t: 返回状态
  */
uint8_t bsp_can1_send_msg(uint32_t id_type, uint32_t basic_id, uint32_t ex_id, uint8_t *data, uint32_t data_len)
{
    uint32_t tx_mailbox = 0;
    CAN_TxHeaderTypeDef send_msg_hdr;

    send_msg_hdr.StdId = basic_id;     // 设置标准ID
    send_msg_hdr.ExtId = ex_id;        // 设置扩展ID
    send_msg_hdr.IDE = id_type;        // 设置ID类型
    send_msg_hdr.RTR = CAN_RTR_DATA;   // 设置数据帧
    send_msg_hdr.DLC = data_len;       // 设置数据长度
    send_msg_hdr.TransmitGlobalTime = DISABLE; // 禁用全局时间

    /* 发送CAN消息 */
if (HAL_CAN_AddTxMessage(&hcan1, &send_msg_hdr, data, &tx_mailbox) != HAL_OK)
{
    /* 发送失败，输出错误信息 */
    //printf("CAN1 Message Send Error\r\n");
    return 1;
}
else
{
    /* 发送成功，输出提示信息 */
    printf("CAN1 Message Sent Successfully\r\n");
    return BSP_CAN_OK;
}

}

/* USER CODE END 1 */
