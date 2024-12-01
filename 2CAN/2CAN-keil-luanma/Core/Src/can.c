/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * ���ļ��������CAN���ͺ����Ĵ������������˷��ͺ����еı���ʹ�����⡣
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#define BSP_CAN_OK  0  // ��ʾ�ɹ�״̬�ĺ궨��

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
  * @brief  ����CAN1������
  * @retval uint8_t: ����״̬
  */
uint8_t bsp_can1_filter_config(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterActivation = ENABLE;                   // ���������
    filter.FilterMode = CAN_FILTERMODE_IDMASK;          // ������ģʽ����ʶ������ģʽ
    filter.FilterScale = CAN_FILTERSCALE_32BIT;         // ������λ��32λ
    filter.FilterBank = 0;                              // ����������
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;     // ���䵽FIFO0
    filter.FilterIdHigh = 0x0000;                       // ��ʶ����λ
    filter.FilterIdLow = 0x0000;                        // ��ʶ����λ
    filter.FilterMaskIdHigh = 0x0000;                   // ���α�ʶ����λ
    filter.FilterMaskIdLow = 0x0000;                    // ���α�ʶ����λ

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        /* ���ù�����ʧ�ܣ����ش���״̬ */
        return 1;
    }
    else
    {
        /* ���óɹ������سɹ�״̬ */
        return BSP_CAN_OK;
    }
}

/**
  * @brief  ����CAN��Ϣ
  * @param  id_type: ��ʶ�����ͣ���׼ID����չID��
  * @param  basic_id: ��׼ID
  * @param  ex_id: ��չID
  * @param  data: Ҫ���͵�����
  * @param  data_len: ���ݳ���
  * @retval uint8_t: ����״̬
  */
uint8_t bsp_can1_send_msg(uint32_t id_type, uint32_t basic_id, uint32_t ex_id, uint8_t *data, uint32_t data_len)
{
    uint32_t tx_mailbox = 0;
    CAN_TxHeaderTypeDef send_msg_hdr;

    send_msg_hdr.StdId = basic_id;     // ���ñ�׼ID
    send_msg_hdr.ExtId = ex_id;        // ������չID
    send_msg_hdr.IDE = id_type;        // ����ID����
    send_msg_hdr.RTR = CAN_RTR_DATA;   // ��������֡
    send_msg_hdr.DLC = data_len;       // �������ݳ���
    send_msg_hdr.TransmitGlobalTime = DISABLE; // ����ȫ��ʱ��

    /* ����CAN��Ϣ */
if (HAL_CAN_AddTxMessage(&hcan1, &send_msg_hdr, data, &tx_mailbox) != HAL_OK)
{
    /* ����ʧ�ܣ����������Ϣ */
    //printf("CAN1 Message Send Error\r\n");
    return 1;
}
else
{
    /* ���ͳɹ��������ʾ��Ϣ */
    printf("CAN1 Message Sent Successfully\r\n");
    return BSP_CAN_OK;
}

}

/* USER CODE END 1 */
