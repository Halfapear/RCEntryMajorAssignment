/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   这个文件提供了CAN的函数原型和全局变量声明
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

/* 防止递归包含 -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H
#ifdef __cplusplus
extern "C" {
#endif

  /* 包含的头文件 ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"  // 根据您的STM32系列选择正确的头文件

  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* CAN句柄 */
  extern CAN_HandleTypeDef hcan1;

  /* USER CODE BEGIN Private defines */

  /* 定义接收和发送的消息头结构体 */
  extern CAN_TxHeaderTypeDef TxHeader;
  extern CAN_RxHeaderTypeDef RxHeader;
  extern uint8_t CAN_TxData[8];
  extern uint8_t CAN_RxData[8];

  /* USER CODE END Private defines */

  /* 函数原型 ------------------------------------------------------------------*/
  void MX_CAN1_Init(void);

  /* USER CODE BEGIN Prototypes */

  /* CAN 接收完成回调函数 */
  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);

  /* CAN 发送一组数据 */
  uint8_t CAN1_Send_Msg(uint8_t* msg, uint8_t len);

  /* CAN 接收数据查询 */
  uint8_t CAN1_Receive_Msg(uint8_t *buf);

  /* 初始化 CAN 过滤器，接收所有消息 */
  void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /* __CAN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
