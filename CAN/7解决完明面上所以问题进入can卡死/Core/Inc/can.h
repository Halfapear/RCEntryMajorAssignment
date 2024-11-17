/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   此文件包含了 can.c 文件中函数的原型声明以及相关变量的外部声明。
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

#ifndef __CAN_H
#define __CAN_H

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
#ifndef CAN_TX_DATA_LENGTH
#define CAN_TX_DATA_LENGTH 8U  // CAN 发送数据长度
#endif

#ifndef CAN_RX_DATA_LENGTH
#define CAN_RX_DATA_LENGTH 8U  // CAN 接收数据长度
#endif

/* USER CODE BEGIN Private defines */
/* 在此处添加其他私有定义 */
/* USER CODE END Private defines */

/* 外部变量声明 ------------------------------------------------------------*/

/* CAN 句柄 */
extern CAN_HandleTypeDef hcan1;

/* CAN 消息头结构体 */
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;

/* CAN 数据缓冲区 */
extern uint8_t CAN_TxData[CAN_TX_DATA_LENGTH];
extern uint8_t CAN_RxData[CAN_RX_DATA_LENGTH];

/* USER CODE BEGIN EXTERN_Variables */

/* 在此处添加其他外部变量的声明 */

/* 例如，如果有 moto_chassis 数组，需要在 main.h 或其他合适的头文件中声明 */
/* extern MotoChassisType moto_chassis[MAX_MOTO_CHASSIS]; */

/* USER CODE END EXTERN_Variables */

/* 函数原型声明 -------------------------------------------------------------*/

/* CAN 初始化函数 */
void MX_CAN1_Init(void);

/* CAN 过滤器初始化函数，接收所有消息 */
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);

/* CAN 发送消息函数
 * msg: 指向发送数据的指针
 * len: 数据长度 (最大为8)
 * 返回值:
 *   0 - 发送成功
 *   1 - 发送失败
 */
uint8_t CAN1_Send_Msg(uint8_t* msg, uint8_t len);

/* CAN 接收消息查询函数
 * buf: 指向接收数据的缓冲区
 * 返回值:
 *   0 - 无数据接收
 *   >0 - 接收到的数据长度
 */
uint8_t CAN1_Receive_Msg(uint8_t *buf);

/* CAN 接收完成回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);

/* USER CODE BEGIN Prototypes */

/* 在此处添加其他函数原型 */

/* 例如，中断服务程序的原型声明（如果需要在其他文件中引用） */
void CAN1_RX0_IRQHandler(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
