/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * 版权�?�? (c) 2024 STMicroelectronics.
  * 保留�?有权利�??
  *
  * 此软件根据根目录下的 LICENSE 文件中的条款进行许可�?
  * 如果没有随此软件组件提供 LICENSE 文件，则按原样提供�??
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define BSP_CAN_OK  0  // 表示成功状�?�的宏定�?
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/**
  * @brief  配置CAN1的过滤器
  * @retval BSP_CAN_OK 表示成功
  */
uint8_t bsp_can1_filter_config(void);

/**
  * @brief  发�?�CAN1消息
  * @param  id_type   CAN_ID_STD �? CAN_ID_EXT
  * @param  basic_id  标准ID
  * @param  ex_id     扩展ID
  * @param  data      发�?�的数据
  * @param  data_len  数据长度（最�?8�?
  * @retval BSP_CAN_OK 表示成功
  */
uint8_t bsp_can1_send_msg(uint32_t id_type, uint32_t basic_id, uint32_t ex_id, uint8_t *data, uint32_t data_len);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

