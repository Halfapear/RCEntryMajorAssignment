/******************************************************************************
/// @brief  电机控制相关函数
*******************************************************************************/

#include "can.h"
#include "motor.h"

#include <stdio.h>

moto_measure_t moto_chassis[motor_num] = {
  {2000, 2000, 2000, 2000},
  {2000, 2000, 2000, 2000},
  {2000, 2000, 2000, 2000},
  {2000, 2000, 2000, 2000}
};


void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);


uint8_t get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  // 从 CAN 接收 FIFO0 中获取消息
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    return 0; // 接收错误
  }

  ptr->last_angle = ptr->angle;
  ptr->angle = (uint16_t)(RxData[0] << 8 | RxData[1]);
  ptr->real_current = (int16_t)(RxData[2] << 8 | RxData[3]);
  ptr->speed_rpm = ptr->real_current; // 根据您的应用需求
  ptr->given_current = (int16_t)(RxData[4] << 8 | RxData[5]) / -5;
  ptr->hall = RxData[6];
  get_total_angle(ptr);

  printf("get_moto_measure called for motor %ld\n", ptr - moto_chassis);
  printf("angle: %u, speed_rpm: %d, total_angle: %ld\n", ptr->angle, ptr->speed_rpm, ptr->total_angle);

  return RxHeader.DLC;
}



void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  // 从 CAN 接收 FIFO0 中获取消息
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    // 处理接收错误
    return;
  }

  ptr->angle = (uint16_t)(RxData[0] << 8 | RxData[1]);
  ptr->offset_angle = ptr->angle;

  printf("get_moto_offset called for motor %ld\n", ptr - moto_chassis);
  printf("offset_angle: %u\n", ptr->offset_angle);


}



#define ABS(x)	( (x)>0 ? (x) : -(x) )
// 更新电机总角度
void get_total_angle(moto_measure_t *p){

  int res1, res2, delta;

  printf("get_total_angle success \r\n");

  if(p->angle < p->last_angle){			// 可能的情况
    res1 = p->angle + 8192 - p->last_angle;	// 正转，delta=+
    res2 = p->angle - p->last_angle;				// 反转	delta=-
  }else{	// angle > last
    res1 = p->angle - 8192 - p->last_angle ;// 反转	delta -
    res2 = p->angle - p->last_angle;				// 正转	delta +
  }
  // 不管正反转，肯定是转的角度小的那个是真的
  if(ABS(res1)<ABS(res2))
    delta = res1;
  else
    delta = res2;

  p->total_angle += delta;
  p->last_angle = p->angle;



}

// 设置电机电流值
uint8_t set_moto_current(CAN_HandleTypeDef* hcan, int16_t SID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4){

  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;

  TxHeader.StdId = SID;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  uint8_t data[8];
  data[0] = iq1 >> 8;
  data[1] = iq1;
  data[2] = iq2 >> 8;
  data[3] = iq2;
  data[4] = iq3 >> 8;
  data[5] = iq3;
  data[6] = iq4 >> 8;
  data[7] = iq4;

  if(HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK){
    return 1;     // 发送失败
  }
  else {
    printf("set_moto_current success \r\n");
  return 0;       // 发送成功
  }
}
