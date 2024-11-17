/******************************************************************************
/// @brief  电机控制相关函数
*******************************************************************************/

#include "can.h"
#include "motor.h"

moto_measure_t moto_chassis[motor_num] = {0};		// 4 个底盘电机

void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

// 获取电机测量值
uint8_t get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
  ptr->last_angle = ptr->angle;
  ptr->angle = (uint16_t)(CAN_RxData[0]<<8 | CAN_RxData[1]);
  ptr->real_current  = (int16_t)(CAN_RxData[2]<<8 | CAN_RxData[3]);
  ptr->speed_rpm = ptr->real_current;	// 这里是因为两种电调对应位不一样的信息
  ptr->given_current = (int16_t)(CAN_RxData[4]<<8 | CAN_RxData[5])/-5;
  ptr->hall = CAN_RxData[6];

  get_total_angle(ptr);

  return RxHeader.DLC;
}

// 获取电机初始偏移值
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
  ptr->angle = (uint16_t)(CAN_RxData[0]<<8 | CAN_RxData[1]);
  ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x)>0 ? (x) : -(x) )
// 更新电机总角度
void get_total_angle(moto_measure_t *p){

  int res1, res2, delta;
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

  if(HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
    return 1;     // 发送失败

  return 0;       // 发送成功
}
