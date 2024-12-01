#ifndef __TRANSMISSION_H
#define __TRANSMISSION_H
#include "usart.h"
#include "motor.h"

typedef struct{
	float kp;
	float ki;
	float kd;
	float kpv;
	float kiv;
	float kdv;
}motorPara_t;

extern u8 control_mode;
extern float set_position[4];	// 电机位置全局变量
extern float set_speed[4]; 	// 电机速度全局变量
extern float set_current[4]; 	// 电机电流全局变量
extern motorPara_t motorPara[4];
extern motorPara_t defalutPara;

extern u8 pid1_change_flag;	// pid参数更改改了
extern u8 pid2_change_flag;

#define POSITION_MODE 0
#define SPEED_MODE 1
#define CURRENT_MODE 2

u8 ComSendData(u8 fun,u8* data,u8 len);

u8 uploadMoto12Info(void);
//u8 uploadMoto34Info(void);
u8 uploadControl12Info(void);
//u8 uploadControl34Info(void);
u8 uploadPID1Info(void);
u8 uploadPID2Info(void);

void analyseData(u8 *data);

#endif

