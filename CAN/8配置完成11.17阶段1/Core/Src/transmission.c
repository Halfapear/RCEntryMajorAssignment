#include "transmission.h"
#include "string.h"

uint8_t control_mode = 1;		// 默认是速度模式
motorPara_t motorPara[4];
motorPara_t defalutPara = {0.04, 0.001, 0, 1.5, 0.1, 0};

float set_position[4];	// 电机位置全局变量
float set_speed[4]; 	// 电机速度全局变量
float set_current[4]; 	// 电机电流全局变量

u8 pid1_change_flag = 0;	// pid参数更改改了
u8 pid2_change_flag = 0;

u8 ComSendData(u8 fun, u8 *data, u8 len)
{
	u8 send_buf[25] = {0};
	u8 i;
	if (len > 20)
		return 1;		   //最多20字节数据
	send_buf[0] = 0XAA;	   //帧头
	send_buf[1] = 0XFF;	   //帧头 上行
	send_buf[2] = fun; //功能字
	send_buf[3] = len; //数据长度
	for (i = 0; i < len; i++)
		send_buf[4 + i] = data[i]; //复制数据
	for (i = 0; i < len + 4; i++)
		send_buf[24] += send_buf[i]; //计算校验和
									 // if (HAL_UART_Transmit(&UART3_Handler, send_buf,25,1000) == HAL_OK)
									 //     return 0;
									 // else return 1;
	for (i = 0; i < 25; i++)
	{
		Uart3SendChar(send_buf[i]);
	}
	return 0;
}

u8 uploadMoto12Info(void)
{
	s16 paramWrite[10] = {0};
	paramWrite[0] = (s32)(moto_chassis[0].total_angle);
	paramWrite[1] = (s32)(moto_chassis[0].total_angle) >> 16;
	paramWrite[2] = (s16)(moto_chassis[0].speed_rpm);
	paramWrite[3] = (s16)(moto_chassis[0].given_current);
	paramWrite[4] = (s32)(moto_chassis[1].total_angle) >> 16;
	paramWrite[5] = (s32)(moto_chassis[1].total_angle);
	paramWrite[6] = (s16)(moto_chassis[1].speed_rpm);
	paramWrite[7] = (s16)(moto_chassis[1].given_current);

	ComSendData(0x01, (u8 *)&paramWrite[0], 16);

	return 0;
}

//u8 uploadMoto34Info(void)
//{
//	s16 paramWrite[10] = {0};
//	paramWrite[0] = (s32)(moto_chassis[2].total_angle);
//	paramWrite[1] = (s32)(moto_chassis[2].total_angle) >> 16;
//	paramWrite[2] = (s16)(moto_chassis[2].speed_rpm);
//	paramWrite[3] = (s16)(moto_chassis[2].given_current);
//	paramWrite[4] = (s32)(moto_chassis[3].total_angle) >> 16;
//	paramWrite[5] = (s32)(moto_chassis[3].total_angle);
//	paramWrite[6] = (s16)(moto_chassis[3].speed_rpm);
//	paramWrite[7] = (s16)(moto_chassis[3].given_current);

//	ComSendData(0x02, (u8 *)&paramWrite[0], 16);

//	return 0;
//}

u8 uploadControl12Info(void)
{
	s16 paramWrite[10] = {0};
	paramWrite[0] = (s16)control_mode;
	paramWrite[1] = (s32)(set_position[0]);
	paramWrite[2] = (s32)(set_position[0]) >> 16;
	paramWrite[3] = (s16)(set_speed[0]);
	paramWrite[4] = (s16)(set_current[0]);

	paramWrite[5] = (s16)control_mode;
	paramWrite[6] = (s32)(set_position[1]);
	paramWrite[7] = (s32)(set_position[1]) >> 16;
	paramWrite[8] = (s16)(set_speed[1]);
	paramWrite[9] = (s16)(set_current[1]);

	ComSendData(0x03, (u8 *)&paramWrite[0], 20);

	return 0;
}

//u8 uploadControl34Info(void)
//{
//	s16 paramWrite[10] = {0};
//	paramWrite[0] = (s16)control_mode;
//	paramWrite[1] = (s32)(set_position[2]);
//	paramWrite[2] = (s32)(set_position[2]) >> 16;
//	paramWrite[3] = (s16)(set_speed[2]);
//	paramWrite[4] = (s16)(set_current[2]);

//	paramWrite[5] = (s16)control_mode;
//	paramWrite[6] = (s32)(set_position[3]);
//	paramWrite[7] = (s32)(set_position[3]) >> 16;
//	paramWrite[8] = (s16)(set_speed[3]);
//	paramWrite[9] = (s16)(set_current[3]);

//	ComSendData(0x04, (u8 *)&paramWrite[0], 20);

//	return 0;
//}


u8 uploadPID1Info(void)
{
	s16 paramWrite[10] = {0};
	paramWrite[0] = (s16)(motorPara[0].kp *1000);
	paramWrite[1] = (s16)(motorPara[0].ki *1000);
	paramWrite[2] = (s16)(motorPara[0].kd *1000);
	paramWrite[3] = (s16)(motorPara[0].kpv *1000);
	paramWrite[4] = (s16)(motorPara[0].kiv *1000);
	paramWrite[5] = (s16)(motorPara[0].kdv *1000);

	ComSendData(0x05, (u8 *)&paramWrite[0], 12);

	return 0;
}

u8 uploadPID2Info(void)
{
	s16 paramWrite[10] = {0};
	paramWrite[0] = (s16)(motorPara[1].kp *1000);
	paramWrite[1] = (s16)(motorPara[1].ki *1000);
	paramWrite[2] = (s16)(motorPara[1].kd *1000);
	paramWrite[3] = (s16)(motorPara[1].kpv *1000);
	paramWrite[4] = (s16)(motorPara[1].kiv *1000);
	paramWrite[5] = (s16)(motorPara[1].kdv *1000);

	ComSendData(0x06, (u8 *)&paramWrite[0], 12);

	return 0;
}
// 解包，从一帧数据的功能字开始解析。 function + len + Data + sum
//								  1 		 1	  len    1
void analyseData(u8 *data)
{
	if (data[2] == 0x01) 	// PID帧
	{
		motorPara[0].kp = (float)((u16)data[5] << 8 | (u16)data[4]) / 1000.0f;
		motorPara[0].ki = (float)((u16)data[7] << 8 | (u16)data[6]) / 1000.0f;
		motorPara[0].kd = (float)((u16)data[9] << 8 | (u16)data[8]) / 1000.0f;
		motorPara[0].kpv = (float)((u16)data[11] << 8 | (u16)data[10]) / 1000.0f;
		motorPara[0].kiv = (float)((u16)data[13] << 8 | (u16)data[12]) / 1000.0f;
		motorPara[0].kdv = (float)((u16)data[15] << 8 | (u16)data[14]) / 1000.0f;
		pid1_change_flag = 1;			
	}


	if (data[2] == 0x02)  
	{
		motorPara[1].kp = (float)((u16)data[5] << 8 | (u16)data[4]) / 1000.0f;
		motorPara[1].ki = (float)((u16)data[7] << 8 | (u16)data[6]) / 1000.0f;
		motorPara[1].kd = (float)((u16)data[9] << 8 | (u16)data[8]) / 1000.0f;
		motorPara[1].kpv = (float)((u16)data[11] << 8 | (u16)data[10]) / 1000.0f;
		motorPara[1].kiv = (float)((u16)data[13] << 8 | (u16)data[12]) / 1000.0f;
		motorPara[1].kdv = (float)((u16)data[15] << 8 | (u16)data[14]) / 1000.0f;
		
	}

	if (data[2] == 0x03) // 控制帧
	{
		control_mode = (s16)((u16)data[5] << 8 | (u16)data[4]) ;
		set_position[0] = ((s32)((u32)data[9] << 24 | (u32)data[8] << 16 | (u32)data[7] << 8 | (u32)data[6]));
		set_speed[0] =  ((s16)((u16)data[11] << 8 | (u16)data[10]));
		set_current[0] = ((s16)((u16)data[13] << 8 | (u16)data[12]));
		
		set_position[1]=set_position[0]; 
		set_speed[1]=set_speed[0]; 
		set_current[1]=set_current[0];
	}
	printf("analyseData success \r\n ");

}
