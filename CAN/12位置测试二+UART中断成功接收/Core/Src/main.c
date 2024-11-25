/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
u32 run_cnt = 0;
u8 i;
pid_t pid_speed[4];		   //电机速度PID�???
pid_t pid_position[4];	//电机电流PID�???

float set_speed_temp;			   //加减速时的临时设定�?�度
int16_t delta;					   //设定速度与实际�?�度的差�???
int16_t max_speed_change = 500;   //电机单次�???大变化�?�度，加减�?�用
// 500经测试差不多是最大加速区间，即从零打到最大�?�度不异常的�???大�??


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
  PUTCHAR_PROTOTYPE
  {
    HAL_UART_Transmit(&huart3 , (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
  }
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PID_TypeDef motor_pid[4];
int32_t set_spd = 0;

// 定义接收缓冲区大�?
#define UART1_RX_BUFFER_SIZE  1

// 接收缓冲�?
uint8_t UART1_RX_Buffer[UART1_RX_BUFFER_SIZE];


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  const uint8_t SendBuffer[4]={0x03,0xFC,0xFC,0x03};  // 帧头
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	// 启动 UART3 的接收中断模式
	HAL_UART_Receive_IT(&huart3, USART3_RX_BUF, USART_REC_LEN);


	my_can_filter_init_recv_all(&hcan1);


// ����CANģ��
if (HAL_CAN_Start(&hcan1) != HAL_OK)
{
    Error_Handler();
}


  // 启用发�?�完成中断（Mailbox 0�???1 �??? 2�???
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }


	uint8_t test_data[] = "UART Test\r\n";
	HAL_UART_Transmit(&huart1, test_data, sizeof(test_data) - 1, HAL_MAX_DELAY);
	printf("Init Success\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //PID初始�???, 电机参数都设为默认参�???
  for (i = 0; i < 4; i++)
  {
    memcpy(&motorPara[i], &defalutPara, 24);
  }

  //PID初始�???
  for (i = 0; i < 4; i++)
  {
    PID_struct_init(&pid_position[i], POSITION_PID, 8000, 2000, motorPara[i].kp, motorPara[i].ki, motorPara[i].kd);   //4 motos angular position closeloop.  �???大输�???8000对应电机转�??8000rpm
    PID_struct_init(&pid_speed[i], POSITION_PID, 16384, 16384, motorPara[i].kpv, motorPara[i].kiv, motorPara[i].kdv); //4 motos angular rate closeloop.
  }

  // 初始化期望位置�?��?�度、电流都�???0
  set_position[0] = set_position[1] = set_position[2] = set_position[3] = 2000.0f; 	// -8000-8000，双向转
  set_speed[0] = set_speed[1] = set_speed[2] = set_speed[3] = 0; 				// -8000-8000，双向转
  set_current[0] = set_current[1] = set_current[2] = set_current[3] = 0;		// -8000-8000，双向转

  while (1)
  {
 // 力矩控制模式
		if (control_mode == CURRENT_MODE)
		{
			set_moto_current(&hcan1, 0x200, set_current[0], set_current[1], set_current[2], set_current[3]);
		}

		// 速度控制模式
		if (control_mode == SPEED_MODE)
		{
			for (i = 0; i < 2; i++)
			{
				//加减�???
				delta = (int16_t)set_speed[i] - moto_chassis[i].speed_rpm;
				if (delta > max_speed_change)
					set_speed_temp = (float)(moto_chassis[i].speed_rpm + max_speed_change);
				else if (delta < -max_speed_change)
					set_speed_temp = (float)(moto_chassis[i].speed_rpm - max_speed_change);
				else
					set_speed_temp = set_speed[i];
				pid_calc(&pid_speed[i], (float)moto_chassis[i].speed_rpm, set_speed_temp);
				printf("Motor %d - Speed Control:\n", i);
				printf("  target_speed: %f, speed_rpm: %d\n", pid_position[i].pos_out, moto_chassis[i].speed_rpm);
				printf("  speed_error: %f\n", pid_speed[i].err[NOW]);
				printf("  speed_output (current command): %f\n", pid_speed[i].pos_out);
			}

			set_moto_current(&hcan1, 0x200, (s16)(pid_speed[0].pos_out),
							 (s16)(pid_speed[1].pos_out),
							 (s16)(pid_speed[2].pos_out),
							 (s16)(pid_speed[3].pos_out));
			printf("  Motor %d current command: %d\n", i, (int16_t)(pid_speed[i].pos_out));
			printf("in speed mode\r\n");
		}

			if (control_mode == POSITION_MODE)
		{
			for(i = 0; i<2; i++)
			{
				// 位置�???
				pid_calc(&pid_position[i], (float)moto_chassis[i].total_angle, set_position[i]);
				printf("Motor %d - Position Control:\n", i);
				printf("  set_position: %f, total_angle: %ld\n", set_position[i], moto_chassis[i].total_angle);
				printf("  position_error: %f\n", pid_position[i].err[NOW]);
				printf("  position_output (target_speed): %f\n", pid_position[i].pos_out);
				// 速度�???
				pid_calc(&pid_speed[i], (float)moto_chassis[i].speed_rpm, pid_position[i].pos_out);
				printf("Motor %d - Speed Control:\n", i);
				printf("  target_speed: %f, speed_rpm: %d\n", pid_position[i].pos_out, moto_chassis[i].speed_rpm);
				printf("  speed_error: %f\n", pid_speed[i].err[NOW]);
				printf("  speed_output (current command): %f\n", pid_speed[i].pos_out);
			}

			set_moto_current(&hcan1, 0x200, (s16)(pid_speed[0].pos_out),
							 (s16)(pid_speed[1].pos_out),
							 (s16)(pid_speed[2].pos_out),
							 (s16)(pid_speed[3].pos_out));
				printf("  Motor %d current command: %d\n", i, (int16_t)(pid_speed[i].pos_out));
			printf("in position mode\r\n");

		}

		if (pid1_change_flag)
		{
			PID_struct_init(&pid_position[i], POSITION_PID, 8000, 2000, motorPara[0].kp, motorPara[0].ki, motorPara[0].kd);   //4 motos angular position closeloop.  �???大输�???8000对应电机转�??8000rpm
			PID_struct_init(&pid_speed[i], POSITION_PID, 16384, 16384, motorPara[0].kpv, motorPara[0].kiv, motorPara[0].kdv); //4 motos angular rate closeloop.
			pid1_change_flag = 0;
		}
		if (pid2_change_flag)
		{
			PID_struct_init(&pid_position[i], POSITION_PID, 8000, 2000, motorPara[1].kp, motorPara[1].ki, motorPara[1].kd);   //4 motos angular position closeloop.  �???大输�???8000对应电机转�??8000rpm
			PID_struct_init(&pid_speed[i], POSITION_PID, 16384, 16384, motorPara[1].kpv, motorPara[1].kiv, motorPara[1].kdv); //4 motos angular rate closeloop.
			pid1_change_flag = 0;
		}


//		// 上传数据�???20ms上传�???�???
		if (run_cnt % 4 == 0)
		{
			uploadMoto12Info();
			uploadControl12Info();
			uploadPID1Info();
			uploadPID2Info();
			printf("ki %f angle %d\r\n", motorPara[0].kiv, moto_chassis[0].total_angle);
		}


		run_cnt++;
		HAL_Delay(5); // 采样周期

  	printf("control_mode: %d\r\n", control_mode);
  	printf("set_position[0]: %f\n", set_position[0]);
  	printf("set_speed[0]: %f\n", set_speed[0]);
  	printf("set_current[0]: %f\n", set_current[0]);
  	printf("iq1: %d, iq2: %d, iq3: %d, iq4: %d\n", (s16)(pid_speed[0].pos_out),
							 (s16)(pid_speed[1].pos_out),
							 (s16)(pid_speed[2].pos_out),
							 (s16)(pid_speed[3].pos_out));


    /* UART 输出 */
  	/*
    uint8_t test_data[] = "UART Test\r\n";
    HAL_UART_Transmit(&huart3, test_data, sizeof(test_data) - 1, HAL_MAX_DELAY);

    printf("Printf Success\r\n");
  	*/
    HAL_Delay(1000);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* PID calculation & motor output */
    /*
    for(int i=0; i<4; i++)
    {
      motor_pid[i].target = set_spd;
      motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //根据设定值进行PID计算�???
    }
    set_moto_current(&hcan1, motor_pid[0].output,   //将PID的计算结果�?�过CAN发�?�到电机
                        motor_pid[1].output,
                        motor_pid[2].output,
                        motor_pid[3].output);
    */
  }


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
