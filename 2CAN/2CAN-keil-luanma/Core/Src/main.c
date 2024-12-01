/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * 本程序在原有的基础上添加了CAN总线的测试代码，并确保UART波特率设置为115200。
  * 在关键位置添加了printf语句用于调试和检验。
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
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 定义标准ID和扩展ID类型的宏
#define ID_TYPE_STANDARD  CAN_ID_STD
#define ID_TYPE_EXTENDED  CAN_ID_EXT

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t send_data[8] = {0};  // 存放要发送的CAN数据
uint8_t id_type_std = 1;     // 用于标识标准或扩展ID的切换

/* 新增变量，用于接收CAN数据 */
CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* 新增CAN接收回调函数的声明 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* 启动CAN1 */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* 启动失败，输出错误信息 */
    printf("CAN1 Start Error\r\n");
    Error_Handler();
  }
  else
  {
    /* 启动成功，输出提示信息 */
    printf("CAN1 Started Successfully\r\n");
  }

  /* 配置CAN1过滤器 */
  if (bsp_can1_filter_config() != BSP_CAN_OK)
  {
    /* 过滤器配置失败，输出错误信息 */
    printf("CAN1 Filter Configuration Error\r\n");
    Error_Handler();
  }
  else
  {
    /* 配置成功，输出提示信息 */
    printf("CAN1 Filter Configured Successfully\r\n");
  }

  /* 激活CAN1接收中断通知 */
 if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    /* 激活失败，输出错误信息 */
    printf("CAN1 Notification Activation Error\r\n");
    Error_Handler();
  }
  else
  {
    /* 激活成功，输出提示信息 */
    printf("CAN1 Notification Activated\r\n");
  }

  /* 初始化发送数据 */
  for (uint8_t i = 0; i < 8; i++)
  {
    send_data[i] = i;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 输出循环提示 */
    //printf("Sending CAN message...\r\n");

    /* 发送CAN消息 */
    if (bsp_can1_send_msg(ID_TYPE_STANDARD, 0x123, 0x00, send_data, 8) != BSP_CAN_OK)
    {
      /* 发送失败，输出错误信息 */
      //printf("CAN1 Message Send Error\r\n");
    }
    else
    {
      /* 发送成功，输出提示信息 */
      //printf("CAN1 Message Sent\r\n");
    }

    /* 发送数据递增，便于观察数据变化 */
    for (uint8_t i = 0; i < 8; i++)
    {
      send_data[i]++;
    }

    /* 延时1秒 */
    HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

/**
  * @brief  CAN接收回调函数，当接收到新的CAN消息时调用
  * @param  hcan: CAN句柄
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* 从FIFO0中获取CAN消息 */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    /* 获取消息失败，输出错误信息 */
    //printf("CAN1 Receive Error\r\n");
  }
  else
  {
    /* 获取消息成功，输出接收到的消息ID和数据 */
		/* 
    printf("Received CAN message:\r\n");
    printf("ID: 0x%X\r\n", rx_header.StdId);
    printf("Data: ");
    for (uint8_t i = 0; i < rx_header.DLC; i++)
    {
      printf("%02X ", rx_data[i]);
    }
    printf("\r\n");
		*/
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 用户可以在此处添加自己的错误处理代码 */
  __disable_irq();
  while (1)
  {
    // 错误发生时，进入死循环
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
