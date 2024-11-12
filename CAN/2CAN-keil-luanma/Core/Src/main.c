/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * ��������ԭ�еĻ����������CAN���ߵĲ��Դ��룬��ȷ��UART����������Ϊ115200��
  * �ڹؼ�λ�������printf������ڵ��Ժͼ��顣
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

// �����׼ID����չID���͵ĺ�
#define ID_TYPE_STANDARD  CAN_ID_STD
#define ID_TYPE_EXTENDED  CAN_ID_EXT

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t send_data[8] = {0};  // ���Ҫ���͵�CAN����
uint8_t id_type_std = 1;     // ���ڱ�ʶ��׼����չID���л�

/* �������������ڽ���CAN���� */
CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* ����CAN���ջص����������� */
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

  /* ����CAN1 */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* ����ʧ�ܣ����������Ϣ */
    printf("CAN1 Start Error\r\n");
    Error_Handler();
  }
  else
  {
    /* �����ɹ��������ʾ��Ϣ */
    printf("CAN1 Started Successfully\r\n");
  }

  /* ����CAN1������ */
  if (bsp_can1_filter_config() != BSP_CAN_OK)
  {
    /* ����������ʧ�ܣ����������Ϣ */
    printf("CAN1 Filter Configuration Error\r\n");
    Error_Handler();
  }
  else
  {
    /* ���óɹ��������ʾ��Ϣ */
    printf("CAN1 Filter Configured Successfully\r\n");
  }

  /* ����CAN1�����ж�֪ͨ */
 if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    /* ����ʧ�ܣ����������Ϣ */
    printf("CAN1 Notification Activation Error\r\n");
    Error_Handler();
  }
  else
  {
    /* ����ɹ��������ʾ��Ϣ */
    printf("CAN1 Notification Activated\r\n");
  }

  /* ��ʼ���������� */
  for (uint8_t i = 0; i < 8; i++)
  {
    send_data[i] = i;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ���ѭ����ʾ */
    //printf("Sending CAN message...\r\n");

    /* ����CAN��Ϣ */
    if (bsp_can1_send_msg(ID_TYPE_STANDARD, 0x123, 0x00, send_data, 8) != BSP_CAN_OK)
    {
      /* ����ʧ�ܣ����������Ϣ */
      //printf("CAN1 Message Send Error\r\n");
    }
    else
    {
      /* ���ͳɹ��������ʾ��Ϣ */
      //printf("CAN1 Message Sent\r\n");
    }

    /* �������ݵ��������ڹ۲����ݱ仯 */
    for (uint8_t i = 0; i < 8; i++)
    {
      send_data[i]++;
    }

    /* ��ʱ1�� */
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
  * @brief  CAN���ջص������������յ��µ�CAN��Ϣʱ����
  * @param  hcan: CAN���
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* ��FIFO0�л�ȡCAN��Ϣ */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    /* ��ȡ��Ϣʧ�ܣ����������Ϣ */
    //printf("CAN1 Receive Error\r\n");
  }
  else
  {
    /* ��ȡ��Ϣ�ɹ���������յ�����ϢID������ */
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
  /* �û������ڴ˴�����Լ��Ĵ�������� */
  __disable_irq();
  while (1)
  {
    // ������ʱ��������ѭ��
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
