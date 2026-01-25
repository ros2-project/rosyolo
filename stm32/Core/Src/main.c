/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buf[7];                   // 수신 버퍼
int16_t left_speed, right_speed;     // 모터 속도
#define PWM_ARR 3359

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
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // PWM 시작 (왼쪽 모터)

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // PWM 시작 (오른쪽 모터)

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // PWM 시작 (왼쪽 서보 모터)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // PWM 시작 (왼쪽 서보 모터)

  HAL_UART_Receive_IT(&huart3, rx_buf, 7);  // 7바이트 데이터를 수신하기 위해 인터럽트 설정
  HAL_GPIO_WritePin(STBY1_GPIO_Port, STBY1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STBY2_GPIO_Port, STBY2_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)  // USART3에서 데이터 수신 완료 확인
    {
        // 데이터 패킷 헤더 확인 (0xAA)
        if (rx_buf[0] == 0xAA) {
            uint8_t id = rx_buf[1];  // 패킷 ID


            // 모터 제어 패킷 (id == 0x01)
            if (id == 0x01) {
                HAL_UART_Transmit(&huart3, rx_buf, 7, HAL_MAX_DELAY);  // 수신한 데이터를 그대로 송신
                // 리틀 엔디안 방식으로 속도 추출
                left_speed  = (int16_t)(rx_buf[2] | (rx_buf[3] << 8));
                right_speed = (int16_t)(rx_buf[4] | (rx_buf[5] << 8));


                // 체크섬 확인
                uint8_t checksum = rx_buf[6];
                uint8_t calc = id ^ rx_buf[2] ^ rx_buf[3] ^ rx_buf[4] ^ rx_buf[5];

                // 체크섬이 일치하면 모터 제어
                if (checksum == calc) {
                    /* ===== 왼쪽 모터 제어 ===== */
                    if (left_speed >= 0) {
                        HAL_GPIO_WritePin(LEFT_IN1_GPIO_Port, LEFT_IN1_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(LEFT_IN2_GPIO_Port, LEFT_IN2_Pin, GPIO_PIN_RESET);
                        TIM3->CCR3 = ((int32_t)left_speed * PWM_ARR) / 32000;  // 왼쪽 모터 PWM 속도 설정
                    } else {
                        HAL_GPIO_WritePin(LEFT_IN1_GPIO_Port, LEFT_IN1_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LEFT_IN2_GPIO_Port, LEFT_IN2_Pin, GPIO_PIN_SET);
                        TIM3->CCR3 = ((int32_t)(-left_speed) * PWM_ARR) / 32000;  // 왼쪽 모터 반대 방향 설정
                    }

                    /* ===== 오른쪽 모터 제어 ===== */
                    if (right_speed >= 0) {
                        HAL_GPIO_WritePin(RIGHT_IN1_GPIO_Port, RIGHT_IN1_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(RIGHT_IN2_GPIO_Port, RIGHT_IN2_Pin, GPIO_PIN_RESET);
                        TIM3->CCR4 = ((int32_t)right_speed * PWM_ARR) / 32000;  // 오른쪽 모터 PWM 속도 설정
                    } else {
                        HAL_GPIO_WritePin(RIGHT_IN1_GPIO_Port, RIGHT_IN1_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(RIGHT_IN2_GPIO_Port, RIGHT_IN2_Pin, GPIO_PIN_SET);
                        TIM3->CCR4 = ((int32_t)(-right_speed) * PWM_ARR) / 32000;  // 오른쪽 모터 반대 방향 설정
                    }
                }
            }
            else if (id == 0x02) {
                HAL_UART_Transmit(&huart3, rx_buf, 7, HAL_MAX_DELAY);  // 수신한 데이터를 그대로 송신

				TIM3->CCR3=0;
				TIM3->CCR4=0;

				TIM4->CCR2 = 1500; // 오른쪽 서보 모터 피는 것
				TIM4->CCR1 = 1500; // 왼쪽 서보 모터 피는 것

				TIM4->CCR1 = 2500; // 왼쪽 서보 모터 접는것
				TIM4->CCR2 = 550; // 오른쪽 서보 모터 접는것


                        }
        }
    }

    /* 다음 패킷 수신 준비 */
    HAL_UART_Receive_IT(&huart3, rx_buf, 7);  // 계속해서 데이터를 비동기적으로 수신
}
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
#ifdef USE_FULL_ASSERT
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
