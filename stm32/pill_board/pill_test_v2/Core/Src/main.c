/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Pill Dispenser + Lid Feeding System)
  * : Fixed include typo and applied servo 2 safety margin (1200) to prevent lockup
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3; // 서보모터 제어용 TIM3

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// [시스템 1] 알약 투하 변수
uint8_t pill_dispensed = 0;
volatile uint32_t tim_toggle_count  = 0;
volatile uint32_t tim_target_toggles = 0;

// [시스템 2] 뚜껑 투입 변수
uint8_t cap_processed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// TIM2 인터럽트: 스텝모터 가감속 펄스 생성
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (tim_toggle_count < tim_target_toggles)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,
          (tim_toggle_count % 2 == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      tim_toggle_count++;

      if (tim_toggle_count < 50)
        __HAL_TIM_SET_AUTORELOAD(&htim2, 1500 - 24 * tim_toggle_count - 1);
      else
        __HAL_TIM_SET_AUTORELOAD(&htim2, 299);
    }
    else
    {
      __HAL_TIM_SET_AUTORELOAD(&htim2, 299);
      HAL_TIM_Base_Stop_IT(&htim2);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    }
  }
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init(); // TIM3 활성화

  /* USER CODE BEGIN 2 */
  // 서보모터용 PWM 채널 1, 2 시작
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // 보드가 처음 켜졌을 때 안전 초기화 위치 설정 (하드웨어 기계적 락 방지 마진 반영)
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 600);   // 서보 1번: 대기 위치 (약 0도 부근)
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2400);  // ⭐ 서보 2번: 원상태인 정회전 끝(약 180도 부근) 위치 대기

  // L298N 액추에이터 초기 정지 상태
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // =========================================================================
    // [시스템 1] 기존 알약 투하 제어 로직 (기존 설정 유지)
    // =========================================================================
    GPIO_PinState sensor_out = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    static GPIO_PinState prev_sensor_out = GPIO_PIN_SET;
    const char *msg;

    if (sensor_out == GPIO_PIN_RESET)  // LOW: 알약 센서 장애물 감지됨
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // LED ON

      if (prev_sensor_out != GPIO_PIN_RESET)
      {
        msg = "[SYSTEM 1] PILL DETECTED (LOW)\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
      }

      if (pill_dispensed == 0)
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);  // ENA LOW: 활성화
        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);  // DIR 설정

        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim2, 1499);
        tim_toggle_count    = 0;
        tim_target_toggles  = 533 * 2;  // 60도 회전
        HAL_TIM_Base_Start_IT(&htim2);
        while (tim_toggle_count < tim_target_toggles);

        pill_dispensed = 1;
        HAL_Delay(1000);
      }
    }
    else  // HIGH: 알약 센서 공백
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // LED OFF
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);    // ENA HIGH: 발열 방지

      if (prev_sensor_out != GPIO_PIN_SET)
      {
        msg = "[SYSTEM 1] PILL CLEARED (HIGH)\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
      }
      pill_dispensed = 0;
      HAL_Delay(10);
    }
    prev_sensor_out = sensor_out;


    // =========================================================================
    // [시스템 2] 신규 알약 뚜껑 투입 제어 시퀀스 (원상태 = 정회전 끝 대기)
    // =========================================================================
    GPIO_PinState cap_sensor = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9); // 뚜껑 센서(PB9)
    static GPIO_PinState prev_cap_sensor = GPIO_PIN_SET;

    if (cap_sensor == GPIO_PIN_RESET) // ⭐ [LOW]: 뚜껑 센서 감지됨 ➡️ 역회전 제어 시퀀스 시작
    {
      if (prev_cap_sensor != GPIO_PIN_RESET)
      {
        msg = "[SYSTEM 2] CAP DETECTED\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
      }

      if (cap_processed == 0)
      {
    	  // 1. 서보모터 1번: 0도 대기(600) 상태에서 -> 거의 최대 각도(2200)로 확 꺾기
    	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2200);
    	  HAL_Delay(1000);

    	  // 2. 서보모터 2번: 180도 대기(2400) 상태에서 -> 완전히 반대쪽 끝(800)으로 확 제끼기
    	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 800);
    	  HAL_Delay(1000);

    	  // 3. 서보모터 2번 미세 조절 (필요 없으면 이 단계를 지우거나 값 조정)
    	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1400);
    	  HAL_Delay(800);

        // 4. L298N 제어: 액추에이터 6초 전진 (IN1=HIGH, IN2=LOW)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_Delay(6000);

        // 5. L298N 제어: 액추에이터 6초 후진 (IN1=LOW, IN2=HIGH)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(6500);

        // 6. 액추에이터 완전 정지 (IN1=LOW, IN2=LOW)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

        // 7. 시퀀스 완료 후 원상태 복귀값 인가
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 600);  // 서보 1번 초기화
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2400); // 서보 2번 원상태 복귀 (정회전 끝 마진 반영)
        HAL_Delay(1000); // 복귀 완료를 위한 대기시간

        cap_processed = 1;
      }
    }
    else // ⭐ [HIGH]: 뚜껑 센서 미감지 ➡️ 대기 상태
    {
      if (prev_cap_sensor != GPIO_PIN_SET)
      {
        msg = "[SYSTEM 2] CAP CLEARED -> RETURNING TO HOME\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);

        // 무한 반복 호출로 모터 제어 레지스터 회로가 먹통이 되는 버그 방지 (상태 변화 시 딱 한 번만 펄스 갱신)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 600);   // 서보 1번 평상시 위치
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2400);  // ⭐ 서보 2번 원상태 고정
      }

      cap_processed = 0; // 플래그 초기화
      HAL_Delay(50);     // CPU 점유율 과부하 방지 딜레이
    }
    prev_cap_sensor = cap_sensor;

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief TIM2 Initialization Function (기존 설정 유지)
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 299;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function (서보모터 PWM 주파수 50Hz 생성용)
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;                   // 1MHz 카운팅 클럭 생성
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;                   // 20,000 카운트 = 20ms 주기 (50Hz)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* PWM Channels Configuration */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function (기존 유지)
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 출력 핀 초기 레벨 설정 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /* PC0 ~ PC4 출력 핀 설정 (스텝모터 및 L298N 제어용) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PA5 내장 디버깅 LED 출력 설정 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA6(TIM3_CH1), PA7(TIM3_CH2) 서보모터 PWM 출력 얼터네이트 핀 설정 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // TIM3 매핑 고정
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB8 (알약 센서) 및 PB9 (뚜껑 센서) 입력 핀 설정 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
