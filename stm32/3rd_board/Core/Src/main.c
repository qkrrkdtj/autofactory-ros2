/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Combined Multi-Task System
  * : System 1 (Cap Dispenser & Press Sequence via Servos & Dual L298N)
  * : System 2 (Relay & Actuator Sequence Control)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcp2515.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ========================================================================= */
/* [시스템 2 매크로 정의]                                                    */
/* ========================================================================= */
#define SENSOR2_DETECTED() (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET)
#define RELAY_ON()        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define RELAY_OFF()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

/* 2번 액추에이터 제어 (IN3: PB12, IN4: PB15) - PB13 핀 불량으로 PB15 대체 */
#define ACT2_FORWARD()    do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); } while(0)
#define ACT2_BACKWARD()   do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   } while(0)
#define ACT2_STOP()       do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); } while(0)

/* ========================================================================= */
/* [시스템 1: 뚜껑 공급 및 압착 제어 매크로 정의 - PC2 핀 매핑 수정 완료]     */
/* ========================================================================= */
/* PB8: 뚜껑 감지 센서 1 */
#define SENSOR1_CAP_DETECTED() (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)

/* 뚜껑 공급 액추에이터 제어 (IN1: PC2, IN2: PC3) 👈 PC2로 수정 완료 */
#define NEW_ACT_FORWARD()  do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); } while(0)
#define NEW_ACT_BACKWARD() do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);   } while(0)
#define NEW_ACT_STOP()     do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); } while(0)

/* 뚜껑 압착 액추에이터 제어 (IN3: PC0, IN4: PC5) - PC1, PC4 핀 불량으로 PC5 대체 */
#define PRESS_ACT_FORWARD()  do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); } while(0)
#define PRESS_ACT_BACKWARD() do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   } while(0)
#define PRESS_ACT_STOP()     do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); } while(0)

/* 서보모터 펄스 제어 편의 매크로 (TIM3 하드웨어 매핑) */
#define SET_SERVO1_PULSE(p) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (p))
#define SET_SERVO2_PULSE(p) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (p))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
MCP2515_HandleTypeDef hmcp2515;

// 시스템 1 제어 상태 플래그
uint8_t cap_sequence_done = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void CANPingTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n=== System Main Boot Up Completed (Newlib Reentrant Enabled) ===\r\n");

  // 서보모터용 PWM 하드웨어 드라이버 구동 시작
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // 시스템 1 하드웨어 대기 홈(Home) 위치 빌드 및 정지 상태 고정 (원복 완료)
  SET_SERVO1_PULSE(2300);
  SET_SERVO2_PULSE(2300);
  NEW_ACT_STOP();
  PRESS_ACT_STOP();

  // 시스템 2 초기 하드웨어 상태 빌드
  RELAY_ON();
  ACT2_STOP();

  // MCP2515 CAN 컨트롤러 인터페이스 구성
  hmcp2515.hspi    = &hspi1;
  hmcp2515.cs_port = MCP2515_CS_GPIO_Port;
  hmcp2515.cs_pin  = MCP2515_CS_Pin;
  hmcp2515.osc_hz  = MCP2515_OSC_8MHZ;

  if (MCP2515_AutoDetectOsc(&hmcp2515) != HAL_OK)
  {
    printf("[CAN] MCP2515 initialization failed!\r\n");
    MCP2515_PrintDiag(&hmcp2515);
  }
  else
  {
    MCP2515_SetNormalMode(&hmcp2515);
    printf("[CAN] MCP2515 is ready on SPI1 (%lu MHz)\r\n", (unsigned long)(hmcp2515.osc_hz / 1000000U));
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  static const osThreadAttr_t canping_attr = {
    .name       = "CANPing",
    .stack_size = 256U * 4U,
    .priority   = osPriorityBelowNormal,
  };
  osThreadNew(CANPingTask, NULL, &canping_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 PC3 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void CANPingTask(void *argument)
{
  (void)argument;
  uint8_t  tx_counter = 0U;
  uint32_t last_tx    = 0U;

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    if ((now - last_tx) >= 1000U)
    {
      last_tx = now;
      tx_counter++;

      MCP2515_CanMsg tx = {
        .id       = 0x003U,
        .dlc      = 2U,
        .data     = {0x03U, tx_counter},
        .extended = false,
      };
      HAL_StatusTypeDef s = MCP2515_Send(&hmcp2515, &tx);
      if (s != HAL_OK)
        MCP2515_PrintDiag(&hmcp2515);
    }

    MCP2515_CanMsg rx = {0};
    if (MCP2515_Receive(&hmcp2515, &rx, 5U) == HAL_OK)
    {
      printf("[RX CAN] ID=0x%03lX dlc=%u\r\n", (unsigned long)rx.id, rx.dlc);
    }

    osDelay(50U);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  (void)argument;
  static GPIO_PinState prev_sensor1_state = GPIO_PIN_SET;

  for (;;)
  {
    /* ========================================================================= */
    /* [시스템 1] 뚜껑 공급 + 신규 압착 메커니즘 통합 제어 시퀀스                */
    /* ========================================================================= */
    if (SENSOR1_CAP_DETECTED())
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

      if (cap_sequence_done == 0)
      {
        // 중복 방지를 위해 진입하자마자 플래그 잠금
        cap_sequence_done = 1;
        printf("[SYS 1] Triggered -> Starting Full Cap & Press Sequence\r\n");

        RELAY_OFF();

        // STEP 1. 서보모터 1번 구동 (대기 2300 -> 역회전 하강 상태 800)
        SET_SERVO1_PULSE(800);
        osDelay(2000);

        // STEP 2. 서보모터 1번 미세 보정 위치 복귀 (원복 완료)
        SET_SERVO1_PULSE(2300);
        osDelay(2000);

        // STEP 3. 뚜껑 공급용 기계식 액추되이터 전진 (9초)
        NEW_ACT_FORWARD();
        osDelay(9000);

        NEW_ACT_STOP();
        osDelay(300);

        // STEP 4. 뚜껑 공급용 기계식 액추에이터 후진 복귀 (9.5초)
        NEW_ACT_BACKWARD();
        osDelay(9500);

        NEW_ACT_STOP();
        osDelay(500);

        // STEP 5. 신규 압착 액추에이터 전진 구동 (위에서 꾹 누르기 - 9초)
        printf("[SYS 1] Pressing Mechanism Active...\r\n");
        PRESS_ACT_FORWARD();
        osDelay(9000);

        PRESS_ACT_STOP();
        osDelay(300);

        // STEP 6. 신규 압착 액추에이터 복귀 후진 구동 (9초)
        PRESS_ACT_BACKWARD();
        osDelay(9000);

        PRESS_ACT_STOP();
        osDelay(200);
        printf("[SYS 1] Pressing Mechanism Completed.\r\n");

        // STEP 7. 서보모터 2번 꺾기 구동 (대기 1000 -> 회전 1500)
        SET_SERVO2_PULSE(1000);
        osDelay(2000);

        // STEP 8. 전체 서보축 최종 안전 대기 홈(Home) 위치 초기화 복귀 (원복 완료)
        SET_SERVO1_PULSE(2300);
        osDelay(2000);
        SET_SERVO2_PULSE(2300);
        osDelay(1000);

        RELAY_ON();
        printf("[SYS 1] Sequence Completed Successfully.\r\n");
      }

      // 시스템 2가 스케줄링될 수 있도록 대기 처리
      osDelay(50);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

      if (prev_sensor1_state != GPIO_PIN_SET)
      {
        SET_SERVO1_PULSE(2300);
        SET_SERVO2_PULSE(2300);
        NEW_ACT_STOP();
        PRESS_ACT_STOP();
        printf("[SYS 1] Sensor Cleared -> System Standby\r\n");
      }
      cap_sequence_done = 0;
    }
    prev_sensor1_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);


    /* ========================================================================= */
    /* [기존 유지] 시츄에이션 2: 기존 액추에이터 2번 및 릴레이 제어              */
    /* ========================================================================= */
    if (SENSOR2_DETECTED())
    {
      printf("[SEQ 2] Sensor 2 Active! -> Relay OFF, Actuator 2 Moving.\r\n");

      RELAY_OFF();
      osDelay(50);

      ACT2_FORWARD();
      osDelay(10000);

      ACT2_STOP();
      osDelay(300);

      ACT2_BACKWARD();
      osDelay(13000);

      ACT2_STOP();
      osDelay(50);

      RELAY_ON();
      printf("[SEQ 2] Actuator 2 Completed! -> Relay ON.\r\n");
      osDelay(500);

      while (SENSOR2_DETECTED())
      {
        osDelay(100);
      }
    }

    osDelay(20);
  }
}
/* USER CODE END Header_StartDefaultTask */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
