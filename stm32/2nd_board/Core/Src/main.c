/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Pill Dispenser — TB6600 Stepper + IR Sensor (FreeRTOS)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mcp2515.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPS_PER_DISPENSE  (520U * 2U)  /* 520스텝 × 2토글 (1/16 마이크로스텝) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
MCP2515_HandleTypeDef hmcp2515;

osMutexId_t       canMutex;              /* MCP2515 SPI 동시 접근 방지 */
osSemaphoreId_t   sem_can_rx;            /* MCP2515 INT → CanRxTask 즉시 기상 */
osMessageQueueId_t containerQueue;       /* 1번 보드 약통 정보 수신 큐 (최대 8) */

osSemaphoreId_t   sem_dispense_done;     /* TIM2 ISR → Task: 스텝 완료 신호 */
osSemaphoreId_t   sem_uart_trigger;      /* UART ISR → Task: 수동 트리거 신호 */
osSemaphoreId_t   sem_pill_trigger;      /* PB8 IR 센서 EXTI → Task: 약통 도착 신호 */
volatile uint32_t tim_toggle_count   = 0U;
volatile uint32_t tim_target_toggles = 0U;
static uint8_t    uart_rx_buf;           /* UART 수신 1바이트 버퍼 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void Dispense_Once(void);
static void CanRxTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* 2번 보드가 STOP을 보낸 뒤에만 RESUME을 1회 전송한다. */
static uint8_t g_belt_stopped_by_pill = 0U;

static void PillSensor_DrainPending(void)
{
  while (osSemaphoreAcquire(sem_pill_trigger, 0U) == osOK)
  {
  }
}

static void PillSensor_WaitClearAndRearm(uint32_t debounce_ms)
{
  while (HAL_GPIO_ReadPin(SENSOR_PILL_GPIO_Port, SENSOR_PILL_Pin) == GPIO_PIN_RESET)
    osDelay(10U);

  osDelay(debounce_ms);
  PillSensor_DrainPending();
}

static HAL_StatusTypeDef Belt_SendStop(MCP2515_CanMsg *can_msg)
{
  can_msg->data[0] = CAN_CMD_BELT_STOP;
  osMutexAcquire(canMutex, osWaitForever);
  HAL_StatusTypeDef st = MCP2515_Send(&hmcp2515, can_msg);
  osMutexRelease(canMutex);
  if (st == HAL_OK)
    g_belt_stopped_by_pill = 1U;
  return st;
}

static HAL_StatusTypeDef Belt_SendResume(MCP2515_CanMsg *can_msg)
{
  if (g_belt_stopped_by_pill == 0U)
    return HAL_BUSY;

  can_msg->data[0] = CAN_CMD_BELT_RESUME;
  osMutexAcquire(canMutex, osWaitForever);
  HAL_StatusTypeDef st = MCP2515_Send(&hmcp2515, can_msg);
  osMutexRelease(canMutex);
  if (st == HAL_OK)
    g_belt_stopped_by_pill = 0U;
  return st;
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
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n=== Pill Dispenser Start ===\r\n");

  HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_SET);  /* 초기: 모터 비활성 */

  hmcp2515.hspi    = &hspi1;
  hmcp2515.cs_port = MCP2515_CS_GPIO_Port;
  hmcp2515.cs_pin  = MCP2515_CS_Pin;
  hmcp2515.osc_hz  = MCP2515_OSC_8MHZ;

  if (MCP2515_AutoDetectOsc(&hmcp2515) != HAL_OK)
  {
    /* AutoDetect 실패 시 8MHz/125kbps로 명시적 초기화 후 Normal 모드 진입 */
    printf("[CAN] AutoDetect failed — forcing 125kbps\r\n");
    MCP2515_PrintDiag(&hmcp2515);
    MCP2515_SetNormalMode(&hmcp2515);
  }
  else
  {
    MCP2515_SetNormalMode(&hmcp2515);
    printf("[CAN] MCP2515 ready\r\n");
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  canMutex = osMutexNew(NULL);
  if (canMutex == NULL) Error_Handler();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  sem_dispense_done = osSemaphoreNew(1U, 0U, NULL);
  if (sem_dispense_done == NULL) Error_Handler();

  sem_uart_trigger = osSemaphoreNew(1U, 0U, NULL);
  if (sem_uart_trigger == NULL) Error_Handler();

  /* PB8 IR 센서 (EXTI line 8) 세마포어 — 생성 후 NVIC 활성화 */
  sem_pill_trigger = osSemaphoreNew(1U, 0U, NULL);
  if (sem_pill_trigger == NULL) Error_Handler();

  __HAL_GPIO_EXTI_CLEAR_IT(SENSOR_PILL_Pin);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* MCP2515 INT (PB0) 세마포어 — 생성 후 NVIC 활성화 */
  sem_can_rx = osSemaphoreNew(2U, 0U, NULL);
  if (sem_can_rx == NULL) Error_Handler();

  __HAL_GPIO_EXTI_CLEAR_IT(MCP2515_INT_Pin);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USART2 인터럽트 활성화 */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  containerQueue = osMessageQueueNew(8U, sizeof(ContainerInfo), NULL);
  if (containerQueue == NULL) Error_Handler();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  static const osThreadAttr_t can_rx_attr = {
    .name       = "CanRxTask",
    .stack_size = 256U * 4U,
    .priority   = osPriorityAboveNormal,
  };
  if (osThreadNew(CanRxTask, NULL, &can_rx_attr) == NULL)
    Error_Handler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* MCP2515 최대 클럭 10MHz, 84MHz/16=5.25MHz로 재초기화 */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
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

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* PB8 EXTI NVIC를 CubeMX가 조기 활성화하므로, sem_pill_trigger 생성 전까지 비활성화 */
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  /* PB0 = MCP2515 INT: FALLING EXTI — NVIC는 sem_can_rx 생성 후 별도 활성화 */
  GPIO_InitStruct.Pin  = MCP2515_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCP2515_INT_GPIO_Port, &GPIO_InitStruct);

  /* ENA(PC2)를 HIGH(비활성)로 초기화: CubeMX가 PC0~2를 모두 LOW로 설정하므로 재설정 */
  HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_SET);

  /* CS(PA4)를 HIGH(비활성)로 초기화: SPI 첫 트랜잭션 전 CS 보장 */
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MCP2515_INT_Pin)
    osSemaphoreRelease(sem_can_rx);
  else if (GPIO_Pin == SENSOR_PILL_Pin)
    osSemaphoreRelease(sem_pill_trigger);
}

/* 1번 보드가 CAN 0x101로 전송하는 약통 정보(seq, color)를 수신해 containerQueue에 적재한다.
   MCP2515 INT 핀(PB0) FALLING 엣지로 즉시 기상하며, 100ms 타임아웃 폴링을 폴백으로 사용한다. */
static void CanRxTask(void *argument)
{
  (void)argument;
  MCP2515_CanMsg rx;

  printf("[CAN-RX] Task started\r\n");

  for (;;)
  {
    osSemaphoreAcquire(sem_can_rx, 100U);  /* 100ms 폴링 폴백 */

    osMutexAcquire(canMutex, osWaitForever);
    HAL_StatusTypeDef st = MCP2515_Receive(&hmcp2515, &rx, 1U);
    osMutexRelease(canMutex);

    if (st != HAL_OK)
      continue;

    if (rx.id != CAN_ID_1ST_TX || rx.dlc < 3U)
      continue;

    ContainerInfo info = { .seq = rx.data[1], .color = rx.data[2] };

    if (osMessageQueuePut(containerQueue, &info, 0U, 0U) == osOK)
    {
      uint32_t cnt = osMessageQueueGetCount(containerQueue);
      printf("[Q:containerQueue] push seq=%u color=%c  [total %lu]\r\n",
             info.seq, info.color, (unsigned long)cnt);
    }
    else
    {
      printf("[Q:containerQueue] FULL — dropped seq=%u color=%c\r\n",
             info.seq, info.color);
    }
  }
}

/* 스텝모터를 STEPS_PER_DISPENSE 만큼 회전시켜 알약 1정을 투하한다.
   TIM2 ISR이 완료 시 sem_dispense_done을 방출할 때까지 태스크가 블록된다. */
static void Dispense_Once(void)
{
  tim_toggle_count   = 0U;
  tim_target_toggles = STEPS_PER_DISPENSE;

  HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_SET);
  osDelay(10U);

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, 1499U);
  HAL_TIM_Base_Start_IT(&htim2);

  osSemaphoreAcquire(sem_dispense_done, osWaitForever);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (uart_rx_buf == 'd' || uart_rx_buf == 'D')
      osSemaphoreRelease(sem_uart_trigger);

    HAL_UART_Receive_IT(&huart2, &uart_rx_buf, 1);  /* 다음 수신 재등록 */
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/*
 * 터미널 커맨드:
 *   d/D : 디스펜스 (가감속 풀 시퀀스)
 *   s/S : 극저속 테스트 — 10스텝, 1스텝당 500ms (탈조·배선 확인)
 *   e/E : ENA 토글 — 홀딩 토크 ON/OFF 확인 (ENA 극성 확인)
 *   p/P : 단발 펄스 1개 (PUL 핀 배선 확인)
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;

  HAL_UART_Receive_IT(&huart2, &uart_rx_buf, 1);
  printf("[PILL] Ready.\r\n");
  printf("  d = dispense  s = slow-test(10steps)  e = ENA toggle  p = single pulse\r\n");

  /* 부팅 시 스퓨리어스 EXTI 엣지로 쌓인 트리거 토큰 제거 */
  PillSensor_DrainPending();

  static GPIO_PinState prev_pill_sensor = GPIO_PIN_SET;
  static uint8_t        pill_armed      = 1U;

  for (;;)
  {
    /* ── 진단 커맨드 처리 ──────────────────────────────────── */
    uint8_t cmd = uart_rx_buf;  /* 스냅샷 (ISR에서 덮어쓸 수 있음) */

    if (cmd == 's' || cmd == 'S')
    {
      uart_rx_buf = 0;
      printf("[DBG] Slow test: ENA LOW, 10 steps @ 500ms/step\r\n");
      HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_RESET);
      osDelay(50U);
      for (int i = 0; i < 10; i++)
      {
        HAL_GPIO_WritePin(STEPPER_PUL_GPIO_Port, STEPPER_PUL_Pin, GPIO_PIN_SET);
        osDelay(5U);
        HAL_GPIO_WritePin(STEPPER_PUL_GPIO_Port, STEPPER_PUL_Pin, GPIO_PIN_RESET);
        osDelay(495U);
        printf("[DBG] Step %d/10\r\n", i + 1);
      }
      HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_SET);
      printf("[DBG] Slow test done\r\n");
      continue;
    }

    if (cmd == 'e' || cmd == 'E')
    {
      uart_rx_buf = 0;
      static uint8_t ena_state = 1U;  /* 1 = HIGH(비활성) */
      ena_state ^= 1U;
      HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin,
                        ena_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      printf("[DBG] ENA -> %s  (모터 축을 손으로 돌려보세요)\r\n",
             ena_state ? "HIGH(비활성)" : "LOW(활성-홀딩)");
      osDelay(10U);
      continue;
    }

    if (cmd == 'p' || cmd == 'P')
    {
      uart_rx_buf = 0;
      printf("[DBG] Single pulse: ENA LOW -> PUL HIGH 5ms -> LOW\r\n");
      HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_RESET);
      osDelay(10U);
      HAL_GPIO_WritePin(STEPPER_PUL_GPIO_Port, STEPPER_PUL_Pin, GPIO_PIN_SET);
      osDelay(5U);
      HAL_GPIO_WritePin(STEPPER_PUL_GPIO_Port, STEPPER_PUL_Pin, GPIO_PIN_RESET);
      printf("[DBG] Pulse sent. Did you hear a click?\r\n");
      osDelay(10U);
      continue;
    }

    /* ── 센서 / UART 디스펜스 트리거 ───────────────────────── */
    GPIO_PinState pill_now     = HAL_GPIO_ReadPin(SENSOR_PILL_GPIO_Port, SENSOR_PILL_Pin);
    uint8_t       pill_falling = (pill_now == GPIO_PIN_RESET && prev_pill_sensor == GPIO_PIN_SET);
    uint8_t       sem_pending  = (osSemaphoreAcquire(sem_pill_trigger, 0U) == osOK);
    uint8_t       uart_trig    = (osSemaphoreAcquire(sem_uart_trigger, 0U) == osOK);
    uint8_t       sensor_trig  = 0U;

    /* 재무장 전에는 센서 트리거 무시, FALLING 엣지 + LOW일 때만 1회 인정 */
    if (pill_armed && pill_now == GPIO_PIN_RESET && (pill_falling || sem_pending))
      sensor_trig = 1U;
    else if (sem_pending)
      PillSensor_DrainPending();

    if (sensor_trig || uart_trig)
    {
      /* ── 약통 정보 dequeue (최대 200ms 대기) ─────────────────── */
      ContainerInfo info = { .seq = 0U, .color = '?' };
      if (osMessageQueueGet(containerQueue, &info, NULL, 200U) == osOK)
      {
        uint32_t cnt = osMessageQueueGetCount(containerQueue);
        printf("[Q:containerQueue] pop  seq=%u color=%c  [remain %lu]\r\n",
               info.seq, info.color, (unsigned long)cnt);
      }
      printf("[PILL] Container seq=%u color=%c\r\n", info.seq, info.color);

      if (sensor_trig)
        pill_armed = 0U;

      /* ── CAN: 1번 보드 벨트 정지 요청 ──────────────────────── */
      MCP2515_CanMsg can_msg = {
        .id       = CAN_ID_2ND_TX,
        .dlc      = 1U,
        .data     = {CAN_CMD_BELT_STOP},
        .extended = false,
      };
      HAL_StatusTypeDef tx_st = Belt_SendStop(&can_msg);
      if (tx_st == HAL_OK)
        printf("[CAN] Belt STOP sent\r\n");
      else
        printf("[CAN] Belt STOP FAILED (TX error)\r\n");
      osDelay(200U);  /* 1번 보드가 벨트를 정지할 시간 확보 */

      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

      /* ── 알약 2정 투입 ───────────────────────────────────────── */
      Dispense_Once();
      printf("[PILL] Pill 1 dispensed\r\n");

      osDelay(500U);  /* 알약 간격 */

      Dispense_Once();
      printf("[PILL] Pill 2 dispensed\r\n");

      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_SET);  /* 모터 비활성 */

      osDelay(500U);  /* 알약 투입 완료 후 벨트 재개 전 안정화 대기 */

      /* ── CAN: STOP을 보낸 경우에만 RESUME 1회 전송 ─────────── */
      tx_st = Belt_SendResume(&can_msg);
      if (tx_st == HAL_OK)
        printf("[CAN] Belt RESUME sent\r\n");
      else if (tx_st == HAL_BUSY)
        printf("[CAN] Belt RESUME skipped (no prior STOP)\r\n");
      else
        printf("[CAN] Belt RESUME FAILED (TX error)\r\n");

      if (sensor_trig)
      {
        PillSensor_WaitClearAndRearm(500U);
        pill_armed = 1U;
      }
    }
    else
    {
      /* ── 감지 없음: 모터 비활성화 (발열 방지) ────────────── */
      HAL_GPIO_WritePin(STEPPER_ENA_GPIO_Port, STEPPER_ENA_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      osDelay(10U);
    }

    prev_pill_sensor = pill_now;
  }
  /* USER CODE END 5 */
}

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
  if (htim->Instance == TIM2)
  {
    if (tim_toggle_count < tim_target_toggles)
    {
      HAL_GPIO_WritePin(STEPPER_PUL_GPIO_Port, STEPPER_PUL_Pin,
                        (tim_toggle_count % 2U == 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      tim_toggle_count++;

      /* 가속 구간: 처음 50토글에서 주기를 1500μs → 300μs로 선형 감소 */
      if (tim_toggle_count < 50U)
        __HAL_TIM_SET_AUTORELOAD(&htim2, 1500U - 24U * tim_toggle_count - 1U);
      else
        __HAL_TIM_SET_AUTORELOAD(&htim2, 299U);
    }
    else
    {
      /* 완료: 타이머 정지, 펄스 LOW, 세마포어 방출 */
      __HAL_TIM_SET_AUTORELOAD(&htim2, 299U);
      HAL_TIM_Base_Stop_IT(&htim2);
      HAL_GPIO_WritePin(STEPPER_PUL_GPIO_Port, STEPPER_PUL_Pin, GPIO_PIN_RESET);
      osSemaphoreRelease(sem_dispense_done);
    }
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
