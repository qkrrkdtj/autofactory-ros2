/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcp2515.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_1ST_TX     0x101U
#define CAN_MSG_CONTAINER 0x01U

/* 신규 추가: PA3 시작 적외선 센서 정의 */
#define START_SENSOR_GPIO_Port GPIOA
#define START_SENSOR_Pin       GPIO_PIN_3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

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

BeltStatus g_belt_status;

osMutexId_t beltMutex;
osMutexId_t piRspMutex;
osMutexId_t canMutex;         /* MCP2515 SPI 동시 접근 방지 */
osSemaphoreId_t sem_can_int;  /* MCP2515 INT 핀 낙하 → CanRxTask 즉시 기상 */

volatile bool g_belt_ext_stop = false;  /* 2번 보드 벨트 정지 요청 플래그 */

static volatile bool g_sensor_monitoring = false;
static volatile bool g_sensor_triggered  = false;

static uint8_t  g_pi_rsp_log[PI_RSP_LOG_SIZE];
static uint16_t g_pi_rsp_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void BeltTask(void *argument);
static void DisplayTask(void *argument);
static void CanRxTask(void *argument);
static void SensorMonitoring_Enable(void);
static void SensorMonitoring_Disable(void);
static bool Sensor_ConfirmDetected(void);
static void BeltStatus_Update(uint8_t stage, bool belt_on);
static void PiUart_FlushRx(void);
static void HandleObstacleDetected(void);

/* 신규 추가: 시작 센서 상태 읽기 함수 프로토타입 */
static bool ReadStartSensor(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* 기존 센서 검출 확인 (비전 검사 트리거용) */
bool ReadSensor(void)
{
  return (HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) == GPIO_PIN_RESET);
}

/* 신규 추가: PA5 시작 센서 상태 읽기 (물체 감지 시 true 반환) */
static bool ReadStartSensor(void)
{
  return (HAL_GPIO_ReadPin(START_SENSOR_GPIO_Port, START_SENSOR_Pin) == GPIO_PIN_RESET);
}

/* Arm EXTI interrupt for sensor detection. */
static void SensorMonitoring_Enable(void)
{
  g_sensor_triggered  = false;
  g_sensor_monitoring = false;
  __HAL_GPIO_EXTI_CLEAR_IT(SENSOR_Pin);
  g_sensor_monitoring = true;
}

/* Disarm EXTI interrupt and clear pending trigger flag. */
static void SensorMonitoring_Disable(void)
{
  g_sensor_monitoring = false;
  g_sensor_triggered  = false;
  __HAL_GPIO_EXTI_CLEAR_IT(SENSOR_Pin);
}

/* Read sensor three times with 1ms gaps to reject noise. */
static bool Sensor_ConfirmDetected(void)
{
  if (!ReadSensor()) return false;
  osDelay(1U);
  if (!ReadSensor()) return false;
  osDelay(1U);
  return ReadSensor();
}

/* EXTI callback:
   GPIO_PIN_0 (PB0) — MCP2515 INT: 메시지 수신 즉시 CanRxTask 기상
   GPIO_PIN_1 (PB1) — 컨베이어 센서: 릴레이 즉시 차단 후 BeltTask에 플래그 전달 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MCP2515_INT_Pin)
  {
    osSemaphoreRelease(sem_can_int);
    return;
  }

  if (GPIO_Pin != SENSOR_Pin || !g_sensor_monitoring)
    return;
  if (HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) != GPIO_PIN_RESET)
    return;

  for (volatile uint32_t i = 0U; i < 200U; i++)
    __NOP();

  if (HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) != GPIO_PIN_RESET)
    return;

  g_sensor_monitoring = false;
  g_sensor_triggered  = true;
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
}

/* Update belt state for DisplayTask consumption. */
static void BeltStatus_Update(uint8_t stage, bool belt_on)
{
  osMutexAcquire(beltMutex, osWaitForever);
  g_belt_status.machine_stage = stage;
  g_belt_status.belt_running  = belt_on ? BELT_STATE_RUNNING : BELT_STATE_STOPPED;
  osMutexRelease(beltMutex);
}

/* Drain any leftover bytes from the UART RX buffer. */
static void PiUart_FlushRx(void)
{
  uint8_t dummy;
  while (HAL_UART_Receive(&huart2, &dummy, 1U, 1U) == HAL_OK) {}
}

/* Send 'S' to Raspberry Pi immediately, receive one-byte inference result. */
static void HandleObstacleDetected(void)
{
  uint8_t tx_byte = UART_CMD_START;
  uint8_t rx_byte = UART_RSP_NG;

  HAL_StatusTypeDef rx_status;
  osDelay(500U);  /* 벨트 정지 후 카메라 안정화 대기 */
  PiUart_FlushRx();
  HAL_UART_Transmit(&huart2, &tx_byte, 1U, HAL_MAX_DELAY);
  rx_status = HAL_UART_Receive(&huart2, &rx_byte, 1U, UART_RX_TIMEOUT_MS);

  if (rx_status != HAL_OK ||
      (rx_byte != UART_RSP_RED && rx_byte != UART_RSP_BLUE && rx_byte != UART_RSP_NG))
    rx_byte = UART_RSP_NG;

  osMutexAcquire(piRspMutex, osWaitForever);
  g_pi_rsp_log[g_pi_rsp_count % PI_RSP_LOG_SIZE] = rx_byte;
  g_pi_rsp_count++;
  osMutexRelease(piRspMutex);

  /* 비전 검사 결과(색상 바이트)를 CAN으로 브로드캐스트 */
  static uint8_t s_can_seq = 0U;
  s_can_seq++;

  MCP2515_CanMsg can_tx = {
    .id       = CAN_ID_1ST_TX,
    .dlc      = 3U,
    .data     = {CAN_MSG_CONTAINER, s_can_seq, rx_byte},
    .extended = false,
  };
  osMutexAcquire(canMutex, osWaitForever);
  MCP2515_Send(&hmcp2515, &can_tx);
  osMutexRelease(canMutex);
}

/* Belt state machine.
   IDLE    : PA5 적외선 감지 센서 확인 -> 감지 시 1초 대기 -> FORWARD 전이
   FORWARD : holds actuator1 forward for 8s, then transitions to RUNNING.
   RUNNING : actuator1 backward, belt ON, sensor monitoring active. */
static void BeltTask(void *argument)
{
  (void)argument;

  const uint32_t forward_ms = 8000U;

  uint8_t  current_stage    = MACHINE_STAGE_IDLE;
  uint32_t stage_start_tick = osKernelGetTickCount();
  bool     sensor_active    = false;

  for (;;)
  {
    uint32_t current_tick = osKernelGetTickCount();

    switch (current_stage)
    {
      case MACHINE_STAGE_IDLE:
        SensorMonitoring_Disable();

        /* 1. 신규 PA5 적외선 센서 감지 상태 확인 */
        if (ReadStartSensor() == true)
        {
          /* 채터링 노이즈 방지를 위해 20ms 후 재확인 */
          osDelay(20U);
          if (ReadStartSensor() == true)
          {
            /* 2. 감지 후 1초(1000ms) 지연 대기 */
            osDelay(1000U);

            /* 3. 공정 구동(FORWARD 단계)으로 전이 */
            current_stage    = MACHINE_STAGE_FORWARD;
            stage_start_tick = osKernelGetTickCount();
            sensor_active    = false;
            printf("[SYSTEM] Start Sensor Triggered! 1s delayed. Starting FORWARD stage.\r\n");
          }
        }
        break;

      case MACHINE_STAGE_FORWARD:
        SensorMonitoring_Disable();
        if ((current_tick - stage_start_tick) >= forward_ms)
        {
          current_stage = MACHINE_STAGE_RUNNING;
          sensor_active = true;
          SensorMonitoring_Enable();
        }
        break;

      case MACHINE_STAGE_RUNNING:
        if (sensor_active && g_sensor_triggered)
        {
          if (Sensor_ConfirmDetected())
          {
            SensorMonitoring_Disable();
            sensor_active = false;

            HandleObstacleDetected();

            sensor_active = true;
            HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
            osDelay(SENSOR_REARM_DELAY_MS);
            SensorMonitoring_Enable();
          }
          else
          {
            g_sensor_triggered = false;
            SensorMonitoring_Enable();
          }
        }
        break;

      default:
        SensorMonitoring_Disable();
        current_stage = MACHINE_STAGE_IDLE;
        sensor_active = false;
        break;
    }

    bool belt_on = false;
    switch (current_stage)
    {
      case MACHINE_STAGE_IDLE:
        HAL_GPIO_WritePin(RELAY_GPIO_Port,    RELAY_Pin,    GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ACT1_IN1_GPIO_Port, ACT1_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ACT1_IN2_GPIO_Port, ACT1_IN2_Pin, GPIO_PIN_SET);
        break;

      case MACHINE_STAGE_FORWARD:
        HAL_GPIO_WritePin(RELAY_GPIO_Port,    RELAY_Pin,    GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ACT1_IN1_GPIO_Port, ACT1_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ACT1_IN2_GPIO_Port, ACT1_IN2_Pin, GPIO_PIN_RESET);
        break;

      case MACHINE_STAGE_RUNNING:
        belt_on = sensor_active && !g_belt_ext_stop;
        HAL_GPIO_WritePin(RELAY_GPIO_Port,    RELAY_Pin,    belt_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ACT1_IN1_GPIO_Port, ACT1_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ACT1_IN2_GPIO_Port, ACT1_IN2_Pin, GPIO_PIN_SET);
        break;

      default:
        break;
    }

    BeltStatus_Update(current_stage, belt_on);

    /* 기존 1U에서 10U로 변경하여 CPU 과점유 방지 (성능 최적화 적용) */
    osDelay(current_stage == MACHINE_STAGE_RUNNING ? 10U : 50U);
  }
}

static void DisplayTask(void *argument)
{
  (void)argument;
  for (;;)
    osDelay(osWaitForever);
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
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  HAL_Delay(500U);

  /* ── 1단계: UART 동작 확인 ── */
  printf("\r\n=== 1st Board Start ===\r\n");

  /* ── 2단계: MCP2515 SPI 초기화 확인 ── */
  hmcp2515.hspi    = &hspi1;
  hmcp2515.cs_port = MCP2515_CS_GPIO_Port;
  hmcp2515.cs_pin  = MCP2515_CS_Pin;
  hmcp2515.osc_hz  = MCP2515_OSC_8MHZ;

  if (MCP2515_AutoDetectOsc(&hmcp2515) != HAL_OK)
  {
    printf("[CAN] MCP2515 init FAILED — SPI 배선/CS 핀 확인\r\n");
    MCP2515_PrintDiag(&hmcp2515);
  }
  else
  {
    MCP2515_SetNormalMode(&hmcp2515);
    printf("[CAN] MCP2515 ready (8 MHz)\r\n");
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  beltMutex  = osMutexNew(NULL);
  piRspMutex = osMutexNew(NULL);
  canMutex   = osMutexNew(NULL);
  if (beltMutex == NULL || piRspMutex == NULL || canMutex == NULL)
    Error_Handler();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  sem_can_int = osSemaphoreNew(2U, 0U, NULL);  /* 최대 2 (MCP2515 RX 버퍼 수) */
  if (sem_can_int == NULL) Error_Handler();

  /* 세마포어 생성 후 NVIC 활성화 — ISR에서 NULL 접근 방지 */
  __HAL_GPIO_EXTI_CLEAR_IT(MCP2515_INT_Pin);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
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
  static const osThreadAttr_t belt_attr = {
    .name       = "BeltTask",
    .stack_size = 512U * 4U,
    .priority   = osPriorityNormal,
  };
  static const osThreadAttr_t display_attr = {
    .name       = "Display",
    .stack_size = 512U * 4U,
    .priority   = osPriorityBelowNormal,
  };
  static const osThreadAttr_t can_rx_attr = {
    .name       = "CanRxTask",
    .stack_size = 256U * 4U,
    .priority   = osPriorityNormal,
  };
  if (osThreadNew(BeltTask,    NULL, &belt_attr)    == NULL ||
      osThreadNew(DisplayTask, NULL, &display_attr) == NULL ||
      osThreadNew(CanRxTask,   NULL, &can_rx_attr)  == NULL)
    Error_Handler();
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 (B1 버튼은 더이상 사용하지 않으나 하드웨어 초기화 유지) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 (MCP2515 CS) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ── 변경 구역: PA5 핀을 시작 센서(입력)로 지정 ── */
  GPIO_InitStruct.Pin = START_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;     /* 노이즈 유입 방지를 위해 내부 풀업 설정 */
  HAL_GPIO_Init(START_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 (기존 인터럽트 핀들) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB8 PB9 (액추에이터 제어 등) */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Set actuator1 default position (backward) */
  HAL_GPIO_WritePin(ACT1_IN2_GPIO_Port, ACT1_IN2_Pin, GPIO_PIN_SET);

  /* CS must be HIGH (inactive) before first SPI transaction */
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void CanRxTask(void *argument)
{
  (void)argument;
  MCP2515_CanMsg rx;
  uint32_t diag_tick = 0U;

  printf("[CAN] CanRxTask started — waiting for INT\r\n");

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();
    if (now - diag_tick >= 10000U)
    {
      diag_tick = now;
      osMutexAcquire(canMutex, osWaitForever);
      printf("[CAN-DIAG] periodic ---\r\n");
      MCP2515_PrintDiag(&hmcp2515);
      osMutexRelease(canMutex);
    }

    osSemaphoreAcquire(sem_can_int, 100U);

    osMutexAcquire(canMutex, osWaitForever);
    HAL_StatusTypeDef st = MCP2515_Receive(&hmcp2515, &rx, 1U);
    osMutexRelease(canMutex);

    if (st == HAL_OK && rx.id == CAN_ID_2ND_TX && rx.dlc >= 1U)
    {
      if (rx.data[0] == CAN_CMD_BELT_STOP)
      {
        g_belt_ext_stop = true;
        printf("[CAN] BELT_STOP received\r\n");
      }
      else if (rx.data[0] == CAN_CMD_BELT_RESUME)
      {
        g_belt_ext_stop = false;
        printf("[CAN] BELT_RESUME received\r\n");
      }
      else
      {
        printf("[CAN] Unknown cmd id=0x%03lX data=0x%02X\r\n",
               (unsigned long)rx.id, rx.data[0]);
      }
    }
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;
  for (;;)
  {
    osDelay(osWaitForever);
  }
  /* USER CODE END 5 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
