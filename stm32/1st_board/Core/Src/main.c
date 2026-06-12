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
/* constants moved to main.h (shared with freertos.c) */
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
uint8_t               g_node_id;
NodeState             node_states[NODE_COUNT + 1U];

osMutexId_t        spiMutex;
osMutexId_t        stateMutex;
osMessageQueueId_t canTxQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef Flash_ReadNodeId(uint8_t *node_id);
static HAL_StatusTypeDef Flash_WriteNodeId(uint8_t node_id);
static HAL_StatusTypeDef Flash_EraseNodeId(void);
static uint8_t SelectAndSaveNodeId(void);
static void SensorTxTask(void *argument);
static void CANManagerTask(void *argument);
static void DisplayTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_Print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static HAL_StatusTypeDef Flash_ReadNodeId(uint8_t *node_id)
{
  uint32_t magic = *(__IO uint32_t *)FLASH_NODE_ID_ADDR;
  uint32_t id    = *(__IO uint32_t *)(FLASH_NODE_ID_ADDR + 4U);

  if (magic == FLASH_NODE_ID_MAGIC && id >= 1U && id <= NODE_COUNT)
  {
    *node_id = (uint8_t)id;
    return HAL_OK;
  }
  return HAL_ERROR;
}

static HAL_StatusTypeDef Flash_EraseNodeId(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;

  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector       = FLASH_SECTOR_7;
  erase.NbSectors    = 1U;

  HAL_FLASH_Unlock();
  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
  HAL_FLASH_Lock();
  return status;
}

static HAL_StatusTypeDef Flash_WriteNodeId(uint8_t node_id)
{
  if (Flash_EraseNodeId() != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_FLASH_Unlock();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_NODE_ID_ADDR,       FLASH_NODE_ID_MAGIC);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_NODE_ID_ADDR + 4U,  (uint32_t)node_id);
  HAL_FLASH_Lock();
  return HAL_OK;
}

static uint8_t SelectAndSaveNodeId(void)
{
  uint8_t stored_id = 0U;

  if (Flash_ReadNodeId(&stored_id) == HAL_OK)
  {
    printf("Stored Node ID found: Node %u\r\n", stored_id);
    printf("Hold USER button 3s to reset, or wait...\r\n");

    bool held = true;
    for (uint32_t i = 0U; i < 30U; i++)
    {
      if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET)
      {
        held = false;
        break;
      }
      HAL_Delay(100);
    }

    if (!held)
    {
      return stored_id;
    }

    printf("Node ID reset.\r\n");
    Flash_EraseNodeId();
  }

  printf("Node ID selection:\r\n");
  printf("  Press USER button 1~4 times, then wait 2s to confirm.\r\n");
  printf("  Waiting for first press...\r\n");

  while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET)
  {
    HAL_Delay(10);
  }

  uint8_t count = 0U;
  uint32_t last_press = HAL_GetTick();

  while (1)
  {
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
    {
      count++;
      printf("  Press count: %u\r\n", count);
      last_press = HAL_GetTick();
      while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
      {
        HAL_Delay(10);
      }
      HAL_Delay(50);
    }

    if ((HAL_GetTick() - last_press) >= 2000U)
    {
      break;
    }
  }

  if (count < 1U || count > NODE_COUNT)
  {
    printf("Invalid count (%u), defaulting to Node 1\r\n", count);
    count = 1U;
  }

  if (Flash_WriteNodeId(count) == HAL_OK)
  {
    printf("Node %u saved to Flash.\r\n", count);
  }

  return count;
}

bool ReadSensor(void)
{
  return (HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) == GPIO_PIN_RESET);
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

  UART_Print("\r\n\r\n=== STM32 CAN Test ===\r\n");
  UART_Print("Serial: 115200 8N1\r\n");
  UART_Print("Waiting 2s... (open terminal, then press RESET)\r\n");
  HAL_Delay(2000);

  hmcp2515.hspi = &hspi1;
  hmcp2515.cs_port = MCP2515_CS_GPIO_Port;
  hmcp2515.cs_pin = MCP2515_CS_Pin;
  hmcp2515.osc_hz = MCP2515_OSC_8MHZ;

  printf("\r\nMCP2515 CAN Multi-Node (%u boards)\r\n", NODE_COUNT);
  printf("Auto-detect crystal (8/16 MHz), bitrate: 125kbps\r\n");

  if (MCP2515_AutoDetectOsc(&hmcp2515) != HAL_OK)
  {
    printf("MCP2515 init failed\r\n");
    MCP2515_PrintDiag(&hmcp2515);
    Error_Handler();
  }

  printf("Crystal: %lu MHz detected\r\n",
         (unsigned long)(hmcp2515.osc_hz / 1000000U));

  if (MCP2515_SetNormalMode(&hmcp2515) != HAL_OK)
  {
    printf("Normal mode failed\r\n");
    Error_Handler();
  }

  g_node_id = SelectAndSaveNodeId();
  printf("Node %u ready. Starting FreeRTOS...\r\n", g_node_id);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  spiMutex   = osMutexNew(NULL);
  stateMutex = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  canTxQueue = osMessageQueueNew(8U, sizeof(MCP2515_CanMsg), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  static const osThreadAttr_t sensorTx_attr = {
    .name       = "SensorTx",
    .stack_size = 256U * 4U,
    .priority   = osPriorityNormal,
  };
  static const osThreadAttr_t canMgr_attr = {
    .name       = "CANMgr",
    .stack_size = 512U * 4U,
    .priority   = osPriorityAboveNormal,
  };
  static const osThreadAttr_t display_attr = {
    .name       = "Display",
    .stack_size = 512U * 4U,
    .priority   = osPriorityBelowNormal,
  };
  osThreadNew(SensorTxTask,   NULL, &sensorTx_attr);
  osThreadNew(CANManagerTask, NULL, &canMgr_attr);
  osThreadNew(DisplayTask,    NULL, &display_attr);
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
    HAL_Delay(1000);
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /* **[추가된 설정]** 부팅 시 릴레이의 초기 출력 레벨 설정 (Active Low 기준 초기 OFF 상태 유지) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* **[추가된 설정]** PB10 (릴레이 제어 핀) GPIO 설정 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* CS pin must start HIGH (inactive) — CubeMX generates RESET incorrectly */
  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);

  /* Re-apply pull-ups for INT (PB0) and SENSOR (PB1) */
  GPIO_InitStruct.Pin  = MCP2515_INT_Pin | SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void SensorTxTask(void *argument)
{
  (void)argument;
  bool     last_sensor  = false;
  uint32_t last_tx_tick = 0U;

  osMutexAcquire(stateMutex, osWaitForever);
  node_states[g_node_id].valid = true;
  osMutexRelease(stateMutex);

  for (;;)
  {
    bool     sensor_now = ReadSensor();
    uint32_t now        = osKernelGetTickCount();

    osMutexAcquire(stateMutex, osWaitForever);
    node_states[g_node_id].sensor = sensor_now;
    osMutexRelease(stateMutex);

    if (sensor_now != last_sensor || (now - last_tx_tick) >= 200U)
    {
      last_sensor  = sensor_now;
      last_tx_tick = now;

      MCP2515_CanMsg msg = {
          .id       = CAN_ID_BASE + g_node_id,
          .dlc      = 1U,
          .data     = {sensor_now ? 0x01U : 0x00U},
          .extended = false,
      };
      osMessageQueuePut(canTxQueue, &msg, 0U, 0U);
    }
    osDelay(50U);
  }
}

static void CANManagerTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    MCP2515_CanMsg tx_msg;
    if (osMessageQueueGet(canTxQueue, &tx_msg, NULL, 0U) == osOK)
    {
      osMutexAcquire(spiMutex, osWaitForever);
      if (MCP2515_Send(&hmcp2515, &tx_msg) != HAL_OK)
      {
        MCP2515_RecoverBus(&hmcp2515);
      }
      osMutexRelease(spiMutex);
    }

    MCP2515_CanMsg rx_msg = {0};
    osMutexAcquire(spiMutex, osWaitForever);
    HAL_StatusTypeDef rx_status = MCP2515_Receive(&hmcp2515, &rx_msg, 5U);
    osMutexRelease(spiMutex);

    if (rx_status == HAL_OK)
    {
      uint32_t src = rx_msg.id - CAN_ID_BASE;
      if (src >= 1U && src <= NODE_COUNT && src != (uint32_t)g_node_id && rx_msg.dlc >= 1U)
      {
        osMutexAcquire(stateMutex, osWaitForever);
        node_states[src].sensor = (rx_msg.data[0] != 0U);
        node_states[src].valid  = true;
        osMutexRelease(stateMutex);
      }
    }

    osDelay(5U);
  }
}

static void DisplayTask(void *argument)
{
  (void)argument;
  NodeState snapshot[NODE_COUNT + 1U];

  for (;;)
  {
    osMutexAcquire(stateMutex, osWaitForever);
    for (uint8_t i = 0U; i <= NODE_COUNT; i++)
      snapshot[i] = node_states[i];
    osMutexRelease(stateMutex);

    for (uint8_t i = 1U; i <= NODE_COUNT; i++)
    {
      const char *label;
      if (!snapshot[i].valid)
        label = "---";
      else
        label = snapshot[i].sensor ? "DETECTED" : "CLEAR";

      if (i == 1U)
        printf("N%u[%-8s]", i, label);
      else
        printf(" N%u[%-8s]", i, label);
    }
    printf("\r\n");

    osDelay(500U);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // 시스템 상태를 관리하는 열거형
  typedef enum {
    STAGE_FORWARD,       // 평상시: 컨베이어 작동 + 5초 전진
    STAGE_BACKWARD,      // 평상시: 컨베이어 작동 + 5초 후진
    STAGE_EMERGENCY_LOCK // 비상 상황: 컨베이어 정지 + 10초 강제 후진
  } ActuatorStage;

  ActuatorStage current_stage = STAGE_FORWARD;
  uint32_t stage_start_tick = osKernelGetTickCount();

  for (;;)
  {
    bool detected = ReadSensor();
    uint32_t current_tick = osKernelGetTickCount();

    /* =============================================================
     * 1. 상태 및 타이머 제어 로직 (State Machine)
     * ============================================================= */
    if (current_stage == STAGE_EMERGENCY_LOCK)
    {
      // [비상 모드] 벨트가 멈추고 액추에이터가 후진하는 중... 5초 체크
      if ((current_tick - stage_start_tick) >= 5000U)
      {
        // 10초가 지났고, 센서 앞에 장애물도 완전히 청소되었다면 평상시로 복귀
        if (!ReadSensor())
        {
          current_stage = STAGE_FORWARD;
          stage_start_tick = current_tick;
          printf("[SYSTEM] Emergency Clear! Belt RESTART / Actuator FORWARD\r\n");
        }
        else
        {
          // 10초가 지났는데도 여전히 물체가 감지 중이라면 비상 상태(후진/벨트정지) 연장
          stage_start_tick = current_tick;
          printf("[SYSTEM] Obstacle still remains. Extending Emergency...\r\n");
        }
      }
    }
    else
    {
      // [평상시 모드] 컨베이어가 잘 돌고 있는 와중에 물체가 감지되면 즉시 비상 모드 진입
      if (detected)
      {
        current_stage = STAGE_EMERGENCY_LOCK;
        stage_start_tick = current_tick; // 10초 카운트다운 시작
        printf("[ALERT] Obstacle Detected! Conveyor STOP / Actuator Emergency Backward\r\n");
      }
      else
      {
        // 평상시: 컨베이어 벨트는 계속 도는 와중에, 액추에이터만 5초 간격 교대 작동
        if ((current_tick - stage_start_tick) >= 5000U)
        {
          if (current_stage == STAGE_FORWARD)
          {
            current_stage = STAGE_BACKWARD;
            printf("[NORMAL] 5s passed. Actuator -> BACKWARD (Conveyor Running)\r\n");
          }
          else
          {
            current_stage = STAGE_FORWARD;
            printf("[NORMAL] 5s passed. Actuator -> FORWARD (Conveyor Running)\r\n");
          }
          stage_start_tick = current_tick;
        }
      }
    }

    /* =============================================================
     * 2. 하드웨어 출력 제어 (릴레이 및 L298N 드라이빙)
     * ============================================================= */
    if (current_stage == STAGE_EMERGENCY_LOCK)
    {
      /* -----------------------------------------------------------
       * [비상 상황] 컨베이어 벨트 즉시 정지 & 액추에이터 10초 후진
       * ----------------------------------------------------------- */
      // Active High 릴레이 기준: LOW = 릴레이 완전히 OFF (컨베이어 정지)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

      // L298N 모터 드라이버 후진 신호 인가
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // IN1 = LOW
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);    // IN2 = HIGH
    }
    else
    {
      /* -----------------------------------------------------------
       * [평상시] 컨베이어 벨트 무조건 작동 (ON) & 액추에이터 5초 교대
       * ----------------------------------------------------------- */
      // Active High 릴레이 기준: HIGH = 릴레이 항상 ON (컨베이어 계속 작동)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

      if (current_stage == STAGE_FORWARD)
      {
        // 5초 동안 전진 구동
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // IN1 = HIGH
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // IN2 = LOW
      }
      else
      {
        // 5초 동안 후진 구동
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // IN1 = LOW
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // IN2 = HIGH
      }
    }

    // 50ms 주기로 센서 및 타이머 상태를 매우 빠르게 스캔
    osDelay(50U);
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
  * where the assert_param error has occurred.
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
