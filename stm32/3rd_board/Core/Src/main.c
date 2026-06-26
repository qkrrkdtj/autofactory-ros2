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

/* 2번 액추에이터 제어 (IN3: PB12, IN4: PB13) */
#define ACT2_FORWARD()    do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); } while(0)
#define ACT2_BACKWARD()   do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   } while(0)
#define ACT2_STOP()       do { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); } while(0)

/* ========================================================================= */
/* [시스템 1: 뚜껑 공급 및 압착 제어 매크로 정의 - PC2 핀 매핑 수정 완료]     */
/* ========================================================================= */
/* PB8: 뚜껑 감지 센서 1 */
#define SENSOR1_CAP_DETECTED() (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)

/* 뚜껑 공급 액추에이터 제어 (IN1: PC2, IN2: PC3) 👈 PC2로 수정 완료 */
#define NEW_ACT_FORWARD()  do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); } while(0)
#define NEW_ACT_BACKWARD() do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);   } while(0)
#define NEW_ACT_STOP()     do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); } while(0)

/* 뚜껑 압착 액추에이터 제어 (IN3: PC0, IN4: PC1) */
#define PRESS_ACT_FORWARD()  do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); } while(0)
#define PRESS_ACT_BACKWARD() do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);   } while(0)
#define PRESS_ACT_STOP()     do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); } while(0)

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
  .stack_size = 256 * 4,
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

// OS 커널 시스템 타이머용 콜백 브릿지
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
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
  SET_SERVO2_PULSE(1000);
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

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  static const osThreadAttr_t canping_attr = {
    .name       = "CANPing",
    .stack_size = 256U * 4U,
    .priority   = osPriorityBelowNormal,
  };
  osThreadNew(CANPingTask, NULL, &canping_attr);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here */
  while (1)
  {
  }
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

        // STEP 1. 서보모터 1번 구동 (대기 2300 -> 역회전 하강 상태 800)
        SET_SERVO1_PULSE(800);
        osDelay(2000);

        // STEP 2. 서보모터 1번 미세 보정 위치 복귀 (원복 완료)
        SET_SERVO1_PULSE(2300);
        osDelay(2000);

        // STEP 3. 뚜껑 공급용 기계식 액추되이터 전진 (6초)
        NEW_ACT_FORWARD();
        osDelay(6000);

        NEW_ACT_STOP();
        osDelay(300);

        // STEP 4. 뚜껑 공급용 기계식 액추에이터 후진 복귀 (6.5초)
        NEW_ACT_BACKWARD();
        osDelay(6500);

        NEW_ACT_STOP();
        osDelay(500);

        // STEP 5. 신규 압착 액추에이터 전진 구동 (위에서 꾹 누르기 - 4초)
        printf("[SYS 1] Pressing Mechanism Active...\r\n");
        PRESS_ACT_FORWARD();
        osDelay(4000);

        PRESS_ACT_STOP();
        osDelay(300);

        // STEP 6. 신규 압착 액추에이터 복귀 후진 구동 (4초)
        PRESS_ACT_BACKWARD();
        osDelay(4000);

        PRESS_ACT_STOP();
        osDelay(200);
        printf("[SYS 1] Pressing Mechanism Completed.\r\n");

        // STEP 7. 서보모터 2번 꺾기 구동 (대기 1000 -> 회전 1500)
        SET_SERVO2_PULSE(1500);
        osDelay(2000);

        // STEP 8. 전체 서보축 최종 안전 대기 홈(Home) 위치 초기화 복귀 (원복 완료)
        SET_SERVO1_PULSE(2300);
        osDelay(2000);
        SET_SERVO2_PULSE(1000);
        osDelay(1000);

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
        SET_SERVO1_PULSE(2300);  // 원복 완료
        SET_SERVO2_PULSE(1000);
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
      osDelay(8000);

      ACT2_STOP();
      osDelay(300);

      ACT2_BACKWARD();
      osDelay(11000);

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
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/**
  * @brief TIM3 Initialization Function
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
}

/**
  * @brief USART2 Initialization Function
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
  HAL_UART_Init(&huart2);
}

/**
  * @brief SPI1 Initialization Function
  */
static void MX_SPI1_Init(void)
{
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
  HAL_SPI_Init(&hspi1);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 출력 레벨 초기 클리어 (PC4 제거하고 PC2로 일괄 수정 완료) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /* MCP2515 CS Pin (PA4) 초기화 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PC0, PC1, PC2, PC3 출력 포트 일괄 세팅 완료 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PA5 상태 제어 LED */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB4, PB5 서보모터용 PWM 출력 핀 설정 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 시스템 2 L298N 및 Relay 출력 포트 세팅 (PB12, PB13, PB14) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 입력 핀 설정 (PB0: CAN INT, PB2: 센서2, PB8: 센서1 👈 인풋 모드로 확인 완료) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_8;
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
