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

/* 뚜껑 공급 액추에이터 제어 (IN1: PC0, IN2: PC5) */
#define NEW_ACT_FORWARD()  do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); } while(0)
#define NEW_ACT_BACKWARD() do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   } while(0)
#define NEW_ACT_STOP()     do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); } while(0)

/* 뚜껑 압착 액추에이터 제어 (IN3: PC2, IN4: PC3) */
#define PRESS_ACT_FORWARD()  do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); } while(0)
#define PRESS_ACT_BACKWARD() do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);   } while(0)
#define PRESS_ACT_STOP()     do { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); } while(0)

/* 서보모터 펄스 제어 편의 매크로 (TIM3 하드웨어 매핑)
 * 서보1 (PB4, TIM3_CH1) : 뚜껑 색상 선택 — 약통 색상에 맞는 뚜껑 위치로 이동
 * 서보2 (PB1, TIM3_CH4) : 약통 스토퍼 — 약통을 멈춰 정위치 고정 */
#define SET_SERVO1_PULSE(p) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (p))  /* 서보1: 뚜껑 색상 선택 */
#define SET_SERVO2_PULSE(p) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (p))  /* 서보2: 약통 스토퍼 */

/* 서보1(뚜껑 색상 선택) 펄스 위치 (us) — 현장 튜닝 시 이 값만 수정 */
#define SERVO1_PULSE_RED_CAP   1040U   /* 빨간 뚜껑 */
#define SERVO1_PULSE_BLUE_CAP  2300U  /* 파란 뚜껑 (그 외 색상) */
#define SERVO1_PULSE_HOME      SERVO1_PULSE_RED_CAP  /* 대기/홈 복귀 */

/* 서보2(약통 스토퍼) 펄스·램핑 — 현장 튜닝 시 이 값만 수정 */
#define SERVO2_PULSE_OPEN     2050U  /* 스토퍼 개방 (약통 통과) */
#define SERVO2_PULSE_CLOSED    820U  /* 스토퍼 차단 (약통 고정) */
#define SERVO2_PULSE_HOME      SERVO2_PULSE_CLOSED    /* 대기/홈 위치 */
#define SERVO2_RAMP_STEP         10U  /* 램핑 1스텝당 펄스 변화량 (us) */
#define SERVO2_RAMP_DELAY_MS     20U  /* 램핑 1스텝당 대기 (ms) */

/* 큐 깊이 */
#define SYS1_QUEUE_DEPTH         8U   /* 공정1(뚜껑) 약통 정보 대기 큐 */
#define SYS2_QUEUE_DEPTH         8U   /* 공정2(분류) 약통 정보 대기 큐 */

/* 공정1(뚜껑/압착) 타이밍 (ms) */
#define CAP_ALIGN_BELT_MS         2000U  /* PB8 감지 후 정위치까지 벨트 추가 가동 */
#define SERVO1_MOVE_MS            2000U  /* 서보1(뚜껑 색상) 이동 후 안정 대기 */
#define NEW_ACT_FORWARD_MS        7600U  /* 뚜껑 공급 액추에이터 전진 */
#define NEW_ACT_BACKWARD_MS       8000U  /* 뚜껑 공급 액추에이터 후진(홈) */
#define NEW_ACT_NUDGE_MS          2000U  /* 뚜껑 공급: 중간 후진/전진 넛지 시간 */
#define PRESS_ACT_FORWARD_1ST_MS  3000U  /* 압착 1차 전진 (약통 위치 고정용, 살짝 내려옴) */
#define PRESS_ACT_FORWARD_2ND_MS  2300U  /* 압착 2차 전진 (끝까지 압착, 1차+2차 = 5.3초) */
#define PRESS_ACT_BACKWARD_MS     6000U  /* 압착 액추에이터 후진(홈) */
#define CAP_PASS_AFTER_PRESS_MS   2000U  /* 압착 완료 후 약통 통과 대기 */
#define ACT_STOP_SHORT_MS          300U  /* 액추에이터 정지 후 짧은 안정화 */
#define ACT_STOP_MED_MS            500U  /* 액추에이터 정지 후 중간 안정화 */
#define ACT_STOP_BRIEF_MS          200U  /* 액추에이터 정지 후 최소 안정화 */

/* 공정2(분류) 타이밍 (ms) */
#define ACT2_FORWARD_MS       10000U  /* 분류 액추에이터 전진 */
#define ACT2_BACKWARD_MS      13000U  /* 분류 액추에이터 후진(홈) */

/* 부팅·초기화 */
#define INIT_ACTUATOR_HOME_MS ACT2_BACKWARD_MS  /* 부팅 시 홈 후진 (가장 긴 후진 기준) */

/* NG(불량) 약통 우회 동작 타이밍 */
#define NG_CAP_RELEASE_DELAY_MS  1000U  /* 공정1: 스토퍼 개방 전 대기 */
#define NG_CAP_PASS_MS           2000U  /* 공정1: NG 스토퍼 개방 후 약통 통과 대기 */
#define NG_REJECT_BELT_MS        3000U  /* 공정2: 불량 박스까지 벨트 가동 시간 */
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

osMutexId_t        canMutex;            /* MCP2515 SPI 동시 접근 방지 */
osSemaphoreId_t    sem_can_rx;          /* MCP2515 INT 핀 낙하 → CANPingTask 즉시 기상 */
osMessageQueueId_t sys1Queue;           /* 공정1(뚜껑): CAN 수신 → PB8 처리 대기 (최대 8) */
osMessageQueueId_t sys2Queue;           /* 공정2(액추에이터2): 공정1 완료 → PB2 처리 대기 (최대 8) */

uint8_t cap_sequence_done = 0;

/* 공유 벨트(릴레이 PB14) 조정 플래그 — 정지 우선(stop-wins)
 *  - *_belt_stop : 해당 공정의 기계 시퀀스 중 벨트 정지 요청 (어느 쪽이든 1이면 정지)
 *  - *_belt_run  : 큐가 비어도 벨트를 가동해야 하는 구간(정위치/NG 배출). 정지 요청에는 양보
 * 우선순위: 정지요청 > (가동요청 또는 큐 작업) > 정지.
 * 벨트 릴레이는 BeltMgrTask 단독으로 기록한다(다중 기록 경쟁 방지). */
volatile uint8_t cap_belt_stop  = 0U;
volatile uint8_t sort_belt_stop = 0U;
volatile uint8_t cap_belt_run   = 0U;
volatile uint8_t sort_belt_run  = 0U;
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
static void SortTask(void *argument);           /* 공정2: 분류 액추에이터 (PB2) */
static void BeltMgrTask(void *argument);        /* 공유 벨트 릴레이 단독 제어 */
static void CAN_SendSlotAvailable(void);        /* 공정1 완료 → 1번 보드에 역압력 해제 신호 */
static void CapActuators_SetIdleBackward(void); /* 공정1 액추에이터 대기 = 후진(홈) */
static void SortActuator_SetIdleBackward(void); /* 공정2 액추에이터 대기 = 후진(홈) */
static void AllActuators_SetIdleBackward(void); /* 전체 액추에이터 대기 = 후진(홈) */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* 약통 seq 연속성 검증: 직전 seq+1과 다르면 누락/중복으로 보고 후 현재 값으로 재동기화한다.
 * last_seq/have_last는 호출부의 static 상태를 가리킨다. */
static void Seq_CheckContinuity(const char *tag, uint8_t seq,
                                uint8_t *last_seq, uint8_t *have_last)
{
  if (*have_last)
  {
    uint8_t expected = (uint8_t)(*last_seq + 1U);
    if (seq != expected)
      printf("%s 약통 번호 불연속! (기대 %u, 수신 %u) — 정보 누락 가능, 재동기화\r\n",
             tag, expected, seq);
  }
  *last_seq  = seq;
  *have_last = 1U;
}

/* 서보2(약통 스토퍼) 현재 펄스폭(us) - 램핑 이동의 시작점으로 사용 */
static uint16_t g_servo2_pulse = SERVO2_PULSE_HOME;

static void CapActuators_SetIdleBackward(void)
{
  NEW_ACT_BACKWARD();
  PRESS_ACT_BACKWARD();
}

static void SortActuator_SetIdleBackward(void)
{
  ACT2_BACKWARD();
}

static void AllActuators_SetIdleBackward(void)
{
  NEW_ACT_BACKWARD();
  PRESS_ACT_BACKWARD();
  ACT2_BACKWARD();
}

/* 공정1 완료 시 1번 보드로 수용 가능 슬롯 신호를 전송한다.
 * 이 신호를 받은 1번 보드는 다음 약통 사이클을 시작할 수 있다. */
static void CAN_SendSlotAvailable(void)
{
  MCP2515_CanMsg tx = {
    .id       = CAN_ID_3RD_TX,
    .dlc      = 1U,
    .data     = {CAN_CMD_SLOT_AVAILABLE},
    .extended = false,
  };
  osMutexAcquire(canMutex, osWaitForever);
  (void)MCP2515_Send(&hmcp2515, &tx);
  osMutexRelease(canMutex);
  printf("[슬롯] 1번 보드에 투입 가능 신호 전송\r\n");
}

/* 서보2(약통 스토퍼)를 target까지 step(us)씩 끊어 보내며 천천히 이동시킨다.
 * step을 작게 / step_delay_ms를 크게 할수록 더 느리고 부드럽게 움직인다. */
static void Servo2_MoveRamp(uint16_t target, uint16_t step, uint16_t step_delay_ms)
{
  int32_t cur = (int32_t)g_servo2_pulse;
  int32_t tgt = (int32_t)target;
  int32_t st  = (step == 0U) ? 1 : (int32_t)step;

  if (cur < tgt)
  {
    for (int32_t p = cur + st; p < tgt; p += st)
    {
      SET_SERVO2_PULSE((uint16_t)p);
      osDelay(step_delay_ms);
    }
  }
  else
  {
    for (int32_t p = cur - st; p > tgt; p -= st)
    {
      SET_SERVO2_PULSE((uint16_t)p);
      osDelay(step_delay_ms);
    }
  }

  SET_SERVO2_PULSE(target);
  g_servo2_pulse = target;
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
  printf("\r\n=== [3번 보드] 시작 (공정1 뚜껑/압착 + 공정2 분류) ===\r\n");

  // 서보모터용 PWM 하드웨어 드라이버 구동 시작
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // 시스템 1 하드웨어 대기 홈(Home) 위치 빌드 및 정지 상태 고정 (원복 완료)
  SET_SERVO1_PULSE(SERVO1_PULSE_HOME);
  SET_SERVO2_PULSE(SERVO2_PULSE_HOME);
  AllActuators_SetIdleBackward();

  // 시스템 2 초기 하드웨어 상태 빌드
  RELAY_OFF();

  // MCP2515 CAN 컨트롤러 인터페이스 구성
  hmcp2515.hspi    = &hspi1;
  hmcp2515.cs_port = MCP2515_CS_GPIO_Port;
  hmcp2515.cs_pin  = MCP2515_CS_Pin;
  hmcp2515.osc_hz  = MCP2515_OSC_8MHZ;

  if (MCP2515_AutoDetectOsc(&hmcp2515) != HAL_OK)
  {
    printf("[통신] CAN 초기화 실패\r\n");
    MCP2515_PrintDiag(&hmcp2515);
  }
  else
  {
    MCP2515_SetNormalMode(&hmcp2515);
    printf("[통신] CAN 연결 완료 (%lu MHz)\r\n", (unsigned long)(hmcp2515.osc_hz / 1000000U));
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  canMutex = osMutexNew(NULL);
  if (canMutex == NULL) Error_Handler();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  sem_can_rx = osSemaphoreNew(2U, 0U, NULL);
  if (sem_can_rx == NULL) Error_Handler();

  __HAL_GPIO_EXTI_CLEAR_IT(MCP2515_INT_Pin);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  sys1Queue = osMessageQueueNew(SYS1_QUEUE_DEPTH, sizeof(ContainerInfo), NULL);
  sys2Queue = osMessageQueueNew(SYS2_QUEUE_DEPTH, sizeof(ContainerInfo), NULL);
  if (sys1Queue == NULL || sys2Queue == NULL) Error_Handler();
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

  /* 공정2(분류)를 공정1(StartDefaultTask)과 병렬 실행 */
  static const osThreadAttr_t sort_attr = {
    .name       = "SortTask",
    .stack_size = 256U * 4U,
    .priority   = osPriorityNormal,
  };
  osThreadNew(SortTask, NULL, &sort_attr);

  /* 공유 벨트 릴레이를 단독으로 제어하는 매니저 태스크 */
  static const osThreadAttr_t beltmgr_attr = {
    .name       = "BeltMgr",
    .stack_size = 128U * 4U,
    .priority   = osPriorityAboveNormal,
  };
  osThreadNew(BeltMgrTask, NULL, &beltmgr_attr);
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 PC3 PC4
                           PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MCP2515_INT_Pin)
    osSemaphoreRelease(sem_can_rx);
}

static void CANPingTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    osSemaphoreAcquire(sem_can_rx, 100U);  /* INT 낙하 즉시 기상, 100ms 타임아웃 폴백 */

    MCP2515_CanMsg rx = {0};
    osMutexAcquire(canMutex, osWaitForever);
    HAL_StatusTypeDef rx_st = MCP2515_Receive(&hmcp2515, &rx, 1U);
    osMutexRelease(canMutex);
    if (rx_st == HAL_OK)
    {
      if (rx.id == CAN_ID_1ST_TX && rx.dlc >= 3U && rx.data[0] == CAN_MSG_CONTAINER)
      {
        ContainerInfo info = { .seq = rx.data[1], .color = (char)rx.data[2] };
        if (osMessageQueuePut(sys1Queue, &info, 0U, 0U) == osOK)
        {
          printf("[수신] 약통 도착 (번호 %u, 색상 %c) — 공정1 대기 %lu개\r\n",
                 info.seq, info.color,
                 (unsigned long)osMessageQueueGetCount(sys1Queue));
        }
        else
        {
          printf("[수신] 공정1 대기열 가득참 — 약통 번호 %u 누락\r\n", info.seq);
        }
      }
      else if (rx.id == CAN_ID_2ND_TX || rx.id == CAN_ID_3RD_TX)
      {
        /* 다른 보드(1번)가 처리할 벨트/슬롯 제어 메시지 — 조용히 무시 */
      }
      else
      {
        printf("[통신] 알 수 없는 메시지 무시 (ID=0x%03lX)\r\n", (unsigned long)rx.id);
      }
    }
  }
}

/* [공정2] 분류 액추에이터(ACT2, PB2) — 공정1(StartDefaultTask)과 병렬 실행.
 * 벨트 릴레이는 직접 만지지 않고 sort_belt_stop / sort_belt_run 플래그로 BeltMgrTask에 요청한다. */
static void SortTask(void *argument)
{
  (void)argument;
  uint8_t sys2_sequence_done = 0U;

  for (;;)
  {
    if (SENSOR2_DETECTED())
    {
      if (sys2_sequence_done == 0U)
      {
        sys2_sequence_done = 1U;

        static uint8_t s2_last_seq  = 0U;
        static uint8_t s2_have_last = 0U;

        ContainerInfo info = { .seq = 0U, .color = '?' };
        if (osMessageQueueGet(sys2Queue, &info, NULL, 0U) == osOK)
        {
          printf("[공정2] 약통 확인 (번호 %u, 색상 %c) — 대기 %lu개\r\n",
                 info.seq, info.color,
                 (unsigned long)osMessageQueueGetCount(sys2Queue));
          Seq_CheckContinuity("[공정2]", info.seq, &s2_last_seq, &s2_have_last);
        }
        else
        {
          printf("[공정2] 대기열 비어있음 — 감지된 약통 처리\r\n");
        }

        if (info.color == 'N')
        {
          /* NG: 분류 액추에이터 생략, 벨트 계속 가동해 불량 박스로 배출 */
          printf("[공정2] 불량 약통 — 분류 건너뛰고 불량 박스로 배출\r\n");
          sort_belt_run = 1U;
          osDelay(NG_REJECT_BELT_MS);
          sort_belt_run = 0U;

          /* NG는 공정2까지 완료된 뒤 슬롯 반환 (공정1에서는 반환하지 않음) */
          CAN_SendSlotAvailable();
          printf("[공정2] 불량 배출 완료 — 다음 약통 투입 가능\r\n");
        }
        else
        {
          printf("[공정2] 약통 감지 — 벨트 정지 후 분류 동작 시작\r\n");

          sort_belt_stop = 1U;              /* 분류 액추에이터 동작 동안 벨트 정지 요청 */
          osDelay(50);

          printf("[공정2] 분류 액추에이터 전진\r\n");
          ACT2_FORWARD();
          osDelay(ACT2_FORWARD_MS);

          printf("[공정2] 분류 액추에이터 정지\r\n");
          ACT2_STOP();
          osDelay(ACT_STOP_SHORT_MS);

          printf("[공정2] 분류 액추에이터 후진\r\n");
          ACT2_BACKWARD();
          osDelay(ACT2_BACKWARD_MS);

          SortActuator_SetIdleBackward();
          osDelay(50);

          sort_belt_stop = 0U;
          printf("[공정2] 분류 완료 — 벨트 재가동\r\n");
          osDelay(ACT_STOP_MED_MS);
        }
      }

      osDelay(50);
    }
    else
    {
      sys2_sequence_done = 0U;
      SortActuator_SetIdleBackward();
      osDelay(20);
    }
  }
}

/* 공유 벨트 릴레이(PB14) 단독 제어 — 정지 우선(stop-wins)
 *  정지요청(어느 공정이든) > (가동요청 또는 큐 작업) > 정지 */
static void BeltMgrTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    uint8_t stop = (cap_belt_stop || sort_belt_stop);
    uint8_t run  = (cap_belt_run  || sort_belt_run);
    uint8_t work = (osMessageQueueGetCount(sys1Queue) > 0U) ||
                   (osMessageQueueGetCount(sys2Queue) > 0U);

    if (!stop && (run || work))
      RELAY_ON();
    else
      RELAY_OFF();

    osDelay(20);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* [공정1] 뚜껑 공급 + 신규 압착 — 공정2(SortTask)와 병렬 실행.
   * 벨트 릴레이는 직접 만지지 않고 cap_belt_stop / cap_belt_run 플래그로 BeltMgrTask에 요청한다. */
  (void)argument;
  static GPIO_PinState prev_sensor1_state = GPIO_PIN_SET;

  /* 부팅 시 모든 액추에이터를 후진(홈) 구동 후 대기 방향 유지 */
  printf("[초기화] 모든 액추에이터 후진(홈)\r\n");
  AllActuators_SetIdleBackward();
  osDelay(INIT_ACTUATOR_HOME_MS);
  AllActuators_SetIdleBackward();
  printf("[초기화] 액추에이터 홈 복귀 완료\r\n");

  for (;;)
  {
    if (SENSOR1_CAP_DETECTED())
    {
      if (cap_sequence_done == 0)
      {
        cap_sequence_done = 1;

        /* 공정1 큐에서 약통 정보 꺼내기 */
        static uint8_t s1_last_seq  = 0U;
        static uint8_t s1_have_last = 0U;

        ContainerInfo info = { .seq = 0U, .color = '?' };
        if (osMessageQueueGet(sys1Queue, &info, NULL, 0U) == osOK)
        {
          printf("[공정1] 약통 확인 (번호 %u, 색상 %c) — 대기 %lu개\r\n",
                 info.seq, info.color,
                 (unsigned long)osMessageQueueGetCount(sys1Queue));
          Seq_CheckContinuity("[공정1]", info.seq, &s1_last_seq, &s1_have_last);
        }
        else
        {
          printf("[공정1] 대기열 비어있음 — 감지된 약통 처리\r\n");
        }

        if (info.color == 'N')
        {
          /* NG: 뚜껑/압착 생략. 벨트 가동 유지 후 스토퍼(서보2) 개방으로 약통 통과 */
          printf("[공정1] 불량 약통 — 뚜껑/압착 건너뛰고 통과시킴\r\n");
          cap_belt_run = 1U;                /* 큐가 비어도 벨트 가동(정지 요청에는 양보) */
          osDelay(NG_CAP_RELEASE_DELAY_MS);
          Servo2_MoveRamp(SERVO2_PULSE_OPEN, SERVO2_RAMP_STEP, SERVO2_RAMP_DELAY_MS);  /* 스토퍼 개방 → 약통 전진 */
          osDelay(NG_CAP_PASS_MS);
          Servo2_MoveRamp(SERVO2_PULSE_HOME, SERVO2_RAMP_STEP, SERVO2_RAMP_DELAY_MS);  /* 스토퍼 원위치 복귀 */
          cap_belt_run = 0U;
        }
        else
        {
          /* 감지 후 정위치까지 벨트를 더 진행시킨 뒤, 공정 동안 벨트 정지 요청 */
          cap_belt_run = 1U;
          osDelay(CAP_ALIGN_BELT_MS);
          cap_belt_run  = 0U;
          cap_belt_stop = 1U;

          // STEP 1. 스토퍼 개방 (압착 기구가 약통을 고정하므로 공정 내내 오픈 유지)
          printf("[공정1] 스토퍼 오픈\r\n");
          Servo2_MoveRamp(SERVO2_PULSE_OPEN, SERVO2_RAMP_STEP, SERVO2_RAMP_DELAY_MS);

          // STEP 2. 압착 1차 (약통 위치 고정 — 살짝 내려옴)
          printf("[공정1] 압착 1차 전진\r\n");
          PRESS_ACT_FORWARD();
          osDelay(PRESS_ACT_FORWARD_1ST_MS);
          PRESS_ACT_STOP();
          osDelay(ACT_STOP_SHORT_MS);

          // STEP 3. 뚜껑 색상 선택
          uint16_t cap_pulse = (info.color == 'R') ? SERVO1_PULSE_RED_CAP : SERVO1_PULSE_BLUE_CAP;
          printf("[공정1] 뚜껑 색상 선택: %s 뚜껑\r\n",
                 (info.color == 'R') ? "빨강" : "파랑/기타");
          SET_SERVO1_PULSE(cap_pulse);
          osDelay(SERVO1_MOVE_MS);

          // STEP 4. 뚜껑 공급 (전진 → 넛지 후진 → 넛지 전진 → 후진 복귀)
          printf("[공정1] 뚜껑 공급 액추에이터 전진\r\n");
          NEW_ACT_FORWARD();
          osDelay(NEW_ACT_FORWARD_MS);

          printf("[공정1] 뚜껑 공급 액추에이터 넛지 후진\r\n");
          NEW_ACT_BACKWARD();
          osDelay(NEW_ACT_NUDGE_MS);

          printf("[공정1] 뚜껑 공급 액추에이터 넛지 전진\r\n");
          NEW_ACT_FORWARD();
          osDelay(NEW_ACT_NUDGE_MS);

          printf("[공정1] 뚜껑 공급 액추에이터 후진\r\n");
          NEW_ACT_BACKWARD();
          osDelay(NEW_ACT_BACKWARD_MS);
          NEW_ACT_BACKWARD();                /* 대기 상태 유지 */
          osDelay(ACT_STOP_MED_MS);

          // 서보1 홈 복귀 (뚜껑 공급 완료 후)
          SET_SERVO1_PULSE(SERVO1_PULSE_HOME);
          osDelay(SERVO1_MOVE_MS);

          // STEP 5. 압착 2차 (끝까지 압착 — 1차 + 2차 합산 5.3초)
          printf("[공정1] 압착 2차 전진\r\n");
          PRESS_ACT_FORWARD();
          osDelay(PRESS_ACT_FORWARD_2ND_MS);
          PRESS_ACT_STOP();
          osDelay(ACT_STOP_SHORT_MS);

          // STEP 6. 압착 해제 (후진 복귀)
          printf("[공정1] 압착 후진 (해제)\r\n");
          PRESS_ACT_BACKWARD();
          osDelay(PRESS_ACT_BACKWARD_MS);
          PRESS_ACT_BACKWARD();              /* 대기 상태 유지 */
          osDelay(ACT_STOP_BRIEF_MS);

          // STEP 7. 벨트 재가동 (스토퍼 이미 오픈 상태 — 약통 바로 배출)
          printf("[공정1] 벨트 재가동 (약통 배출 중)\r\n");
          cap_belt_stop = 0U;
          cap_belt_run  = 1U;
          osDelay(CAP_PASS_AFTER_PRESS_MS);

          // STEP 8. 스토퍼 닫기 (약통 통과 완료 후 차단)
          printf("[공정1] 스토퍼 닫기\r\n");
          Servo2_MoveRamp(SERVO2_PULSE_HOME, SERVO2_RAMP_STEP, SERVO2_RAMP_DELAY_MS);
          cap_belt_run = 0U;

          CapActuators_SetIdleBackward();
          osDelay(ACT_STOP_BRIEF_MS);
          printf("[공정1] 뚜껑 압착 완료\r\n");
        }

        /* 정상/NG 공통: 공정2(PB2)에서 처리하도록 큐로 넘김 */
        if (osMessageQueuePut(sys2Queue, &info, 0U, 0U) == osOK)
          printf("[공정1→2] 약통 전달 (번호 %u, 색상 %c) — 공정2 대기 %lu개\r\n",
                 info.seq, info.color,
                 (unsigned long)osMessageQueueGetCount(sys2Queue));
        else
          printf("[공정1→2] 공정2 대기열 가득참 — 약통 번호 %u 누락\r\n", info.seq);

        /* 정상 약통만 공정1 완료 시 슬롯 반환. NG는 공정2 배출 완료 후 반환 */
        if (info.color != 'N')
          CAN_SendSlotAvailable();

        printf("[공정1] 완료\r\n");
      }

      osDelay(50);
    }
    else
    {
      if (prev_sensor1_state != GPIO_PIN_SET)
      {
        SET_SERVO1_PULSE(SERVO1_PULSE_HOME);
        Servo2_MoveRamp(SERVO2_PULSE_HOME, SERVO2_RAMP_STEP, SERVO2_RAMP_DELAY_MS);
        CapActuators_SetIdleBackward();
        cap_belt_stop = 0U;                 /* 안전: 정지 요청 해제 */
        cap_belt_run  = 0U;
        printf("[공정1] 센서 비어있음 — 대기 상태\r\n");
      }
      cap_sequence_done = 0;
    }
    prev_sensor1_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);

    osDelay(20);
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
