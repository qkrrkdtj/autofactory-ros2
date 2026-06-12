/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"

/* 8스텝(Half-step) 구동 시퀀스 - 오리지널 데이터 */
const uint8_t half_step_seq[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

/* 스텝 모터 핀 제어 함수 (PC0 ~ PC3 기준) */
void step_motor(int step) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, half_step_seq[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, half_step_seq[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, half_step_seq[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, half_step_seq[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* Infinite loop */
  while (1)
  {
    // [검증] 블루투스나 외부 보드에서 신호(PA1 = HIGH)가 들어왔을 때만 실행
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
    {
      // 신호 감지 후 원래 타이밍 확보를 위한 0.5초 대기
      HAL_Delay(500);

      // -------------------------------------------------------------
      // 지호 님의 오리지널 구동 코드 메커니즘 100% 그대로 복사 이식
      // -------------------------------------------------------------

      // 1. 정방향으로 1칸(약 60도) 회전
      for(int i = 0; i < 85; i++) {
        for(int step = 0; step < 8; step++) {
          step_motor(step);
          HAL_Delay(2);
        }
      }

      // 대기하는 동안 모터 전자석 전원 완벽 차단
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
      HAL_Delay(1000); // 1초 대기

      // 2. 역방향으로 1칸(약 60도) 회전하여 원위치 (이래야 위상이 완벽히 리셋됨)
      for(int i = 0; i < 85; i++) {
        for(int step = 7; step >= 0; step--) {
          step_motor(step);
          HAL_Delay(2);
        }
      }

      // 대기 중 전원 차단
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
      HAL_Delay(1000); // 1초 대기

      // -------------------------------------------------------------

      // [연동 락] 회전 완료 신호를 상대 보드에 전송 (PA2 = HIGH)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

      // 상대 보드가 확인하고 신호(PA1)를 내려줄 때까지 대기 (무한 연타 방지)
      while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
        HAL_Delay(10);
      }

      // 핸드셰이킹 끝났으니 내 완료 핀도 초기화
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    }
    else
    {
      // 신호가 없을 때는 코드를 아무것도 수행하지 않고 10ms씩 쉬면서 대기
      HAL_Delay(10);
    }
  }
}

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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // 모터 핀 초기화 (PC0 ~ PC3)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // 연동 핀 초기화 (PA1: 수신, PA2: 송신)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; // 기본 상태에서 노이즈 안 타게 풀다운
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
