/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
  uint8_t machine_stage;
  uint8_t belt_running;
} BeltStatus;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MACHINE_STAGE_IDLE    0U
#define MACHINE_STAGE_FORWARD 1U
#define MACHINE_STAGE_RUNNING 2U
#define BELT_STATE_STOPPED    0U
#define BELT_STATE_RUNNING    1U

#define UART_CMD_START        'S'
#define UART_RSP_RED          'R'   /* 빨간 약통 (OK) */
#define UART_RSP_BLUE         'B'   /* 파란 약통 (OK) */
#define UART_RSP_NG           'N'   /* 불량 (NG) */
#define UART_RX_TIMEOUT_MS    1000U
#define PI_RSP_LOG_SIZE       16U
#define SENSOR_REARM_DELAY_MS 500U

/* 약통 투입·비전 검사 타이밍 (ms) — 현장 튜닝 시 이 값만 수정 */
#define ACT1_FORWARD_MS           8000U  /* 액추에이터1 전진 유지 시간 */
#define START_SENSOR_DEBOUNCE_MS    20U  /* PA9 공급 센서 채터링 방지 재확인 */
#define START_SENSOR_HOLD_MS      1000U  /* PA9 감지 후 약통 안착 대기 */
#define VISION_STABILIZE_MS        500U  /* 비전 검사 전 카메라 안정화 대기 */
#define POST_VISION_BELT_MS        500U  /* 비전 검사 후 약통 밀어주기 시간 */

/* 1번 보드 → 전체 CAN 약통 정보 브로드캐스트 (0x101) */
#define CAN_ID_1ST_TX        0x101U
#define CAN_MSG_CONTAINER    0x01U   /* data[0] 메시지 타입 */
#define CAN_COLOR_RED        'R'     /* 빨간 약통 (정상) */
#define CAN_COLOR_BLUE       'B'     /* 파란 약통 (정상) */
#define CAN_COLOR_NG         'N'     /* 불량 */

/* 2번 보드 → 1번 보드 CAN 벨트 제어 프로토콜 */
#define CAN_ID_2ND_TX        0x102U
#define CAN_CMD_BELT_STOP    0x10U  /* 벨트 정지 요청 */
#define CAN_CMD_BELT_RESUME  0x11U  /* 벨트 재개 요청 */

/* 3번 보드 → 1번 보드 CAN 역압력 프로토콜
 * 3번 보드 공정1 완료 시 전송 → 1번 보드가 다음 약통 투입을 허가받음 */
#define CAN_ID_3RD_TX           0x103U
#define CAN_CMD_SLOT_AVAILABLE  0x20U
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern BeltStatus      g_belt_status;
extern osMutexId_t     beltMutex;
extern osMutexId_t     canMutex;
extern osSemaphoreId_t sem_can_int;
extern volatile bool   g_belt_ext_stop;
bool ReadSensor(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SENSOR_Pin         GPIO_PIN_1
#define SENSOR_GPIO_Port   GPIOB
#define RELAY_Pin          GPIO_PIN_10
#define RELAY_GPIO_Port    GPIOB
#define ACT1_IN1_Pin       GPIO_PIN_14
#define ACT1_IN1_GPIO_Port GPIOB
#define ACT1_IN2_Pin       GPIO_PIN_15
#define ACT1_IN2_GPIO_Port GPIOB
#define MCP2515_CS_Pin       GPIO_PIN_4
#define MCP2515_CS_GPIO_Port GPIOA
#define MCP2515_INT_Pin      GPIO_PIN_0
#define MCP2515_INT_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
