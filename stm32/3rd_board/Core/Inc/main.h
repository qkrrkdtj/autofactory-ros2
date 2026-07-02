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
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
  bool valid;
  bool sensor;
} NodeState;

typedef struct
{
  uint8_t seq;
  char    color;
} ContainerInfo;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define NODE_COUNT          4U
#define CAN_ID_BASE         0x010U

#define FLASH_NODE_ID_ADDR  0x08060000U
#define FLASH_NODE_ID_MAGIC 0xA5A55A5AU

#define CAN_ID_1ST_TX        0x101U  /* 1번 보드 → 전체: 약통 정보 브로드캐스트 */
#define CAN_MSG_CONTAINER    0x01U   /* data[0] 메시지 타입 식별자 */
#define CAN_COLOR_RED        'R'     /* 빨간 약통 (정상) */
#define CAN_COLOR_BLUE       'B'     /* 파란 약통 (정상) */
#define CAN_COLOR_NG         'N'     /* 불량 */
/* 2번 보드 → 1번 보드 CAN 벨트 제어 (3번 보드는 수신만, 처리 안 함) */
#define CAN_ID_2ND_TX           0x102U

/* 3번 보드 → 1번 보드 CAN 역압력 프로토콜
 * 공정1 완료 시 전송 → 1번 보드가 다음 약통 투입을 허가받음 */
#define CAN_ID_3RD_TX           0x103U
#define CAN_CMD_SLOT_AVAILABLE  0x20U
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
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
#define MCP2515_CS_Pin GPIO_PIN_4
#define MCP2515_CS_GPIO_Port GPIOA
#define MCP2515_INT_Pin GPIO_PIN_0
#define MCP2515_INT_GPIO_Port GPIOB
#define SENSOR_Pin GPIO_PIN_1
#define SENSOR_GPIO_Port GPIOB

#define ACT1_IN1_Pin       GPIO_PIN_8
#define ACT1_IN1_GPIO_Port GPIOB
#define ACT1_IN2_Pin       GPIO_PIN_9
#define ACT1_IN2_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
