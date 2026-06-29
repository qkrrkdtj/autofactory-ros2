/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
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
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

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
/* 약통 정보 (1번 보드 CAN 메세지에서 수신) */
typedef struct {
  uint8_t seq;    /* 시퀀스 번호 */
  uint8_t color;  /* 'R'=빨강, 'B'=파랑, 'N'=NG, '?'=미수신 */
} ContainerInfo;

/* TB6600 스텝모터 제어 핀 */
#define STEPPER_PUL_Pin       GPIO_PIN_0   /* PC0: 펄스 */
#define STEPPER_PUL_GPIO_Port GPIOC
#define STEPPER_DIR_Pin       GPIO_PIN_1   /* PC1: 방향 */
#define STEPPER_DIR_GPIO_Port GPIOC
#define STEPPER_ENA_Pin       GPIO_PIN_2   /* PC2: 활성화 (LOW=ON) */
#define STEPPER_ENA_GPIO_Port GPIOC

/* 알약 IR 센서 (NPN형: 감지 시 LOW) */
#define SENSOR_PILL_Pin       GPIO_PIN_8
#define SENSOR_PILL_GPIO_Port GPIOB

/* MCP2515 CAN 컨트롤러 */
#define MCP2515_CS_Pin        GPIO_PIN_4   /* PA4: SPI CS */
#define MCP2515_CS_GPIO_Port  GPIOA
#define MCP2515_INT_Pin       GPIO_PIN_0   /* PB0: INT (EXTI0 FALLING) */
#define MCP2515_INT_GPIO_Port GPIOB

/* CAN 메세지 ID */
#define CAN_ID_1ST_TX        0x101U  /* 1번 보드 → 전체: 약통 정보 (seq, is_ok, color) */

/* PA5 = SPI1_SCK (AF5 고정) — LED 기능 비활성, 핀 정의만 유지 */
#define LED_Pin               GPIO_PIN_5
#define LED_GPIO_Port         GPIOA

/* 1번 보드 → 2번 보드 CAN 벨트 제어 프로토콜 */
#define CAN_ID_2ND_TX        0x102U
#define CAN_CMD_BELT_STOP    0x10U  /* 벨트 정지 요청 */
#define CAN_CMD_BELT_RESUME  0x11U  /* 벨트 재개 요청 */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
