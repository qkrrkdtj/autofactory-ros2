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

#define UART_CMD_START     'S'
#define UART_RSP_OK        'O'
#define UART_RSP_NG        'N'
#define UART_PI_DELAY_MS   500U
#define UART_RX_TIMEOUT_MS 1000U
#define PI_RSP_LOG_SIZE    16U
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern BeltStatus  g_belt_status;
extern osMutexId_t beltMutex;
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
#define B1_Pin             GPIO_PIN_13
#define B1_GPIO_Port       GPIOC
#define SENSOR_Pin         GPIO_PIN_1
#define SENSOR_GPIO_Port   GPIOB
#define RELAY_Pin          GPIO_PIN_10
#define RELAY_GPIO_Port    GPIOB
#define ACT1_IN1_Pin       GPIO_PIN_8
#define ACT1_IN1_GPIO_Port GPIOB
#define ACT1_IN2_Pin       GPIO_PIN_9
#define ACT1_IN2_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
