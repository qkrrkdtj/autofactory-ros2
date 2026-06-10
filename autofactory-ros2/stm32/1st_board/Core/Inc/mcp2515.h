#ifndef MCP2515_H
#define MCP2515_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define MCP2515_OSC_8MHZ   8000000U
#define MCP2515_OSC_16MHZ  16000000U

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  uint32_t osc_hz;
} MCP2515_HandleTypeDef;

typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  bool extended;
} MCP2515_CanMsg;

HAL_StatusTypeDef MCP2515_Init(MCP2515_HandleTypeDef *dev);
HAL_StatusTypeDef MCP2515_SetNormalMode(MCP2515_HandleTypeDef *dev);
HAL_StatusTypeDef MCP2515_SetLoopbackMode(MCP2515_HandleTypeDef *dev);
HAL_StatusTypeDef MCP2515_Send(MCP2515_HandleTypeDef *dev, const MCP2515_CanMsg *msg);
HAL_StatusTypeDef MCP2515_Receive(MCP2515_HandleTypeDef *dev, MCP2515_CanMsg *msg, uint32_t timeout_ms);
HAL_StatusTypeDef MCP2515_LoopbackTest(MCP2515_HandleTypeDef *dev);
HAL_StatusTypeDef MCP2515_AutoDetectOsc(MCP2515_HandleTypeDef *dev);
HAL_StatusTypeDef MCP2515_RecoverBus(MCP2515_HandleTypeDef *dev);
void MCP2515_PrintDiag(MCP2515_HandleTypeDef *dev);

#endif /* MCP2515_H */
