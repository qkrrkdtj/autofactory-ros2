#include "mcp2515.h"
#include <stdio.h>

#define MCP2515_RESET       0xC0U
#define MCP2515_READ        0x03U
#define MCP2515_WRITE       0x02U
#define MCP2515_BITMOD      0x05U
#define MCP2515_READ_STATUS 0xA0U

#define MCP2515_CANCTRL     0x0FU
#define MCP2515_CANSTAT     0x0EU
#define MCP2515_CNF1        0x2AU
#define MCP2515_CNF2        0x29U
#define MCP2515_CNF3        0x28U
#define MCP2515_CANINTF     0x2CU
#define MCP2515_CANINTE     0x2BU
#define MCP2515_RXB0CTRL    0x60U
#define MCP2515_RXB0SIDH    0x61U
#define MCP2515_RXB1CTRL    0x70U
#define MCP2515_TXB0CTRL    0x30U
#define MCP2515_TXB0SIDH    0x31U
#define MCP2515_EFLG        0x2DU

#define MCP2515_MODE_CONFIG   0x80U
#define MCP2515_MODE_LOOPBACK 0x40U
#define MCP2515_MODE_NORMAL   0x00U

#define MCP2515_RXB1SIDH    0x71U
#define MCP2515_RX0IF         0x01U
#define MCP2515_RX1IF         0x02U
#define MCP2515_TXREQ         0x08U
#define MCP2515_TXERR         0x10U
#define MCP2515_ABTF          0x40U
#define MCP2515_TXBO          0x20U

#define MCP2515_RXM0SIDH    0x20U
#define MCP2515_RXM0SIDL    0x21U
#define MCP2515_RXM1SIDH    0x24U
#define MCP2515_RXM1SIDL    0x25U
#define MCP2515_RXF0SIDH    0x00U
#define MCP2515_RXF0SIDL    0x01U
#define MCP2515_RXF1SIDH    0x04U
#define MCP2515_RXF1SIDL    0x05U
#define MCP2515_RXF2SIDH    0x08U
#define MCP2515_RXF2SIDL    0x09U
#define MCP2515_RXF3SIDH    0x10U
#define MCP2515_RXF3SIDL    0x11U
#define MCP2515_RXF4SIDH    0x14U
#define MCP2515_RXF4SIDL    0x15U
#define MCP2515_RXF5SIDH    0x18U
#define MCP2515_RXF5SIDL    0x19U

static void MCP2515_CS_Low(MCP2515_HandleTypeDef *dev)
{
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void MCP2515_CS_High(MCP2515_HandleTypeDef *dev)
{
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef MCP2515_SPI_Transfer(MCP2515_HandleTypeDef *dev,
                                              const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  return HAL_SPI_TransmitReceive(dev->hspi, (uint8_t *)tx, rx, len, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef MCP2515_Reset(MCP2515_HandleTypeDef *dev)
{
  uint8_t cmd = MCP2515_RESET;
  uint8_t dummy = 0U;

  MCP2515_CS_Low(dev);
  HAL_StatusTypeDef status = MCP2515_SPI_Transfer(dev, &cmd, &dummy, 1U);
  MCP2515_CS_High(dev);
  HAL_Delay(10);
  return status;
}

static uint8_t MCP2515_ReadReg(MCP2515_HandleTypeDef *dev, uint8_t reg)
{
  uint8_t tx[3] = {MCP2515_READ, reg, 0U};
  uint8_t rx[3] = {0U};

  MCP2515_CS_Low(dev);
  MCP2515_SPI_Transfer(dev, tx, rx, 3U);
  MCP2515_CS_High(dev);
  return rx[2];
}

static void MCP2515_WriteReg(MCP2515_HandleTypeDef *dev, uint8_t reg, uint8_t value)
{
  uint8_t tx[3] = {MCP2515_WRITE, reg, value};
  uint8_t rx[3] = {0U};

  MCP2515_CS_Low(dev);
  MCP2515_SPI_Transfer(dev, tx, rx, 3U);
  MCP2515_CS_High(dev);
}

static void MCP2515_BitModify(MCP2515_HandleTypeDef *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
  uint8_t tx[4] = {MCP2515_BITMOD, reg, mask, value};
  uint8_t rx[4] = {0U};

  MCP2515_CS_Low(dev);
  MCP2515_SPI_Transfer(dev, tx, rx, 4U);
  MCP2515_CS_High(dev);
}

static HAL_StatusTypeDef MCP2515_SetMode(MCP2515_HandleTypeDef *dev, uint8_t mode)
{
  MCP2515_WriteReg(dev, MCP2515_CANCTRL, mode);

  for (uint32_t i = 0U; i < 100U; i++)
  {
    if ((MCP2515_ReadReg(dev, MCP2515_CANSTAT) & 0xE0U) == mode)
    {
      return HAL_OK;
    }
    HAL_Delay(1);
  }

  return HAL_ERROR;
}

static void MCP2515_ConfigureFilters(MCP2515_HandleTypeDef *dev)
{
  MCP2515_WriteReg(dev, MCP2515_RXB0CTRL, 0xC0U);
  MCP2515_WriteReg(dev, MCP2515_RXB1CTRL, 0xC0U);
}

static void MCP2515_ConfigureBitTiming(MCP2515_HandleTypeDef *dev)
{
  if (dev->osc_hz == MCP2515_OSC_16MHZ)
  {
    MCP2515_WriteReg(dev, MCP2515_CNF1, 0x07U);
    MCP2515_WriteReg(dev, MCP2515_CNF2, 0xB1U);
    MCP2515_WriteReg(dev, MCP2515_CNF3, 0x85U);
  }
  else
  {
    MCP2515_WriteReg(dev, MCP2515_CNF1, 0x03U);
    MCP2515_WriteReg(dev, MCP2515_CNF2, 0xB1U);
    MCP2515_WriteReg(dev, MCP2515_CNF3, 0x85U);
  }
}

HAL_StatusTypeDef MCP2515_Init(MCP2515_HandleTypeDef *dev)
{
  if (dev == NULL || dev->hspi == NULL)
  {
    return HAL_ERROR;
  }

  if (dev->osc_hz != MCP2515_OSC_8MHZ && dev->osc_hz != MCP2515_OSC_16MHZ)
  {
    dev->osc_hz = MCP2515_OSC_8MHZ;
  }

  if (MCP2515_Reset(dev) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2515_SetMode(dev, MCP2515_MODE_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }

  MCP2515_ConfigureBitTiming(dev);
  MCP2515_ConfigureFilters(dev);
  MCP2515_WriteReg(dev, MCP2515_CANINTE, 0x00U);
  MCP2515_WriteReg(dev, MCP2515_CANINTF, 0x00U);
  MCP2515_BitModify(dev, MCP2515_EFLG, 0xC0U, 0x00U);

  return HAL_OK;
}

HAL_StatusTypeDef MCP2515_RecoverBus(MCP2515_HandleTypeDef *dev)
{
  MCP2515_BitModify(dev, MCP2515_TXB0CTRL, MCP2515_TXREQ, 0U);

  if (MCP2515_SetMode(dev, MCP2515_MODE_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }

  MCP2515_WriteReg(dev, MCP2515_CANINTF, 0x00U);
  MCP2515_BitModify(dev, MCP2515_EFLG, 0xC0U, 0x00U);
  MCP2515_BitModify(dev, MCP2515_TXB0CTRL, MCP2515_TXREQ, 0U);
  HAL_Delay(10);

  if (MCP2515_SetMode(dev, MCP2515_MODE_NORMAL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_Delay(10);
  return ((MCP2515_ReadReg(dev, MCP2515_EFLG) & MCP2515_TXBO) == 0U) ? HAL_OK : HAL_ERROR;
}

/* std_id: 수신할 11비트 CAN ID
 * std_mask: 1=해당 비트 반드시 일치, 0=무시 (완전 일치: 0x7FFU) */
HAL_StatusTypeDef MCP2515_SetAcceptanceFilter(MCP2515_HandleTypeDef *dev,
                                              uint16_t std_id,
                                              uint16_t std_mask)
{
  if (MCP2515_SetMode(dev, MCP2515_MODE_CONFIG) != HAL_OK)
    return HAL_ERROR;

  uint8_t id_sidh   = (uint8_t)((std_id   >> 3) & 0xFFU);
  uint8_t id_sidl   = (uint8_t)((std_id   << 5) & 0xE0U);  /* EXIDE=0: standard frame */
  uint8_t mask_sidh = (uint8_t)((std_mask >> 3) & 0xFFU);
  uint8_t mask_sidl = (uint8_t)((std_mask << 5) & 0xE0U);

  MCP2515_WriteReg(dev, MCP2515_RXM0SIDH, mask_sidh);
  MCP2515_WriteReg(dev, MCP2515_RXM0SIDL, mask_sidl);
  MCP2515_WriteReg(dev, MCP2515_RXM1SIDH, mask_sidh);
  MCP2515_WriteReg(dev, MCP2515_RXM1SIDL, mask_sidl);

  MCP2515_WriteReg(dev, MCP2515_RXF0SIDH, id_sidh);
  MCP2515_WriteReg(dev, MCP2515_RXF0SIDL, id_sidl);
  MCP2515_WriteReg(dev, MCP2515_RXF1SIDH, id_sidh);
  MCP2515_WriteReg(dev, MCP2515_RXF1SIDL, id_sidl);
  MCP2515_WriteReg(dev, MCP2515_RXF2SIDH, id_sidh);
  MCP2515_WriteReg(dev, MCP2515_RXF2SIDL, id_sidl);
  MCP2515_WriteReg(dev, MCP2515_RXF3SIDH, id_sidh);
  MCP2515_WriteReg(dev, MCP2515_RXF3SIDL, id_sidl);
  MCP2515_WriteReg(dev, MCP2515_RXF4SIDH, id_sidh);
  MCP2515_WriteReg(dev, MCP2515_RXF4SIDL, id_sidl);
  MCP2515_WriteReg(dev, MCP2515_RXF5SIDH, id_sidh);
  MCP2515_WriteReg(dev, MCP2515_RXF5SIDL, id_sidl);

  /* RXM=00(필터 사용), RXB0에 BUKT=1(RXB0 가득 시 RXB1으로 롤오버) */
  MCP2515_WriteReg(dev, MCP2515_RXB0CTRL, 0x04U);
  MCP2515_WriteReg(dev, MCP2515_RXB1CTRL, 0x00U);

  return HAL_OK;
}

void MCP2515_PrintDiag(MCP2515_HandleTypeDef *dev)
{
  uint8_t canstat = MCP2515_ReadReg(dev, MCP2515_CANSTAT);
  uint8_t eflg = MCP2515_ReadReg(dev, MCP2515_EFLG);
  uint8_t txb0 = MCP2515_ReadReg(dev, MCP2515_TXB0CTRL);
  uint8_t canintf = MCP2515_ReadReg(dev, MCP2515_CANINTF);
  uint8_t cnf1 = MCP2515_ReadReg(dev, MCP2515_CNF1);
  uint8_t cnf2 = MCP2515_ReadReg(dev, MCP2515_CNF2);
  uint8_t cnf3 = MCP2515_ReadReg(dev, MCP2515_CNF3);

  printf("  CANSTAT=0x%02X mode=0x%02X\r\n", canstat, canstat & 0xE0U);
  printf("  EFLG=0x%02X%s%s%s%s\r\n", eflg,
         (eflg & 0x80U) ? " RX1OVR" : "",
         (eflg & 0x40U) ? " RX0OVR" : "",
         (eflg & MCP2515_TXBO) ? " BUS-OFF" : "",
         (eflg & MCP2515_TXERR) ? " TXERR" : "");
  printf("  TXB0CTRL=0x%02X%s%s\r\n", txb0,
         (txb0 & MCP2515_TXREQ) ? " TXREQ" : "",
         (txb0 & MCP2515_ABTF) ? " ABTF" : "");
  printf("  CANINTF=0x%02X%s%s\r\n", canintf,
         (canintf & MCP2515_RX0IF) ? " RX0IF" : "",
         (canintf & MCP2515_RX1IF) ? " RX1IF" : "");
  printf("  CNF=0x%02X 0x%02X 0x%02X (125kbps)\r\n", cnf1, cnf2, cnf3);
}

static void MCP2515_ClearFlags(MCP2515_HandleTypeDef *dev)
{
  MCP2515_WriteReg(dev, MCP2515_CANINTF, 0x00U);
  MCP2515_BitModify(dev, MCP2515_EFLG, 0xC0U, 0x00U);
}

static void MCP2515_ParseRxBuffer(uint8_t sidh, uint8_t sidl, uint8_t dlc,
                                  const uint8_t *data, MCP2515_CanMsg *msg)
{
  msg->extended = ((sidl & 0x08U) != 0U);
  if (msg->extended)
  {
    msg->id = ((uint32_t)sidh << 21) |
              ((uint32_t)(sidl & 0xE0U) << 13) |
              ((uint32_t)(sidl & 0x03U) << 16);
  }
  else
  {
    msg->id = ((uint32_t)sidh << 3) | ((uint32_t)(sidl >> 5));
  }

  msg->dlc = dlc;
  for (uint8_t j = 0U; j < dlc && j < 8U; j++)
  {
    msg->data[j] = data[j];
  }
}

static HAL_StatusTypeDef MCP2515_ReadRxBuffer(MCP2515_HandleTypeDef *dev,
                                              uint8_t sidh_reg,
                                              uint8_t intf_flag,
                                              MCP2515_CanMsg *msg)
{
  uint8_t sidh = MCP2515_ReadReg(dev, sidh_reg);
  uint8_t sidl = MCP2515_ReadReg(dev, sidh_reg + 1U);
  uint8_t dlc = MCP2515_ReadReg(dev, sidh_reg + 4U) & 0x0FU;
  uint8_t data[8];

  for (uint8_t j = 0U; j < dlc && j < 8U; j++)
  {
    data[j] = MCP2515_ReadReg(dev, sidh_reg + 5U + j);
  }

  MCP2515_ParseRxBuffer(sidh, sidl, dlc, data, msg);
  MCP2515_BitModify(dev, MCP2515_CANINTF, intf_flag, 0x00U);
  return HAL_OK;
}

HAL_StatusTypeDef MCP2515_SetLoopbackMode(MCP2515_HandleTypeDef *dev)
{
  if (MCP2515_SetMode(dev, MCP2515_MODE_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return MCP2515_SetMode(dev, MCP2515_MODE_LOOPBACK);
}

HAL_StatusTypeDef MCP2515_SetNormalMode(MCP2515_HandleTypeDef *dev)
{
  if (MCP2515_SetMode(dev, MCP2515_MODE_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return MCP2515_SetMode(dev, MCP2515_MODE_NORMAL);
}

HAL_StatusTypeDef MCP2515_Send(MCP2515_HandleTypeDef *dev, const MCP2515_CanMsg *msg)
{
  if (dev == NULL || msg == NULL || msg->dlc > 8U)
  {
    return HAL_ERROR;
  }

  uint8_t mode = MCP2515_ReadReg(dev, MCP2515_CANSTAT) & 0xE0U;
  if (mode != MCP2515_MODE_NORMAL && mode != MCP2515_MODE_LOOPBACK)
  {
    return HAL_ERROR;
  }

  if (mode == MCP2515_MODE_NORMAL &&
      (MCP2515_ReadReg(dev, MCP2515_EFLG) & MCP2515_TXBO) != 0U)
  {
    MCP2515_RecoverBus(dev);
  }

  MCP2515_BitModify(dev, MCP2515_TXB0CTRL, MCP2515_TXREQ, 0U);

  uint8_t sidh;
  uint8_t sidl;

  if (msg->extended)
  {
    sidh = (uint8_t)((msg->id >> 21) & 0xFFU);
    sidl = (uint8_t)(((msg->id >> 13) & 0xE0U) | ((msg->id >> 16) & 0x03U) | 0x08U);
  }
  else
  {
    sidh = (uint8_t)((msg->id >> 3) & 0xFFU);
    sidl = (uint8_t)((msg->id << 5) & 0xE0U);
  }

  MCP2515_WriteReg(dev, MCP2515_TXB0SIDH, sidh);
  MCP2515_WriteReg(dev, MCP2515_TXB0SIDH + 1U, sidl);
  MCP2515_WriteReg(dev, MCP2515_TXB0SIDH + 4U, msg->dlc & 0x0FU);

  for (uint8_t i = 0U; i < msg->dlc; i++)
  {
    MCP2515_WriteReg(dev, MCP2515_TXB0SIDH + 5U + i, msg->data[i]);
  }

  MCP2515_BitModify(dev, MCP2515_TXB0CTRL, MCP2515_TXREQ, MCP2515_TXREQ);

  for (uint32_t i = 0U; i < 200U; i++)
  {
    uint8_t txb0 = MCP2515_ReadReg(dev, MCP2515_TXB0CTRL);

    if ((txb0 & MCP2515_TXREQ) == 0U)
    {
      if ((txb0 & (MCP2515_TXERR | MCP2515_ABTF)) != 0U)
      {
        return HAL_ERROR;
      }
      return HAL_OK;
    }
    HAL_Delay(1);
  }

  return HAL_TIMEOUT;
}

HAL_StatusTypeDef MCP2515_Receive(MCP2515_HandleTypeDef *dev, MCP2515_CanMsg *msg, uint32_t timeout_ms)
{
  if (dev == NULL || msg == NULL)
  {
    return HAL_ERROR;
  }

  for (uint32_t i = 0U; i < (timeout_ms * 100U); i++)
  {
    uint8_t intf = MCP2515_ReadReg(dev, MCP2515_CANINTF);

    if ((intf & MCP2515_RX0IF) != 0U)
    {
      return MCP2515_ReadRxBuffer(dev, MCP2515_RXB0SIDH, MCP2515_RX0IF, msg);
    }

    if ((intf & MCP2515_RX1IF) != 0U)
    {
      return MCP2515_ReadRxBuffer(dev, MCP2515_RXB1SIDH, MCP2515_RX1IF, msg);
    }

    if ((i % 100U) == 0U)
    {
      HAL_Delay(1);
    }
  }

  return HAL_TIMEOUT;
}

HAL_StatusTypeDef MCP2515_LoopbackTest(MCP2515_HandleTypeDef *dev)
{
  MCP2515_CanMsg tx_msg = {
      .id = 0x123U,
      .dlc = 4U,
      .data = {0xDEU, 0xADU, 0xBEU, 0xEFU},
      .extended = false,
  };
  MCP2515_CanMsg rx_msg = {0};

  if (MCP2515_SetLoopbackMode(dev) != HAL_OK)
  {
    return HAL_ERROR;
  }

  MCP2515_ClearFlags(dev);

  if (MCP2515_Send(dev, &tx_msg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2515_Receive(dev, &rx_msg, 100U) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  if (rx_msg.id != tx_msg.id || rx_msg.dlc != tx_msg.dlc)
  {
    return HAL_ERROR;
  }

  for (uint8_t i = 0U; i < tx_msg.dlc; i++)
  {
    if (rx_msg.data[i] != tx_msg.data[i])
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef MCP2515_AutoDetectOsc(MCP2515_HandleTypeDef *dev)
{
  const uint32_t freqs[2] = {MCP2515_OSC_8MHZ, MCP2515_OSC_16MHZ};

  for (uint8_t i = 0U; i < 2U; i++)
  {
    dev->osc_hz = freqs[i];
    if (MCP2515_Init(dev) != HAL_OK)
    {
      continue;
    }
    if (MCP2515_LoopbackTest(dev) == HAL_OK)
    {
      return HAL_OK;
    }
  }

  return HAL_ERROR;
}
