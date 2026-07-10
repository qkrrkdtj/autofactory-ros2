# STM32 보드 물리 배선

> MCU: **STM32F411** (1·2·3번 보드 공통)  
> 외부 배선이 필요한 핀만 기록

---

## 공통 — MCP2515 CAN (1·2·3번 보드)

| MCU 핀 | MCP2515 핀 |
|--------|------------|
| PA4 | CS |
| PA5 | SCK |
| PA6 | MISO (SO) |
| PA7 | MOSI (SI) |
| PB0 | INT |
| CANH / CANL | 보드 간 버스 연결 |

---

## 1번 보드

| MCU 핀 | 연결 대상 |
|--------|-----------|
| PA2 | Raspberry Pi RX |
| PA3 | Raspberry Pi TX |
| PA9 | 공급 IR 센서 (NPN) |
| PB1 | 비전 IR 센서 (NPN) |
| PB10 | 벨트 릴레이 |
| PC8 | 액추에이터1 IN1 (L298N) |
| PC9 | 액추에이터1 IN2 (L298N) |

---

## 2번 보드

| MCU 핀 | 연결 대상 |
|--------|-----------|
| PC0 | TB6600 PUL |
| PC1 | TB6600 DIR |
| PC2 | TB6600 ENA |
| PB8 | 약통 IR 센서 (NPN) |

---

## 3번 보드

| MCU 핀 | 연결 대상 |
|--------|-----------|
| PB8 | 공정1 IR 센서 (NPN) |
| PB2 | 공정2 IR 센서 (NPN) |
| PB4 | 서보1 (TIM3_CH1 PWM) |
| PB1 | 서보2 (TIM3_CH4 PWM) |
| PB14 | 벨트 릴레이 |
| PC0 | 뚜껑 공급 IN1 (L298N) |
| PC5 | 뚜껑 공급 IN2 (L298N) |
| PC2 | 압착 IN3 (L298N) |
| PC3 | 압착 IN4 (L298N) |
| PB12 | 분류 IN3 (L298N) |
| PB15 | 분류 IN4 (L298N) |

---

## Raspberry Pi

| 연결 | 포트 |
|------|------|
| 1번 보드 USART2 | /dev/ttyACM0 (115200) |
| USB 카메라 | /dev/video0 |
