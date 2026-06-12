# 🏭 Automated Packaging & Inspection System

본 프로젝트는 **3개의 STM32F411RE**와 **2개의 Raspberry Pi**를 활용하여 제품의 투입, 내용물 공급, 가공, 검사 및 최종 분류 과정을 자동화한 스마트 제조 시스템입니다.

STM32 보드들은 **MCP2515 CAN Bus 네트워크**를 통해 실시간으로 데이터를 공유하며, Raspberry Pi는 비전 판정 및 로봇팔 연동을 담당하는 상위 제어 장치로 동작합니다.

---

# 🏗️ System Architecture

```text
       [ Raspberry Pi (Board 4) ]
          Vision Inspection
                  │
         (UART: 3-Byte Packet)
                  │
                  ▼
       [ STM32F411RE (Board 1) ] ─── [ STM32F411RE (Board 2) ] ─── [ STM32F411RE (Board 3) ]
       └─────────────────────────────── CAN BUS (MCP2515) ───────────────────────────────┘
                                                                           │
                                                                           ▼
                                                              [ Raspberry Pi (Board 5) ]
                                                            Final Control & Robot Arm
```

## System Overview

### STM32 CAN Network

* Board 1 : Main Controller
* Board 2 : Material Supply Controller
* Board 3 : Processing & Sorting Controller

### Raspberry Pi Systems

* Board 4 : Vision Inspection System
* Board 5 : Final Conveyor & Robot Arm Controller

---

# 🎛️ Board Specifications & Roles

---

# 1️⃣ Board 1 — Front-End Control & Main Controller

## MCU

STM32F411RE

## Responsibilities

* 공정 시작 제어
* 약통 투입 관리
* 메인 컨베이어 벨트 구동
* 적외선 센서 감지
* 비전 판정 결과 수신
* CAN 메시지 브로드캐스트

## Hardware Components

### UART Communication (Board 4 ↔ Board 1)

```text
Raspberry Pi TX ─────► STM32 RX
Raspberry Pi RX ◄───── STM32 TX
```

* Logic Level : 3.3V
* Level Shifter : Not Required

### Packet Structure

| Byte   | Value                          |
| ------ | ------------------------------ |
| Header | 0xAA                           |
| Data   | 0x01 (Good) / 0x00 (Defective) |
| Tail   | 0xFF                           |

Example

```text
AA 01 FF  → Good Product
AA 00 FF  → Defective Product
```

### Bottle Pusher Actuator

#### Driver

* L298N

#### Control Signals

| Signal | Function          |
| ------ | ----------------- |
| IN1    | Direction Control |
| IN2    | Direction Control |
| ENA    | PWM Speed Control |

### Conveyor Control

#### Relay

* JQC-3FF-S-Z

#### Protection Circuit

* NPN Transistor Driver
* Flyback Diode

### Sensors

* Infrared Sensor
* Bottle Detection
* Process Trigger

---

# 2️⃣ Board 2 — Material Supply Control

## MCU

STM32F411RE

## Responsibilities

Board 1의 CAN 메시지를 기반으로:

* 약 공급
* 뚜껑 공급
* 정밀 액추에이터 제어

## Hardware Components

### Medicine Dispenser

#### Stepper Motor

* 28BYJ-48

#### Driver

* ULN2003

### Cap Supply Actuator

#### Driver

* L298N (HW-095)

### Communication

* MCP2515 CAN Controller
* CAN Event-Driven Operation

---

# 3️⃣ Board 3 — Processing & First Sorting Control

## MCU

STM32F411RE

## Responsibilities

Board 1의 CAN 데이터를 기반으로:

* 압착 공정 수행
* 컨베이어 이송
* 정상품 / 불량품 1차 분류
* 불량품 패스 처리

## Hardware Components

### Compression Actuator

* L298N Motor Driver

### Sorting Actuator

* L298N Motor Driver

### Sensors

* Infrared Sensors
* Compression Position Detection
* Sorting Position Detection

### Communication

* MCP2515 CAN Controller

---

# 4️⃣ Board 4 — Vision Inspection System

## Main Controller

Raspberry Pi

## Responsibilities

* 카메라 영상 취득
* 비전 기반 제품 검사
* 정상 / 불량 판정
* 판정 결과 UART 전송

## Communication

### UART (Board 4 ↔ Board 1)

```text
Raspberry Pi TX ─────► STM32 RX
Raspberry Pi RX ◄───── STM32 TX
```

### Packet Structure

| Byte   | Value       |
| ------ | ----------- |
| Header | 0xAA        |
| Data   | 0x01 / 0x00 |
| Tail   | 0xFF        |

### Inspection Result

| Value | Description       |
| ----- | ----------------- |
| 0x01  | Good Product      |
| 0x00  | Defective Product |

판정 결과는 Board 1로 전송되며, 이후 CAN 네트워크 전체에 공유됩니다.

---

# 5️⃣ Board 5 — Final Conveyor Control & Robot Arm Integration

## Main Controller

Raspberry Pi

## Responsibilities

* 최종 제품 위치 감지
* 컨베이어 정지 제어
* 로봇팔 연동
* Pick & Place 트리거

## Hardware Components

### Infrared Sensor

* 최종 위치 감지
* 로봇팔 적재 위치 감지

### Relay Module

* JQC-3FF-S-Z
* 최종 컨베이어 전원 제어

## Operation Flow

```text
Product Arrives
        │
        ▼
IR Detection
        │
        ▼
Conveyor Stop
        │
        ▼
Robot Arm Trigger
        │
        ▼
Pick & Place
```

## Robot Arm Interface

### Functions

* Product Arrival Notification
* Robot Arm Start Signal
* Conveyor Resume Signal

### Interface

```text
TBD (GPIO / UART / TCP-IP / ROS2)
```

---

# 📡 STM32 CAN Communication Specification

Board 1 ~ Board 3은 동일한 MCP2515 연결 구조를 사용합니다.

| MCP2515 Pin | STM32F411RE Pin |
| ----------- | --------------- |
| SCK         | PA5             |
| MISO        | PA6             |
| MOSI        | PA7             |
| CS          | PA4             |
| INT         | PB0             |

## SPI Configuration

| Item           | Value                |
| -------------- | -------------------- |
| Peripheral     | SPI1                 |
| CAN Controller | MCP2515              |
| Network Type   | CAN Bus              |
| Topology       | Multi-node Broadcast |

---

# 🔄 System Workflow

```text
Bottle Detected
        │
        ▼
      Board 1
        │
        ▼
Board 4 (Vision Inspection)
        │
        ▼
UART Result
        │
        ▼
      Board 1
        │
        ▼
   CAN Broadcast
        │
        ├────────► Board 2
        │              │
        │              └─ Medicine & Cap Supply
        │
        └────────► Board 3
                       │
                       ├─ Compression
                       ├─ Sorting
                       └─ Conveyor Transfer
                                   │
                                   ▼
                         Board 5 (Raspberry Pi)
                                   │
                                   ├─ Final Detection
                                   ├─ Conveyor Stop
                                   └─ Robot Arm Control
```

---

# 🛠️ Hardware Stack

## Controllers

* STM32F411RE × 3
* Raspberry Pi × 2

## Communication

* MCP2515 CAN Bus
* SPI1
* UART

## Actuators

* DC Motor
* 28BYJ-48 Stepper Motor
* L298N Motor Driver
* ULN2003 Driver
* Relay Module

## Sensors

* Infrared Detection Sensors

---

# 🎯 Project Objectives

* Multi-node CAN-based distributed control
* Real-time production process synchronization
* Vision-based quality inspection
* Automated product sorting
* Robot arm integration for smart manufacturing
* Modular and scalable embedded system architecture
