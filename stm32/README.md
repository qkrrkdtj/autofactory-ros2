# 🏭 Automated Packaging & Inspection System

본 프로젝트는 복수의 **STM32F411RE** 보드 간의 **MCP2515 CAN 통신**과 메인 판단 장치인 **Raspberry Pi**와의 **UART 통신**을 연동하여 제품을 실시간으로 판정, 이송 및 분류하는 자동화 시스템입니다.

---

# 🏗️ System Architecture

```text
       [ Raspberry Pi (Board 4) ]
                   │
         (UART: 3-Byte Packet)
                   │
                   ▼
       [ STM32F411RE (Board 1) ] ─── [ STM32F411RE (Board 2) ] ─── [ STM32F411RE (Board 3) ]
       └─────────────────────────────── CAN BUS (MCP2515) ───────────────────────────────┘
```

### System Overview

* **Board 1, 2, 3** : MCP2515 기반 CAN 네트워크를 구성하여 실시간 공정 데이터를 공유
* **Board 4 (Raspberry Pi)** : 비전 판정 및 최종 공정 제어를 담당하는 상위 제어 객체
* Raspberry Pi는 CAN 네트워크에 참여하지 않고 Board 1과 UART로 직접 통신

---

# 🎛️ Board Specifications & Roles

---

## 1️⃣ Board 1 — Front-End Control & Main Controller

### MCU

* STM32F411RE

### Responsibilities

* 공정 시작 제어
* 약통 투입 관리
* 메인 컨베이어 벨트 제어
* 적외선 센서 기반 투입 감지
* Raspberry Pi 판정 결과 수신
* CAN 메시지 브로드캐스트

### Hardware Components

#### UART Communication (Raspberry Pi ↔ STM32)

```text
Raspberry Pi TX ─────► STM32 RX
Raspberry Pi RX ◄───── STM32 TX
```

* Logic Level : 3.3V
* Level Shifter : Not Required

##### Packet Structure

| Byte   | Value                          |
| ------ | ------------------------------ |
| Header | 0xAA                           |
| Data   | 0x01 (Good) / 0x00 (Defective) |
| Tail   | 0xFF                           |

Example:

```text
AA 01 FF  → Good Product
AA 00 FF  → Defective Product
```

#### Bottle Pusher Actuator

* Driver : L298N
* Motor Type : DC Motor

| Signal | Function          |
| ------ | ----------------- |
| IN1    | Direction Control |
| IN2    | Direction Control |
| ENA    | PWM Speed Control |

#### Conveyor Control

* Relay : JQC-3FF-S-Z
* NPN Transistor Driver Circuit
* Flyback Diode Protection

#### Sensors

* Infrared Sensor
* Bottle insertion detection
* Process trigger generation

#### CAN Communication

* MCP2515 CAN Controller
* SPI1 Interface

---

## 2️⃣ Board 2 — Material Supply Control

### MCU

* STM32F411RE

### Responsibilities

Board 1이 전송한 CAN 메시지를 기반으로 다음 작업을 수행합니다.

* 약 투하
* 뚜껑 공급
* 액추에이터 정밀 제어

### Hardware Components

#### Medicine Dispenser

* Stepper Motor : 28BYJ-48
* Driver : ULN2003

정밀 위치 제어를 통해 일정량의 약을 공급합니다.

#### Cap Supply Actuator

* Driver : L298N (HW-095)
* DC Motor Mechanism

#### CAN Communication

* MCP2515 CAN Controller
* Event-driven CAN Operation

---

## 3️⃣ Board 3 — Processing & First Sorting Control

### MCU

* STM32F411RE

### Responsibilities

Board 1으로부터 수신한 CAN 데이터를 기반으로 다음 작업을 수행합니다.

* 압착 공정 수행
* 다음 공정으로 이송
* 정상품/불량품 1차 분류
* 불량품 패스 처리

### Hardware Components

#### Compression Actuator

* L298N Motor Driver
* Independent Channel Control

#### Sorting Actuator

* L298N Motor Driver
* Defective Product Bypass

#### Sensors

* Infrared Sensors
* Compression Position Detection
* Sorting Position Detection

#### CAN Communication

* MCP2515 CAN Controller
* Real-time Synchronization

---

## 4️⃣ Board 4 — Vision Inspection & Final Conveyor Control

### Main Controller

* Raspberry Pi

### Responsibilities

Board 4는 CAN 네트워크에 참여하지 않는 독립적인 상위 제어 장치입니다.

주요 역할은 다음과 같습니다.

* 비전 기반 제품 판정
* Board 1으로 UART 결과 전송
* 최종 컨베이어 정지 제어
* 로봇팔 연동 신호 전달

### UART Communication

Board 1과 UART 직결 통신을 수행합니다.

```text
Raspberry Pi TX ─────► STM32 RX
Raspberry Pi RX ◄───── STM32 TX
```

### Vision Inspection Result

| Value | Meaning           |
| ----- | ----------------- |
| 0x01  | Good Product      |
| 0x00  | Defective Product |

판정 결과는 UART를 통해 Board 1으로 전달되며, Board 1은 이를 CAN 네트워크에 브로드캐스트합니다.

### Hardware Components

#### Infrared Sensor

* 최종 제품 도착 감지
* 로봇팔 적재 위치 감지

#### Relay Module

* JQC-3FF-S-Z
* 최종 컨베이어 전원 제어

### Robot Arm Interface

* Product Arrival Notification
* Conveyor Stop Signal
* Robot Arm Trigger Signal

> Interface Type : TBD (GPIO / UART / CAN Extension)

---

# 📡 STM32 CAN Communication Specification

Board 1, Board 2, Board 3은 동일한 MCP2515 연결 구조를 사용합니다.

| MCP2515 Pin | STM32F411RE Pin |
| ----------- | --------------- |
| SCK         | PA5             |
| MISO        | PA6             |
| MOSI        | PA7             |
| CS          | PA4             |
| INT         | PB0             |

### SPI Configuration

* Peripheral : SPI1
* CAN Controller : MCP2515
* Communication Method : CAN Bus
* Network Type : Multi-node Broadcast

---

# 🔄 System Workflow

```text
Bottle Detected
        │
        ▼
      Board 1
        │
        ├─ UART Receive (Vision Result)
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
                          Raspberry Pi
                                   │
                                   ├─ Final Detection
                                   ├─ Conveyor Stop
                                   └─ Robot Arm Handoff
```

---

# 🛠️ Technology Stack

### Embedded Systems

* STM32F411RE
* MCP2515 CAN Controller
* SPI1
* UART

### Actuators

* 12V DC Motor
* 28BYJ-48 Stepper Motor
* Relay Module
* L298N Motor Driver
* ULN2003 Driver

### Sensors

* Infrared Detection Sensor

### Main Controller

* Raspberry Pi

### Communication

* CAN Bus (MCP2515)
* UART
