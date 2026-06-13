# 🏭 Automated Packaging & Inspection System

본 프로젝트는 **4개의 STM32F411RE**와 **2개의 Raspberry Pi**를 활용하여 제품의 투입, 내용물 공급, 가공, 비전 검사 및 최종 분류 과정을 자동화한 스마트 제조 시스템입니다.

STM32 MCU들은 **MCP2515 CAN Bus 네트워크**를 통해 실시간으로 공정 데이터를 공유하며, Raspberry Pi는 고성능 비전 알고리즘 판정 및 로봇팔 연동을 담당하는 상위 제어 장치로 동작합니다.

---

# 🏗️ System Architecture

       [ Raspberry Pi ]                  [ Raspberry Pi ]
  (Vision Processing System)       (Final Robot Arm Controller)
               │                                ▲
     (UART: 3-Byte Packet)                      │
               │                       (GPIO / UART / ROS2)
               ▼                                │
      [ Vision Bridge MCU ]                     │
               │                                │
      (CAN Bus / MCP2515)                       │
               │                                │
               ▼                                │
      [ Main Infeed MCU ] ─── [ Material Supply MCU ] ─── [ Processing & Sorting MCU ]
      └─────────────────────────── CAN BUS (MCP2515) ───────────────────────────────┘

---

# ⚙️ System Overview

### 🔌 STM32 CAN Network (4 Nodes)
* **Main Infeed MCU** : 공정 시작 제어 및 메인 컨베이어, 약통 투입 관리
* **Vision Bridge MCU** : 비전 트리거 및 Raspberry Pi 판정 결과 수신 후 CAN Bus 브로드캐스트
* **Material Supply MCU** : 약품 및 뚜껑 공급 장치 제어
* **Processing & Sorting MCU** : 압착 가공 및 1차 불량품 분류 처리

### 🧠 Raspberry Pi Systems (2 Nodes)
* **Vision Processing System** : 카메라 기반 실시간 AI 비전 검사 (OpenCV / 딥러닝 기반 정상·불량 판정)
* **Final Robot Arm Controller** : 최종 컨베이어 제어 및 로봇팔 Pick & Place 연동

---

# 🎛️ Controller Specifications & Roles

## 1️⃣ Main Infeed MCU (STM32F411RE)
* **Responsibilities**
  * 생산 라인 공정 시작 및 메인 컨베이어 벨트 구동
  * 적외선(IR) 센서 기반 약통 진입 감지
  * 투입 액추에이터 제어 및 CAN Bus로 상태 전송
* **Hardware Components**
  * **Bottle Pusher Actuator** : L298N 모터 드라이버 (IN1/IN2 방향 제어, ENA PWM 속도 제어)
  * **Main Conveyor Control** : JQC-3FF-S-Z 릴레이 (NPN 트랜지스터 + Flyback Diode 보호회로 포함)
  * **Sensors** : IR Sensor (약통 투입 위치 감지)

---

## 2️⃣ Vision Bridge MCU (STM32F411RE)
* **Responsibilities**
  * IR 센서 기반 약통 도착 감지 시 비전 트리거 신호 생성
  * 상위 Raspberry Pi(Vision Processing System)로 검사 신호 전송 및 판정 결과 UART 수신
  * 판정 결과를 CAN 메시지로 변환하여 전체 네트워크에 브로드캐스트
* **UART Communication Specification (with Raspberry Pi)**
  * **Logic Level** : 3.3V (Level Shifter 불필요)
  * **Packet Structure**
    | Byte | Value | Description |
    | :--- | :--- | :--- |
    | **Header** | `0xAA` | 패킷 시작 신호 |
    | **Data** | `0x01` / `0x00` | `0x01`: 정상품 (Good) / `0x00`: 불량품 (Defective) |
    | **Tail** | `0xFF` | 패킷 종료 신호 |
  * *Example*: `AA 01 FF` (정상품) / `AA 00 FF` (불량품)
* **Communication**
  * MCP2515 CAN Controller 연결 (SPI1 기반 통신)

---

## 3️⃣ Material Supply MCU (STM32F411RE)
* **Responsibilities**
  * CAN 네트워크를 통해 수신된 비전 판정 결과를 기반으로 후속 공정 판단
  * 약품 정량 공급 장치 및 캡(뚜껑) 공급 장치 정밀 제어
* **Hardware Components**
  * **Medicine Dispenser** : 28BYJ-48 스테퍼 모터 & ULN2003 드라이버
  * **Cap Supply System** : L298N 모터 드라이버 또는 솔레노이드 드라이버
* **Communication**
  * MCP2515 CAN Controller (인터럽트 구동 기반 실시간 메시지 처리)

---

## 4️⃣ Processing & Sorting MCU (STM32F411RE)
* **Responsibilities**
  * CAN 데이터를 기반으로 최종 가공품 압착(Sealing) 공정 제어
  * 정상품 및 1차 불량품 분류 액추에이터 제어
  * 불량품 패스 처리 및 최종 이송 컨베이어 연동
* **Hardware Components**
  * **Compression & Sorting Actuator** : L298N 모터 드라이버
  * **Sensors** : IR Sensors (압착 위치 및 분류 위치 감지)
* **Communication**
  * MCP2515 CAN Controller 연결

---

## 5️⃣ Vision Processing System (Raspberry Pi)
* **Responsibilities**
  * 카메라로부터 실시간 영상 취득
  * OpenCV 및 AI 기반 알고리즘을 통한 제품 외관/내용물 검사 수행
  * 정상/불량 판정 결과를 UART 3-Byte 패킷 형태로 Vision Bridge MCU에 전송

---

## 6️⃣ Final Robot Arm Controller (Raspberry Pi)
* **Responsibilities**
  * 공정이 끝난 최종 제품의 위치를 IR 센서로 감지
  * 물품 유무에 따른 컨베이어 일시 정지 및 구동 제어
  * 로봇팔과의 인터페이스를 통한 적재 프로세스(Pick & Place) 트리거
* **Robot Interface**
  * GPIO / UART / TCP-IP / ROS2 (프로젝트 환경에 따라 가변 적용)

---

# 📡 STM32 CAN Communication Specification

모든 STM32 보드는 동일한 SPI 핀 맵을 통해 MCP2515 CAN 컨트롤러와 통신합니다.

### Hardware Wiring (MCP2515 ↔ STM32F411RE)
| MCP2515 Pin | STM32F411RE Pin | Function |
| :--- | :--- | :--- |
| **SCK** | PA5 | SPI1 Clock |
| **MISO** | PA6 | SPI1 Master In Slave Out |
| **MOSI** | PA7 | SPI1 Master Out Slave In |
| **CS** | PA4 | SPI1 Chip Select |
| **INT** | PB0 | CAN Receive Interrupt |

### Protocol Configuration
* **Peripheral** : SPI1 Hardware Peripheral
* **Protocol Version** : CAN 2.0B Active
* **Network Topology** : Multi-node Broadcast Network

---

# 🔄 System Workflow

    [1] 약통 투입 감지 (Main Infeed MCU)
             │
             ▼
     메인 컨베이어 구동 시작
             │
             ▼
    [2] 비전 영역 도착 감지 (Vision Bridge MCU)
             │
             ▼
     Raspberry Pi 비전 트리거 신호 송신
             │
             ▼
    [3] AI 비전 검사 수행 (Raspberry Pi)
             │
             ▼
     UART 결과 송신 (3-Byte Packet -> Vision Bridge MCU)
             │
             ▼
    [4] 판정 데이터 CAN Bus 브로드캐스트 (Vision Bridge MCU)
             │
             ├───► Material Supply MCU (판정 결과에 따른 약품 및 캡 투입)
             │
             └───► Processing & Sorting MCU (압착 가공 및 1차 정상/불량 분류)
             │
             ▼
    [5] 최종 물류 처리 (Final Robot Arm Controller)
             │
             ├── IR 센서 제품 감지 시 컨베이어 정지
             └── 로봇팔 연동을 통한 Pick & Place (최종 적재) 수행

---

# 🚀 Summary
* **분산 제어 최적화** : 4개의 STM32 MCU가 CAN Bus를 통해 역할을 분담하여 실시간 동기화 구현
* **지능형 공정 검사** : Raspberry Pi의 비전 솔루션을 활용하여 정밀 불량품 판정 및 전처리 자동화
* **모듈화 구조** : 공정 단계별 하드웨어 및 소프트웨어가 독립적인 노드로 구성되어 확장 및 유지보수 용이
* **스마트 팩토리 구현** : 투입부터 검사, 가공, 로봇 arm 적재까지 연동된 엔드투엔드 제조 시스템
