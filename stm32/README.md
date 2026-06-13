🏭 Automated Packaging & Inspection System

본 프로젝트는 4개의 STM32F411RE와 2개의 Raspberry Pi를 활용하여
제품의 투입, 내용물 공급, 가공, 비전 검사 및 최종 분류 과정을 자동화한 스마트 제조 시스템입니다.

STM32 보드들은 MCP2515 CAN Bus 네트워크를 통해 실시간으로 공정 데이터를 공유하며,
Raspberry Pi는 고성능 비전 알고리즘 판정 및 로봇팔 연동을 담당하는 상위 제어 장치로 동작합니다.

🏗️ System Architecture
    [ Raspberry Pi ]

(Vision Processing System) (Final Control & Robot Arm)
│ ▲
(UART: 3-Byte Packet) │
│ (GPIO / UART / ROS2)
▼ │
[ Vision Bridge MCU ] │
│ │
(CAN Bus / MCP2515) │
│ │
▼ │
[ Main Infeed MCU ] ─── [ Material Supply MCU ] ─── [ Processing & Sorting MCU ]
└─────────────────────────── CAN BUS (MCP2515) ───────────────────────────────┘

⚙️ System Overview
🔌 STM32 CAN Network (4 Nodes)
Main Infeed MCU : 공정 시작 제어 및 메인 컨베이어, 약통 투입 관리
Vision Bridge MCU : 비전 트리거 및 Raspberry Pi 결과 수신/브로드캐스트
Material Supply MCU : 약품 및 뚜껑 공급 장치 제어
Processing & Sorting MCU : 압착 가공 및 1차 불량품 분류
🧠 Raspberry Pi Systems (2 Nodes)
Vision Processing System
카메라 기반 실시간 AI 비전 검사
OpenCV / 딥러닝 기반 정상/불량 판정
Final Robot Arm Controller
최종 컨베이어 제어
로봇팔 Pick & Place 연동
🎛️ Controller Specifications & Roles
1️⃣ Main Infeed MCU (STM32F411RE)
Responsibilities
생산 라인 공정 시작 및 컨베이어 제어
IR 센서 기반 약통 진입 감지
투입 액추에이터 제어
CAN Bus 상태 전송
Hardware Components

Bottle Pusher Actuator

Driver: L298N
IN1: 방향 제어
IN2: 방향 제어
ENA: PWM 속도 제어

Main Conveyor Control

Relay: JQC-3FF-S-Z
NPN 트랜지스터 + Flyback Diode 보호회로

Sensors

IR Sensor: 약통 투입 위치 감지
2️⃣ Vision Bridge MCU (STM32F411RE)
Responsibilities
IR 센서 기반 비전 트리거
Raspberry Pi로 검사 신호 전송
UART 결과 수신
CAN Bus 브로드캐스트

UART Packet Format

Header : 0xAA
Data : 0x01 (Good) / 0x00 (Defective)
Tail : 0xFF

Example
AA 01 FF → 정상
AA 00 FF → 불량

Communication

MCP2515 CAN Controller
SPI1 기반 통신
3️⃣ Material Supply MCU (STM32F411RE)
Responsibilities
CAN 메시지 기반 공정 판단
약품 정량 투입
캡 공급 제어

Hardware

Medicine Dispenser

Stepper Motor 28BYJ-48
ULN2003 Driver

Cap Supply System

L298N or Solenoid Driver

Communication

MCP2515 CAN (Interrupt 기반)
4️⃣ Processing & Sorting MCU (STM32F411RE)
Responsibilities
압착(Sealing) 공정 제어
불량품 1차 분류
최종 컨베이어 이송 제어

Hardware

L298N Motor Driver
IR Sensors
5️⃣ Vision Processing System (Raspberry Pi)
Responsibilities
카메라 영상 처리
OpenCV / AI 기반 검사
UART 3-byte 결과 송신

Output

0x01 : Good
0x00 : Defective
6️⃣ Final Robot Arm Controller (Raspberry Pi)
Responsibilities
최종 제품 감지
컨베이어 정지
로봇팔 Pick & Place 수행

Robot Interface

GPIO / UART / TCP-IP / ROS2 중 선택
📡 STM32 CAN Communication Specification

MCP2515 Wiring

SCK → PA5
MISO → PA6
MOSI → PA7
CS → PA4
INT → PB0

SPI Configuration

SPI1
CAN 2.0B
Multi-node broadcast
🔄 System Workflow

[1] 약통 투입 감지 (Main Infeed MCU)
│
▼
컨베이어 시작
│
▼
비전 영역 도착 감지 (Vision Bridge MCU)
│
▼
Raspberry Pi 비전 트리거
│
▼
AI 검사 수행
│
▼
UART 결과 송신 (3-byte)
│
▼
CAN Bus 브로드캐스트
│
├── Material Supply MCU (투입/캡)
│
└── Processing MCU (압착 + 분류)
│
▼
Final Robot Arm Controller
│
├── IR 감지 → 컨베이어 정지
└── Pick & Place 수행

🚀 Summary
STM32 기반 CAN 분산 제어 시스템
Raspberry Pi 기반 AI 비전 검사
산업용 스마트 팩토리 자동화 구조
로봇팔 기반 최종 물류 처리
