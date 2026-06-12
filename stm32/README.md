# STM32

# 🔧 STM32 Control Network

본 시스템의 핵심 제어부는 **3개의 STM32F411RE 보드(Board 1~3)** 로 구성되며, 각 보드는 **MCP2515 CAN Controller**를 통해 실시간으로 데이터를 공유하며 협력 동작합니다.

---

## 🚌 STM32 CAN Network Architecture

```text
[ STM32F411RE (Board 1) ] ─── [ STM32F411RE (Board 2) ] ─── [ STM32F411RE (Board 3) ]
└──────────────────────── MCP2515 CAN BUS ────────────────────────┘
```

* **Board 1** : Main Controller & Front-End Control
* **Board 2** : Medicine / Cap Supply Control
* **Board 3** : Processing & First Sorting Control

각 STM32 보드는 CAN Bus를 통해 공정 상태를 공유하며, 이벤트 기반으로 액추에이터를 제어합니다.

---

## 📡 Common CAN Configuration

모든 STM32F411RE 보드는 동일한 MCP2515 연결 구조를 사용합니다.

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
* Topology : Multi-node Broadcast Network

---

# 1️⃣ Board 1 — Front-End Control & Main Controller

### Responsibilities

* Production process start control
* Bottle insertion management
* Main conveyor operation
* IR sensor monitoring
* Vision inspection result reception from Raspberry Pi
* CAN message broadcast to Board 2 and Board 3

### Hardware Components

#### UART Communication (Raspberry Pi ↔ STM32)

```text
Raspberry Pi TX ─────► STM32 RX
Raspberry Pi RX ◄───── STM32 TX
```

* Voltage Level : 3.3V
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

---

# 2️⃣ Board 2 — Material Supply Control

### Responsibilities

Board 1으로부터 수신한 CAN 메시지를 기반으로 다음 작업을 수행합니다.

* Medicine dispensing
* Cap feeding
* Precise actuator synchronization

### Hardware Components

#### Medicine Dispenser

* Stepper Motor : 28BYJ-48
* Driver : ULN2003

기준 위치 제어를 통해 정량의 약을 투하합니다.

#### Cap Supply Actuator

* Motor Driver : L298N (HW-095)
* DC Motor Driven Mechanism

#### Communication

* MCP2515 CAN Controller
* Event-driven operation via CAN messages

---

# 3️⃣ Board 3 — Processing & First Sorting Control

### Responsibilities

Board 1이 전송한 CAN 데이터를 기반으로 다음 작업을 수행합니다.

* Product compression process
* Conveyor transfer control
* First-stage sorting
* Defective product bypass

### Hardware Components

#### Compression Actuator

* L298N Motor Driver
* Independent Channel Control

#### Sorting Actuator

* L298N Motor Driver
* Defective product bypass mechanism

#### Sensors

* Infrared Sensors
* Compression position detection
* Sorting position detection

#### Communication

* MCP2515 CAN Controller
* Real-time process synchronization through CAN Bus

---

## 🔄 STM32 Communication Flow

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
      │             │
      │             └─ Medicine & Cap Supply
      │
      └────────► Board 3
                    │
                    └─ Compression & Sorting
```

모든 STM32 노드는 CAN Bus 기반 이벤트 전달 구조를 사용하여 공정 상태를 공유하고, 각 단계의 액추에이터를 독립적으로 제어합니다.
