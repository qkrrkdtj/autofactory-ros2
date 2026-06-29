# 2nd Board CAN Implementation Guide

## 1. 시스템 개요

약통 패키징 자동화 공정. 총 3개 보드, 2개 컨베이어 벨트.

```
         ┌──[컨베이어 벨트 1]─────────────────────────────┐   ┌──[벨트 2]──┐
약통투입 ──► [1번 보드] ──────────► [2번 보드] ────────────► [3번 보드] ──►
             비전검사                알약투입 + 뚜껑투입       뚜껑압착 + 분류
             OK/NG 판별              (OK만 공정 수행)          (OK만 공정 수행)

                         CAN Bus (125kbps, MCP2515)
              1번 ─────────────────────────────────── 3번
                        └──── 2번 ────┘
```

**CAN 통신의 목적:**
- 1번 보드가 비전 검사 결과(OK/NG)를 브로드캐스트
- 2번, 3번 보드가 이를 수신하여 FIFO 큐에 저장
- 각 보드의 센서가 약통을 감지하면 큐에서 꺼내 OK/NG 판단 후 공정 수행 여부 결정
- NG 약통은 어떤 공정도 수행하지 않고 벨트를 계속 구동하여 통과시킴

**FIFO 보장:** 컨베이어 벨트 특성상 약통은 투입된 순서대로 각 스테이션에 도달하므로, CAN 큐의 순서와 실제 약통 도착 순서가 항상 일치함.

---

## 2. CAN 프로토콜 명세

### CAN 설정

| 항목 | 값 |
|------|----|
| 비트레이트 | 125 kbps |
| 프레임 | Standard (11-bit ID) |
| CAN Controller | MCP2515 (SPI 연결) |
| 오실레이터 | 8 MHz (자동 감지) |

### CAN ID 체계

```
0x1XX = 1번 보드 발신
0x2XX = 2번 보드 발신
0x3XX = 3번 보드 발신
```

### 메시지 타입 정의

#### `CONTAINER_DETECT` — 1번 보드 → 전체 브로드캐스트

약통이 1번 보드 센서에 감지되고 비전 검사 완료 직후 전송.

```
CAN ID : 0x101
DLC    : 3

Data[0] = 0x01               (MSG_TYPE = CONTAINER_DETECT)
Data[1] = seq                (0~255 롤링 카운터, 약통 고유 번호)
Data[2] = 0x01 / 0x00        (OK=1, NG=0)
```

#### `PROCESS_DONE` — 각 보드 → 전체 브로드캐스트 (선택, 모니터링용)

각 보드에서 공정이 완료되거나 약통이 스테이션을 통과할 때 전송.

| 보드 | CAN ID |
|------|--------|
| 2번  | 0x201  |
| 3번  | 0x301  |

```
CAN ID : 0x201 (2번 보드)
DLC    : 3

Data[0] = 0x02               (MSG_TYPE = PROCESS_DONE)
Data[1] = seq                (수신한 seq 그대로)
Data[2] = 0x01 / 0x00        (processed=1, skipped=0)
```

---

## 3. 2번 보드 역할 및 동작 정의

### 공정 흐름

```
센서 감지
    │
    ▼
canRxQueue에서 dequeue
    │
    ├─ OK ─► 벨트 정지 → 알약 투입 액추에이터 → 뚜껑 투입 액추에이터 → 벨트 재개
    │
    └─ NG ─► 아무것도 안 함 (벨트 계속) → 센서 해제 대기
    │
    ▼
(선택) PROCESS_DONE 전송
```

### NG 약통 처리 원칙

- 벨트를 멈추지 않음
- 액추에이터 동작 없음
- 센서에서 약통이 빠져나갈 때까지 대기 후 다음 감지 준비

---

## 4. 데이터 구조

```c
/* ContainerInfo: 약통 추적 단위 */
typedef struct {
    uint8_t seq;    /* 약통 고유 번호 (1번 보드 부여) */
    uint8_t is_ok;  /* 1=OK, 0=NG */
} ContainerInfo;
```

---

## 5. FreeRTOS 태스크 구조

```
┌─────────────────────┐   canRxQueue   ┌──────────────────────────┐
│  CANRxTask          │ ─────────────► │  SequenceTask            │
│                     │ ContainerInfo  │                           │
│  MCP2515_Receive()  │                │  센서 감지 → dequeue     │
│  → ID=0x101 필터    │                │  OK  → 공정 수행         │
│  → canRxQueue enqueue│               │  NG  → 통과 대기         │
│                     │                │  (선택) PROCESS_DONE TX  │
└─────────────────────┘                └──────────────────────────┘

canRxQueue 크기: 8~16 (동시 운반 가능한 최대 약통 수 이상)
```

### 태스크 우선순위

| 태스크 | 우선순위 | 스택 |
|--------|----------|------|
| CANRxTask | `osPriorityBelowNormal` | 256 × 4 bytes |
| SequenceTask | `osPriorityNormal` | 512 × 4 bytes |

---

## 6. 하드웨어 핀 배치

3번 보드와 동일한 MCP2515 + SPI1 구성을 사용.

| 핀 | 기능 |
|----|------|
| PA4 | MCP2515 CS (GPIO Output, 초기값 HIGH) |
| PA5 | SPI1 SCK |
| PA6 | SPI1 MISO |
| PA7 | SPI1 MOSI |
| PB0 | MCP2515 INT (GPIO Input, PULLUP) |

> **MCP2515 모듈 VCC는 반드시 5V 연결** (TJA1050 트랜시버 요구사항)

CubeMX 설정은 3rd_board의 `stm32_can.ioc`를 참고하여 동일하게 구성.

---

## 7. 구현 체크리스트

### CubeMX
- [ ] SPI1 활성화 (Full-Duplex Master, Prescaler 16, CPOL Low, CPHA 1Edge)
- [ ] PA4 GPIO Output (MCP2515 CS)
- [ ] PB0 GPIO Input (MCP2515 INT)
- [ ] FreeRTOS CMSIS-RTOS v2 활성화
- [ ] TIM11 Timebase (SysTick 대신)

### main.h USER CODE
- [ ] `MCP2515_CS_Pin`, `MCP2515_CS_GPIO_Port` define
- [ ] `MCP2515_INT_Pin`, `MCP2515_INT_GPIO_Port` define
- [ ] `ContainerInfo` typedef

### main.c USER CODE
- [ ] `#include "mcp2515.h"` 추가
- [ ] `MCP2515_HandleTypeDef hmcp2515` 전역 선언
- [ ] `osMessageQueueId_t canRxQueue` 전역 선언
- [ ] `MX_GPIO_Init_2`에 CS HIGH 초기화, PB0 PULLUP 재적용
- [ ] `USER CODE BEGIN 2`에 MCP2515 AutoDetect + SetNormalMode
- [ ] `CANRxTask` 구현 (ID=0x101 수신 → canRxQueue enqueue)
- [ ] `SequenceTask` 구현 (센서 감지 → dequeue → OK/NG 분기)
- [ ] RTOS_QUEUES에 `canRxQueue = osMessageQueueNew(16, sizeof(ContainerInfo), NULL)`
- [ ] RTOS_THREADS에 CANRxTask 등록

### 검증
- [ ] 루프백 테스트 PASS
- [ ] 1번 보드와 CAN 핑 테스트 PASS
- [ ] OK 약통: 공정 수행 확인
- [ ] NG 약통: 통과 확인

---

## 8. 참고 코드 (3rd_board 패턴)

3rd_board는 동일한 패턴으로 먼저 구현되어 있음. 아래 파일을 참고:

```
stm32/3rd_board/Core/Src/main.c   — MCP2515 초기화, CANPingTask, SequenceTask
stm32/3rd_board/Core/Inc/main.h   — 핀 define, ContainerInfo typedef
stm32/3rd_board/Core/Src/mcp2515.c — MCP2515 드라이버 (동일 파일 복사 사용)
stm32/3rd_board/Core/Inc/mcp2515.h — 동일
```
