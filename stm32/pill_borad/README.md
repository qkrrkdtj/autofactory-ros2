# STM32(NUCLEO-F411RE) 보드를 통한 스텝 모터(28BYJ-48) 및 드라이버(ULN2003) 제어

NUCLEO-F411RE 보드와 ULN2003 드라이버를 활용하여 28BYJ-48 스텝 모터를 부드럽게 정방향/역방향으로 회전시키는 기본적인 구동 테스트 프로젝트입니다.

---

## 🛠 1. 개발 환경 (Development Environment)
- **OS:** Ubuntu 24.04 LTS
- **IDE:** STM32CubeIDE (Version 2.1.1) / STM32CubeMX (Standalone)
- **Language:** C (HAL 라이브러리 사용)

---

## 🔌 2. 하드웨어 구성 및 결선 (Hardware & Wiring)

### 주요 부품
- **MCU:** STM32 NUCLEO-F411RE
- **Motor:** 28BYJ-48 스텝 모터 (5V)
- **Driver:** ULN2003 모터 드라이버 보드
- **Power:** 외부 전원 (AA 건전지 홀더)

### 핀 맵 (Pin Mapping)
| NUCLEO-F411RE | ULN2003 Driver | 설명 |
| :--- | :--- | :--- |
| **PC0** | IN1 | 모터 제어 신호 1 |
| **PC1** | IN2 | 모터 제어 신호 2 |
| **PC2** | IN3 | 모터 제어 신호 3 |
| **PC3** | IN4 | 모터 제어 신호 4 |
| **GND** | - (GND) | **공통 접지 (GND 공유 필수)** |
| **외부 전원 (+)** | + (5-12V) | 모터 구동용 외부 전원 공급 |

> ⚠️ **주의:** 스텝 모터 구동 시 전류 소모로 인한 MCU 보드 리셋을 방지하기 위해 반드시 외부 전원(AA 건전지 등)을 사용해야 하며, MCU의 GND와 외부 전원의 (-)극, 드라이버의 GND를 모두 하나로 묶어주는 **공통 접지(Common GND)** 작업이 필수적입니다.

---

## ⚙️ 3. 핵심 구현 내용 (Key Implementation)
- **8스텝(Half-Step) 구동 시퀀스:** 모터가 가장 부드럽고 정밀하게 돌 수 있도록 8단계 상여자 방식을 배열(`half_step_seq[8][4]`)로 구현했습니다.
- **감속 기어비 반영:** 28BYJ-48 모터의 내부 기어비(1:64)와 스텝 수(64)를 고려하여, 출력축 기준 **1바퀴(360도) 회전에 필요한 4,096 스텝**을 이중 `for`문 구조로 정밀 제어했습니다.
- **최적의 펄스 간격:** 탈조(Step Loss) 현상 없이 토크를 확보할 수 있는 안정적인 속도인 `HAL_Delay(2)`(스텝당 2ms 간격)를 적용했습니다.

---

## 🚀 4. 트러블슈팅 및 해결 과정 (Troubleshooting)

### 1) Ubuntu 24.04 환경에서 STM32CubeIDE 새 프로젝트 생성 오류
- **문제:** IDE 내에서 새 STM32 프로젝트 생성 시 보드 선택 창(Target Selector)이 뜨지 않고 이클립스 기본 빈 프로젝트 메뉴만 반복 출력되는 플러그인 호환성 버그 발생.
- **해결:** 단독 실행형(Standalone) **STM32CubeMX 프로그램**을 따로 실행하여 보드 및 핀 설정을 마친 뒤, 툴체인을 `STM32CubeIDE`로 지정하여 코드를 생성(`GENERATE CODE`)했습니다. 이후 IDE에서 `File -> Import -> Existing Projects into Workspace` 메뉴를 통해 불러오는 방식으로 완벽히 우회했습니다.

### 2) 우분투 환경에서 ST-Link USB 권한 거부 오류 (`No ST-LINK detected`)
- **문제:** 보드를 연결하고 다운로드 시 USB 포트 접근 권한 문제로 업로드가 실패함.
- **해결:** 터미널에서 현재 사용자를 `dialout` 그룹에 추가하여 권한 문제를 해결했습니다.
  ```bash
  sudo usermod -aG dialout $USER
