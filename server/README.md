# 관제 서버 (server_pkg)

관제서버 하나가 **OMX(TCP) · TurtleBot(ROS2 액션/토픽 + SSH bringup) · 라즈베리파이(SSH)** 를 모두 오케스트레이션하는 구조입니다. 이 문서는 그 셋을 묶어서 실행하는 방법을 다룹니다. STM32(임베디드 펌웨어)는 이 스택과 독립적으로 동작하며 관련 내용은 [`stm32/`](../stm32/)를 참고하세요.

전체 시스템 구성도는 [최상위 README의 "시스템 구성"](../README.md#시스템-구성)을 참고하세요.

## 목차

- [사전 준비](#사전-준비)
- [빌드](#빌드-관제서버-pc)
- [실행 순서](#실행-순서)
- [종료](#종료)
- [대시보드](#대시보드)
- [설정 파일 레퍼런스](#설정-파일-레퍼런스)
- [트러블슈팅](#트러블슈팅)

## 사전 준비

1. **네트워크**: 관제서버 PC가 OMX1 PC, OMX2 PC, TurtleBot1/2, Raspberry Pi5와 같은 네트워크에서 통신 가능해야 합니다.
2. **SSH 키 등록**: `ssh_launcher.py`가 비밀번호 입력 없이 SSH로 접속을 시도합니다. 관제서버 → TurtleBot1/2, Raspberry Pi5로 `ssh-copy-id` 등을 이용해 키 기반 접속을 미리 설정해두세요.
3. **설정 파일 확인/수정** (자세한 항목은 [설정 파일 레퍼런스](#설정-파일-레퍼런스) 참고):
   - `server_pkg/config.py` — OMX1/OMX2 IP·포트·정책 이름
   - `server_pkg/ssh_launcher.py` — TurtleBot1/2·Raspberry Pi5 SSH 접속 정보
   - `server_pkg/waypoints.yaml` — A~E 웨이포인트 좌표 (SLAM으로 맵을 먼저 만들고 좌표를 확정해야 함)
4. **OMX PC 요구 사항**: GPU, `torch`, `lerobot`, `opencv-python`, `numpy`, front/wrist 카메라 2대, 정렬 기준 파일(`canonical_ref.json`), 트레이 카운터(`box_counter.py`/`box_counter1.py`/`box_counter_back.py`) — 모두 [`omx/`](../omx/) 참고.

## 빌드 (관제서버 PC)

```bash
cd server
colcon build
source install/setup.bash
```

- ROS2 의존성: `rclpy`, `nav2_msgs`, `action_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs` (ROS2 설치에 포함)
- Python 의존성: `flask`, `pyyaml` (`setup.py`의 `install_requires`)

## 실행 순서

1. **OMX1 PC, OMX2 PC** 각각에서 실행 서버를 기동합니다.

   ```bash
   cd omx
   python3 omx_executor_server.py omx1   # OMX1 PC에서
   python3 omx_executor_server.py omx2   # OMX2 PC에서
   ```

   각각 자신의 포트(`config.py`의 `OMX_CONFIGS[...]["port"]`, 기본 9001/9002)에서 관제서버의 TCP 접속을 대기합니다.

2. **TurtleBot1/2, Raspberry Pi5는 수동으로 켤 필요가 없습니다.** 관제서버(`modi_bridge`)가 시작되면 `ssh_launcher.py`가 자동으로 각 기기에 SSH 접속해 `bringup`(터틀봇), `stepper.py`(라즈베리파이)를 원격 실행하고, `/keepout_filter_mask` 토픽이 뜰 때까지 대기한 뒤 대시보드에 "준비 완료"로 표시합니다.

3. **관제서버에서 메인 브릿지 실행**:

   ```bash
   ros2 run server_pkg modi_bridge
   ```

   - ROS2 도메인 3개(30=관제, 31=robot1, 32=robot2)를 브릿지
   - OMX1/OMX2로 TCP 연결 시도(끊기면 자동 재연결)
   - Flask 대시보드를 `5000`번 포트에 기동

4. 브라우저에서 `http://<관제서버 IP>:5000` 접속 → 투입할 로봇(1·2) 선택 → **미션 시작**.

> 참고: `control_server.py`는 OMX 연결만 단독으로 테스트하기 위한 구버전 CLI 스크립트입니다(`omx1 run` / `omx2 run` / `status`). 평소 실행에는 `modi_bridge` 하나만 쓰면 됩니다.

## 종료

관제서버 터미널에서 `Ctrl+C` → 진행 중이던 이동을 취소하고, `kill_all()`이 SSH로 TurtleBot/Raspberry Pi의 원격 프로세스까지 정리(SIGINT)합니다.

## 대시보드

![alt text](../docs/images/dashboard.png)

### 페이지

| 경로 | 내용 |
| --- | --- |
| `/` | 실시간 관제 대시보드 (지도, 로봇 위치, 로그) |
| `/stats` | 공정 통계 페이지 |

### 기능

1. **로봇 선택 투입** — 터틀봇 한 대만으로 공정을 시작하거나 두 대를 모두 투입할 수 있습니다.
2. **중단 / 재개** — 공정 중 로봇을 멈추고, 멈춘 위치에서 그대로 재개할 수 있습니다.
3. **실시간 위치 반영** — 지도에 각 로봇의 현재 위치가 실시간으로 표시됩니다(`/pose_stream`).
4. **배터리 실시간 표시** (`/battery_stream`)
5. **진행 로그 실시간 스트림** (`/log_stream`)
6. **A / C 지점 수동 강제 출발** — OMX 카메라의 출발 조건(트레이 개수)이 충족되지 않아도 대시보드 버튼으로 강제로 다음 지점으로 보낼 수 있습니다(`/signal/<wp>`).
7. **공정 이력 조회** — 로봇/성공여부로 필터링해 최근 공정 로그를 조회(`/api/stats/logs`), 요약 통계 조회(`/api/stats/summary`) — `stats_db`(SQLite)에 누적 저장됩니다.

## 설정 파일 레퍼런스

| 파일 | 항목 | 설명 |
| --- | --- | --- |
| `server_pkg/config.py` | `OMX_CONFIGS["omx1"/"omx2"].host/port` | 관제서버가 접속할 OMX PC의 IP·포트 |
| | `policy_name` | 실행할 정책 이름(로그 표시용) — omx1: `pick1_ep0_400`(loading), omx2: `classify1_ep0_800`(sorting) |
| | `depart_target_count` | 출발 판정 기준 개수 — A(omx1): 3(트레이 채워지면 출발), C(omx2): 0(트레이 비면 출발) |
| | `POLICY_TIMEOUT`, `CONNECT_TIMEOUT`, `RECONNECT_INTERVAL` | TCP 연결/정책 실행 타임아웃 설정 |
| `server_pkg/waypoints.yaml` | `A`~`E`, `E'` | Nav2 맵 좌표계 기준 웨이포인트(x, y, oz, ow). A=적재, B=분류 앞 대기, C=분류, D=분류 후 탈출, E=적재 앞 대기, E'=로봇2 시작 위치 |
| `server_pkg/ssh_launcher.py` | `DEVICES` | `turtlebot1`, `turtlebot2`, `raspberry`의 SSH host/user/실행 스크립트/종료 시 찾을 프로세스명 |

## 트러블슈팅

- **OMX가 계속 `DISCONNECTED`로 뜸** → `config.py`의 `host`/`port`가 실제 OMX PC와 일치하는지, 방화벽이 막고 있지 않은지, 해당 OMX PC에서 `omx_executor_server.py`가 떠 있는지 확인하세요.
- **`ssh_launcher.py`가 "SSH 접속 실패" 출력** → `DEVICES`의 IP/계정이 맞는지, 대상 기기가 켜져 있는지, SSH 키 기반 접속이 설정돼 있는지 확인하세요.
- **터틀봇이 "준비 완료"로 안 넘어감(무한 대기)** → `wait_for_topic_by_domain`이 `/keepout_filter_mask` 토픽을 계속 기다리는 상태입니다. 해당 로봇의 `ROS_DOMAIN_ID`(31/32)가 맞는지, Nav2 launch가 정상적으로 떴는지 로봇 쪽 로그를 확인하세요.
- **로봇이 A/C에서 계속 "출발조건 미충족 — 재추론"만 반복** → OMX 카메라가 트레이 개수를 목표치(`depart_target_count`)와 다르게 세고 있는 것입니다. 대시보드의 A/C 수동 해제 버튼으로 강제 출발시키거나, `omx/debug_departure/`에 저장되는 판정 디버그 이미지로 원인을 확인하세요.
- **정렬(`check_alignment`)이 계속 실패(`tray_found=False`)** → OMX PC에 `canonical_ref.json`(정렬 기준점)이 없거나 잘못됐을 수 있습니다. `omx/capture_canonical_ref.py`, `omx/calibrate_pixel_to_cm.py`로 기준을 다시 잡아보세요.
- **정렬은 되는데 위치 보정이 과도하게 커서 중단됨** → `apply_alignment_offset`/`apply_alignment_yaw`에 안전 상한(30cm/30도)이 있습니다. 오프셋이 비정상적으로 크게 측정된다면 정렬 기준(`canonical_ref.json`)이나 카메라 정렬 자체를 재점검하세요.
