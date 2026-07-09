"""
공통 설정 파일

OMX1 / OMX2 PC의 IP, 포트, 정책 이름, 로봇 실행 관련 설정을 한 곳에서 관리합니다.
실제 값이 확정되면 여기만 수정하면 됩니다.
"""

OMX_CONFIGS = {
    "omx1": {
        # ---- 네트워크 (관제서버 <-> OMX1 PC 통신) ----
        "host": "192.168.100.83",   # TODO: 실제 OMX1 PC IP로 교체
        "port": 9001,

        # ---- 역할 ----
        "policy_name": "pick1_ep0_400",   # 적재 정책 (로그/메시지용 짧은 이름)
        "role": "loading",

        # ---- 로봇 실행 관련 (OMX1 PC 로컬에서만 사용) ----
        "robot_port": "/dev/omx_follower",          # TODO: 실제 시리얼 포트 확인 필요
        "policy_path": "angrynose/pick1_ep0_400",     # HuggingFace repo_id (로컬 캐시 사용)
        "dataset_repo_id": "angrynose/pick1",          # 학습에 사용한 데이터셋 repo_id (메타데이터만 사용, action 이름 순서 확인용)
        "home_position": {
            "shoulder_pan.pos": 2.125,
            "shoulder_lift.pos": -64.542,
            "elbow_flex.pos": 55.556,
            "wrist_flex.pos": 51.844,
            "wrist_roll.pos": 3.687,
            "gripper.pos": 50.095,
        },
        "cameras": {
            "observation.images.front": {"index_or_path": '/dev/cam_front', "fps": 30, "width": 640, "height": 480},
            "observation.images.wrist": {"index_or_path": '/dev/cam_wrist', "fps": 30, "width": 640, "height": 480},
        },
        "depart_target_count": 3,                          # A: 3개 차면 출발
        "tray_camera_key": "observation.images.wrist",     # 트레이 보는 캠(cam_wrist)
    },
    "omx2": {
        # ---- 네트워크 (관제서버 <-> OMX2 PC 통신) ----
        "host": "192.168.100.165",   # TODO: 실제 OMX2 PC IP로 교체
        "port": 9002,

        # ---- 역할 ----
        "policy_name": "classify1_ep0_800",   # 분류 정책 (로그/메시지용 짧은 이름)
        "role": "sorting",

        # ---- 로봇 실행 관련 (OMX2 PC 로컬에서만 사용) ----
        "robot_port": "/dev/omx_follower",          # TODO: 실제 시리얼 포트 확인 필요 (OMX1과 PC가 다르면 보통 같은 경로명일 수 있음)
        "policy_path": "angrynose/classify1_ep0_800",   # HuggingFace repo_id (로컬 캐시 사용)
        "dataset_repo_id": "angrynose/classify1",   # TODO: 실제 학습 데이터셋 repo_id 확인 필요 (train_config.json의 dataset.repo_id 값)
        "home_position": {
            "shoulder_pan.pos": -0.757,
            "shoulder_lift.pos": -63.175,
            "elbow_flex.pos": 54.579,
            "wrist_flex.pos": 55.389,
            "wrist_roll.pos": -2.173,
            "gripper.pos": 50.134,
        },
        "cameras": {
            "observation.images.front": {"index_or_path": '/dev/cam_front', "fps": 30, "width": 640, "height": 480},
            "observation.images.wrist": {"index_or_path": '/dev/cam_wrist', "fps": 30, "width": 640, "height": 480},
        },
        "depart_target_count": 0,                          # C: 트레이 비면 출발
        "tray_camera_key": "observation.images.wrist",     # 트레이 보는 캠(cam_wrist)
    },
}

# 관절 키 순서 고정 (action tensor <-> dict 변환 시 사용)
JOINT_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]

# 로봇 실행 관련 안전/타이밍 설정
HOME_THRESHOLD = 5.0       # degree, 이 값 이내면 home 도달로 판정
MAX_EPISODE_STEPS = 3000    # policy 한 사이클의 최대 스텝 수 (안전망)
HOMING_TIMEOUT = 25.0       # home 복귀 시도 최대 시간(초). 넘으면 실패 처리
STEP_DELAY = 0.03           # 각 스텝 사이 대기 시간(초)

# 연결 관련 타임아웃 설정 (초)
CONNECT_TIMEOUT = 5.0          # 최초 TCP 연결 시도 타임아웃
RECONNECT_INTERVAL = 3.0       # 연결 끊겼을 때 재연결 시도 간격
ALIGN_TIMEOUT = 30.0           # 정렬 측정 요청 후 응답을 기다리는 최대 시간
                                 # (OMX 쪽 카메라 프레임 확보/검출 재시도가 10초를 넘길 수 있음)
POLICY_TIMEOUT = 60.0          # 정책 실행 요청 후 완료 메시지를 기다리는 최대 시간
                                 # (이 시간 넘으면 "응답 없음" 으로 간주하고 경고)

# 메시지 구분자 (한 줄 = 한 메시지, 줄바꿈으로 구분)
MESSAGE_DELIMITER = b"\n"

# ── 컨베이어 물건 감지 센서 (라즈베리파이 → 관제서버 UDP) ──
CONVEYOR_SENSOR_PORT = 9100        # UDP 수신 포트 (라즈베리파이 stepper.py가 이 포트로 전송)
CONVEYOR_SENSOR_FRESH_SEC = 2.0    # 이 시간 내 수신이 없으면 신호 무효(끊김) 처리

