"""
check_robot_state.py

OMX 로봇을 현재 자세(예: home 위치로 손으로 옮겨놓은 상태)에서
현재 관절값을 읽어서 출력합니다.

robot_executor.py 의 HOME_POSITION 딕셔너리에 그대로 복사해 넣을 수 있는
형태로 출력해줍니다.

사용법:
    1. 로봇을 원하는 home 자세로 둔다 (수동으로 옮기거나, 이미 그 자세인 상태)
    2. python3 check_robot_state.py --port /dev/omx_follower
    3. 출력된 HOME_POSITION 딕셔너리를 robot_executor.py 에 복사
"""

import argparse
import time

from lerobot.robots.utils import make_robot_from_config
from lerobot.robots.omx_follower import OmxFollowerConfig

JOINT_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]


def main():
    parser = argparse.ArgumentParser(description="OMX 로봇의 현재 관절값을 읽어서 출력합니다.")
    parser.add_argument("--port", required=True, help="로봇 시리얼 포트 (예: /dev/omx_follower)")
    parser.add_argument(
        "--samples", type=int, default=5,
        help="몇 번 읽어서 평균낼지 (기본 5회, 약간의 노이즈 완화용)"
    )
    parser.add_argument(
        "--interval", type=float, default=0.2,
        help="샘플 간 대기 시간(초, 기본 0.2초)"
    )
    args = parser.parse_args()

    print(f"[INIT] 로봇 연결 중... (port={args.port})")
    robot = make_robot_from_config(OmxFollowerConfig(port=args.port))
    robot.connect()
    print("[INIT] 로봇 연결 완료")

    try:
        print(f"[READ] {args.samples}회 측정 시작 (간격 {args.interval}초)")
        all_samples = []
        for i in range(args.samples):
            obs = robot.get_observation()
            sample = {k: obs[k] for k in JOINT_KEYS}
            all_samples.append(sample)
            print(f"  [{i+1}/{args.samples}] " +
                  ", ".join(f"{k}={v:.3f}" for k, v in sample.items()))
            time.sleep(args.interval)

        # 평균값 계산
        avg = {
            k: sum(s[k] for s in all_samples) / len(all_samples)
            for k in JOINT_KEYS
        }

        print("\n" + "=" * 60)
        print("아래 내용을 robot_executor.py 의 HOME_POSITION 에 복사하세요:")
        print("=" * 60)
        print("HOME_POSITION = {")
        for k in JOINT_KEYS:
            print(f'    "{k}": {avg[k]:.3f},')
        print("}")
        print("=" * 60)

    finally:
        robot.disconnect()
        print("[SYS] 로봇 연결 해제")


if __name__ == "__main__":
    main()
