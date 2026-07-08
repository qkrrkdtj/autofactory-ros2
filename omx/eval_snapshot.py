#!/usr/bin/env python3
"""
eval_snapshot.py — 캐노니컬 위치 테스트용 단독 스크립트

목적:
  트레이를 손으로 캐노니컬(가장 잘 되던) 위치에 놓고 정책을 1회 실행한 뒤,
  원점(home) 복귀를 감지하면 wrist 프레임 한 장을 저장한다.
  차량 도킹/미션 루프 없이 "오차 ~0에서 정책이 되는가"를 반복 측정하기 위한 도구.

omx_executor_server.py 에서 아래 부분만 떼어내 단독 실행하도록 재구성:
  - init_robot_and_policy  (로봇 연결 + 정책/전처리/후처리 로드)
  - _run_policy_steps      (정책 실행 루프 + home 복귀 debounce 감지)
  - _go_home_safely        (home 복귀 보장)
  - _grab_tray_frame_bgr   (wrist 프레임 → BGR numpy)
  - count_in_tray          (저장 시 개수 즉시 확인용, omx별 파일 자동 선택)
TCP / 프로토콜 / 미션 매니저는 전부 제외했다.

사용법:
  python3 eval_snapshot.py omx2      # 분류 (기본 테스트 대상)
  python3 eval_snapshot.py omx1      # 적재

흐름:
  스크립트가 뜨면 매 trial 마다 입력을 기다린다.
    Enter        → 정책 1회 실행 → home 복귀 → wrist 사진 저장 → 다음 trial
    오프셋 라벨   → 예: 1.0L (왼쪽 1cm) 라고 치면 파일명에 박혀 3단계 오프셋 테스트에 유용
    q            → 종료
  각 trial 시작 전에 팔을 home 으로 먼저 보내 시작 자세를 통일한다.

저장물 (eval_snapshots/ 아래):
  trialNN_<omx>_<label>_<ts>.png            원본 wrist 프레임 (centroid/분석용)
  trialNN_<omx>_<label>_<ts>_annotated.png  ROI/검출박스/카운트 오버레이 (즉시 확인용)

필요 패키지: torch, lerobot, opencv-python(cv2), numpy
필요 파일  : config.py, box_counter.py(omx2) / box_counter1.py(omx1)
             (omx_executor_server.py 와 같은 폴더에서 실행하면 됨)
"""

import os
import re
import sys
import time

import cv2
import numpy as np
import torch

from lerobot.robots.utils import make_robot_from_config
from lerobot.robots.omx_follower import OmxFollowerConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.utils.control_utils import predict_action
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.datasets.utils import build_dataset_frame

from config import (
    OMX_CONFIGS,
    MAX_EPISODE_STEPS, HOMING_TIMEOUT, STEP_DELAY, HOME_THRESHOLD,
)

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "eval_snapshots")
HOME_IGNORE_KEYS = {"gripper.pos"}   # 그리퍼는 사이클마다 변동 → home 판정 제외
HOME_RETURN_STREAK = 10              # 이 횟수만큼 연속 home 이면 복귀로 확정


class PolicyEvalSnapshot:
    def __init__(self, omx_id: str):
        if omx_id not in OMX_CONFIGS:
            raise ValueError(f"알 수 없는 omx_id: {omx_id} (가능: {list(OMX_CONFIGS.keys())})")

        self.omx_id = omx_id
        self.cfg = OMX_CONFIGS[omx_id]

        self.robot = None
        self.policy = None
        self.preprocessor = None
        self.postprocessor = None
        self.ds_features = None
        self.cam_key_map = None
        self.device = None

        self.tray_camera_key = self.cfg.get("tray_camera_key", "observation.images.wrist")
        
        # home 도달 판정에서 제외할 관절
        # omx2는 원점 복귀 시 좌우(shoulder_pan)가 약간 돌아가 있어도 인정.
        if self.omx_id == "omx2":
            self._home_ignore = {"gripper.pos", "shoulder_pan.pos"}
        else:
            self._home_ignore = {"gripper.pos"}

        # ── 카운터 선택 (omx별 독립 파일) ──
        if self.omx_id == "omx1":
            from box_counter1 import count_in_tray
        else:
            from box_counter import count_in_tray
        self._count_in_tray = count_in_tray

        # ── 정책 종류/override 분기 (omx_executor_server 와 동일) ──
        if self.omx_id == "omx1" or self.omx_id == "omx2":
            self._policy_class = ACTPolicy
            self._policy_overrides = ["--device=cuda"]
            self._use_amp = False
        else:
            self._policy_class = DiffusionPolicy
            self._policy_overrides = [
                "--device=cuda",
                "--use_amp=true",
                "--noise_scheduler_type=DDIM",
                "--num_inference_steps=10",
                "--n_action_steps=15",
            ]
            self._use_amp = True

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------
    def init_robot_and_policy(self):
        print(f"[{self.omx_id}] 로봇 연결 중... (port={self.cfg['robot_port']})")
        camera_configs = {}
        for cam_key, cam_cfg in self.cfg["cameras"].items():
            camera_configs[cam_key] = OpenCVCameraConfig(
                index_or_path=cam_cfg["index_or_path"],
                fps=cam_cfg["fps"],
                width=cam_cfg["width"],
                height=cam_cfg["height"],
            )
        self.robot = make_robot_from_config(
            OmxFollowerConfig(port=self.cfg["robot_port"], cameras=camera_configs)
        )
        self.robot.connect()
        print(f"[{self.omx_id}] 로봇 연결 완료")

        policy_path = self.cfg["policy_path"]
        print(f"[{self.omx_id}] Policy 로드 중... ({policy_path}, class={self._policy_class.__name__})")
        policy_cfg = PreTrainedConfig.from_pretrained(
            policy_path, cli_overrides=self._policy_overrides,
        )
        self.policy = self._policy_class.from_pretrained(policy_path, config=policy_cfg)
        self.policy.eval()
        self.device = torch.device(policy_cfg.device)
        print(f"[{self.omx_id}] Policy 로드 완료 (device: {self.device})")

        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=policy_path,
            preprocessor_overrides={"device_processor": {"device": policy_cfg.device}},
        )

        ds_meta = LeRobotDatasetMetadata(repo_id=self.cfg["dataset_repo_id"])
        self.ds_features = ds_meta.features

        self.cam_key_map = {}
        for cam_key in self.cfg["cameras"]:
            short_key = cam_key.removeprefix("observation.images.")
            self.cam_key_map[short_key] = cam_key
        print(f"[{self.omx_id}] 준비 완료\n")

    def shutdown(self):
        if self.robot is not None:
            self.robot.disconnect()
            print(f"[{self.omx_id}] 로봇 연결 해제")

    # ------------------------------------------------------------------
    # 정책 실행 + home
    # ------------------------------------------------------------------
    def _build_observation_frame(self, raw_obs: dict) -> dict:
        renamed_obs = {}
        for key, value in raw_obs.items():
            if key in self.cam_key_map.values():
                short_key = key.removeprefix("observation.images.")
                renamed_obs[short_key] = value
            else:
                renamed_obs[key] = value
        return build_dataset_frame(self.ds_features, renamed_obs, prefix="observation")

    def _is_at_home(self, obs: dict, home_position: dict) -> bool:
        for key, target in home_position.items():
            if key in self._home_ignore:
                continue
            current = obs.get(key)
            if current is None:
                return False
            if abs(current - target) > HOME_THRESHOLD:
                return False
        return True

    def _run_policy_steps(self) -> int:
        print(f"[{self.omx_id}] 정책 실행 시작")
        self.policy.reset()
        home_position = self.cfg["home_position"]
        has_left_home = False
        home_streak = 0

        step = 0
        for step in range(MAX_EPISODE_STEPS):
            raw_obs = self.robot.get_observation()
            observation = self._build_observation_frame(raw_obs)
            action_tensor = predict_action(
                observation=observation,
                policy=self.policy,
                device=self.device,
                preprocessor=self.preprocessor,
                postprocessor=self.postprocessor,
                use_amp=self._use_amp,
            )
            action_dict = make_robot_action(action_tensor, self.ds_features)
            self.robot.send_action(action_dict)

            if self._is_at_home(raw_obs, home_position):
                if has_left_home:
                    home_streak += 1
                    if home_streak >= HOME_RETURN_STREAK:
                        print(f"[{self.omx_id}] home 복귀 감지 — {step+1} 스텝에서 종료")
                        break
            else:
                has_left_home = True
                home_streak = 0
            time.sleep(STEP_DELAY)
        return step + 1

    def _go_home_safely(self) -> bool:
        home_position = self.cfg["home_position"]
        start = time.time()
        last_obs = None
        while time.time() - start < HOMING_TIMEOUT:
            obs = self.robot.get_observation()
            last_obs = obs
            if self._is_at_home(obs, home_position):
                return True
            self.robot.send_action(home_position)
            time.sleep(STEP_DELAY)

        # ── 타임아웃 시: 어느 관절이 얼마나 벗어났는지 출력 ──
        print(f"[{self.omx_id}] home 타임아웃 — 관절별 오차(deg):")
        for key, target in home_position.items():
            cur = last_obs.get(key) if last_obs else None
            if cur is None:
                print(f"    {key}: obs에 없음 (키 불일치?)")
                continue
            mark = "IGNORE" if key in self._home_ignore else f"{abs(cur - target):.2f}"
            print(f"    {key}: cur={cur:.2f} tgt={target:.2f} diff={mark}")
        return False

    # ------------------------------------------------------------------
    # 프레임 캡처 (wrist → BGR)
    # ------------------------------------------------------------------
    def _grab_tray_frame_bgr(self):
        try:
            raw_obs = self.robot.get_observation()
            img = raw_obs.get(self.tray_camera_key)
            if img is None:
                print(f"[{self.omx_id}] observation에 '{self.tray_camera_key}' 키 없음 "
                      f"(가능 키: {list(raw_obs.keys())})")
                return None
            if hasattr(img, "detach"):
                img = img.detach().cpu().numpy()
            img = np.asarray(img)
            if img.ndim == 3 and img.shape[0] in (1, 3) and img.shape[2] not in (1, 3):
                img = np.transpose(img, (1, 2, 0))
            if img.dtype != np.uint8:
                if img.max() <= 1.0:
                    img = (img * 255.0).clip(0, 255).astype(np.uint8)
                else:
                    img = img.clip(0, 255).astype(np.uint8)
            if img.ndim == 3 and img.shape[2] == 3:
                img = img[:, :, ::-1].copy()   # RGB → BGR
            return img
        except Exception as e:
            print(f"[{self.omx_id}] 프레임 변환 실패: {e}")
            return None

    def _count_with_boxes(self, frame):
        try:
            return self._count_in_tray(frame, return_boxes=True)
        except TypeError:
            return self._count_in_tray(frame), [], None

    # ------------------------------------------------------------------
    # 한 trial: home → 정책 → home → 캡처/저장
    # ------------------------------------------------------------------
    def run_trial(self, trial_no: int, label: str):
        print(f"\n===== trial {trial_no} (label='{label or '-'}') 시작 =====")

        print(f"[{self.omx_id}] 시작 자세로 home 이동 중...")
        if not self._go_home_safely():
            print(f"[{self.omx_id}] ⚠ 시작 home 실패 — 그래도 진행")

        steps = self._run_policy_steps()
        homed = self._go_home_safely()
        if steps >= MAX_EPISODE_STEPS:
            print(f"[{self.omx_id}] ⚠ MAX_EPISODE_STEPS({MAX_EPISODE_STEPS}) 도달")
        if not homed:
            print(f"[{self.omx_id}] ⚠ 종료 home 복귀 실패 (timeout {HOMING_TIMEOUT}s)")

        # ── home 상태에서 wrist 한 장 ──
        frame = self._grab_tray_frame_bgr()
        if frame is None:
            print(f"[{self.omx_id}] ⚠ 프레임 획득 실패 — 저장 건너뜀")
            return

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        safe_label = re.sub(r"[^0-9A-Za-z._-]", "", label) or "none"
        ts = time.strftime("%Y%m%d_%H%M%S")
        stem = f"trial{trial_no:02d}_{self.omx_id}_{safe_label}_{ts}"

        raw_path = os.path.join(OUTPUT_DIR, f"{stem}.png")
        cv2.imwrite(raw_path, frame)

        # ── 카운트 오버레이 (즉시 확인용) ──
        counts, boxes, roi = self._count_with_boxes(frame)
        vis = frame.copy()
        if roi is not None:
            x, y, w, h = roi
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 255), 2)
        for color, (bx, by, bw, bh) in boxes:
            bgr = (255, 0, 0) if color == "blue" else (0, 0, 255)
            cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), bgr, 2)
        lbl = (f"{self.omx_id} tray={counts.get('tray_found')} "
               f"R={counts.get('red', 0)} B={counts.get('blue', 0)} "
               f"total={counts.get('total', 0)}")
        cv2.putText(vis, lbl, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        anno_path = os.path.join(OUTPUT_DIR, f"{stem}_annotated.png")
        cv2.imwrite(anno_path, vis)

        print(f"[{self.omx_id}] 저장: {raw_path}")
        print(f"[{self.omx_id}] 카운트: {counts}")
        print(f"===== trial {trial_no} 완료 (성공/실패는 직접 기록) =====")

def main():
    omx_id = sys.argv[1] if len(sys.argv) >= 2 else "omx2"
    if omx_id not in OMX_CONFIGS:
        print(f"사용법: python3 eval_snapshot.py [{'|'.join(OMX_CONFIGS.keys())}]")
        sys.exit(1)

    ev = PolicyEvalSnapshot(omx_id)
    ev.init_robot_and_policy()

    print("──────────────────────────────────────────────")
    print(f" 캐노니컬 위치 테스트 — {omx_id}")
    print("  트레이 배치 후 Enter → 정책 실행 → home → 사진 저장")
    print("  오프셋 라벨 입력 가능 (예: 1.0L, 0.5R) / q = 종료")
    print("──────────────────────────────────────────────")

    trial_no = 0
    try:
        while True:
            label = input(f"\n[trial {trial_no + 1}] 트레이 배치 후 Enter (라벨/q): ").strip()
            if label.lower() == "q":
                break
            trial_no += 1
            try:
                ev.run_trial(trial_no, label)
            except Exception as e:
                print(f"[{omx_id}] trial {trial_no} 실행 중 예외: {e}")
    except (EOFError, KeyboardInterrupt):
        print("\n종료 요청 수신")
    finally:
        ev.shutdown()


if __name__ == "__main__":
    main()
