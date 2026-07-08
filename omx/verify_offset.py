#!/usr/bin/env python3
"""
verify_offset.py — 정렬 오프셋(fwd, lat cm) 계산 검증 (차량 이동 없음)

미션에 붙이기 전에, 픽셀→cm 변환과 '부호'가 맞는지 눈으로 확인하는 도구.
차는 움직이지 않는다. Enter 칠 때마다 현재 wrist 트레이 중심을 재서,
기준점(캐노니컬)과의 차이를 (fwd, lat) cm 로 환산해 출력한다.

검증 방법:
  1) 차/트레이를 캐노니컬에 정확히 → 오프셋이 ≈(0, 0) 이어야 함.
     (0 에서 크게 벗어나면 기준점이 낡음 → capture_canonical_ref.py 재측정)
  2) 트레이를 '왼쪽으로 1cm' 밀고 Enter → '왼쪽 ~1cm' 로 읽혀야 함.
     '오른쪽'으로 읽히면 부호 반대 (lat_sign 반대로 재캘리브레이션).
  3) 트레이를 '앞으로 1cm'(카메라/팔 쪽) 밀고 Enter → '앞으로 ~1cm'.
  → 방향·크기가 맞으면 변환 검증 완료. 그 다음 차량 이동 배관으로 진행.

출력 두 가지:
  · 트레이 오프셋 : 지금 트레이가 캐노니컬에서 얼마나 벗어났나(민 방향과 일치해야 함)
  · 차량 보정량   : 이 오프셋을 지우려면 차를 어디로 얼마나 움직여야 하나(오프셋의 반대)

부호 규약(캘리브레이션과 동일):
  fwd > 0 = 앞(카메라/팔 쪽),  lat > 0 = 왼쪽  (lat_sign 반영됨)

사용법:
  python3 verify_offset.py omx2
"""

import json
import os
import sys
import time

import numpy as np

from lerobot.robots.utils import make_robot_from_config
from lerobot.robots.omx_follower import OmxFollowerConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

from config import OMX_CONFIGS, HOMING_TIMEOUT, STEP_DELAY, HOME_THRESHOLD

HERE = os.path.dirname(os.path.abspath(__file__))
REF_PATH = os.path.join(HERE, "canonical_ref.json")

N_FRAMES = 20
FRAME_DELAY = 0.04

# 정책 성공 밴드 (측정값): 앞뒤 ±0.5cm, 좌우 ±1.0cm
FWD_TOL = 0.5
LAT_TOL = 1.0


def _fwd_word(v):
    return f"앞으로 {v:.2f}cm" if v >= 0 else f"뒤로 {abs(v):.2f}cm"


def _lat_word(v):
    return f"왼쪽으로 {v:.2f}cm" if v >= 0 else f"오른쪽으로 {abs(v):.2f}cm"


class OffsetVerifier:
    def __init__(self, omx_id: str):
        if omx_id not in OMX_CONFIGS:
            raise ValueError(f"알 수 없는 omx_id: {omx_id}")
        self.omx_id = omx_id
        self.cfg = OMX_CONFIGS[omx_id]
        self.robot = None
        self.tray_camera_key = self.cfg.get("align_camera_key",
                                            self.cfg.get("tray_camera_key", "observation.images.wrist"))

        if omx_id == "omx2":
            self._home_ignore = {"gripper.pos", "shoulder_pan.pos"}
        else:
            self._home_ignore = {"gripper.pos"}

        from box_counter_back import count_in_tray
        self._count_in_tray = count_in_tray

        self.ref, self.Minv = self._load_ref()

    def _load_ref(self):
        if not os.path.exists(REF_PATH):
            raise FileNotFoundError(f"{REF_PATH} 없음 — 먼저 기준점/캘리브레이션 필요")
        with open(REF_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        entry = data.get(self.omx_id, {})
        pix = entry.get("pix2cm")
        if pix is None:
            raise KeyError(f"{self.omx_id}.pix2cm 없음 — calibrate_pixel_to_cm.py 먼저 실행")

        # 정렬 목표점: 기준점(cx,cy) 우선, 없으면 캘리브레이션 canon_px 폴백
        if "cx" in entry and "cy" in entry:
            ref = np.array([entry["cx"], entry["cy"]], dtype=float)
            print(f"[{self.omx_id}] 기준점(cx,cy) 사용: ({ref[0]:.1f}, {ref[1]:.1f})")
        else:
            ref = np.array(pix["canon_px"], dtype=float)
            print(f"[{self.omx_id}] ⚠ 기준점(cx,cy) 없음 → 캘리브레이션 canon_px 폴백: "
                  f"({ref[0]:.1f}, {ref[1]:.1f})")
            print(f"[{self.omx_id}]   정확도 위해 capture_canonical_ref.py 재측정 권장")

        Minv = np.array(pix["Minv"], dtype=float)
        print(f"[{self.omx_id}] Minv =\n{Minv}")
        return ref, Minv

    # ------------------------------------------------------------------
    def connect(self):
        camera_configs = {}
        for cam_key, cam_cfg in self.cfg["cameras"].items():
            camera_configs[cam_key] = OpenCVCameraConfig(
                index_or_path=cam_cfg["index_or_path"],
                fps=cam_cfg["fps"], width=cam_cfg["width"], height=cam_cfg["height"],
            )
        self.robot = make_robot_from_config(
            OmxFollowerConfig(port=self.cfg["robot_port"], cameras=camera_configs)
        )
        self.robot.connect()
        print(f"[{self.omx_id}] 로봇 연결 완료")

    def disconnect(self):
        if self.robot is not None:
            self.robot.disconnect()

    def _is_at_home(self, obs, hp):
        for key, target in hp.items():
            if key in self._home_ignore:
                continue
            cur = obs.get(key)
            if cur is None or abs(cur - target) > HOME_THRESHOLD:
                return False
        return True

    def go_home(self):
        hp = self.cfg["home_position"]
        start = time.time()
        while time.time() - start < HOMING_TIMEOUT:
            obs = self.robot.get_observation()
            if self._is_at_home(obs, hp):
                return True
            self.robot.send_action(hp)
            time.sleep(STEP_DELAY)
        return False

    def _grab_bgr(self):
        raw = self.robot.get_observation()
        img = raw.get(self.tray_camera_key)
        if img is None:
            return None
        if hasattr(img, "detach"):
            img = img.detach().cpu().numpy()
        img = np.asarray(img)
        if img.ndim == 3 and img.shape[0] in (1, 3) and img.shape[2] not in (1, 3):
            img = np.transpose(img, (1, 2, 0))
        if img.dtype != np.uint8:
            img = (img * 255.0).clip(0, 255).astype(np.uint8) if img.max() <= 1.0 \
                  else img.clip(0, 255).astype(np.uint8)
        if img.ndim == 3 and img.shape[2] == 3:
            img = img[:, :, ::-1].copy()
        return img

    def measure_centroid(self):
        xs, ys = [], []
        for _ in range(N_FRAMES):
            frame = self._grab_bgr()
            if frame is None:
                continue
            try:
                counts, _b, _r = self._count_in_tray(frame, omx_id=self.omx_id, return_boxes=True)
            except TypeError:
                counts = self._count_in_tray(frame, omx_id=self.omx_id)
            cx, cy = counts.get("tray_cx"), counts.get("tray_cy")
            if counts.get("tray_found") and cx is not None and cy is not None:
                xs.append(cx)
                ys.append(cy)
            time.sleep(FRAME_DELAY)
        if len(xs) < max(3, N_FRAMES // 4):
            return None
        return np.array([float(np.median(xs)), float(np.median(ys))])

    def offset_cm(self, cur):
        """트레이 오프셋(fwd,lat cm) = Minv @ (현재 - 기준). 민 방향과 같은 부호."""
        return self.Minv @ (cur - self.ref)

    # ------------------------------------------------------------------
    def run(self):
        print(f"\n팔을 home 으로 보냅니다...")
        if not self.go_home():
            print(f"[{self.omx_id}] ⚠ home 실패 — 시점 어긋나면 오프셋 부정확")

        print("\n─────────────────────────────────────────────")
        print(" 오프셋 검증 (차량 이동 없음) — Enter: 측정 / q: 종료")
        print(f" 성공 밴드: 앞뒤 ±{FWD_TOL}cm, 좌우 ±{LAT_TOL}cm")
        print("─────────────────────────────────────────────")

        while True:
            s = input("\n[측정] 트레이 배치 후 Enter (q=종료): ").strip()
            if s.lower() == "q":
                break
            cur = self.measure_centroid()
            if cur is None:
                print("  ✗ 트레이 측정 실패 — 위치/조명 확인")
                continue

            dpx = cur - self.ref
            fwd, lat = self.offset_cm(cur)
            in_band = abs(fwd) <= FWD_TOL and abs(lat) <= LAT_TOL

            print(f"  현재중심=({cur[0]:.1f},{cur[1]:.1f})  기준=({self.ref[0]:.1f},{self.ref[1]:.1f})  "
                  f"픽셀차=({dpx[0]:+.1f},{dpx[1]:+.1f})")
            print(f"  ▶ 트레이 오프셋 : {_fwd_word(fwd)}, {_lat_word(lat)}   "
                  f"(민 방향과 일치해야 정상)")
            print(f"  ▶ 차량 보정량   : {_fwd_word(-fwd)}, {_lat_word(-lat)}   "
                  f"(이만큼 움직이면 정렬)")
            if in_band:
                print(f"  ✓ 정책 성공 밴드 안 — 보정 불필요")
            else:
                out = []
                if abs(fwd) > FWD_TOL:
                    out.append(f"앞뒤 {abs(fwd):.2f}cm(>{FWD_TOL})")
                if abs(lat) > LAT_TOL:
                    out.append(f"좌우 {abs(lat):.2f}cm(>{LAT_TOL})")
                print(f"  ✗ 밴드 밖: {', '.join(out)} — 보정 필요")


def main():
    omx_id = sys.argv[1] if len(sys.argv) >= 2 else "omx2"
    if omx_id not in OMX_CONFIGS:
        print(f"사용법: python3 verify_offset.py [{'|'.join(OMX_CONFIGS.keys())}]")
        return
    v = OffsetVerifier(omx_id)
    v.connect()
    try:
        v.run()
    except (EOFError, KeyboardInterrupt):
        print("\n중단됨")
    finally:
        v.disconnect()


if __name__ == "__main__":
    main()
