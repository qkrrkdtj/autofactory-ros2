#!/usr/bin/env python3
"""
calibrate_pixel_to_cm.py — 수동 픽셀→cm 2x2 캘리브레이션 (omx별)

트레이 중심의 '픽셀 오차'를 차량 이동량 '(fwd, lat) cm'로 바꾸는 2x2 변환을
만든다. 차량을 손으로 알려진 거리만큼 움직이며 세 지점에서 트레이 중심을
측정하고, 그 차이로 행렬을 세운 뒤 역행렬(px→cm)까지 계산해 저장한다.

  [dpx_fwd, dpx_lat]   ← 차를 앞/옆으로 STEP_CM 움직였을 때의 픽셀 변화(2x2 열벡터)
  M (cm→px) = [[dpx_fwd_x, dpx_lat_x],
               [dpx_fwd_y, dpx_lat_y]]
  Minv (px→cm) = inv(M)   ← 런타임에 (기준점-현재) 픽셀차 → (fwd,lat) cm

측정 3지점 (차량을 손으로 이동, 자 대고 정확히):
  1) CANON : 캐노니컬 위치 (기준점과 같은 자리)
  2) FWD   : 캐노니컬에서 '앞으로' 정확히 STEP_CM cm  (카메라/팔 쪽 방향)
  3) LAT   : 캐노니컬에서 '옆으로' 정확히 STEP_CM cm  (기본: 왼쪽 +)
  ※ FWD/LAT 측정 전 반드시 캐노니컬로 되돌린 뒤 한 방향만 이동할 것.
    (대각선/누적 이동 금지 — 축 분리가 깨진다)

전제:
  - box_counter(1).count_in_tray 가 tray_cx/tray_cy(전체프레임 px) 반환하도록 수정됨.
  - 팔은 home 자세 (기준점 측정 때와 동일 시점). 스크립트가 home 으로 보냄.
  - capture_canonical_ref.py 로 기준점이 이미 저장돼 있으면 좋음(필수는 아님).

사용법:
  python3 calibrate_pixel_to_cm.py omx2 [--step 2.0] [--lat-sign +1]
     --step      : 이동 거리 cm (기본 2.0)
     --lat-sign  : 옆 이동 방향. +1=왼쪽(기본), -1=오른쪽.
                   실제로 민 방향과 부호만 맞으면 됨(측정 중 물어봄).

출력:
  canonical_ref.json 의 해당 omx 항목에 'pix2cm' 추가:
     { "step_cm":.., "M":[[..]], "Minv":[[..]],
       "canon_px":[cx,cy], "fwd_px":[..], "lat_px":[..] }
"""

import argparse
import json
import os
import time

import cv2
import numpy as np

from lerobot.robots.utils import make_robot_from_config
from lerobot.robots.omx_follower import OmxFollowerConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

from config import OMX_CONFIGS, HOMING_TIMEOUT, STEP_DELAY, HOME_THRESHOLD

HERE = os.path.dirname(os.path.abspath(__file__))
REF_PATH = os.path.join(HERE, "canonical_ref.json")
VIS_DIR = os.path.join(HERE, "canonical_ref")

N_FRAMES = 40
FRAME_DELAY = 0.05


class PixelToCmCalibrator:
    def __init__(self, omx_id: str, step_cm: float, lat_sign: int):
        if omx_id not in OMX_CONFIGS:
            raise ValueError(f"알 수 없는 omx_id: {omx_id} (가능: {list(OMX_CONFIGS.keys())})")
        self.omx_id = omx_id
        self.cfg = OMX_CONFIGS[omx_id]
        self.step_cm = step_cm
        self.lat_sign = lat_sign
        self.robot = None
        self.tray_camera_key = self.cfg.get("align_camera_key",
                                            self.cfg.get("tray_camera_key", "observation.images.wrist"))

        if omx_id == "omx2":
            self._home_ignore = {"gripper.pos", "shoulder_pan.pos"}
        else:
            self._home_ignore = {"gripper.pos"}

        from box_counter_back import count_in_tray
        self._count_in_tray = count_in_tray

    # ------------------------------------------------------------------
    def connect(self):
        print(f"[{self.omx_id}] 로봇 연결 중... (port={self.cfg['robot_port']})")
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
            print(f"[{self.omx_id}] 로봇 연결 해제")

    def _is_at_home(self, obs, hp):
        for key, target in hp.items():
            if key in self._home_ignore:
                continue
            cur = obs.get(key)
            if cur is None or abs(cur - target) > HOME_THRESHOLD:
                return False
        return True

    def go_home(self) -> bool:
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

    def _centroid(self, frame):
        try:
            counts, _b, roi = self._count_in_tray(frame, omx_id=self.omx_id, return_boxes=True)
        except TypeError:
            counts, roi = self._count_in_tray(frame, omx_id=self.omx_id), None
        return counts.get("tray_cx"), counts.get("tray_cy"), roi, counts

    def measure_point(self, name: str):
        """현재 위치에서 N_FRAMES 중앙값 트레이 중심을 측정. (cx, cy) 반환(실패 시 None)."""
        xs, ys, last = [], [], None
        for _ in range(N_FRAMES):
            frame = self._grab_bgr()
            if frame is None:
                continue
            cx, cy, roi, counts = self._centroid(frame)
            last = (frame, roi)
            if counts.get("tray_found") and cx is not None and cy is not None:
                xs.append(cx)
                ys.append(cy)
            time.sleep(FRAME_DELAY)
        if len(xs) < max(5, N_FRAMES // 4):
            print(f"  ✗ [{name}] 유효 측정 부족({len(xs)}/{N_FRAMES}) — 트레이/조명 확인")
            return None
        cx_med, cy_med = float(np.median(xs)), float(np.median(ys))
        sx = float(np.median(np.abs(np.array(xs) - cx_med)))
        sy = float(np.median(np.abs(np.array(ys) - cy_med)))
        print(f"  ✓ [{name}] 중심=({cx_med:.1f}, {cy_med:.1f})  spread ±{sx:.2f}/{sy:.2f}px  (n={len(xs)})")
        if last is not None:
            self._save_vis(name, last[0], cx_med, cy_med, last[1])
        return np.array([cx_med, cy_med])

    def _save_vis(self, name, frame, cx, cy, roi):
        os.makedirs(VIS_DIR, exist_ok=True)
        vis = frame.copy()
        if roi is not None:
            x, y, w, h = roi
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.drawMarker(vis, (int(round(cx)), int(round(cy))), (0, 0, 255),
                       cv2.MARKER_CROSS, 24, 2)
        cv2.putText(vis, f"{self.omx_id} {name} ({cx:.0f},{cy:.0f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        path = os.path.join(VIS_DIR, f"calib_{self.omx_id}_{name}_{time.strftime('%H%M%S')}.png")
        cv2.imwrite(path, vis)

    # ------------------------------------------------------------------
    def run(self):
        print(f"\n=== {self.omx_id} 픽셀→cm 캘리브레이션 (step={self.step_cm}cm) ===")
        print("팔을 home 으로 보냅니다...")
        if not self.go_home():
            print("⚠ home 실패 — 시점 어긋나면 캘리브레이션 부정확")

        print("\n[1/3] 차량을 '캐노니컬' 위치에 정확히 두세요.")
        input("     준비되면 Enter: ")
        p_canon = self.measure_point("CANON")
        if p_canon is None:
            return

        print(f"\n[2/3] 차량을 캐노니컬에서 '앞으로' 정확히 {self.step_cm}cm 미세요.")
        print("     (앞 = 트레이가 카메라/팔 쪽으로 가까워지는 방향)")
        input("     이동 후 Enter: ")
        p_fwd = self.measure_point("FWD")
        if p_fwd is None:
            return

        lat_dir = "왼쪽" if self.lat_sign > 0 else "오른쪽"
        print(f"\n[3/3] 캐노니컬로 되돌린 뒤, '옆({lat_dir})'으로 정확히 {self.step_cm}cm 미세요.")
        print("     ※ 반드시 캐노니컬 복귀 후 옆으로만! (앞뒤 이동 섞지 말 것)")
        input("     이동 후 Enter: ")
        p_lat = self.measure_point("LAT")
        if p_lat is None:
            return

        # ── 2x2 행렬 구성 ──
        # 열1 = 차를 +step_cm '앞으로' 움직였을 때 픽셀 변화
        # 열2 = 차를 +step_cm '옆으로'(lat_sign 방향) 움직였을 때 픽셀 변화
        # → +lat_sign 을 항상 '+옆(왼쪽 기준)'으로 정규화하기 위해 lat_sign 으로 나눔
        d_fwd = (p_fwd - p_canon) / self.step_cm                    # px per +1cm 앞
        d_lat = (p_lat - p_canon) / (self.step_cm * self.lat_sign)  # px per +1cm 왼쪽

        M = np.column_stack([d_fwd, d_lat])   # cm→px : [px] = M @ [fwd_cm, lat_cm]
        det = float(np.linalg.det(M))
        print(f"\nM (cm→px) =\n{M}")
        print(f"det(M) = {det:.3f}")
        if abs(det) < 1e-3:
            print("✗ det≈0 — 두 이동 방향이 픽셀상 거의 같음(축 분리 실패). "
                  "이동 거리를 키우거나 방향을 다시 확인하세요. 저장 안 함.")
            return

        Minv = np.linalg.inv(M)               # px→cm : [fwd,lat] = Minv @ [dpx_x, dpx_y]
        print(f"Minv (px→cm) =\n{Minv}")

        # 감(感) 확인용: 1cm 이동이 몇 px 인지
        print(f"\n참고: 앞 1cm ≈ {np.linalg.norm(d_fwd):.1f}px, "
              f"옆 1cm ≈ {np.linalg.norm(d_lat):.1f}px 이동")

        self._save(p_canon, p_fwd, p_lat, M, Minv, det)

    def _save(self, p_canon, p_fwd, p_lat, M, Minv, det):
        data = {}
        if os.path.exists(REF_PATH):
            try:
                with open(REF_PATH, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception:
                data = {}
        entry = data.get(self.omx_id, {})
        entry["pix2cm"] = {
            "step_cm": self.step_cm,
            "lat_sign": self.lat_sign,
            "canon_px": [round(float(p_canon[0]), 2), round(float(p_canon[1]), 2)],
            "fwd_px": [round(float(p_fwd[0]), 2), round(float(p_fwd[1]), 2)],
            "lat_px": [round(float(p_lat[0]), 2), round(float(p_lat[1]), 2)],
            "M": [[round(float(v), 4) for v in row] for row in M],
            "Minv": [[round(float(v), 4) for v in row] for row in Minv],
            "det": round(det, 4),
            "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        data[self.omx_id] = entry
        with open(REF_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        print(f"\n[{self.omx_id}] 저장: {REF_PATH} (키: {self.omx_id}.pix2cm)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("omx_id", nargs="?", default="omx2")
    ap.add_argument("--step", type=float, default=2.0, help="이동 거리 cm (기본 2.0)")
    ap.add_argument("--lat-sign", type=int, default=1, choices=[1, -1],
                    help="옆 이동 방향 +1=왼쪽(기본) -1=오른쪽")
    args = ap.parse_args()

    if args.omx_id not in OMX_CONFIGS:
        print(f"사용법: python3 calibrate_pixel_to_cm.py [{'|'.join(OMX_CONFIGS.keys())}] "
              f"[--step 2.0] [--lat-sign +1]")
        return

    cal = PixelToCmCalibrator(args.omx_id, args.step, args.lat_sign)
    cal.connect()
    try:
        cal.run()
    except (EOFError, KeyboardInterrupt):
        print("\n중단됨")
    finally:
        cal.disconnect()


if __name__ == "__main__":
    main()
