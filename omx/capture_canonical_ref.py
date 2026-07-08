#!/usr/bin/env python3
"""
capture_canonical_ref.py — 캐노니컬 기준점(트레이 무게중심) 측정/저장

정렬 보정의 '기준점'을 만든다. 차량/트레이를 가장 잘 되던(캐노니컬) 위치에
정확히 두고, 팔을 home 자세로 보낸 뒤 wrist 프레임 여러 장에서 트레이 초록
무게중심 픽셀 좌표를 재서 '중앙값'을 canonical_ref.json 에 저장한다.

런타임 정렬 페이즈는 이 기준점과 현재 무게중심의 픽셀 차이를 (fwd,lat) cm로
바꿔 차량을 보정한다. 따라서 이 기준점이 흔들리면 그 뒤가 전부 흔들린다.
그래서 한 장이 아니라 여러 장 중앙값을 쓰고, 분산(spread)을 함께 출력해
기준점의 신뢰도를 눈으로 확인한다.

전제:
  - 차량/트레이가 캐노니컬 위치에 정확히 놓여 있어야 한다(바닥 테이프 표시 권장).
  - box_counter(1).count_in_tray 가 counts 에 tray_cx/tray_cy(전체프레임 px)를
    반환하도록 수정돼 있어야 한다.
  - omx_executor_server.py 와 같은 폴더에서 실행(config/box_counter import).

사용법:
  python3 capture_canonical_ref.py omx2     # 분류(C) — 먼저 이거
  python3 capture_canonical_ref.py omx1     # 적재(A)

출력:
  canonical_ref.json                              기준점 (omx_id 별로 갱신)
  canonical_ref/<omx>_<ts>.png                    ROI+무게중심 표시 확인 이미지
"""

import json
import os
import sys
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

N_FRAMES = 40          # 측정에 쓸 프레임 수
FRAME_DELAY = 0.05     # 프레임 간 간격(초)
SPREAD_WARN_PX = 4.0   # 중앙값 대비 이 px 이상 흔들리면 경고(기준점 불안정)


class CanonicalRefCapturer:
    def __init__(self, omx_id: str):
        if omx_id not in OMX_CONFIGS:
            raise ValueError(f"알 수 없는 omx_id: {omx_id} (가능: {list(OMX_CONFIGS.keys())})")
        self.omx_id = omx_id
        self.cfg = OMX_CONFIGS[omx_id]
        self.robot = None
        self.tray_camera_key = self.cfg.get("align_camera_key",
                                            self.cfg.get("tray_camera_key", "observation.images.wrist"))

        # home 판정 제외 관절 (eval_snapshot 과 동일 규칙)
        if omx_id == "omx2":
            self._home_ignore = {"gripper.pos", "shoulder_pan.pos"}
        else:
            self._home_ignore = {"gripper.pos"}

        # 정렬은 omx1/omx2 모두 후방 캠(box_counter_back). ROI 는 내부에서 omx_id 분기.
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

    def _is_at_home(self, obs, home_position):
        for key, target in home_position.items():
            if key in self._home_ignore:
                continue
            cur = obs.get(key)
            if cur is None or abs(cur - target) > HOME_THRESHOLD:
                return False
        return True

    def go_home(self) -> bool:
        """기준점은 반드시 home 시점에서 재야 한다(box_counter ROI 가 home 시점 기준)."""
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
            img = img[:, :, ::-1].copy()   # RGB → BGR
        return img

    def _centroid_of(self, frame):
        try:
            counts, _boxes, roi = self._count_in_tray(frame, omx_id=self.omx_id, return_boxes=True)
        except TypeError:
            counts, roi = self._count_in_tray(frame, omx_id=self.omx_id), None
        cx, cy = counts.get("tray_cx"), counts.get("tray_cy")
        angle = counts.get("tray_angle")
        return cx, cy, angle, roi, counts

    # ------------------------------------------------------------------
    def capture(self):
        print("\n캐노니컬 위치에 차량/트레이가 정확히 놓였는지 확인하세요.")
        input(f"[{self.omx_id}] 준비되면 Enter (팔을 home 으로 보낸 뒤 측정): ")

        print(f"[{self.omx_id}] home 이동 중...")
        if not self.go_home():
            print(f"[{self.omx_id}] ⚠ home 복귀 실패 — 시점이 어긋나면 기준점이 부정확할 수 있음")

        print(f"[{self.omx_id}] {N_FRAMES} 프레임 측정 중...")
        xs, ys, angs, last_frame, last_roi = [], [], [], None, None
        tray_missing = 0
        for _ in range(N_FRAMES):
            frame = self._grab_bgr()
            if frame is None:
                continue
            cx, cy, angle, roi, counts = self._centroid_of(frame)
            last_frame, last_roi = frame, roi
            if not counts.get("tray_found") or cx is None or cy is None:
                tray_missing += 1
                continue
            xs.append(cx)
            ys.append(cy)
            if angle is not None:
                angs.append(angle)
            time.sleep(FRAME_DELAY)

        n = len(xs)
        if n < max(5, N_FRAMES // 4):
            print(f"[{self.omx_id}] ✗ 유효 측정 부족 ({n}/{N_FRAMES}, tray_found 실패 {tray_missing}). "
                  f"트레이 위치/조명/ROI 확인 필요. 저장 안 함.")
            return

        xs, ys = np.array(xs), np.array(ys)
        cx_med, cy_med = float(np.median(xs)), float(np.median(ys))
        # 분산은 중앙값편차(MAD 근사)로 — 이상치에 덜 민감
        spread_x = float(np.median(np.abs(xs - cx_med)))
        spread_y = float(np.median(np.abs(ys - cy_med)))
        ang_med = float(np.median(angs)) if angs else None
        print(f"[{self.omx_id}] 기준점: cx={cx_med:.1f} cy={cy_med:.1f} "
              f"ang={ang_med:.2f}도 " if ang_med is not None else f"ang=None "
              f"(유효 {n}/{N_FRAMES}, spread ±{spread_x:.1f}/{spread_y:.1f}px)")

        print(f"[{self.omx_id}] 기준점: cx={cx_med:.1f} cy={cy_med:.1f} "
              f"(유효 {n}/{N_FRAMES}, spread ±{spread_x:.1f}/{spread_y:.1f}px)")
        if spread_x > SPREAD_WARN_PX or spread_y > SPREAD_WARN_PX:
            print(f"[{self.omx_id}] ⚠ 기준점 흔들림 큼(>{SPREAD_WARN_PX}px). 조명/트레이 고정 확인 후 재측정 권장.")

        self._save_json(cx_med, cy_med, ang_med, n, spread_x, spread_y, last_roi)
        self._save_vis(last_frame, cx_med, cy_med, last_roi)

    def _save_json(self, cx, cy, ang_med, n, sx, sy, roi):
        data = {}
        if os.path.exists(REF_PATH):
            try:
                with open(REF_PATH, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception:
                data = {}
        entry = data.get(self.omx_id, {})   # 기존 pix2cm 등 보존
        entry.update({
            "cx": round(cx, 2), "cy": round(cy, 2),
            "angle": round(ang_med, 3) if ang_med is not None else None,
            "roi": list(roi) if roi is not None else None,
            "n_used": n, "spread_px": [round(sx, 2), round(sy, 2)],
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        })
        data[self.omx_id] = entry
        
        with open(REF_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        print(f"[{self.omx_id}] 저장: {REF_PATH}")

    def _save_vis(self, frame, cx, cy, roi):
        os.makedirs(VIS_DIR, exist_ok=True)
        vis = frame.copy()
        if roi is not None:
            x, y, w, h = roi
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 255), 2)
        ix, iy = int(round(cx)), int(round(cy))
        cv2.drawMarker(vis, (ix, iy), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(vis, (ix, iy), 6, (0, 0, 255), 2)
        cv2.putText(vis, f"{self.omx_id} ref ({cx:.0f},{cy:.0f})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        path = os.path.join(VIS_DIR, f"{self.omx_id}_{time.strftime('%Y%m%d_%H%M%S')}.png")
        cv2.imwrite(path, vis)
        print(f"[{self.omx_id}] 확인 이미지: {path}")


def main():
    omx_id = sys.argv[1] if len(sys.argv) >= 2 else "omx2"
    if omx_id not in OMX_CONFIGS:
        print(f"사용법: python3 capture_canonical_ref.py [{'|'.join(OMX_CONFIGS.keys())}]")
        sys.exit(1)

    cap = CanonicalRefCapturer(omx_id)
    cap.connect()
    try:
        cap.capture()
    except (EOFError, KeyboardInterrupt):
        print("\n중단됨")
    finally:
        cap.disconnect()


if __name__ == "__main__":
    main()
