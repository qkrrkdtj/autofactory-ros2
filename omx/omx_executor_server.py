"""
OMX PC 측에서 실행하는 TCP 서버 (실제 로봇 구동 버전 - lerobot 공식 API 사용)

흐름:
    robot.get_observation()  (raw dict)
    -> predict_action(...)   (정규화 + policy.select_action + 역정규화, 전부 lerobot 제공)
    -> make_robot_action(...) (action tensor -> robot이 쓰는 dict 형태로 변환)
    -> robot.send_action(...)

[추가] 출발 검증(check_departure) 처리:
    관제가 cycle_done 이후 check_departure 를 보내면,
    트레이를 보는 카메라(cam_wrist에 물려있음) 프레임을 잡아 count_in_tray 로
    빨강/파랑 개수를 세고, 자리별 조건(config의 depart_target_count)으로 직접 판정해서
    depart_check 응답을 돌려준다.

    ※ OpenCV 카운터는 omx별로 파일이 분리되어 있다(각 PC 독립 튜닝):
        omx1(적재/A) → box_counter1.count_in_tray  (트레이가 3개 차면 출발)
        omx2(분류/C) → box_counter.count_in_tray   (트레이가 비면 출발)
      __init__ 에서 omx_id 에 따라 알맞은 모듈을 골라 self._count_in_tray 로 보관한다.

    ※ 정책 종류도 omx별로 다르다:
        omx1(적재/A) → ACT 정책      (ACTPolicy)
        omx2(분류/C) → Diffusion 정책 (DiffusionPolicy, DDIM/추론스텝 override 필요)
      __init__ 에서 omx_id 에 따라 정책 클래스/override/use_amp 를 골라 보관한다.

사용법:
    python3 omx_executor_server.py omx1
    python3 omx_executor_server.py omx2

필요 패키지: torch, lerobot, opencv-python(cv2), numpy
필요 파일  : box_counter.py(omx2) / box_counter1.py(omx1) — 각 PC에 해당 파일이 있어야 함
"""

import os
import socket
import sys
import threading
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
    OMX_CONFIGS, MESSAGE_DELIMITER,
    MAX_EPISODE_STEPS, HOMING_TIMEOUT, STEP_DELAY, HOME_THRESHOLD, JOINT_KEYS,
)
from protocol import (
    decode_message, encode_message,
    make_ack_response, make_cycle_done, make_depart_check_response,
)
# count_in_tray 는 omx_id 에 따라 __init__ 에서 box_counter / box_counter1 중 골라 import 한다.


DEBUG_IMAGE_DIR = os.path.join(os.path.dirname(__file__), "debug_departure")


class OmxExecutorServer:
    def __init__(self, omx_id: str):
        if omx_id not in OMX_CONFIGS:
            raise ValueError(f"알 수 없는 omx_id: {omx_id} (가능한 값: {list(OMX_CONFIGS.keys())})")

        self.omx_id = omx_id
        self.cfg = OMX_CONFIGS[omx_id]
        self.host = "0.0.0.0"  # 모든 인터페이스에서 수신 (관제서버가 접속해옴)
        self.port = self.cfg["port"]
        self.busy = False  # 현재 정책 실행 중인지 여부 (TCP 메시지 레벨의 busy)

        self.robot = None
        self.policy = None
        self.preprocessor = None
        self.postprocessor = None
        self.ds_features = None
        self.cam_key_map = None
        self.device = None  # 정책 config 에서 확정된 실행 device (init 에서 채움)

        # ── 출발 검증용 설정 ──
        # tray_camera_key: 트레이를 보는 카메라의 observation 키
        # depart_target_count: 이 개수와 같으면 출발 (A: 3, C: 0)
        self.tray_camera_key = self.cfg.get("tray_camera_key", "observation.images.wrist")
        self.depart_target_count = self.cfg.get("depart_target_count", 0)

        # ── home 도달 판정에서 제외할 관절 (omx별로 다름) ──
        # config에 없으면 gripper만 제외(기존 동작과 동일).
        self.home_ignore_keys = set(
            self.cfg.get("home_ignore_keys", ["gripper.pos"])
        )

        # ── 출발 검증용 OpenCV 카운터 선택 (omx별 독립 파일/튜닝) ──
        # omx1(적재/A): box_counter1.count_in_tray  (트레이 3개 차면 출발)
        # omx2(분류/C): box_counter.count_in_tray   (트레이 비면 출발)
        # 각 PC 에 자기 파일만 있어도 되도록 조건부 import 사용.
        if self.omx_id == "omx1":
            from box_counter1 import count_in_tray
        else:
            from box_counter import count_in_tray
        self._count_in_tray = count_in_tray

        # ── 정책 종류 분기 (omx1=ACT, omx2=Diffusion) ──
        # omx1(적재/A): ACT 정책(pick1_ep0_400). diffusion 전용 인자는 ACTConfig 에 없으므로
        #   override 에 넣으면 draccus 파싱이 터진다 → device 만 cuda 로 지정.
        # omx2(분류/C): Diffusion 정책. noise_scheduler_type / num_inference_steps 는 모델
        #   __init__ 에서 스케줄러를 만들 때 읽히므로 반드시 from_pretrained 전에 주입해야 한다.
        if self.omx_id == "omx1" or self.omx_id == "omx2":
            self._policy_class = ACTPolicy
            self._policy_overrides = [
                "--device=cuda",
            ]
            self._use_amp = False
        # else:
        #     self._policy_class = DiffusionPolicy
        #     self._policy_overrides = [
        #         "--device=cuda",
        #         "--use_amp=true",
        #         "--noise_scheduler_type=DDIM",
        #         "--num_inference_steps=10",
        #         "--n_action_steps=15",
        #     ]
        #     self._use_amp = True

    # ------------------------------------------------------------------
    # 초기화: 로봇 연결 + 정책/전처리기/후처리기 로드 (서버 시작 시 한 번만)
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
            OmxFollowerConfig(
                port=self.cfg["robot_port"],
                cameras=camera_configs,
            )
        )
        self.robot.connect()
        print(f"[{self.omx_id}] 로봇 연결 완료")

        policy_path = self.cfg["policy_path"]
        print(f"[{self.omx_id}] Policy 로드 중... ({policy_path}, class={self._policy_class.__name__})")
        # 추론용 config override = lerobot CLI 의 --policy.* 인자와 동일 효과.
        # diffusion(omx2) 의 noise_scheduler_type / num_inference_steps 는 모델 __init__ 에서
        # 스케줄러를 만들 때 읽히므로 반드시 from_pretrained 전에 config 로 주입해야 한다
        # (객체 생성 후 policy.config 만 바꾸면 이미 만들어진 스케줄러엔 반영 안 됨).
        # omx1(ACT) 는 device 만 지정하고 나머지는 저장된 config 그대로 로드된다.
        policy_cfg = PreTrainedConfig.from_pretrained(
            policy_path,
            cli_overrides=self._policy_overrides,
        )
        self.policy = self._policy_class.from_pretrained(policy_path, config=policy_cfg)
        self.policy.eval()

        # device 는 정책 config 에서 최종 확정된 값을 따른다.
        self.device = torch.device(policy_cfg.device)
        print(f"[{self.omx_id}] Policy 로드 완료 (device: {self.device})")

        print(f"[{self.omx_id}] 전처리/후처리 파이프라인 로드 중...")
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=policy_path,
            preprocessor_overrides={"device_processor": {"device": policy_cfg.device}},
        )
        print(f"[{self.omx_id}] 전처리/후처리 파이프라인 로드 완료")

        dataset_repo_id = self.cfg["dataset_repo_id"]
        print(f"[{self.omx_id}] 데이터셋 메타데이터 로드 중... ({dataset_repo_id})")
        ds_meta = LeRobotDatasetMetadata(repo_id=dataset_repo_id)
        self.ds_features = ds_meta.features
        print(f"[{self.omx_id}] action 이름 확인: {self.ds_features.get('action', {}).get('names')}")

        self.cam_key_map = {}
        for cam_key in self.cfg["cameras"]:
            short_key = cam_key.removeprefix("observation.images.")
            self.cam_key_map[short_key] = cam_key

    def shutdown_robot(self):
        if self.robot is not None:
            self.robot.disconnect()
            print(f"[{self.omx_id}] 로봇 연결 해제")

    # ------------------------------------------------------------------
    # TCP 서버
    # ------------------------------------------------------------------

    def start(self):
        self.init_robot_and_policy()

        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, self.port))
        server_sock.listen(1)
        print(f"[{self.omx_id}] 서버 시작 - {self.host}:{self.port} 에서 대기 중...")

        try:
            while True:
                conn, addr = server_sock.accept()
                print(f"[{self.omx_id}] 관제서버 접속됨: {addr}")
                self._handle_connection(conn)
                print(f"[{self.omx_id}] 연결 종료됨, 재접속 대기...")
        finally:
            self.shutdown_robot()

    def _handle_connection(self, conn: socket.socket):
        buffer = b""
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break  # 관제서버가 연결을 닫음
                buffer += chunk

                while MESSAGE_DELIMITER in buffer:
                    line, buffer = buffer.split(MESSAGE_DELIMITER, 1)
                    if not line.strip():
                        continue
                    self._handle_message(conn, line)
        except (ConnectionResetError, BrokenPipeError) as e:
            print(f"[{self.omx_id}] 연결 오류: {e}")
        finally:
            conn.close()

    def _handle_message(self, conn: socket.socket, line: bytes):
        try:
            msg = decode_message(line)
        except Exception as e:
            print(f"[{self.omx_id}] 메시지 파싱 실패: {e} / raw={line}")
            return

        cmd = msg.get("cmd")
        if cmd == "execute_policy":
            self._on_execute_policy(conn, msg)
        elif cmd == "check_departure":
            self._on_check_departure(conn, msg)
        else:
            print(f"[{self.omx_id}] 알 수 없는 cmd: {msg}")

    def _on_execute_policy(self, conn: socket.socket, msg: dict):
        request_id = msg["request_id"]
        policy_name = msg["policy_name"]

        if self.busy:
            ack = make_ack_response(request_id, accepted=False, reason="이미 정책 실행 중")
            conn.sendall(encode_message(ack))
            print(f"[{self.omx_id}] 요청 거부 (busy): {policy_name}")
            return

        ack = make_ack_response(request_id, accepted=True)
        conn.sendall(encode_message(ack))
        print(f"[{self.omx_id}] 요청 수락: {policy_name} (request_id={request_id})")

        self.busy = True
        thread = threading.Thread(
            target=self._run_policy_and_notify,
            args=(conn, request_id, policy_name),
            daemon=True,
        )
        thread.start()

    # ------------------------------------------------------------------
    # [추가] 출발 검증 (OpenCV)
    # ------------------------------------------------------------------

    def _on_check_departure(self, conn: socket.socket, msg: dict):
        """트레이 프레임을 잡아 개수를 세고, 자리별 조건으로 출발 여부 판정 후 회신.

        cycle_done 직후(팔 원점)에만 호출되므로 빠르게 동기 처리한다.
        """
        request_id = msg["request_id"]
        wp = msg.get("wp")

        frame = self._grab_tray_frame_bgr()
        if frame is None:
            resp = make_depart_check_response(
                request_id, wp, depart_ok=False, counts={}, tray_found=False
            )
            conn.sendall(encode_message(resp))
            print(f"[{self.omx_id}] 출발검증: 트레이 프레임 획득 실패 (camera_key={self.tray_camera_key})")
            return

        counts, boxes, roi = self._count_tray_with_debug(frame)
        tray_found = bool(counts.get("tray_found"))
        total = int(counts.get("total", 0))

        # 자리별 조건 판정: 트레이를 찾았고(total이 신뢰 가능) + 목표 개수와 일치
        depart_ok = bool(tray_found and total == self.depart_target_count)
        debug_path = self._save_departure_debug_image(frame, wp, counts, boxes, roi, depart_ok)

        resp = make_depart_check_response(
            request_id, wp, depart_ok=depart_ok, counts=counts, tray_found=tray_found
        )
        conn.sendall(encode_message(resp))
        print(f"[{self.omx_id}] 출발검증 wp={wp} counts={counts} "
              f"target={self.depart_target_count} -> depart_ok={depart_ok} debug={debug_path}")

    def _count_tray_with_debug(self, frame):
        """카운트와 함께 ROI/박스 정보를 얻는다. 구버전 카운터도 방어적으로 지원한다."""
        try:
            return self._count_in_tray(frame, return_boxes=True)
        except TypeError:
            counts = self._count_in_tray(frame)
            return counts, [], None

    def _save_departure_debug_image(self, frame, wp, counts, boxes, roi, depart_ok):
        """출발검증 결과를 눈으로 확인할 수 있게 ROI/검출 박스를 이미지로 저장한다."""
        try:
            os.makedirs(DEBUG_IMAGE_DIR, exist_ok=True)
            vis = frame.copy()
            if roi is not None:
                x, y, w, h = roi
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 255), 3)

            for color, (bx, by, bw, bh) in boxes:
                bgr = (255, 0, 0) if color == "blue" else (0, 0, 255)
                cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), bgr, 3)
                cv2.putText(vis, color, (bx, max(20, by - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)

            status = "OK" if depart_ok else "WAIT"
            label = (f"{self.omx_id} wp={wp} {status} "
                     f"tray={counts.get('tray_found')} R={counts.get('red', 0)} "
                     f"B={counts.get('blue', 0)} total={counts.get('total', 0)} "
                     f"target={self.depart_target_count}")
            cv2.putText(vis, label, (10, 32), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 255, 0) if depart_ok else (0, 180, 255), 2)

            ts = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{ts}_{self.omx_id}_wp{wp}_{status}_total{counts.get('total', 0)}.png"
            path = os.path.join(DEBUG_IMAGE_DIR, filename)
            cv2.imwrite(path, vis)
            return path
        except Exception as e:
            print(f"[{self.omx_id}] 출발검증 디버그 이미지 저장 실패: {e}")
            return ""

    def _grab_tray_frame_bgr(self):
        """트레이 보는 카메라 프레임을 OpenCV용 BGR uint8 numpy로 변환해서 반환.

        lerobot observation의 이미지는 보통 RGB(HWC, uint8) numpy 이지만,
        구현/버전에 따라 torch 텐서·CHW·float[0,1]일 수 있어 방어적으로 변환한다.
        키를 못 찾거나 변환 실패하면 None.
        """
        try:
            raw_obs = self.robot.get_observation()
            img = raw_obs.get(self.tray_camera_key)
            if img is None:
                print(f"[{self.omx_id}] observation에 '{self.tray_camera_key}' 키 없음 "
                      f"(가능 키: {list(raw_obs.keys())})")
                return None

            # torch 텐서 -> numpy
            if hasattr(img, "detach"):
                img = img.detach().cpu().numpy()
            img = np.asarray(img)

            # CHW -> HWC (채널이 맨 앞이고 마지막축이 3이 아닌 경우)
            if img.ndim == 3 and img.shape[0] in (1, 3) and img.shape[2] not in (1, 3):
                img = np.transpose(img, (1, 2, 0))

            # float[0,1] -> uint8
            if img.dtype != np.uint8:
                if img.max() <= 1.0:
                    img = (img * 255.0).clip(0, 255).astype(np.uint8)
                else:
                    img = img.clip(0, 255).astype(np.uint8)

            # RGB -> BGR (lerobot 카메라는 RGB로 반환, cv2는 BGR 기대)
            if img.ndim == 3 and img.shape[2] == 3:
                img = img[:, :, ::-1].copy()

            return img
        except Exception as e:
            print(f"[{self.omx_id}] 트레이 프레임 변환 실패: {e}")
            return None

    # ------------------------------------------------------------------
    # 실제 로봇 실행 로직
    # ------------------------------------------------------------------

    def _run_policy_and_notify(self, conn: socket.socket, request_id: str, policy_name: str):
        success = False
        message = ""
        try:
            steps_used = self._run_policy_steps()
            homed_ok = self._go_home_safely()

            if not homed_ok:
                success = False
                message = f"home 복귀 실패 (timeout {HOMING_TIMEOUT}초)"
                print(f"[{self.omx_id}] {message}")
            else:
                success = True
                if steps_used >= MAX_EPISODE_STEPS:
                    message = f"MAX_EPISODE_STEPS({MAX_EPISODE_STEPS}) 도달 후 강제 종료"
                    print(f"[{self.omx_id}] 경고: {message}")
                else:
                    message = f"{steps_used} 스텝 후 완료"
                print(f"[{self.omx_id}] 정책 실행 완료(원점복귀): {policy_name} ({message})")

        except Exception as e:
            success = False
            message = f"실행 중 예외 발생: {e}"
            print(f"[{self.omx_id}] {message}")

        finally:
            self.busy = False
            done_msg = make_cycle_done(request_id, policy_name, success=success, message=message)
            try:
                conn.sendall(encode_message(done_msg))
            except OSError as e:
                print(f"[{self.omx_id}] cycle_done 전송 실패 (연결 끊김): {e}")

    def _run_policy_steps(self) -> int:
        print(f"[{self.omx_id}] Policy 실행 시작")
        self.policy.reset()

        home_position = self.cfg["home_position"]
        has_left_home = False
        home_streak = 0
        HOME_RETURN_STREAK = 10

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
                        print(f"[{self.omx_id}] 정책 동작 후 home 복귀 감지 — {step+1} 스텝에서 종료")
                        break
            else:
                has_left_home = True
                home_streak = 0

            time.sleep(STEP_DELAY)

        return step + 1

    def _go_home_safely(self) -> bool:
        """Home 복귀 시도. HOMING_TIMEOUT 내 도달하면 True, 못하면 False."""
        print(f"[{self.omx_id}] Home position 복귀 중...")
        home_position = self.cfg["home_position"]
        start_time = time.time()

        while time.time() - start_time < HOMING_TIMEOUT:
            obs = self.robot.get_observation()
            if self._is_at_home(obs, home_position):
                print(f"[{self.omx_id}] Home 도달")
                return True
            self.robot.send_action(home_position)
            time.sleep(STEP_DELAY)

        print(f"[{self.omx_id}] Home 복귀 타임아웃 ({HOMING_TIMEOUT}초)")
        return False

    # HOME_IGNORE_KEYS = {"gripper.pos"}

    # @classmethod
    def _is_at_home(cls, obs: dict, home_position: dict) -> bool:
        for key, target in home_position.items():
            if key in cls.HOME_IGNORE_KEYS:
                continue
            current = obs.get(key)
            if current is None:
                return False
            if abs(current - target) > HOME_THRESHOLD:
                return False
        return True

    def _build_observation_frame(self, raw_obs: dict = None) -> dict:
        if raw_obs is None:
            raw_obs = self.robot.get_observation()

        renamed_obs = {}
        for key, value in raw_obs.items():
            if key in self.cam_key_map.values():
                short_key = key.removeprefix("observation.images.")
                renamed_obs[short_key] = value
            else:
                renamed_obs[key] = value

        return build_dataset_frame(self.ds_features, renamed_obs, prefix="observation")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in OMX_CONFIGS:
        print(f"사용법: python3 omx_executor_server.py [{'|'.join(OMX_CONFIGS.keys())}]")
        sys.exit(1)

    omx_id = sys.argv[1]
    server = OmxExecutorServer(omx_id)
    server.start()


if __name__ == "__main__":
    main()
