"""
omx_link.py

기존 control_server.py 에서 OmxConnection 만 떼어내 재사용하는 모듈.
(CLI / main 은 제거 — 이제 modi_bridge 가 자동으로 OMX 를 트리거함)

control_server.py 대비 핵심 변경점:
  - import 경로를 패키지 경로(.config / .protocol)로 변경
  - OmxConnection 에 on_cycle_done 콜백 훅 추가
    → 정책 실행이 완료(cycle_done)되면, 등록된 콜백을 호출해서
      외부(미션 매니저)에 "이 자리 작업 끝났다"고 알려줄 수 있음.
"""

import socket
import threading
import time

from .config import OMX_CONFIGS, CONNECT_TIMEOUT, RECONNECT_INTERVAL, MESSAGE_DELIMITER
from .protocol import decode_message, encode_message, make_execute_policy_request


class OmxConnection:
    """OMX1 또는 OMX2 한 대에 대한 TCP 연결 + 상태 관리"""

    def __init__(self, omx_id: str, on_cycle_done=None):
        self.omx_id = omx_id
        self.cfg = OMX_CONFIGS[omx_id]
        self.host = self.cfg["host"]
        self.port = self.cfg["port"]

        self.sock: socket.socket | None = None
        self.connected = False
        self.busy = False
        self.completed_count = 0
        self.pending_request_id: str | None = None

        # ── [추가] 정책 완료 시 호출할 콜백 ──
        # signature: on_cycle_done(omx_id: str, success: bool)
        # OMX 수신 스레드에서 호출되므로, 콜백 안에서 asyncio 루프로 넘길 때는
        # call_soon_threadsafe 를 써야 한다 (미션 매니저가 그렇게 처리함).
        self.on_cycle_done = on_cycle_done

        self._lock = threading.Lock()
        self._stop = False

    def start(self):
        """연결 유지 스레드 시작 (끊기면 재연결 반복)"""
        thread = threading.Thread(target=self._connection_loop, daemon=True)
        thread.start()

    def stop(self):
        self._stop = True
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass

    def _connection_loop(self):
        while not self._stop:
            if not self.connected:
                self._try_connect()
            time.sleep(RECONNECT_INTERVAL)

    def _try_connect(self):
        try:
            sock = socket.create_connection((self.host, self.port), timeout=CONNECT_TIMEOUT)
            sock.settimeout(None)
            self.sock = sock
            self.connected = True
            print(f"[{self.omx_id}] 연결 성공: {self.host}:{self.port}")

            recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            recv_thread.start()
        except OSError as e:
            print(f"[{self.omx_id}] 연결 실패 ({self.host}:{self.port}): {e} - {RECONNECT_INTERVAL}초 후 재시도")
            self.connected = False

    def _recv_loop(self):
        buffer = b""
        try:
            while not self._stop:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                buffer += chunk

                while MESSAGE_DELIMITER in buffer:
                    line, buffer = buffer.split(MESSAGE_DELIMITER, 1)
                    if not line.strip():
                        continue
                    self._handle_message(line)
        except OSError as e:
            print(f"[{self.omx_id}] 수신 중 오류: {e}")
        finally:
            self.connected = False
            self.sock = None
            print(f"[{self.omx_id}] 연결 끊김 - 재연결 대기")

    def _handle_message(self, line: bytes):
        try:
            msg = decode_message(line)
        except Exception as e:
            print(f"[{self.omx_id}] 메시지 파싱 실패: {e} / raw={line}")
            return

        msg_type = msg.get("type")
        if msg_type == "ack":
            self._on_ack(msg)
        elif msg_type == "cycle_done":
            self._on_cycle_done(msg)
        else:
            print(f"[{self.omx_id}] 알 수 없는 메시지 타입: {msg}")

    def _on_ack(self, msg: dict):
        if msg["accepted"]:
            print(f"[{self.omx_id}] 요청 수락됨 (request_id={msg['request_id']})")
        else:
            print(f"[{self.omx_id}] 요청 거부됨: {msg.get('reason')} (request_id={msg['request_id']})")
            with self._lock:
                self.busy = False
                self.pending_request_id = None
            # 거부도 "더 이상 기다릴 필요 없음"이므로 콜백으로 알림 (success=False)
            if self.on_cycle_done is not None:
                self.on_cycle_done(self.omx_id, False)

    def _on_cycle_done(self, msg: dict):
        print(f"[{self.omx_id}] 정책 완료(원점복귀): {msg['policy_name']} "
              f"success={msg['success']} (request_id={msg['request_id']})")
        with self._lock:
            self.busy = False
            self.pending_request_id = None
            if msg["success"]:
                self.completed_count += 1
        print(f"[{self.omx_id}] 누적 완료 횟수: {self.completed_count}")

        # ── [추가] 완료를 외부(미션 매니저)에 알림 ──
        if self.on_cycle_done is not None:
            self.on_cycle_done(self.omx_id, bool(msg["success"]))

    def run_policy(self) -> bool:
        """
        정책 실행 요청을 보냄.
        반환값 True면 "요청 전송됨", False면 "전송 불가(미연결/busy)".
        실제 완료는 비동기로 on_cycle_done 콜백으로 전달됨.
        """
        with self._lock:
            if not self.connected or self.sock is None:
                print(f"[{self.omx_id}] 연결되어 있지 않아 요청을 보낼 수 없습니다.")
                return False
            if self.busy:
                print(f"[{self.omx_id}] 이미 정책 실행 중입니다 (request_id={self.pending_request_id}).")
                return False

            req = make_execute_policy_request(self.cfg["policy_name"])
            try:
                self.sock.sendall(encode_message(req))
            except OSError as e:
                print(f"[{self.omx_id}] 요청 전송 실패: {e}")
                return False

            self.busy = True
            self.pending_request_id = req["request_id"]
            print(f"[{self.omx_id}] 정책 실행 요청 전송: {req['policy_name']} "
                  f"(request_id={req['request_id']})")
            return True

    def status_str(self) -> str:
        state = "BUSY" if self.busy else "IDLE"
        conn_state = "CONNECTED" if self.connected else "DISCONNECTED"
        return (f"{self.omx_id}: {conn_state}, {state}, "
                f"완료횟수={self.completed_count}, host={self.host}:{self.port}")
