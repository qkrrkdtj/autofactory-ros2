"""
OMX PC 측에서 실행하는 TCP 서버.

OMX1, OMX2 각 PC에서 이 스크립트를 실행해두면, 관제서버가 접속해서
정책 실행을 요청하고 완료 알림을 받을 수 있습니다.

사용법:
    python3 omx_dummy_server.py omx1
    python3 omx_dummy_server.py omx2

지금은 실제 OMX 로봇을 움직이지 않고, "정책 실행 중"을 sleep으로 흉내냅니다.
나중에 실제 환경에서는 run_policy() 함수 내부만 lerobot-record 호출로
교체하면 됩니다. (인터페이스/메시지 흐름은 그대로 유지)
"""

import socket
import sys
import threading
import time
import random

from config import OMX_CONFIGS, MESSAGE_DELIMITER
from protocol import decode_message, encode_message, make_ack_response, make_cycle_done


class OmxDummyServer:
    def __init__(self, omx_id: str):
        if omx_id not in OMX_CONFIGS:
            raise ValueError(f"알 수 없는 omx_id: {omx_id} (가능한 값: {list(OMX_CONFIGS.keys())})")

        self.omx_id = omx_id
        self.cfg = OMX_CONFIGS[omx_id]
        self.host = "0.0.0.0"  # 모든 인터페이스에서 수신 (실제 PC에서는 본인 IP로 바인딩됨)
        self.port = self.cfg["port"]
        self.busy = False  # 현재 정책 실행 중인지 여부

    def start(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, self.port))
        server_sock.listen(1)
        print(f"[{self.omx_id}] 서버 시작 - {self.host}:{self.port} 에서 대기 중...")

        while True:
            conn, addr = server_sock.accept()
            print(f"[{self.omx_id}] 관제서버 접속됨: {addr}")
            # 한 번에 하나의 관제 연결만 다루면 충분 (관제서버는 OMX당 1개 연결 유지)
            self._handle_connection(conn)
            print(f"[{self.omx_id}] 연결 종료됨, 재접속 대기...")

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

        if msg.get("cmd") == "execute_policy":
            self._on_execute_policy(conn, msg)
        else:
            print(f"[{self.omx_id}] 알 수 없는 cmd: {msg}")

    def _on_execute_policy(self, conn: socket.socket, msg: dict):
        request_id = msg["request_id"]
        policy_name = msg["policy_name"]

        if self.busy:
            # 이미 실행 중이면 거부 (관제서버는 평소엔 이 상황을 안 만들어야 정상)
            ack = make_ack_response(request_id, accepted=False, reason="이미 정책 실행 중")
            conn.sendall(encode_message(ack))
            print(f"[{self.omx_id}] 요청 거부 (busy): {policy_name}")
            return

        # 1. 즉시 수락 응답
        ack = make_ack_response(request_id, accepted=True)
        conn.sendall(encode_message(ack))
        print(f"[{self.omx_id}] 요청 수락: {policy_name} (request_id={request_id})")

        # 2. 백그라운드 스레드에서 "정책 실행" 흉내내고, 끝나면 cycle_done 전송
        self.busy = True
        thread = threading.Thread(
            target=self._run_policy_and_notify,
            args=(conn, request_id, policy_name),
            daemon=True,
        )
        thread.start()

    def _run_policy_and_notify(self, conn: socket.socket, request_id: str, policy_name: str):
        # ============================================================
        # TODO: 실제 환경에서는 아래 sleep 부분을 lerobot-record 서브프로세스
        # 호출로 교체합니다. 예:
        #
        #   result = subprocess.run(
        #       ["lerobot-record", "--policy", policy_name, ...],
        #       capture_output=True,
        #   )
        #   success = (result.returncode == 0)
        #
        # 지금은 3~6초 사이 랜덤 시간으로 "실행 중"을 흉내냅니다.
        # ============================================================
        duration = random.uniform(3.0, 6.0)
        print(f"[{self.omx_id}] 정책 실행 시작: {policy_name} (예상 {duration:.1f}초)")
        time.sleep(duration)
        success = True  # 더미 단계에서는 항상 성공
        print(f"[{self.omx_id}] 정책 실행 완료(원점복귀): {policy_name}")

        self.busy = False
        done_msg = make_cycle_done(request_id, policy_name, success=success)
        try:
            conn.sendall(encode_message(done_msg))
        except OSError as e:
            print(f"[{self.omx_id}] cycle_done 전송 실패 (연결 끊김): {e}")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in OMX_CONFIGS:
        print(f"사용법: python3 omx_dummy_server.py [{'|'.join(OMX_CONFIGS.keys())}]")
        sys.exit(1)

    omx_id = sys.argv[1]
    server = OmxDummyServer(omx_id)
    server.start()


if __name__ == "__main__":
    main()
