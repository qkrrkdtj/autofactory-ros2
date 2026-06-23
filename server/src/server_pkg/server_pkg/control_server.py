"""
관제서버 메인 스크립트 (지금 단계: OMX1/OMX2 메시지 기반 동작만)

- OMX1, OMX2 PC에 각각 TCP 연결을 상시로 유지합니다 (끊기면 자동 재연결).
- 연결마다 수신 스레드를 두어, ack / cycle_done 메시지를 비동기로 처리합니다.
- CLI로 명령을 입력해서 수동으로 정책 실행을 트리거합니다.

CLI 명령:
    omx1 run        -> OMX1에 정책(pick1_ep0_400) 실행 요청
    omx2 run        -> OMX2에 정책(classify_ep0_316) 실행 요청
    status           -> 현재 OMX1/OMX2 연결 상태 및 busy 여부 출력
    quit / exit       -> 종료
"""

import socket
import threading
import time

from .config import OMX_CONFIGS, CONNECT_TIMEOUT, RECONNECT_INTERVAL, MESSAGE_DELIMITER
from .protocol import decode_message, encode_message, make_execute_policy_request


class OmxConnection:
    """OMX1 또는 OMX2 한 대에 대한 TCP 연결 + 상태 관리"""

    def __init__(self, omx_id: str):
        self.omx_id = omx_id
        self.cfg = OMX_CONFIGS[omx_id]
        self.host = self.cfg["host"]
        self.port = self.cfg["port"]

        self.sock: socket.socket | None = None
        self.connected = False
        self.busy = False  # 마지막으로 알고 있는 OMX의 busy 상태 (요청 보낸 후 ~ 완료 전까지 True)
        self.completed_count = 0  # 완료된 정책 실행 횟수 (적재 카운트 등에 활용 가능)
        self.pending_request_id: str | None = None

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
            sock.settimeout(None)  # 연결 후에는 타임아웃 해제 (정책 실행이 길어도 recv가 끊기지 않도록)
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
                    break  # OMX 서버가 연결을 닫음
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

    def _on_cycle_done(self, msg: dict):
        print(f"[{self.omx_id}] 정책 완료(원점복귀): {msg['policy_name']} "
              f"success={msg['success']} (request_id={msg['request_id']})")
        with self._lock:
            self.busy = False
            self.pending_request_id = None
            if msg["success"]:
                self.completed_count += 1
        print(f"[{self.omx_id}] 누적 완료 횟수: {self.completed_count}")

    def run_policy(self) -> bool:
        """
        정책 실행 요청을 보냄.
        반환값 True면 "요청 전송됨", False면 "전송 불가(미연결/busy)".
        실제 성공 여부는 비동기로 _on_cycle_done에서 처리됨.
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


class ControlServer:
    def __init__(self):
        self.connections: dict[str, OmxConnection] = {
            omx_id: OmxConnection(omx_id) for omx_id in OMX_CONFIGS
        }

    def start(self):
        for conn in self.connections.values():
            conn.start()

    def run_cli(self):
        print("\n=== 관제서버 CLI ===")
        print("명령: omx1 run | omx2 run | status | quit\n")
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n종료합니다.")
                break

            if not line:
                continue

            if line in ("quit", "exit"):
                print("종료합니다.")
                break
            elif line == "status":
                self._print_status()
            elif line in ("omx1 run", "omx2 run"):
                omx_id = line.split()[0]
                self.connections[omx_id].run_policy()
            else:
                print(f"알 수 없는 명령: {line}")
                print("사용 가능: omx1 run | omx2 run | status | quit")

        self._shutdown()

    def _print_status(self):
        for conn in self.connections.values():
            print(conn.status_str())

    def _shutdown(self):
        for conn in self.connections.values():
            conn.stop()


def main():
    server = ControlServer()
    server.start()
    time.sleep(1.0)  # 연결 스레드들이 최초 연결 시도할 시간을 잠깐 줌
    server.run_cli()


if __name__ == "__main__":
    main()
