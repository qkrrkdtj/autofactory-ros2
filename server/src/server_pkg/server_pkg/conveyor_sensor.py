"""
conveyor_sensor.py

라즈베리파이(컨베이어 stepper.py)가 UDP로 쏴주는 물건 감지 상태를 수신·보관.

라즈베리파이 쪽은 0.5초마다 {"present": true/false} JSON을 보낸다.
마지막 수신 시각이 CONVEYOR_SENSOR_FRESH_SEC 를 넘으면(파이 꺼짐/네트워크 단절)
item_present() 는 False 를 반환한다 — 신호가 없으면 '물건 없음'으로 취급.
"""

import json
import socket
import threading
import time

from .config import CONVEYOR_SENSOR_PORT, CONVEYOR_SENSOR_FRESH_SEC


class ConveyorSensorLink:
    """컨베이어 IR 센서 상태 UDP 수신기 (라즈베리파이 → 관제서버)"""

    def __init__(self, port: int = CONVEYOR_SENSOR_PORT,
                 fresh_sec: float = CONVEYOR_SENSOR_FRESH_SEC):
        self.port = port
        self.fresh_sec = fresh_sec
        self._present = False
        self._last_rx = 0.0
        self._stop = False
        self._sock = None

    def start(self):
        thread = threading.Thread(target=self._recv_loop, daemon=True)
        thread.start()

    def stop(self):
        self._stop = True
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass

    def _recv_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(1.0)
        self._sock = sock
        print(f"[conveyor] 컨베이어 센서 UDP 수신 대기 (port={self.port})")

        while not self._stop:
            try:
                data, _addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            self._present = bool(msg.get("present"))
            self._last_rx = time.time()

    def item_present(self) -> bool:
        """물건이 컨베이어 감지 위치에 있고, 신호가 살아있으면 True."""
        return self._present and (time.time() - self._last_rx) < self.fresh_sec

    def alive(self) -> bool:
        """라즈베리파이로부터 신호가 최근에 들어오고 있는지."""
        return (time.time() - self._last_rx) < self.fresh_sec
