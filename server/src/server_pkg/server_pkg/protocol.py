"""
관제서버 <-> OMX PC 간 TCP 메시지 프로토콜

메시지는 JSON 한 줄 + 개행("\n")으로 구분합니다.
예) {"cmd": "execute_policy", "policy_name": "pick1_ep0_400", "request_id": "abc123"}\n

각 함수는 dict <-> bytes 변환만 담당하고, 실제 소켓 송수신은
control_server.py / omx_dummy_server.py 쪽에서 처리합니다.
"""

import json
import uuid

from .config import MESSAGE_DELIMITER


def new_request_id() -> str:
    """요청을 구분하기 위한 짧은 ID 생성"""
    return uuid.uuid4().hex[:8]


def encode_message(message: dict) -> bytes:
    """dict -> 전송용 bytes (JSON + 개행)"""
    return (json.dumps(message, ensure_ascii=False) + "\n").encode("utf-8")


def decode_message(line: bytes) -> dict:
    """수신한 한 줄(bytes) -> dict. 파싱 실패 시 예외 발생시킴 (호출부에서 처리)"""
    return json.loads(line.decode("utf-8").strip())


# ---- 메시지 생성 헬퍼 (관제 -> OMX) ----

def make_execute_policy_request(policy_name: str, request_id: str = None) -> dict:
    """관제 -> OMX: 정책 실행 요청"""
    return {
        "cmd": "execute_policy",
        "policy_name": policy_name,
        "request_id": request_id or new_request_id(),
    }

def make_check_departure_request(wp: str, request_id: str = None) -> dict:
    """관제 -> OMX: 출발 검증(사진 촬영 + 카운트) 요청"""
    return {
        "cmd": "check_departure",
        "wp": wp,
        "request_id": request_id or new_request_id(),
    }

# ---- 메시지 생성 헬퍼 (OMX -> 관제) ----

def make_ack_response(request_id: str, accepted: bool, reason: str = "") -> dict:
    """OMX -> 관제: 요청 수신 즉시 응답 (수락/거부)"""
    return {
        "type": "ack",
        "request_id": request_id,
        "accepted": accepted,
        "reason": reason,
    }


def make_cycle_done(request_id: str, policy_name: str, success: bool, message: str = "") -> dict:
    """OMX -> 관제: 정책 실행 완료(원점 복귀) 알림"""
    return {
        "type": "cycle_done",
        "request_id": request_id,
        "policy_name": policy_name,
        "success": success,
        "message": message,
    }
