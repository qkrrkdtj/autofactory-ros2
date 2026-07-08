"""
관제서버 <-> OMX PC 간 TCP 메시지 프로토콜

메시지는 JSON 한 줄 + 개행("\n")으로 구분합니다.
예) {"cmd": "execute_policy", "policy_name": "pick1_ep0_400", "request_id": "abc123"}\n

각 함수는 dict <-> bytes 변환만 담당하고, 실제 소켓 송수신은
control_server.py / omx_executor_server.py 쪽에서 처리합니다.
"""

import json
import uuid

# server_pkg 안에서는 패키지 상대 임포트(.config), OMX PC에서 단독 스크립트로
# 실행할 때는 절대 임포트(config)로 폴백한다. (두 환경에서 같은 파일 공용)
try:
    from .config import MESSAGE_DELIMITER
except ImportError:
    from config import MESSAGE_DELIMITER


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
    """관제 -> OMX: 출발 검증(OpenCV) 요청.

    OMX PC가 트레이를 찍어서 자리별 조건(A: ==3, C: ==0)을 직접 판정하고
    depart_check 응답으로 돌려준다.
    """
    return {
        "cmd": "check_departure",
        "wp": wp,
        "request_id": request_id or new_request_id(),
    }

def make_check_alignment_request(wp: str, request_id: str = None) -> dict:
    """관제/브릿지 -> OMX: 정렬 오프셋 측정 요청.

    OMX 가 wrist 로 현재 트레이 중심을 재서 canonical_ref(기준점+Minv)로
    '차량 보정량'(fwd,lat cm)을 계산해 alignment 응답으로 돌려준다.
    """
    return {
        "cmd": "check_alignment",
        "wp": wp,
        "request_id": request_id or new_request_id(),
    }

def make_alignment_response(request_id, wp, aligned, fwd_cm, lat_cm,
                            tray_found=False, extra=None, yaw_deg=None):
    return {
        "type": "alignment",
        "request_id": request_id,
        "wp": wp,
        "aligned": aligned,
        "fwd_cm": fwd_cm,
        "lat_cm": lat_cm,
        "yaw_deg": yaw_deg,
        "tray_found": tray_found,
        "extra": extra or {},
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


def make_depart_check_response(request_id: str, wp: str, depart_ok: bool,
                               counts: dict = None, tray_found: bool = False) -> dict:
    """OMX -> 관제: 출발 검증 결과.

    depart_ok : 자리별 조건 충족 여부 (PC가 직접 판정).
    counts    : {"red":n, "blue":n, "total":n} (관제 로그/대시보드 표시용).
    tray_found: 트레이를 화면에서 찾았는지 (False면 카메라 이상 가능성).
    """
    return {
        "type": "depart_check",
        "request_id": request_id,
        "wp": wp,
        "depart_ok": depart_ok,
        "counts": counts or {},
        "tray_found": tray_found,
    }
