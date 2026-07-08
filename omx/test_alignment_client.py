#!/usr/bin/env python3
"""
test_alignment_client.py — executor 정렬측정 단독 테스트

관제/브릿지 없이, OMX executor 서버에 check_alignment 한 번만 보내
alignment 응답(fwd_cm/lat_cm 등)을 출력한다. 응답 값이 verify_offset.py 의
'차량 보정량'과 일치하면 executor 의 측정 로직이 검증된 것.

전제:
  - 같은 PC 에서 omx_executor_server.py omx2 가 이미 떠 있어야 함.
  - canonical_ref.json 이 executor 와 같은 폴더에 있어야 함.

사용법:
  python3 test_alignment_client.py omx2
  python3 test_alignment_client.py omx2 --host 127.0.0.1 --port 9002
"""

import argparse
import socket
import time

from config import OMX_CONFIGS, MESSAGE_DELIMITER
from protocol import (
    encode_message, decode_message,
    make_check_alignment_request,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("omx_id", nargs="?", default="omx2")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--wp", default=None, help="웨이포인트 (기본: omx2→C, omx1→A)")
    args = ap.parse_args()

    if args.omx_id not in OMX_CONFIGS:
        print(f"사용법: python3 test_alignment_client.py [{'|'.join(OMX_CONFIGS.keys())}]")
        return

    port = args.port or OMX_CONFIGS[args.omx_id]["port"]
    wp = args.wp or ("C" if args.omx_id == "omx2" else "A")

    print(f"[test] {args.host}:{port} 연결 시도...")
    sock = socket.create_connection((args.host, port), timeout=5.0)
    sock.settimeout(30.0)
    print("[test] 연결됨")

    req = make_check_alignment_request(wp)
    sock.sendall(encode_message(req))
    print(f"[test] 전송: {req}")

    # 응답 한 줄 수신
    buffer = b""
    resp = None
    start = time.time()
    while time.time() - start < 30.0:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buffer += chunk
        while MESSAGE_DELIMITER in buffer:
            line, buffer = buffer.split(MESSAGE_DELIMITER, 1)
            if not line.strip():
                continue
            msg = decode_message(line)
            if msg.get("type") == "alignment":
                resp = msg
                break
        if resp is not None:
            break

    sock.close()

    if resp is None:
        print("[test] ✗ alignment 응답 없음 (서버 로그 확인)")
        return

    print("\n[test] ===== alignment 응답 =====")
    print(f"  tray_found : {resp.get('tray_found')}")
    print(f"  aligned    : {resp.get('aligned')}")
    print(f"  fwd_cm     : {resp.get('fwd_cm'):+.3f}  (+앞 / -뒤)")
    print(f"  lat_cm     : {resp.get('lat_cm'):+.3f}  (+왼 / -오)")
    extra = resp.get("extra", {})
    if extra:
        print(f"  extra      : {extra}")
    print("============================\n")

    if not resp.get("tray_found"):
        print("[test] ⚠ tray_found=False — 트레이 미검출/기준 미로드. 서버 로그와 트레이 위치 확인.")
    else:
        print("[test] 이 fwd_cm/lat_cm 이 verify_offset.py 의 '차량 보정량'과 같은지 비교하세요.")


if __name__ == "__main__":
    main()
