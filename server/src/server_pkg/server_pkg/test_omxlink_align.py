#!/usr/bin/env python3
"""
test_omxlink_align.py — omx_link.OmxConnection.request_alignment 단독 테스트

omx_link.py 와 같은 폴더에 두고 실행. 브릿지 전체를 띄우지 않고,
OmxConnection 하나만 만들어 서버에 연결한 뒤 정렬 측정을 요청/수신한다.

전제:
  - 같은 PC 에서 omx_executor_server.py omx2 가 이미 떠 있어야 함.

실행:
  python3 test_omxlink_align.py            # omx2 기본, 3회 측정
  python3 test_omxlink_align.py omx2 5     # 5회 측정

※ omx_link.py 가 상대임포트(.config/.protocol)를 쓰는 경우에도 돌도록,
  이 스크립트를 패키지의 일부로 로드하지 않고 폴더를 sys.path 에 넣어
  절대임포트로 끌어온다. (import 에러가 나면 아래 '문제 해결' 참고)
"""

import importlib
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)


def _load_omxconnection():
    """omx_link.OmxConnection 을 import. 상대임포트면 패키지 경로로 재시도."""
    try:
        mod = importlib.import_module("omx_link")
        return mod.OmxConnection
    except ImportError as e:
        # 상대임포트(.config) 때문에 실패하는 경우: 상위 폴더를 sys.path 에 넣고
        # '<패키지명>.omx_link' 로 재시도
        parent = os.path.dirname(HERE)
        pkg = os.path.basename(HERE)
        if parent not in sys.path:
            sys.path.insert(0, parent)
        try:
            mod = importlib.import_module(f"{pkg}.omx_link")
            return mod.OmxConnection
        except ImportError:
            raise e


def main():
    omx_id = sys.argv[1] if len(sys.argv) >= 2 else "omx2"
    n = int(sys.argv[2]) if len(sys.argv) >= 3 else 3
    wp = "C" if omx_id == "omx2" else "A"

    OmxConnection = _load_omxconnection()

    print(f"[test] OmxConnection('{omx_id}') 생성 + 연결...")
    c = OmxConnection(omx_id)
    c.start()
    time.sleep(3.0)   # 연결 스레드가 붙을 시간

    if not c.connected:
        print(f"[test] ✗ 서버에 연결 안 됨 — omx_executor_server.py {omx_id} 가 떠 있는지 확인")
        return

    print(f"[test] 연결됨. {n}회 정렬 측정 요청\n")
    for i in range(n):
        resp = c.request_alignment(wp, timeout=10.0)
        if resp is None:
            print(f"  [{i+1}] ✗ 응답 없음 (타임아웃/미연결)")
        else:
            print(f"  [{i+1}] tray_found={resp.get('tray_found')} aligned={resp.get('aligned')} "
                  f"fwd={resp.get('fwd_cm'):+.2f} lat={resp.get('lat_cm'):+.2f}  extra={resp.get('extra')}")
        time.sleep(0.5)

    print("\n[test] 완료. 이 값이 test_alignment_client.py / verify_offset.py 와 일치하면 ③ 검증 성공.")
    c.stop()


if __name__ == "__main__":
    main()
