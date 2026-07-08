"""
box_counter_back.py

후방 캠(cam_back) '정렬 전용' 트레이 무게중심 검출기.

box_counter.py(omx2/wrist)와 목적이 다르다:
  - wrist 카운터는 원기둥 개수까지 센다(출발판정용).
  - 이건 '정렬'만을 위한 것이라 → 트레이 초록 무게중심(tray_cx, tray_cy)만 낸다.
    (원기둥 카운트 불필요. 후방 캠은 팔 home 시 트레이를 정면에 가깝게 봄)

후방 캠 시점(640x480) 특징:
  - 트레이(초록)가 화면 중앙 상단쯤에 가로로 놓임.
  - 위쪽=차체(검정), 아래쪽=분류함(흰)/팔베이스, 좌하단=파란 원기둥, 우측=흰 선.
    → 이것들을 고정 ROI 로 잘라내고, ROI 안 초록만 본다.

반환 형태는 box_counter.count_in_tray 와 호환:
  {"tray_found": bool, "tray_cx": float|None, "tray_cy": float|None,
   "red": 0, "blue": 0, "total": 0}
  (red/blue/total 은 정렬에선 안 쓰지만, executor/스크립트 공용을 위해 키만 유지)

단독 테스트:
    python3 box_counter_back.py <이미지경로>          # ROI/무게중심 시각화
    python3 box_counter_back.py <이미지경로> --no-viz
"""

import cv2
import numpy as np

# ── 후방 캠 프레임으로 튜닝할 초록 범위 (HSV) ──
# 트레이 초록만 잡고 바닥/그림자는 배제. 실측 프레임 보고 조이면 됨.
GREEN_RANGE = ((35, 60, 40), (90, 255, 255))

# ── 트레이 고정 ROI (x0, y0, x1, y1) — 640x480, 후방 캠 트레이 영역 ──
# omx1(station A) / omx2(station C) 는 후방캠 도킹 위치/각도가 달라
# 트레이가 화면상 다른 위치에 잡힌다. omx_id 별로 ROI 를 분리한다.
# 단독 테스트로 십자가가 트레이 중심에 찍히는지 보고 이 값만 조정.
TRAY_ROI_BY_OMX = {
    "omx1": (40, 160, 390, 300),    # 양 끝 마커까지 포함하도록 좌우 확장
    "omx2": (120, 100, 630, 300),   # 어긋남 대비 좌우 크게 (마커가 벗어나지 않게)
}
TRAY_ROI = TRAY_ROI_BY_OMX["omx2"]  # 기본값(하위호환용, omx_id 없이 호출되던 곳 대비)

TRAY_GREEN_RATIO = 0.03   # ROI 내 초록 비율이 이 값 미만이면 트레이 미검출(도킹 안됨/이상)
MIN_GREEN_AREA = 800      # 무게중심 계산에 쓸 최소 초록 덩어리 면적(px)

# 트레이 흰 테이프(고정 마커) — 원기둥(빨/파)과 확실히 구분되는 흰색.
# HSV 에서 흰색 = 낮은 채도(S) + 높은 명도(V). 조명 반사로 S 가 조금 떠도 잡히게
# S 상한을 넉넉히, V 하한을 높게.
WHITE_RANGE = ((0, 0, 160), (180, 70, 255))
MIN_WHITE_AREA = 150     # 흰 마커로 인정할 최소 면적(px)

# 트레이 중앙 노란 마커 — 초록/검정/회색/빨강/파랑 어디와도 안 겹침.
# 조명 반사로 밝을 때도 잡히게 S/V 하한을 적당히.
MARKER_RANGE = ((18, 90, 90), (38, 255, 255))
MIN_MARKER_AREA = 40     # 마커로 인정할 최소 면적(px). 작은 노란 잡티 배제.

def _clamp_roi(roi, w, h):
    x0, y0, x1, y1 = roi
    return (max(0, min(x0, w)), max(0, min(y0, h)),
            max(0, min(x1, w)), max(0, min(y1, h)))


def _tray_markers(region_bgr, offset=(0, 0)):
    """ROI 안 노란 마커들을 찾아 (중심cx, 중심cy, 각도deg, 마커수) 반환.

    양 끝 마커 2개(x좌표 최소/최대)로:
      - 트레이 중심 = 두 마커의 중점
      - 트레이 각도 = 두 마커를 잇는 선의 기울기(atan2, 도)
    마커가 2개 미만이면 각도는 None. 1개면 그 점을 중심으로(폴백).
    실패 시 (None, None, None, 0).
    """
    ox, oy = offset
    hsv = cv2.cvtColor(region_bgr, cv2.COLOR_BGR2HSV)
    marker = cv2.inRange(hsv, np.array(MARKER_RANGE[0]), np.array(MARKER_RANGE[1]))
    marker = cv2.morphologyEx(marker, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    marker = cv2.morphologyEx(marker, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    cnts, _ = cv2.findContours(marker, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pts = []
    for c in cnts:
        if cv2.contourArea(c) >= MIN_MARKER_AREA:
            M = cv2.moments(c)
            if M["m00"] > 0:
                pts.append((M["m10"] / M["m00"], M["m01"] / M["m00"]))

    if not pts:
        return None, None, None, 0

    # x좌표로 정렬 → 양 끝 선택
    pts.sort(key=lambda p: p[0])

    if len(pts) >= 2:
        xL, yL = pts[0]        # 가장 왼쪽
        xR, yR = pts[-1]       # 가장 오른쪽
        cx = (xL + xR) / 2.0 + ox
        cy = (yL + yR) / 2.0 + oy
        angle = float(np.degrees(np.arctan2(yR - yL, xR - xL)))
        return float(cx), float(cy), angle, len(pts)

    # 마커 1개뿐 → 양 끝 판정 불가 → 측정 실패(정렬은 2개 필수)
    return None, None, None, len(pts)

def count_in_tray(frame_bgr, roi=None, omx_id=None, return_boxes=False, **_ignored):
    if roi is None:
        roi = TRAY_ROI_BY_OMX.get(omx_id, TRAY_ROI)
    """
    후방 캠 프레임에서 트레이 초록 무게중심을 낸다 (정렬 전용).

    반환 : {"tray_found": bool, "tray_cx": float|None, "tray_cy": float|None,
            "red": 0, "blue": 0, "total": 0}
           ROI 안 초록 비율이 너무 낮으면 tray_found=False (도킹 안됨/이상 신호).

    return_boxes=True 이면 box_counter 와 시그니처 호환을 위해
    (counts, boxes, roi_xywh) 를 반환한다(boxes 는 항상 빈 리스트).
    """
    h, w = frame_bgr.shape[:2]
    x0, y0, x1, y1 = _clamp_roi(roi, w, h)
    region = frame_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

    grn = cv2.inRange(hsv, np.array(GREEN_RANGE[0]), np.array(GREEN_RANGE[1]))
    tray_found = bool((grn > 0).mean() >= TRAY_GREEN_RATIO)

    tray_cx, tray_cy, tray_angle, n_markers = _tray_markers(region, offset=(x0, y0))
    if tray_cx is None:
        tray_found = False

    counts = {
        "tray_found": tray_found,
        "tray_cx": tray_cx, "tray_cy": tray_cy,
        "tray_angle": tray_angle, "n_markers": n_markers,
        "red": 0, "blue": 0, "total": 0,
    }

    if return_boxes:
        return counts, [], (x0, y0, x1 - x0, y1 - y0)
    return counts


# ── 단독 테스트용 ──
def _demo(path, omx_id="omx2", viz=True):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(path)
    counts, _boxes, roi = count_in_tray(img, omx_id=omx_id, return_boxes=True)
    print(counts)
    
    if viz:
        x, y, w, h = roi
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cx, cy = counts["tray_cx"], counts["tray_cy"]
        if cx is not None:
            cv2.drawMarker(img, (int(round(cx)), int(round(cy))), (0, 0, 255),
                           cv2.MARKER_CROSS, 26, 2)
            cv2.circle(img, (int(round(cx)), int(round(cy))), 6, (0, 0, 255), 2)
        label = (f"tray={counts['tray_found']} "
                 f"cx={cx if cx is None else round(cx,1)} "
                 f"cy={cy if cy is None else round(cy,1)}"
                 f"ang={counts.get('tray_angle') if counts.get('tray_angle') is None else round(counts.get('tray_angle'),1)} "
                 f"n={counts.get('n_markers')}")
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imwrite("count_result_back.png", img)
        print("시각화 저장: count_result_back.png")
            
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("사용법: python3 box_counter_back.py <이미지경로> [omx1|omx2] [--no-viz]")
        sys.exit(1)
    omx_id = sys.argv[2] if len(sys.argv) >= 3 and sys.argv[2] in ("omx1", "omx2") else "omx2"
    viz = "--no-viz" not in sys.argv
    _demo(sys.argv[1], omx_id=omx_id, viz=viz)
