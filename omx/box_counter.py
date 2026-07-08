"""
box_counter.py

omx2(분류, 지점 C) 전용 OpenCV 색(HSV) 기반 물체 카운트.

[변경 이력]
  이전 버전은 '가장 큰 초록 덩어리 = 트레이' 로 ROI 를 잡았으나, C 지점
  조명이 어두워 어두운 바닥/배경까지 초록 임계(S/V 하한)에 걸려 ROI 가
  화면 왼쪽 전체로 부풀었다. 그 결과 운 좋게(분류함이 오른쪽에 있어서)
  개수가 맞기도 했지만, 프레임이 조금만 달라지면 분류함 안의 물체까지
  세어 개수가 틀어졌다.
  -> omx1(box_counter1.py)과 동일하게 ── 고정 ROI + 거리변환 분리 ── 로 교체.
     home 자세에서 wrist 시점이 일정하고, 트레이는 항상 화면 왼쪽에 도킹됨.

특징:
  - 트레이는 화면 '왼쪽' 고정 ROI. 분류함(오른쪽 흰색 박스)은 ROI 밖이라
    구조적으로 제외된다(분류 끝난 물체를 안 센다).
  - 붙은 동색 원기둥은 거리변환(distanceTransform) 코어로 분리해 정확히 센다.
  - tray_found 는 ROI 안 초록 비율로 판정(트레이가 비면 초록이 잘 보이므로
    출발 판정 순간엔 비율이 높다 → 안정적).

실측 omx2 wrist 프레임(640x480)으로 검증:
  트레이3(파1빨2)/트레이1(파1)/빈트레이 모두 정확히 카운트.
  (분류함 오른쪽 물체는 전부 제외)

핵심 판정(출발 조건은 executor 가 config 로 처리):
  지점 C 는 트레이가 비면(depart_target_count=0) 출발.

제공 함수:
  count_in_tray(frame_bgr) : 고정 ROI 안의 빨강/파랑 개수 + 트레이 존재 여부
                             반환 {"tray_found":bool, "red":n, "blue":n, "total":n}

프레임은 omx_executor_server 가 robot.get_observation() 에서 뽑아
RGB->BGR 변환한 뒤 넘겨준다. (lerobot observation 은 RGB)

단독 테스트:
    python3 box_counter.py <이미지경로>            # ROI/카운트 시각화
    python3 box_counter.py <이미지경로> --no-viz
"""

import cv2
import numpy as np

# ── omx2 wrist 프레임으로 튜닝한 색 범위 (HSV) ──
COLOR_RANGES = {
    "red":  [((0, 80, 50), (10, 255, 255)),
             ((165, 80, 50), (180, 255, 255))],
    "blue": [((100, 100, 60), (130, 255, 255))],
}
# 트레이(초록) 존재 확인용. 왼쪽 ROI의 트레이가 조명/그림자 영향으로
# 채도와 명도가 낮게 잡히는 경우를 고려해 범위를 약간 넓게 둔다.
GREEN_RANGE = ((35, 40, 30), (95, 255, 255))

# ── 트레이 고정 ROI (x0, y0, x1, y1) — 640x480 기준, 화면 왼쪽 트레이 영역 ──
# 왼쪽 가장자리의 라이다 파란 불빛/배경 반사가 약통으로 잡히지 않도록
# x=0부터 시작하지 않고 트레이 안쪽 영역만 본다. 도킹 위치가 바뀌면 여기만 다시 맞춘다.
TRAY_ROI = (90, 40, 270, 340)

MIN_AREA = 400           # 색 덩어리로 인정할 최소 면적(px). 작은 잡티 제거용.
DIST_THRESH = 10.0       # 거리변환 코어 임계(px). 붙은 원기둥 분리 기준.
TRAY_GREEN_RATIO = 0.03  # ROI 내 초록 비율이 이 값 미만이면 트레이 미검출로 간주
                         # (도킹 안됨/카메라 이상 방어. 빈 트레이 실측 0.2+ 라 안전).
MIN_CORE_PX = 8          # 거리변환 코어가 이 픽셀 수 미만이면 노이즈 피크로 무시

def _tray_centroid(grn, offset=(0, 0)):
    m = cv2.morphologyEx(grn, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, None
    ox, oy = offset
    return float(M["m10"] / M["m00"] + ox), float(M["m01"] / M["m00"] + oy)
    
def _is_valid_pill_box(x, y, w, h):
    """색은 맞지만 약통 형태가 아닌 반사/불빛 후보를 제거한다."""
    aspect = w / max(h, 1)
    if w < 18 or h < 40:
        return False
    if w > 130 or h > 170:
        return False
    if aspect < 0.30 or aspect > 1.80:
        return False
    return True


def _color_mask(hsv, ranges):
    m = None
    for lo, hi in ranges:
        mm = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = mm if m is None else cv2.bitwise_or(m, mm)
    return m


def _filled_mask(mask, min_area):
    """반사광 구멍 메우고(CLOSE) 잡티 정리(OPEN) 후, 면적 미달 컨투어 제거."""
    m = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    clean = np.zeros_like(m)
    for c in cnts:
        if cv2.contourArea(c) >= min_area:
            cv2.drawContours(clean, [c], -1, 255, -1)
    return clean


def _count_split(mask, min_area, dist_thresh, return_boxes=False, offset=(0, 0)):
    """붙은 동색 원기둥을 거리변환 코어로 분리해 센다.

    return_boxes=True 면 (count, [bbox,...]) 반환. bbox 는 watershed 로 나눈
    원기둥별 (x, y, w, h) (offset 적용된 전체프레임 좌표).
    """
    clean = _filled_mask(mask, min_area)
    if cv2.countNonZero(clean) == 0:
        return (0, []) if return_boxes else 0

    dist = cv2.distanceTransform(clean, cv2.DIST_L2, 5)
    cores = (dist >= dist_thresh).astype(np.uint8)
    n_cores, core_lab = cv2.connectedComponents(cores)
    valid = [i for i in range(1, n_cores) if (core_lab == i).sum() >= MIN_CORE_PX]
    count = len(valid)

    boxes = []
    if count == 0:
        return (0, boxes) if return_boxes else 0

    markers = np.zeros(clean.shape, np.int32)
    for new_id, old_id in enumerate(valid, start=2):
        markers[core_lab == old_id] = new_id
    unknown = cv2.subtract(clean, (markers > 0).astype(np.uint8) * 255)
    markers[clean == 0] = 1
    markers[unknown == 255] = 0
    cv2.watershed(cv2.cvtColor(clean, cv2.COLOR_GRAY2BGR), markers)
    ox, oy = offset
    for new_id in range(2, count + 2):
        ys, xs = np.where(markers == new_id)
        if xs.size == 0:
            continue
        x, y = xs.min(), ys.min()
        w, h = xs.max() - x + 1, ys.max() - y + 1
        box = (x + ox, y + oy, w, h)
        if _is_valid_pill_box(*box):
            boxes.append(box)

    filtered_count = len(boxes)
    return (filtered_count, boxes) if return_boxes else filtered_count


def _clamp_roi(roi, w, h):
    x0, y0, x1, y1 = roi
    return (max(0, min(x0, w)), max(0, min(y0, h)),
            max(0, min(x1, w)), max(0, min(y1, h)))


def count_in_tray(frame_bgr, roi=TRAY_ROI, min_area=MIN_AREA,
                  dist_thresh=DIST_THRESH, return_boxes=False):
    """
    화면 왼쪽 고정 ROI 안에서 빨강/파랑 원기둥을 센다 (C 출발 판정용).
    붙어 있는 동색 원기둥은 거리변환으로 분리해 정확히 카운트한다.

    반환 : {"tray_found": bool, "red": n, "blue": n, "total": n}
           ROI 안 초록(트레이) 비율이 너무 낮으면 tray_found=False.
           ※ tray_found=False 는 '도킹 안됨/카메라 이상' 신호 (executor 가 출발 보류).
    """
    h, w = frame_bgr.shape[:2]
    x0, y0, x1, y1 = _clamp_roi(roi, w, h)
    region = frame_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

    grn = cv2.inRange(hsv, np.array(GREEN_RANGE[0]), np.array(GREEN_RANGE[1]))
    tray_found = bool((grn > 0).mean() >= TRAY_GREEN_RATIO)
    tray_cx, tray_cy = _tray_centroid(grn, offset=(x0, y0))   # 전체프레임 px

    counts = {"tray_found": tray_found, "tray_cx": tray_cx, "tray_cy": tray_cy}
    all_boxes = []
    for color, ranges in COLOR_RANGES.items():
        mask = _color_mask(hsv, ranges)
        if return_boxes:
            cnt, boxes = _count_split(mask, min_area, dist_thresh,
                                      return_boxes=True, offset=(x0, y0))
            for bx in boxes:
                all_boxes.append((color, bx))
        else:
            cnt = _count_split(mask, min_area, dist_thresh)
        counts[color] = cnt
    counts["total"] = counts["red"] + counts["blue"]

    if return_boxes:
        return counts, all_boxes, (x0, y0, x1 - x0, y1 - y0)
    return counts


# ── 단독 테스트용 ──
def _demo(path, viz=True):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(path)
    counts, boxes, roi = count_in_tray(img, return_boxes=True)
    print(counts)
    if viz:
        bgr = {"red": (0, 0, 255), "blue": (255, 0, 0)}
        x, y, w, h = roi
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
        for color, (bx, by, bw, bh) in boxes:
            cv2.rectangle(img, (bx, by), (bx + bw, by + bh), bgr[color], 2)
        label = f"tray={counts['tray_found']} R={counts['red']} B={counts['blue']} total={counts['total']}"
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imwrite("count_result_omx2.png", img)
        print("시각화 저장: count_result_omx2.png")


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("사용법: python3 box_counter.py <이미지경로> [--no-viz]")
        sys.exit(1)
    _demo(sys.argv[1], viz="--no-viz" not in sys.argv)
