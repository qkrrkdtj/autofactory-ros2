"""
box_counter1.py

omx1(적재, 지점 A) 전용 OpenCV 색(HSV) 기반 물체 카운트.

box_counter.py(omx2/분류, 지점 C)와 알고리즘이 다르다:
  omx2 는 초록 트레이를 색으로 찾아 그 안을 센다.
  omx1 은 가운데 '컨베이어 벨트도 초록색'이라 '가장 큰 초록=트레이' 가정이
  깨진다. 그래서 home 자세에서 도킹 위치가 고정인 점을 이용해
  ── 오른쪽 트레이 영역을 고정 ROI 로 잡고 그 안에서만 원기둥을 센다. ──
  (벨트는 ROI 밖이라 자동 제외)

붙은 동색 원기둥 분리:
  원기둥 윗면 반사광(하이라이트)으로 색 마스크에 구멍이 생겨 MORPH_CLOSE 로
  메우는데, 이게 옆에 붙은 같은 색 원기둥끼리 이어버려 2개->1개로 합쳐졌다
  (예: '파파빨'에서 파랑 2개가 1개로). 이를 막기 위해
  ── 거리변환(distanceTransform)으로 각 원기둥 중심(코어)을 분리해 센다. ──
  실측 omx1 wrist 프레임(640x480)으로 검증(크기 무관 비율 분리):
    파1/파2/파3 및 파파빨(파2+빨1) 모두 정확히 카운트.

핵심 판정(출발 조건은 executor 가 config 로 처리):
  지점 A 는 트레이가 depart_target_count(=3) 개 차면 출발.

제공 함수:
  count_in_tray(frame_bgr) : 고정 ROI 안의 빨강/파랑 개수 + 트레이 존재 여부
                             반환 {"tray_found":bool, "red":n, "blue":n, "total":n}
                             (box_counter.py 와 동일한 반환 형태 -> executor 공용)

프레임은 omx_executor_server 가 robot.get_observation() 에서 뽑아
RGB->BGR 변환한 뒤 넘겨준다. (lerobot observation 은 RGB)

단독 테스트:
    python3 box_counter1.py <이미지경로>            # ROI/카운트 시각화
    python3 box_counter1.py <이미지경로> --no-viz
"""

import cv2
import numpy as np

# ── omx1 wrist 프레임으로 튜닝한 색 범위 (HSV) ──
# 빨강은 hue가 0/180 양 끝으로 갈라지므로 두 구간을 합친다.
# 채도(S) 하한을 130 으로 높였다: 진짜 원기둥은 채도가 매우 높지만(S>=200),
# ROI 안에 들어오는 어두운 배경 구조물/그림자(검은 크레이트 등)는 채도가 낮아
# (실측 S~105) 가짜로 잡혔다. S>=130 이면 진짜만 남고 배경 오검출이 제거된다.
COLOR_RANGES = {
    "red":  [((0, 130, 50), (10, 255, 255)),
             ((165, 130, 50), (180, 255, 255))],
    "blue": [((100, 130, 60), (130, 255, 255))],
}
# 트레이(초록) 존재 확인용. 벨트와 색이 비슷하지만 ROI 안에서만 보므로 무방.
GREEN_RANGE = ((35, 40, 30), (95, 255, 255))

# ── 트레이 고정 ROI (x0, y0, x1, y1) — 640x480 기준, 오른쪽 트레이 영역 ──
# home 도킹 위치가 바뀌면 여기만 다시 맞추면 된다(단독 테스트로 확인).
TRAY_ROI = (480, 120, 600, 400)

MIN_AREA = 600           # 색 덩어리로 인정할 최소 면적(px). 작은 잡티 제거용.
CORE_RATIO = 0.6         # 거리변환 코어 임계(각 덩어리 최대거리 대비 비율).
                         # 고정 px 임계는 원기둥 크기가 프레임마다 달라(도킹 거리)
                         # 큰 원기둥의 두꺼운 접합부를 못 끊었다. 비율 방식은 크기 무관.
                         # 실측상 0.55~0.7 에서 붙은 원기둥이 정확히 분리됨.
TRAY_GREEN_RATIO = 0.05  # ROI 내 초록 픽셀 비율이 이 값 미만이면 트레이 미검출로 간주
                         # (도킹 안됨/카메라 이상 방어. 실측 0.23~0.50 이라 안전한 바닥값)
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


def _count_split(mask, min_area, core_ratio, return_boxes=False, offset=(0, 0)):
    """붙은 동색 원기둥을 거리변환 코어로 분리해 센다.

    각 연결덩어리(component)마다 그 덩어리의 최대거리 대비 core_ratio 비율로
    코어를 잡으므로, 원기둥 크기가 프레임마다 달라도(도킹 거리) 견고하게 분리된다.

    return_boxes=True 면 (count, [bbox,...]) 반환. bbox 는 watershed 로 나눈
    원기둥별 (x, y, w, h) (offset 적용된 전체프레임 좌표).
    """
    clean = _filled_mask(mask, min_area)
    if cv2.countNonZero(clean) == 0:
        return (0, []) if return_boxes else 0

    dist = cv2.distanceTransform(clean, cv2.DIST_L2, 5)

    # ── 덩어리별 최대거리 대비 비율로 코어 추출 (크기 무관) ──
    n_comp, comp_lab = cv2.connectedComponents(clean)
    cores = np.zeros(clean.shape, np.uint8)
    for i in range(1, n_comp):
        comp = comp_lab == i
        peak = dist[comp].max()
        if peak <= 0:
            continue
        cores[comp & (dist >= core_ratio * peak)] = 255

    n_cores, core_lab = cv2.connectedComponents(cores)
    valid = [i for i in range(1, n_cores) if (core_lab == i).sum() >= MIN_CORE_PX]
    count = len(valid)

    boxes = []
    if count == 0:
        return (0, boxes) if return_boxes else 0

    # watershed 로 각 코어 영역을 원기둥 전체로 확장 -> 박스 추출
    markers = np.zeros(clean.shape, np.int32)
    for new_id, old_id in enumerate(valid, start=2):
        markers[core_lab == old_id] = new_id            # 코어 = 전경 마커(2..)
    unknown = cv2.subtract(clean, (markers > 0).astype(np.uint8) * 255)
    markers[clean == 0] = 1                              # 배경 마커 = 1
    markers[unknown == 255] = 0                          # 미지 영역 = 0
    color3 = cv2.cvtColor(clean, cv2.COLOR_GRAY2BGR)
    cv2.watershed(color3, markers)
    ox, oy = offset
    for new_id in range(2, count + 2):
        ys, xs = np.where(markers == new_id)
        if xs.size == 0:
            continue
        x, y, w, h = xs.min(), ys.min(), xs.max() - xs.min() + 1, ys.max() - ys.min() + 1
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
                  core_ratio=CORE_RATIO, return_boxes=False):
    """
    오른쪽 고정 ROI 안에서 빨강/파랑 원기둥을 센다 (A 출발 판정용).
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
            cnt, boxes = _count_split(mask, min_area, core_ratio,
                                      return_boxes=True, offset=(x0, y0))
            for bx in boxes:
                all_boxes.append((color, bx))
        else:
            cnt = _count_split(mask, min_area, core_ratio)
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
        cv2.imwrite("count_result_omx1.png", img)
        print("시각화 저장: count_result_omx1.png")


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("사용법: python3 box_counter1.py <이미지경로> [--no-viz]")
        sys.exit(1)
    _demo(sys.argv[1], viz="--no-viz" not in sys.argv)
