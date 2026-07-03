import os
import json
import time
import threading

from flask import Flask, request, jsonify
from ament_index_python.packages import get_package_share_directory

from server_pkg import stats_db


# ==========================================
# Flask 앱 팩토리
# ==========================================
def create_flask_app(
    start_event: threading.Event,
    mission_holder: dict,
    robot_ready_events: dict,
    shared_state: dict,
    active_proxies: list,
    shared_pose_state: dict,
    shared_battery_state: dict,
):
    
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    html_dir = os.path.join(BASE_DIR, 'html')   
    static_dir = os.path.join(BASE_DIR, 'static')

    app = Flask(__name__,
            static_folder=static_dir,
            static_url_path='/static')

    # DB 초기화 (테이블 없으면 생성)
    stats_db.init_db()

    # ── 대시보드 메인 페이지 (실시간 관제) ──
    @app.route('/')
    def index():
        with open(os.path.join(html_dir, 'dashboard.html'), 'r', encoding='utf-8') as f:
            return f.read()

    # ── 공정 통계 페이지 ──
    @app.route('/stats')
    def stats_page():
        with open(os.path.join(html_dir, 'stats.html'), 'r', encoding='utf-8') as f:
            return f.read()

    # ── 통계 요약 API ──
    @app.route('/api/stats/summary')
    def api_stats_summary():
        return jsonify(stats_db.get_summary())

    # ── 공정 이력 API (robot/status/limit 필터) ──
    @app.route('/api/stats/logs')
    def api_stats_logs():
        robot  = request.args.get('robot')  or None
        status = request.args.get('status') or None
        limit  = request.args.get('limit', default=50, type=int)
        return jsonify(stats_db.get_logs(limit=limit, robot=robot, status=status))

    # ── 현재 미션 상태 조회 (페이지 로드 시 UI 복원용) ──
    @app.route('/state')
    def get_state():
        return jsonify({
            'started':  shared_state.get('started', False),
            'paused':   shared_state.get('paused', False),
            'selected': shared_state.get('selected', [1, 2]),
            'ready': {
                '1': robot_ready_events[1].is_set(),
                '2': robot_ready_events[2].is_set(),
            },
        })

    # ── keepout 준비 완료 SSE ──
    @app.route('/ready_stream')
    def ready_stream():
        def event_stream():
            sent = {1: False, 2: False}
            while True:
                for robot_id, event in robot_ready_events.items():
                    if not sent[robot_id] and event.is_set():
                        yield f'data: ready_{robot_id}\n\n'
                        sent[robot_id] = True
                if all(sent.values()):
                    break
                time.sleep(0.5)
        return app.response_class(event_stream(), mimetype='text/event-stream')

    # ── 미션 로그 SSE ──
    @app.route('/log_stream')
    def log_stream():
        def event_stream():
            while True:
                try:
                    entry = mission_holder['mission'].log_queue.get(timeout=1.0)
                    yield f'data: {json.dumps(entry, ensure_ascii=False)}\n\n'
                except Exception:
                    yield ': keep-alive\n\n'
        return app.response_class(event_stream(), mimetype='text/event-stream')

    # ── 실시간 위치 SSE ──
    @app.route('/pose_stream')
    def pose_stream():
        def event_stream():
            while True:
                yield f'data: {json.dumps(shared_pose_state)}\n\n'
                time.sleep(0.1)
        return app.response_class(event_stream(), mimetype='text/event-stream')

    # ── 배터리 상태 SSE ──
    @app.route('/battery_stream')
    def battery_stream():
        def event_stream():
            while True:
                yield f'data: {json.dumps(shared_battery_state)}\n\n'
                time.sleep(2.0)
        return app.response_class(event_stream(), mimetype='text/event-stream')

    # ── 미션 시작 ──
    @app.route('/start', methods=['POST'])
    def start():
        body = request.get_json(silent=True) or {}
        selected = body.get('selected', [1, 2])

        not_ready = [r for r in selected if not robot_ready_events[r].is_set()]
        if not_ready:
            return jsonify({'status': 'error', 'message': f'Waffle {not_ready} 아직 keepout 준비 안됨'}), 400

        shared_state['selected'] = selected
        shared_state['paused'] = False
        shared_state['started'] = True

        if not start_event.is_set():
            start_event.set()
            return jsonify({'status': 'ok', 'message': '미션 시작!'})
        return jsonify({'status': 'already', 'message': '이미 시작됨'})

    # ── 미션 일시정지 ──
    @app.route('/pause', methods=['POST'])
    def pause():
        shared_state['paused'] = True
        def do_cancel():
            for proxy in active_proxies:
                proxy.cancel_current_goal()
        threading.Thread(target=do_cancel, daemon=True).start()
        return jsonify({'status': 'ok', 'message': '미션 일시정지!'})

    # ── 미션 재개 ──
    @app.route('/resume', methods=['POST'])
    def resume():
        shared_state['paused'] = False
        for proxy in active_proxies:
            proxy._wants_pause = False
        return jsonify({'status': 'ok', 'message': '목표지점 미션 재개!'})

    # ── 웨이포인트 수동 신호 ──
    @app.route('/signal/<wp>', methods=['POST'])
    def signal_wp(wp):
        mission = mission_holder.get('mission')
        if mission is None:
            return jsonify({'status': 'error', 'message': '미션이 아직 생성되지 않았습니다'}), 400

        target = wp.upper()
        if target in ['A', 'C']:
            mission.signal_resume(target)
            return jsonify({'status': 'ok', 'message': f'{target} 대기 해제!'})
        return jsonify({'status': 'error', 'message': '잘못된 웨이포인트입니다.'}), 400

    return app