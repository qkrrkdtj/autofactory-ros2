import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from server_pkg.modi_mission_manager import MissionManager, start_mission
from server_pkg.omx_link import OmxConnection
from server_pkg.ssh_launcher import launch_all, kill_all
import threading
import time
import sys
import os
import asyncio
import time


from nav2_msgs.action import NavigateToPose
from flask import Flask, request, jsonify


# ==========================================
# 1. 액션 프록시 (스레드 씹힘 방지 & 일시정지 적용)
# ==========================================
class ActionProxy:
    def __init__(self, server_node, robot_node, server_action_name, robot_action_name, robot_id, shared_state):
        self.server_node = server_node
        self.robot_node = robot_node
        self.robot_id = robot_id
        self.shared_state = shared_state
        self.active_client_goal_handle = None
        self._wants_pause = False  
        self.last_goal = None
        self.pause_requested = False

        self.cmd_vel_pub = self.robot_node.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.robot_node.create_timer(0.1, self._timer_callback)
        self.client = ActionClient(self.robot_node, NavigateToPose, robot_action_name)
        self.server = ActionServer(self.server_node, NavigateToPose, server_action_name, execute_callback=self.execute_callback)

    def _timer_callback(self):
        # pause 중이면 cmd_vel 0을 계속 발행해서 로봇이 움직이지 못하게 함
        if self._wants_pause:
            self.cmd_vel_pub.publish(Twist())

    async def execute_callback(self, goal_handle):
        if self.robot_id not in self.shared_state.get('selected', []):
            goal_handle.succeed()
            return NavigateToPose.Result()

        self.last_goal = goal_handle.request

        self.active_client_goal_handle = (
            await self.client.send_goal_async(
                self.last_goal
            )
        )

        result_wrapper = await (
            self.active_client_goal_handle
            .get_result_async()
        )

        status = result_wrapper.status

        self.server_node.get_logger().info(
            f"[{self.robot_id}] status={status}"
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.active_client_goal_handle = None
        return result_wrapper.result

    def cancel_current_goal(self):
        # ── Nav2 goal cancel 즉시 ──
        if self.active_client_goal_handle:
            self.active_client_goal_handle.cancel_goal_async()
        # ── cmd_vel 0 즉시 1회 발행 (blocking 없음) ──
        self.cmd_vel_pub.publish(Twist())
        # ── 이후 타이머(0.1s)가 계속 0을 발행하도록 플래그 세팅 ──
        self._wants_pause = True


# ==========================================
# 2. Flask 앱 (kill_all 제거, pause/resume 교체)
# ==========================================
def create_flask_app(
    start_event: threading.Event,
    mission_holder: dict,
    robot_ready_events: dict,
    shared_state: dict,
    active_proxies: list
):
    app = Flask(__name__)

    @app.route('/')
    def index():
        html_path = os.path.join(os.path.dirname(__file__), 'dashboard.html')
        with open(html_path, 'r') as f:
            return f.read()

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

    @app.route('/log_stream')
    def log_stream():
        def event_stream():
            while True:
                try:
                    entry = mission_holder['mission'].log_queue.get(timeout=1.0)
                    import json
                    yield f'data: {json.dumps(entry, ensure_ascii=False)}\n\n'
                except Exception:
                    yield ': keep-alive\n\n'
        return app.response_class(event_stream(), mimetype='text/event-stream')

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

    @app.route('/pause', methods=['POST'])
    def pause():
        shared_state['paused'] = True
        # ── blocking 없이 즉시 반환, cancel은 백그라운드에서 ──
        def do_cancel():
            for proxy in active_proxies:
                proxy.cancel_current_goal()
        threading.Thread(target=do_cancel, daemon=True).start()
        return jsonify({'status': 'ok', 'message': '미션 일시정지!'})

    @app.route('/resume', methods=['POST'])
    def resume():
        shared_state['paused'] = False
        for proxy in active_proxies:
            proxy._wants_pause = False
        return jsonify({'status': 'ok', 'message': '목표지점 미션 재개!'})

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


# ==========================================
# 3. 키보드 입력 스레드
# ==========================================
def keyboard_listener(mission_holder: dict):
    print("\n[키보드] 'A' 또는 'C' 입력 후 Enter = 해당 웨이포인트 대기 해제")
    while True:
        user_input = sys.stdin.readline().strip().upper()
        mission = mission_holder.get('mission')
        if mission is None:
            print("[키보드] 아직 미션이 시작되지 않았습니다.")
            continue
        if user_input in ['A', 'C']:
            mission.signal_resume(user_input)


# ==========================================
# 4. 도메인 셋업 헬퍼
# ==========================================
def setup_domain(domain_id, node_name, args=None):
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain_id, args=args)
    node = Node(node_name, context=ctx)
    executor = MultiThreadedExecutor(context=ctx)
    executor.add_node(node)
    return {'ctx': ctx, 'node': node, 'exc': executor}


# ==========================================
# 5. 메인
# ==========================================
def main(args=None):
    server = setup_domain(30, 'server_bridge', args)
    robot1 = setup_domain(31, 'robot1_bridge', args)
    robot2 = setup_domain(32, 'robot2_bridge', args)
    robot_list = [robot1, robot2]

    shared_state = {
        'selected':[1,2],
        'paused':False,
        'started':False
    }
    
    active_proxies = []

    print("==================================================")
    print(" 🚀 모듈형 다중 로봇 액션 프록시 브릿지 가동 🚀 ")
    print("==================================================")

    for i, target_robot in enumerate(robot_list):
        robot_id = i + 1
        proxy = ActionProxy(
            server_node=server['node'], robot_node=target_robot['node'],
            server_action_name=f'robot{robot_id}/navigate_to_pose',
            robot_action_name='navigate_to_pose', robot_id=robot_id, shared_state=shared_state
        )
        active_proxies.append(proxy)

    start_event = threading.Event()
    mission_holder = {'mission': None}
    robot_ready_events = {1: threading.Event(), 2: threading.Event()}

    flask_app = create_flask_app(start_event, mission_holder, robot_ready_events, shared_state, active_proxies)
    flask_thread = threading.Thread(
        target=lambda: flask_app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False),
        daemon=True
    )
    flask_thread.start()
    print("\n🌐 대시보드: http://localhost:5000")

    def on_robot1_ready():
        robot_ready_events[1].set()
        print("✅ Waffle 1 keepout 확인 완료")

    def on_robot2_ready():
        robot_ready_events[2].set()
        print("✅ Waffle 2 keepout 확인 완료")

    try:
        launch_all(on_robot1_ready=on_robot1_ready, on_robot2_ready=on_robot2_ready)
    except TypeError:
        # ssh_launcher가 개별 콜백을 지원하지 않을 때의 폴백
        def on_all_ready():
            robot_ready_events[1].set()
            robot_ready_events[2].set()
            print("✅ 전체 keepout 확인 완료")
        launch_all(on_all_ready=on_all_ready)

    threading.Thread(target=keyboard_listener, args=(mission_holder,), daemon=True).start()

    domain_systems = [server, robot1, robot2]
    ros_threads = []
    for system in domain_systems:
        t = threading.Thread(target=system['exc'].spin, daemon=True)
        t.start()
        ros_threads.append(t)

    print("\n🔌 OMX1/OMX2 연결 시도 중...")
    omx_connections = {'omx1': OmxConnection('omx1'), 'omx2': OmxConnection('omx2')}
    for omx in omx_connections.values():
        omx.start()

    try:
        print("\n⏳ 대시보드에서 [미션 시작] 버튼을 눌러주세요...")
        while not start_event.is_set():
            time.sleep(0.1)
        print("✅ 시작 신호 수신 — 미션을 시작합니다!")

        mission = MissionManager(
            server['node'], robot1['node'], robot2['node'],
            shared_state=shared_state,
            omx_connections=omx_connections,
        )
        mission_holder['mission'] = mission

        for omx in omx_connections.values():
            omx.on_cycle_done = mission.on_omx_done

        threading.Thread(target=start_mission, args=(mission,), daemon=True).start()

        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n🛑 프로그램 종료 요청(Ctrl+C) 수신 — 로봇 정지 후 종료합니다.")
        # ── 미션 루프 즉시 정지 ──
        shared_state['paused'] = True
        for proxy in active_proxies:
            proxy.cancel_current_goal()
        time.sleep(0.5)  # cancel이 Nav2에 전달될 시간
    finally:
        kill_all()
        for system in domain_systems:
            system['exc'].shutdown()
        for t in ros_threads:
            t.join()
        for system in domain_systems:
            system['node'].destroy_node()
            rclpy.shutdown(context=system['ctx'])

if __name__ == '__main__':
    main()