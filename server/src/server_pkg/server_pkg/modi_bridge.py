import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped  # PoseWithCovarianceStamped 추가
from server_pkg.modi_mission_manager import MissionManager, start_mission
from server_pkg.omx_link import OmxConnection
from server_pkg.ssh_launcher import launch_all, kill_all
import threading
import math
import time
import sys
import os
import asyncio
import json  # JSON 추가

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
        if self.active_client_goal_handle:
            self.active_client_goal_handle.cancel_goal_async()
        self.cmd_vel_pub.publish(Twist())
        self._wants_pause = True


# ==========================================
# 1.5. 실시간 위치 추적 (Pose Tracker) - NEW
# ==========================================
class PoseTracker:
    def __init__(self, robot_node, robot_id, shared_pose_state):
        self.robot_node = robot_node
        self.robot_id = robot_id
        self.shared_pose_state = shared_pose_state
        
        # amcl_pose를 구독하여 맵 상의 위치 획득 (터틀봇/Nav2 기본 토픽)
        self.sub = self.robot_node.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 쿼터니언에서 yaw(평면 회전각, 라디안) 추출
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.shared_pose_state[self.robot_id] = {'x': x, 'y': y, 'theta': yaw}


# ==========================================
# 2. Flask 앱 (Pose 스트리밍 추가)
# ==========================================
def create_flask_app(
    start_event: threading.Event,
    mission_holder: dict,
    robot_ready_events: dict,
    shared_state: dict,
    active_proxies: list,
    shared_pose_state: dict  # 위치 상태 딕셔너리 추가
):
    app = Flask(__name__, static_folder='static') # static 폴더 명시적 설정

    @app.route('/')
    def index():
        html_path = os.path.join(os.path.dirname(__file__), 'dashboard.html')
        with open(html_path, 'r', encoding='utf-8') as f:
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
                    yield f'data: {json.dumps(entry, ensure_ascii=False)}\n\n'
                except Exception:
                    yield ': keep-alive\n\n'
        return app.response_class(event_stream(), mimetype='text/event-stream')

    # ── [NEW] 실시간 위치 데이터 스트리밍 라우트 ──
    @app.route('/pose_stream')
    def pose_stream():
        def event_stream():
            while True:
                # 0.1초마다 현재 저장된 위치 정보를 JSON으로 웹에 쏨
                yield f'data: {json.dumps(shared_pose_state)}\n\n'
                time.sleep(0.1)
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
    
    # [NEW] 로봇들의 현재 위치를 저장할 전역 딕셔너리
    # 빈 상태로 시작 -> amcl_pose가 실제로 들어와야 점이 찍힘 (구동 전엔 표시 안 됨)
    shared_pose_state = {}
    
    active_proxies = []
    pose_trackers = [] # [NEW]

    print("==================================================")
    print(" 🚀 모듈형 다중 로봇 액션 프록시 & 관제 브릿지 가동 🚀 ")
    print("==================================================")

    for i, target_robot in enumerate(robot_list):
        robot_id = i + 1
        
        # 액션 프록시 생성
        proxy = ActionProxy(
            server_node=server['node'], robot_node=target_robot['node'],
            server_action_name=f'robot{robot_id}/navigate_to_pose',
            robot_action_name='navigate_to_pose', robot_id=robot_id, shared_state=shared_state
        )
        active_proxies.append(proxy)
        
        # [NEW] 위치 트래커 생성
        tracker = PoseTracker(
            robot_node=target_robot['node'], robot_id=robot_id, shared_pose_state=shared_pose_state
        )
        pose_trackers.append(tracker)

    start_event = threading.Event()
    mission_holder = {'mission': None}
    robot_ready_events = {1: threading.Event(), 2: threading.Event()}

    # Flask 앱 생성 시 shared_pose_state 전달
    flask_app = create_flask_app(start_event, mission_holder, robot_ready_events, shared_state, active_proxies, shared_pose_state)
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
        shared_state['paused'] = True
        for proxy in active_proxies:
            proxy.cancel_current_goal()
        time.sleep(0.5) 
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