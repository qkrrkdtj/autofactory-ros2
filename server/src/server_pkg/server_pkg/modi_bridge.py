import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from server_pkg.modi_mission_manager import MissionManager, start_mission
from server_pkg.omx_link import OmxConnection
import threading
import time
import sys

from nav2_msgs.action import NavigateToPose

# Flask (시작 버튼용)
from flask import Flask, request, jsonify

# ==========================================
# 1. 액션 프록시 (기존과 동일)
# ==========================================
class ActionProxy:
    def __init__(self, server_node, robot_node, server_action_name, robot_action_name):
        self.server_node = server_node
        self.robot_node = robot_node

        self.client = ActionClient(
            self.robot_node, NavigateToPose, robot_action_name
        )
        self.server = ActionServer(
            self.server_node, NavigateToPose, server_action_name,
            execute_callback=self.execute_callback
        )
        self.server_node.get_logger().info(
            f"🔗 라우팅 완료: [서버] {server_action_name} ➡️ [로봇] {robot_action_name}"
        )

    async def execute_callback(self, goal_handle):
        self.server_node.get_logger().info('서버에서 주행 명령 수신! 해당 로봇으로 전달합니다.')

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.server_node.get_logger().error('로봇의 Nav2 액션 서버를 찾을 수 없습니다.')
            goal_handle.abort()
            return NavigateToPose.Result()

        def proxy_feedback_cb(feedback_msg):
            goal_handle.publish_feedback(feedback_msg.feedback)

        send_goal_future = await self.client.send_goal_async(
            goal_handle.request, feedback_callback=proxy_feedback_cb
        )

        if not send_goal_future.accepted:
            self.server_node.get_logger().error('로봇이 주행을 거부했습니다.')
            goal_handle.abort()
            return NavigateToPose.Result()

        result_wrapper = await send_goal_future.get_result_async()
        status = result_wrapper.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            self.server_node.get_logger().info('주행 성공!')
        elif status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
            self.server_node.get_logger().warn('주행 실패(Aborted)!')
        elif status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            self.server_node.get_logger().warn('주행 취소됨(Canceled)!')
        else:
            goal_handle.abort()
            self.server_node.get_logger().error(f'알 수 없는 에러 상태({status}).')

        return result_wrapper.result


# ==========================================
# 2. Flask 앱 — 시작 버튼 + A,C 신호 수신
# ==========================================
def create_flask_app(start_event: threading.Event, mission_holder: dict):
    app = Flask(__name__)

    @app.route('/')
    def index():
        with open('/tmp/dashboard.html', 'r') as f:
            return f.read()

    @app.route('/start', methods=['POST'])
    def start():
        if not start_event.is_set():
            start_event.set()
            return jsonify({'status': 'ok', 'message': '미션 시작!'})
        return jsonify({'status': 'already', 'message': '이미 시작됨'})

    # ── A, C 신호를 모두 처리할 수 있도록 동적 라우팅 적용 ──
    @app.route('/signal/<wp>', methods=['POST'])
    def signal_wp(wp):
        mission = mission_holder.get('mission')
        if mission is None:
            return jsonify({'status': 'error', 'message': '미션이 아직 생성되지 않았습니다'}), 400
        
        target = wp.upper()
        if target in ['A', 'C']:
            mission.signal_resume(target)
            return jsonify({'status': 'ok', 'message': f'{target} 대기 해제!'})
        else:
            return jsonify({'status': 'error', 'message': '잘못된 웨이포인트입니다.'}), 400

    return app


# ==========================================
# 3. HTML 대시보드 생성
# ==========================================
def write_dashboard_html():
    html = """<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <title>🤖 로봇 미션 대시보드</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', sans-serif;
      background: #0f172a;
      color: #e2e8f0;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      min-height: 100vh;
      gap: 24px;
    }
    h1 { font-size: 1.8rem; letter-spacing: 0.05em; color: #7dd3fc; }
    .card {
      background: #1e293b;
      border: 1px solid #334155;
      border-radius: 16px;
      padding: 32px 48px;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 16px;
      width: 380px;
    }
    .label { font-size: 0.85rem; color: #94a3b8; letter-spacing: 0.08em; }
    button {
      width: 100%;
      padding: 14px;
      border: none;
      border-radius: 10px;
      font-size: 1.1rem;
      font-weight: 700;
      cursor: pointer;
      transition: opacity 0.2s, transform 0.1s;
    }
    button:active { transform: scale(0.97); }
    button:disabled { opacity: 0.4; cursor: not-allowed; }
    #btn-start  { background: #22c55e; color: #fff; }
    .signal-btn { background: #f59e0b; color: #fff; flex: 1; }
    .btn-group { display: flex; gap: 12px; width: 100%; }
    #status {
      font-size: 0.9rem;
      color: #64748b;
      min-height: 1.2em;
      text-align: center;
    }
    .divider { width: 100%; border: none; border-top: 1px solid #334155; }
  </style>
</head>
<body>
  <h1>🤖 로봇 미션 대시보드</h1>

  <div class="card">
    <span class="label">MISSION CONTROL</span>
    <button id="btn-start" onclick="sendStart()">▶ 미션 시작</button>

    <hr class="divider">

    <span class="label">EXTERNAL SIGNALS</span>
    <div class="btn-group">
      <button class="signal-btn" onclick="sendSignal('A')" disabled>✅ A 해제</button>
      <button class="signal-btn" onclick="sendSignal('C')" disabled>✅ C 해제</button>
    </div>

    <p id="status">대기 중...</p>
  </div>

  <script>
    async function sendStart() {
      document.getElementById('btn-start').disabled = true;
      const res = await fetch('/start', { method: 'POST' });
      const data = await res.json();
      setStatus(data.message, '#22c55e');
      
      // 시작 후 A, C 신호 버튼 모두 활성화
      document.querySelectorAll('.signal-btn').forEach(btn => btn.disabled = false);
    }

    async function sendSignal(wp) {
      const res = await fetch(`/signal/${wp}`, { method: 'POST' });
      const data = await res.json();
      setStatus(data.message, '#f59e0b');
    }

    function setStatus(msg, color) {
      const el = document.getElementById('status');
      el.textContent = msg;
      el.style.color = color;
    }
  </script>
</body>
</html>
"""
    with open('/tmp/dashboard.html', 'w') as f:
        f.write(html)

# ==========================================
# 4. 키보드 입력 스레드 (Enter → C 신호)
# ==========================================
def keyboard_listener(mission_holder: dict):
    print("\n[키보드] 터미널에 'A' 또는 'C' 입력 후 Enter = 해당 웨이포인트 대기 해제")
    while True:
        # 사용자의 텍스트 입력을 받아 공백 제거 후 대문자로 변환
        user_input = sys.stdin.readline().strip().upper()
        
        mission = mission_holder.get('mission')
        if mission is None:
            print("[키보드] 아직 미션이 시작되지 않았습니다.")
            continue
            
        if user_input == 'A':
            mission.signal_resume('A')
        elif user_input == 'C':
            mission.signal_resume('C')
        elif user_input:
            print("[키보드] 잘못된 입력입니다. 'A' 또는 'C'를 입력해주세요.")

# ==========================================
# 5. 도메인 셋업 헬퍼
# ==========================================
def setup_domain(domain_id, node_name, args=None):
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain_id, args=args)
    node = Node(node_name, context=ctx)
    executor = MultiThreadedExecutor(context=ctx)
    executor.add_node(node)
    return {'ctx': ctx, 'node': node, 'exc': executor}


# ==========================================
# 6. 메인
# ==========================================
def main(args=None):

    # ── ROS2 도메인 생성 ──
    server = setup_domain(30, 'server_bridge', args)
    robot1 = setup_domain(31, 'robot1_bridge', args)
    robot2 = setup_domain(32, 'robot2_bridge', args)
    robot_list = [robot1, robot2]

    active_proxies = []

    print("==================================================")
    print(" 🚀 모듈형 다중 로봇 액션 프록시 브릿지 가동 🚀 ")
    print("==================================================")

    for i, target_robot in enumerate(robot_list):
        robot_id = i + 1
        proxy = ActionProxy(
            server_node=server['node'],
            robot_node=target_robot['node'],
            server_action_name=f'robot{robot_id}/navigate_to_pose',
            robot_action_name='navigate_to_pose',
        )
        active_proxies.append(proxy)

    # ── 시작 이벤트 & 미션 홀더 ──
    start_event   = threading.Event()   # HTML 버튼이 set()
    mission_holder = {'mission': None}  # 미션 객체 공유

    # ── Flask 서버 스레드 ──
    write_dashboard_html()
    flask_app = create_flask_app(start_event, mission_holder)
    flask_thread = threading.Thread(
        target=lambda: flask_app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False),
        daemon=True
    )
    flask_thread.start()
    print("\n🌐 대시보드: http://localhost:5000")

    # ── 키보드 리스너 스레드 ──
    kb_thread = threading.Thread(
        target=keyboard_listener, args=(mission_holder,), daemon=True
    )
    kb_thread.start()

    # ── ROS2 executor 스레드들 ──
    domain_systems = [server, robot1, robot2]
    ros_threads = []
    for system in domain_systems:
        t = threading.Thread(target=system['exc'].spin, daemon=True)
        t.start()
        ros_threads.append(t)

    # ── OMX 연결 미리 생성 (콜백은 미션 생성 후에 연결) ──
    # 버튼 누르기 전에 미리 붙여놔서 시작 즉시 사용 가능하도록.
    print("\n🔌 OMX1/OMX2 연결 시도 중...")
    omx_connections = {
        'omx1': OmxConnection('omx1'),
        'omx2': OmxConnection('omx2'),
    }
    for omx in omx_connections.values():
        omx.start()

    # ── 시작 신호 대기 (HTML 버튼) ──
    print("\n⏳ 대시보드에서 [미션 시작] 버튼을 눌러주세요...")
    start_event.wait()
    print("✅ 시작 신호 수신 — 미션을 시작합니다!")

    # ── 미션 생성 & 등록 (OMX 연결 주입) ──
    mission = MissionManager(
        server['node'], robot1['node'], robot2['node'],
        omx_connections=omx_connections,
    )
    mission_holder['mission'] = mission   # Flask & 키보드가 참조 가능하도록

    # ── OMX 완료 콜백을 미션에 연결 (닭-달걀 해소: 사후 주입) ──
    # OMX 수신 스레드에서 cycle_done 오면 mission.on_omx_done 호출됨.
    for omx in omx_connections.values():
        omx.on_cycle_done = mission.on_omx_done

    # ── 미션 실행 스레드 ──
    threading.Thread(target=start_mission, args=(mission,), daemon=True).start()

    # ── 메인 루프 ──
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n안전하게 자원을 해제합니다...")
        for system in domain_systems:
            system['exc'].shutdown()
        for t in ros_threads:
            t.join()
        for system in domain_systems:
            system['node'].destroy_node()
            rclpy.shutdown(context=system['ctx'])


if __name__ == '__main__':
    main()
