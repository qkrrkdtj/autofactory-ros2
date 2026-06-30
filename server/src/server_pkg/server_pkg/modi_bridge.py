import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from server_pkg.modi_mission_manager import MissionManager, start_mission
from server_pkg.modi_flask import create_flask_app
from server_pkg.omx_link import OmxConnection
from server_pkg.ssh_launcher import launch_all, kill_all
import threading
import math
import time
import sys
import asyncio

from nav2_msgs.action import NavigateToPose


# ==========================================
# 1. 액션 프록시 (스레드 씹힘 방지 & 일시정지 적용)
# ==========================================
class ActionProxy:
    def __init__(self, server_node, robot_node, server_action_name, robot_action_name, robot_id, shared_state, shared_pose_state):
        self.server_node = server_node
        self.robot_node = robot_node
        self.robot_id = robot_id
        self.shared_state = shared_state
        self.shared_pose_state = shared_pose_state
        self.active_client_goal_handle = None
        self._wants_pause = False
        self.last_goal = None
        self.pause_requested = False

        # 실제 로봇(turtlebot3_node)이 TwistStamped를 구독하므로 타입을 맞춘다
        self.cmd_vel_pub = self.robot_node.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.timer = self.robot_node.create_timer(0.1, self._timer_callback)
        self.client = ActionClient(self.robot_node, NavigateToPose, robot_action_name)
        self.server = ActionServer(self.server_node, NavigateToPose, server_action_name, execute_callback=self.execute_callback)

    # ── TwistStamped 메시지 생성 헬퍼 ──
    def _make_twist_stamped(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.robot_node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    def _timer_callback(self):
        if self._wants_pause:
            self.cmd_vel_pub.publish(self._make_twist_stamped())

    async def execute_callback(self, goal_handle):
        if self.robot_id not in self.shared_state.get('selected', []):
            goal_handle.succeed()
            return NavigateToPose.Result()

        self.last_goal = goal_handle.request

        self.active_client_goal_handle = (
            await self.client.send_goal_async(self.last_goal)
        )

        result_wrapper = await (
            self.active_client_goal_handle.get_result_async()
        )

        status = result_wrapper.status

        self.server_node.get_logger().info(
            f"[{self.robot_id}] status={status}"
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._run_correction_blocking()
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.active_client_goal_handle = None
        return result_wrapper.result

    # ──────────────────────────────────────────────
    # P보정을 별도 스레드에서 돌리고 완료까지 대기
    # (asyncio.sleep을 콜백 스레드에서 쓰면 'no running event loop' 발생 →
    #  threading.Event.wait로 대기)
    # ──────────────────────────────────────────────
    def _run_correction_blocking(self):
        done = threading.Event()

        def worker():
            try:
                self._p_correction_sync()
            finally:
                done.set()

        threading.Thread(target=worker, daemon=True).start()
        done.wait()

    def _p_correction_sync(self):
        # ── 정밀 정차가 필요한 포인트(A, C)에서만 보정 ──
        if self.shared_state.get('current_wp') not in ['A', 'C']:
            return
        if self.last_goal is None:
            return

        target_x = self.last_goal.pose.pose.position.x
        target_y = self.last_goal.pose.pose.position.y
        q = self.last_goal.pose.pose.orientation
        target_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        Kp_lin = 0.3
        Kp_ang = 0.5
        Kd_ang = 0.15
        
        THRESHOLD_XY  = 0.03   # 3cm
        THRESHOLD_YAW = 0.06   # 3.5도
        TIMEOUT = 15.0
        dt = 0.1

        start_time = time.time()
        last_log_dist = None
        last_log_yaw = None
        
        while True:
            if time.time() - start_time > TIMEOUT:
                self.server_node.get_logger().warn(f"[{self.robot_id}] P correction timeout")
                break

            if self._wants_pause:
                break

            pose = self.shared_pose_state.get(self.robot_id)
            if pose is None:
                time.sleep(dt)
                continue

            dx = target_x - pose['x']
            dy = target_y - pose['y']
            dist = math.sqrt(dx**2 + dy**2)
            yaw_err = math.atan2(
                math.sin(target_yaw - pose['theta']),
                math.cos(target_yaw - pose['theta'])
            )

            yaw_deg = math.degrees(yaw_err)
            if (last_log_dist is None
                    or abs(dist - last_log_dist) >= 0.005       # 5mm 이상 변하면
                    or abs(yaw_deg - last_log_yaw) >= 1.0):      # 1도 이상 변하면
                self.server_node.get_logger().info(
                    f"[{self.robot_id}] P보정 중 — dist={dist:.3f}m  yaw={yaw_deg:.1f}°"
                )
                last_log_dist = dist
                last_log_yaw = yaw_deg

            if dist < THRESHOLD_XY and abs(yaw_err) < THRESHOLD_YAW:
                self.server_node.get_logger().info(f"[{self.robot_id}] P correction 완료 ✅")
                break

            angle_to_goal = math.atan2(dy, dx)
            heading_err = math.atan2(
                math.sin(angle_to_goal - pose['theta']),
                math.cos(angle_to_goal - pose['theta'])
            )

            lin_x = 0.0
            ang_z = 0.0
            if dist > THRESHOLD_XY:
                lin_x = max(-0.10, min(0.10, Kp_lin * dist))
                ang_z = max(-0.5,  min(0.5,  Kp_ang * heading_err))
            else:
                # PD 제어
                d_term = 0.0
                if prev_yaw_err is not None:
                    d_term = Kd_ang * (yaw_err - prev_yaw_err) / dt
                az = Kp_ang * yaw_err + d_term

                if abs(yaw_err) < THRESHOLD_YAW:
                    az = 0.0
                elif abs(az) < 0.10:
                    az = 0.10 if az > 0 else -0.10
                ang_z = max(-0.3, min(0.3, az))
            
            self.cmd_vel_pub.publish(self._make_twist_stamped(lin_x, ang_z))
            prev_yaw_err = yaw_err
            time.sleep(dt)

        self.cmd_vel_pub.publish(self._make_twist_stamped())  # 정지

    def cancel_current_goal(self):
        if self.active_client_goal_handle:
            self.active_client_goal_handle.cancel_goal_async()
        self.cmd_vel_pub.publish(self._make_twist_stamped())
        self._wants_pause = True


# ==========================================
# 2. 실시간 위치 추적 (Pose Tracker)
# ==========================================
class PoseTracker:
    def __init__(self, robot_node, robot_id, shared_pose_state):
        self.robot_node = robot_node
        self.robot_id = robot_id
        self.shared_pose_state = shared_pose_state

        self.sub = self.robot_node.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.shared_pose_state[self.robot_id] = {'x': x, 'y': y, 'theta': yaw}


# ==========================================
# 3. 배터리 추적 (Battery Tracker)
# ==========================================
class BatteryTracker:
    def __init__(self, robot_node, robot_id, shared_battery_state):
        self.robot_node = robot_node
        self.robot_id = robot_id
        self.shared_battery_state = shared_battery_state

        self.sub = self.robot_node.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            10
        )

    def battery_callback(self, msg):
        # percentage가 이미 0~100 스케일로 들어오므로 *100 하지 않음
        raw = msg.percentage   # 실제 0~100
        mapped = (raw - 30) / 70 * 100
        mapped = max(0.0, min(100.0, mapped))   # 0~100 범위로 고정
        self.shared_battery_state[self.robot_id] = round(mapped, 1)


# ==========================================
# 4. 키보드 입력 스레드
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
    server = setup_domain(30, 'server_bridge', args)
    robot1 = setup_domain(31, 'robot1_bridge', args)
    robot2 = setup_domain(32, 'robot2_bridge', args)
    robot_list = [robot1, robot2]

    shared_state = {
        'selected': [1, 2],
        'paused': False,
        'started': False
    }
    shared_pose_state    = {}  # { robot_id: {x, y, theta} }
    shared_battery_state = {}  # { robot_id: 퍼센트(float) }

    active_proxies   = []
    pose_trackers    = []
    battery_trackers = []

    print("==================================================")
    print(" 🚀 모듈형 다중 로봇 액션 프록시 & 관제 브릿지 가동 🚀 ")
    print("==================================================")

    for i, target_robot in enumerate(robot_list):
        robot_id = i + 1

        proxy = ActionProxy(
            server_node=server['node'], robot_node=target_robot['node'],
            server_action_name=f'robot{robot_id}/navigate_to_pose',
            robot_action_name='navigate_to_pose',
            robot_id=robot_id,
            shared_state=shared_state,
            shared_pose_state=shared_pose_state,
        )
        active_proxies.append(proxy)

        tracker_p = PoseTracker(
            robot_node=target_robot['node'], robot_id=robot_id,
            shared_pose_state=shared_pose_state,
        )
        pose_trackers.append(tracker_p)

        tracker_b = BatteryTracker(
            robot_node=target_robot['node'], robot_id=robot_id,
            shared_battery_state=shared_battery_state,
        )
        battery_trackers.append(tracker_b)

    start_event        = threading.Event()
    mission_holder     = {'mission': None}
    robot_ready_events = {1: threading.Event(), 2: threading.Event()}

    flask_app = create_flask_app(
        start_event, mission_holder, robot_ready_events,
        shared_state, active_proxies,
        shared_pose_state, shared_battery_state,
    )
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