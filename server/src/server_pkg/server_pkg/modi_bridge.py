import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
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
    def __init__(self, server_node, robot_node, server_action_name, 
                 robot_action_name, robot_id, shared_state, 
                 shared_pose_state,shared_odom_state):
        self.server_node = server_node
        self.robot_node = robot_node
        self.robot_id = robot_id
        self.shared_state = shared_state
        self.shared_pose_state = shared_pose_state
        self.shared_odom_state = shared_odom_state 
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
        if self.shared_state.get('current_wp') not in ['A', 'C']:
            return
        if self.last_goal is None:
            return

        # ── ① amcl로 목표 위치를 '한 번만' 측정 ──
        pose = self.shared_pose_state.get(self.robot_id)
        if pose is None:
            return
        target_x = self.last_goal.pose.pose.position.x
        target_y = self.last_goal.pose.pose.position.y
        q = self.last_goal.pose.pose.orientation
        target_yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

        dx, dy = target_x-pose['x'], target_y-pose['y']
        c, s = math.cos(pose['theta']), math.sin(pose['theta'])
        goal_fwd =  c*dx + s*dy      # +앞 / -뒤
        goal_lat = -s*dx + c*dy      # +왼 / -오
        goal_final_yaw = math.atan2(math.sin(target_yaw-pose['theta']),
                                    math.cos(target_yaw-pose['theta']))

        self.server_node.get_logger().info(
            f"[{self.robot_id}] 보정측정 fwd={goal_fwd*100:.1f}cm lat={goal_lat*100:.1f}cm")

        if math.hypot(goal_fwd, goal_lat) > 0.30:
            self.server_node.get_logger().warn(f"[{self.robot_id}] 거리 과다 중단")
            return

        # ── ② 이동 계획 수립 ──
        LAT_TOL = 0.02
        if abs(goal_lat) < LAT_TOL:
            turn1 = 0.0
            drive = goal_fwd              # +전진 / -후진
            turn2 = goal_final_yaw
        else:
            ang_to_goal = math.atan2(goal_lat, goal_fwd)
            if abs(ang_to_goal) <= math.pi/2:
                turn1 = ang_to_goal
                drive = math.hypot(goal_fwd, goal_lat)     # 전진
            else:
                turn1 = math.atan2(math.sin(ang_to_goal - math.pi),
                                   math.cos(ang_to_goal - math.pi))
                drive = -math.hypot(goal_fwd, goal_lat)    # 후진
            turn2 = math.atan2(math.sin(goal_final_yaw - turn1),
                               math.cos(goal_final_yaw - turn1))

        self.server_node.get_logger().info(
            f"[{self.robot_id}] 계획: 회전1={math.degrees(turn1):+.1f}° "
            f"{'전진' if drive>=0 else '후진'}={abs(drive)*100:.1f}cm "
            f"회전2={math.degrees(turn2):+.1f}°")

        # ── ③ odom으로 순차 실행 ──
        if not self._odom_rotate(turn1):  return
        if not self._odom_drive(drive):   return
        if not self._odom_rotate(turn2):  return

        self.server_node.get_logger().info(f"[{self.robot_id}] 정밀보정 완료 ✅")
        self.cmd_vel_pub.publish(self._make_twist_stamped())


    # ── odom 기준 제자리 회전 ──
    def _odom_rotate(self, angle):
        """angle 라디안만큼 제자리 회전. 부호=방향(+반시계/-시계). 최단방향은 호출부에서 결정."""
        if abs(angle) < 0.02:
            return True
        od0 = self.shared_odom_state.get(self.robot_id)
        if od0 is None:
            self.server_node.get_logger().warn(f"[{self.robot_id}] odom 없음")
            return False
        yaw0 = od0['theta']

        # 누적 회전량 추적 (wrap 방지)
        prev = yaw0
        accumulated = 0.0
        Kp, W_MAX, W_MIN = 1.5, 0.5, 0.12
        start, dt = time.time(), 0.05

        while time.time()-start < 10.0 and not self._wants_pause:
            od = self.shared_odom_state.get(self.robot_id)
            if od is None:
                time.sleep(dt); continue
            # 매 스텝 증분을 누적 (±π 경계 안전)
            d = math.atan2(math.sin(od['theta']-prev), math.cos(od['theta']-prev))
            accumulated += d
            prev = od['theta']

            rem = angle - accumulated      # 남은 회전량 (부호 유지)
            if abs(rem) < 0.008:
                break
            w = Kp*rem
            if abs(w) < W_MIN: w = math.copysign(W_MIN, w)
            w = max(-W_MAX, min(W_MAX, w))
            self.cmd_vel_pub.publish(self._make_twist_stamped(0.0, w))
            time.sleep(dt)

        self.cmd_vel_pub.publish(self._make_twist_stamped())
        time.sleep(0.2)
        return not self._wants_pause


    # ── odom 기준 직진(부호로 전/후진) ──
    def _odom_drive(self, distance):
        if abs(distance) < 0.01:
            return True
        od0 = self.shared_odom_state.get(self.robot_id)
        if od0 is None:
            self.server_node.get_logger().warn(f"[{self.robot_id}] odom 없음")
            return False
        x0, y0, yaw0 = od0['x'], od0['y'], od0['theta']
        sign = 1.0 if distance >= 0 else -1.0
        target = abs(distance)
        Kp, V_MAX, V_MIN = 1.0, 0.08, 0.02
        start, dt = time.time(), 0.05
        while time.time()-start < 12.0 and not self._wants_pause:
            od = self.shared_odom_state.get(self.robot_id)
            if od is None: time.sleep(dt); continue
            mdx, mdy = od['x']-x0, od['y']-y0
            moved = abs(math.cos(yaw0)*mdx + math.sin(yaw0)*mdy)
            rem = target - moved
            if rem < 0.005:
                break
            v = Kp*rem
            if v < V_MIN: v = V_MIN
            v = min(V_MAX, v) * sign          # 부호로 전/후진
            self.cmd_vel_pub.publish(self._make_twist_stamped(v, 0.0))
            time.sleep(dt)
        self.cmd_vel_pub.publish(self._make_twist_stamped())
        time.sleep(0.2)
        return not self._wants_pause

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

class OdomTracker:
    def __init__(self, robot_node, robot_id, shared_odom_state):
        self.robot_node = robot_node
        self.robot_id = robot_id
        self.shared_odom_state = shared_odom_state
        self.sub = self.robot_node.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                         1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.shared_odom_state[self.robot_id] = {'x': p.x, 'y': p.y, 'theta': yaw}
        

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
    shared_odom_state    = {}

    active_proxies   = []
    pose_trackers    = []
    odom_trackers    = [] 
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
            shared_odom_state=shared_odom_state,
        )
        active_proxies.append(proxy)

        tracker_p = PoseTracker(
            robot_node=target_robot['node'], robot_id=robot_id,
            shared_pose_state=shared_pose_state,
        )
        pose_trackers.append(tracker_p)

        tracker_o = OdomTracker(                       # ← 추가
            robot_node=target_robot['node'], robot_id=robot_id,
            shared_odom_state=shared_odom_state,
        )
        odom_trackers.append(tracker_o)
        
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
        print("✅ Waffle 1 준비 완료")

    def on_robot2_ready():
        robot_ready_events[2].set()
        print("✅ Waffle 2 준비 완료")

    try:
        launch_all(on_robot1_ready=on_robot1_ready, on_robot2_ready=on_robot2_ready)
    except TypeError:
        def on_all_ready():
            robot_ready_events[1].set()
            robot_ready_events[2].set()
            print("✅ 전체 확인 완료")
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
            omx.on_depart_check = mission.on_omx_depart_check

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