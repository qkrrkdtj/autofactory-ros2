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
from server_pkg.conveyor_sensor import ConveyorSensorLink
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
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.active_client_goal_handle = None
        return result_wrapper.result

    def apply_alignment_offset(self, fwd_cm, lat_cm):
        """정렬 보정량(cm, +앞/+왼)만큼 차량을 '순수 평행이동'.
        회전1 → 직진 → 회전2(원복)로 분해해 최종 heading 은 유지.
        성공/미소(스킵)=True, 과다·실패·pause=False."""
        fwd, lat = fwd_cm / 100.0, lat_cm / 100.0
        dist = math.hypot(fwd, lat)
        if dist < 0.003:                       # 3mm 미만 → 이동 불필요
            return True
        if dist > 0.30:                        # 과다 이동 방어
            self.server_node.get_logger().warn(
                f"[{self.robot_id}] 정렬 이동 과다({dist*100:.1f}cm) 중단")
            return False

        LAT_TOL = 0.008                        # OMX aligned 판정 lat_tol(1.0cm)보다 작게 유지
        if abs(lat) < LAT_TOL:
            turn1 = 0.0
            drive = fwd
        else:
            ang = math.atan2(lat, fwd)
            if abs(ang) <= math.pi / 2:
                turn1 = ang
                drive = dist
            else:
                turn1 = math.atan2(math.sin(ang - math.pi), math.cos(ang - math.pi))
                drive = -dist
        self.server_node.get_logger().info(
            f"[{self.robot_id}] 정렬이동: 회전1={math.degrees(turn1):+.1f}° "
            f"{'전진' if drive>=0 else '후진'}={abs(drive)*100:.1f}cm "
            f"회전2=원위치복귀")

        # 출발 헤딩을 기억해두고, 회전2는 상대(-turn1)가 아니라
        # '출발 헤딩으로 절대 복귀'로 계산 → 회전/직진 중 누적된 드리프트까지 상쇄
        od0 = self.shared_odom_state.get(self.robot_id)
        if od0 is None:
            self.server_node.get_logger().warn(f"[{self.robot_id}] odom 없음")
            return False
        yaw_start = od0['theta']

        if not self._odom_rotate(turn1): return False
        if not self._odom_drive(drive):  return False

        od1 = self.shared_odom_state.get(self.robot_id)
        if od1 is None:
            self.server_node.get_logger().warn(f"[{self.robot_id}] odom 없음")
            return False
        turn2 = math.atan2(math.sin(yaw_start - od1['theta']),
                           math.cos(yaw_start - od1['theta']))
        if not self._odom_rotate(turn2): return False
        self.cmd_vel_pub.publish(self._make_twist_stamped())
        return not self._wants_pause
    
    def apply_alignment_yaw(self, yaw_deg):
        """트레이 yaw 오차(도)만큼 차량을 제자리 회전해 되돌린다.
        부호: 차량이 CW로 틀어지면 yaw_deg>0 → +각도 CCW 회전으로 원복.
        (measure 쪽에서 확정한 부호 규약: _odom_rotate(+)=CCW)
        미소(스킵)=True, 과다·실패·pause=False."""
        ang = math.radians(yaw_deg)
        if abs(yaw_deg) < 1.0:            # 1도 미만 → 회전 불필요
            return True
        if abs(yaw_deg) > 30.0:           # 과다 회전 방어
            self.server_node.get_logger().warn(
                f"[{self.robot_id}] 정렬 회전 과다({yaw_deg:.1f}도) 중단")
            return False

        self.server_node.get_logger().info(
            f"[{self.robot_id}] 정렬회전: {yaw_deg:+.1f}도 (CCW+)")
        if not self._odom_rotate(ang):
            return False
        self.cmd_vel_pub.publish(self._make_twist_stamped())
        return not self._wants_pause

    # ── odom 기준 제자리 회전 ──
    def _odom_rotate(self, angle):
        """angle 라디안만큼 제자리 회전. 부호=방향(+반시계/-시계). 최단방향은 호출부에서 결정."""
        # 스킵 임계는 미션 매니저 YAW_TOL(1.5도≈0.026rad)보다 반드시 작게 —
        # 아니면 회전 지시가 조용히 무시되는 데드존이 생겨 정렬이 수렴하지 않는다
        if abs(angle) < 0.005:
            return True
        od0 = self.shared_odom_state.get(self.robot_id)
        if od0 is None:
            self.server_node.get_logger().warn(f"[{self.robot_id}] odom 없음")
            return False
        yaw0 = od0['theta']

        # 누적 회전량 추적 (wrap 방지)
        prev = yaw0
        accumulated = 0.0
        Kp, W_MAX, W_MIN = 1.5, 0.5, 0.08
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
            if abs(rem) < 0.004:
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
        # 스킵/정지 임계는 OMX aligned 판정 밴드(앞뒤 0.4cm)보다 반드시 작게 —
        # 아니면 0.4~0.5cm 오차 구간에서 이동 지시가 무시되어 정렬이 수렴하지 않는다
        if abs(distance) < 0.002:
            return True
        od0 = self.shared_odom_state.get(self.robot_id)
        if od0 is None:
            self.server_node.get_logger().warn(f"[{self.robot_id}] odom 없음")
            return False
        x0, y0, yaw0 = od0['x'], od0['y'], od0['theta']
        sign = 1.0 if distance >= 0 else -1.0
        target = abs(distance)
        Kp, V_MAX, V_MIN = 1.0, 0.08, 0.02
        Kp_yaw, W_HOLD_MAX = 1.2, 0.3          # 직진 중 헤딩 유지(드리프트 보정)
        start, dt = time.time(), 0.05
        while time.time()-start < 12.0 and not self._wants_pause:
            od = self.shared_odom_state.get(self.robot_id)
            if od is None: time.sleep(dt); continue
            mdx, mdy = od['x']-x0, od['y']-y0
            moved = abs(math.cos(yaw0)*mdx + math.sin(yaw0)*mdy)
            rem = target - moved
            if rem < 0.002:
                break
            v = Kp*rem
            if v < V_MIN: v = V_MIN
            v = min(V_MAX, v) * sign          # 부호로 전/후진
            yaw_err = math.atan2(math.sin(yaw0-od['theta']),
                                 math.cos(yaw0-od['theta']))
            w = max(-W_HOLD_MAX, min(W_HOLD_MAX, Kp_yaw*yaw_err))
            self.cmd_vel_pub.publish(self._make_twist_stamped(v, w))
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
        'started': False,
        'current_wp': {},
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
        target=lambda: flask_app.run(host='0.0.0.0', port=5000, debug=False,
                                 use_reloader=False, threaded=True),
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

    # 컨베이어 물건 감지 센서 (라즈베리파이 → UDP) 수신 시작
    conveyor_sensor = ConveyorSensorLink()
    conveyor_sensor.start()

    try:
        print("\n⏳ 대시보드에서 [미션 시작] 버튼을 눌러주세요...")
        while not start_event.is_set():
            time.sleep(0.1)
        print("✅ 시작 신호 수신 — 미션을 시작합니다!")

        # active_proxies 를 robot_id 키로 매핑
        proxies = {f'robot{p.robot_id}': p for p in active_proxies}

        mission = MissionManager(
            server['node'], robot1['node'], robot2['node'],
            shared_state=shared_state,
            omx_connections=omx_connections,
            proxies=proxies,
            conveyor_sensor=conveyor_sensor,
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