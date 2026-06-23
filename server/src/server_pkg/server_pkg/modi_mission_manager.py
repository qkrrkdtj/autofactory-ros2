import asyncio
import threading
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import yaml
import os


class MissionManager:
    def __init__(self, server_node, robot1_node, robot2_node,
                 omx_connections=None):
        self.server_node = server_node
        self.robot1_client = ActionClient(robot1_node, NavigateToPose, 'navigate_to_pose')
        self.robot2_client = ActionClient(robot2_node, NavigateToPose, 'navigate_to_pose')

        # ── OMX 연결 (omx_link.OmxConnection 객체들) ──
        # {'omx1': OmxConnection, 'omx2': OmxConnection}
        self.omx_connections = omx_connections or {}

        # ── 웨이포인트 <-> OMX 매핑 ──
        # A(적재) = omx1, C(분류) = omx2
        self.WP_TO_OMX = {'A': 'omx1', 'C': 'omx2'}

        # ── 자리(A/C)별 "현재 진행 중인 OMX 작업 완료 대기표(Future)" ──
        # 도착할 때마다 새 Future를 깔고, OMX 완료 콜백이 이걸 완료시킨다.
        self._omx_pending = {'A': None, 'C': None}

        # ── 자리별 출발 목표 횟수 ──
        # 지금: 동작 N회 완료하면 출발.
        # 나중: 이 값 대신 YOLO 박스 수 판정으로 교체 예정 (A:박스3개, C:박스0개).
        self.DEPART_TARGET = {'A': 3, 'C': 3}

        yaml_path = os.path.join(os.path.dirname(__file__), 'waypoints.yaml')

        with open(yaml_path, 'r') as f:
            raw = yaml.safe_load(f)

        self.WAYPOINTS = {k: tuple(v) for k, v in raw.items()}
        
        print("=== 로드된 웨이포인트 ===")
        for name, (x, y, z, w) in self.WAYPOINTS.items():
            print(f"  {name}: x={x:.4f}, y={y:.4f}, z={z:.4f}, w={w:.4f}")
        print("========================")
        
        self.STEPS = ['A', 'B', 'C', 'D', 'E']

        # ── [수정] 로봇의 현재 위치와 이동 목표(예약)를 분리 ──
        self.robot_current = {
            'robot1': None,
            'robot2': None,
        }
        self.robot_target = {
            'robot1': None,
            'robot2': None,
        }

        # ── 각 로봇의 현재 스텝 인덱스 ──
        self.robot_step = {
            'robot1': 0,
            'robot2': 0,
        }

        # ── robot2 최초 출발 여부 ──
        self.robot2_first_run = True

        # ── asyncio 루프 핸들 (run에서 초기화) ──
        # 다른 스레드(OMX 수신 / Flask / 키보드)에서 루프로 안전하게 넘어올 때 사용
        self._loop = None

    # ──────────────────────────────────────────────
    # 외부에서 호출하는 공개 메서드
    #   - OMX 완료 콜백 (omx_link 의 on_cycle_done 에서 호출)
    #   - Flask / 키보드 수동 오버라이드
    # 어느 쪽이든 "그 자리에서 대기 중인 Future를 완료시킨다"로 통일
    # ──────────────────────────────────────────────
    def signal_resume(self, target_wp):
        """수동 오버라이드: 사람이 Flask/키보드로 A 또는 C 대기를 강제 해제."""
        if self._loop is None:
            self.server_node.get_logger().error('[오류] 아직 미션 루프가 초기화되지 않았습니다.')
            return
        if target_wp not in self._omx_pending:
            self.server_node.get_logger().error(f'[오류] {target_wp}는 유효한 대기 지점이 아닙니다.')
            return
        self._loop.call_soon_threadsafe(self._resolve_pending, target_wp, True)
        self.server_node.get_logger().info(f'[수동신호] {target_wp} 대기 해제 명령 수신!')

    def on_omx_done(self, omx_id, success):
        """OMX 완료 콜백 (omx_link 수신 스레드에서 호출됨).

        omx_id(omx1/omx2)를 웨이포인트(A/C)로 되돌려서 해당 Future를 완료시킨다.
        반드시 call_soon_threadsafe 로 asyncio 루프에 넘긴다.
        """
        if self._loop is None:
            return
        wp = None
        for w, oid in self.WP_TO_OMX.items():
            if oid == omx_id:
                wp = w
                break
        if wp is None:
            return
        self._loop.call_soon_threadsafe(self._resolve_pending, wp, success)

    def _resolve_pending(self, target_wp, success):
        """[asyncio 루프 안에서 실행됨] 그 자리에서 대기 중인 Future를 완료시킨다."""
        fut = self._omx_pending.get(target_wp)
        if fut is not None and not fut.done():
            fut.set_result(success)
        # 완료시킨 Future는 move_robot 쪽에서 None 으로 정리함

    # ──────────────────────────────────────────────
    # 내부 헬퍼
    # ──────────────────────────────────────────────
    def is_waypoint_occupied(self, waypoint, robot_id):
        """해당 웨이포인트에 다른 로봇이 있거나, 그곳으로 이동 중이면 True"""
        for rid in self.robot_current.keys():
            if rid != robot_id:
                if self.robot_current[rid] == waypoint or self.robot_target[rid] == waypoint:
                    return True
        return False

    async def _check_departure(self, wp_name, done_count) -> bool:
        """출발 판정 슬롯.

        지금: 동작 완료 횟수가 자리별 목표(DEPART_TARGET)에 도달하면 True.
        나중: 이 함수 내부를 YOLO 추론 요청/결과 판정으로 교체 예정.
              (예: OMX에 추론 요청 → 박스 수 받아서 A는 ==3, C는 ==0 검사)
              교체 시 move_robot / _run_omx_and_wait 는 건드릴 필요 없음.
        """
        target = self.DEPART_TARGET.get(wp_name, 1)
        return done_count >= target

    async def _run_one_cycle(self, robot_id, wp_name) -> bool:
        """OMX 정책 1회 실행 요청 + cycle_done 대기. 완료 success 반환.

        - 새 Future를 깔고 run_policy() 를 쏜 뒤, 완료 콜백이 그 Future를
          완료시킬 때까지 await.
        - 요청 전송 자체가 실패(미연결/busy)하면 무한대기 방지 위해 True 처리하고 통과.
        """
        omx_id = self.WP_TO_OMX[wp_name]
        omx = self.omx_connections.get(omx_id)

        fut = self._loop.create_future()
        self._omx_pending[wp_name] = fut

        if omx is None:
            self.server_node.get_logger().warn(
                f'[{robot_id}] {wp_name}: OMX({omx_id}) 연결 없음 — 이번 동작 건너뜀'
            )
            self._omx_pending[wp_name] = None
            return True

        sent = omx.run_policy()
        if not sent:
            self.server_node.get_logger().error(
                f'[{robot_id}] {wp_name}: OMX({omx_id}) 요청 전송 실패 — 이번 동작 건너뜀'
            )
            self._omx_pending[wp_name] = None
            return True

        success = await fut
        self._omx_pending[wp_name] = None
        return success

    async def _run_omx_and_wait(self, robot_id, wp_name):
        """A/C 도착 시: 출발 판정이 충족될 때까지 OMX 정책을 반복 실행.

        - 도착한 순간 그 자리엔 이 로봇 한 대뿐 (점유 제어가 보장).
        - 한 사이클(run_policy → cycle_done) 끝날 때마다 _check_departure 로 판정.
        - 지금은 자리별 목표 횟수(DEPART_TARGET)만큼 반복하면 출발.
        - 다음 사이클은 이전 cycle_done 받은 직후 바로 발사 (간격 없음).
        """
        self.server_node.get_logger().warn(
            f'[{robot_id}] {wp_name} 도착 — OMX 작업 시작 (목표 {self.DEPART_TARGET.get(wp_name)}회)'
        )

        done_count = 0
        while True:
            success = await self._run_one_cycle(robot_id, wp_name)
            done_count += 1
            self.server_node.get_logger().info(
                f'[{robot_id}] {wp_name} 동작 {done_count}회 완료 (success={success})'
            )

            if await self._check_departure(wp_name, done_count):
                break

        self.server_node.get_logger().info(
            f'[{robot_id}] {wp_name} 출발 조건 충족 ({done_count}회 완료) — 출발!'
        )

    async def move_robot(self, client, robot_id):
        # ── robot2 전용: 최초 출발 시 E' → E 경유 ──
        if robot_id == 'robot2' and self.robot2_first_run:
            self.robot2_first_run = False

            # E' 이동 예약 -> 출발 -> 도착 처리
            self.robot_target[robot_id] = "E'"
            await self.send_and_wait(client, *self.WAYPOINTS["E'"], label=f"{robot_id}→E'")
            self.robot_current[robot_id] = "E'"
            self.robot_target[robot_id] = None

            # E 이동 예약 -> 출발 -> 도착 처리
            self.robot_target[robot_id] = 'E'
            await self.send_and_wait(client, *self.WAYPOINTS['E'], label=f'{robot_id}→E')
            self.robot_current[robot_id] = 'E'
            self.robot_target[robot_id] = None

            self.robot_step[robot_id] = 0

        # ── 공통 순환 루프 ──
        while True:
            next_step_idx = self.robot_step[robot_id] % len(self.STEPS)
            next_wp = self.STEPS[next_step_idx]

            # ── 1. 목적지 점유 대기 ──
            while self.is_waypoint_occupied(next_wp, robot_id):
                self.server_node.get_logger().warn(
                    f'[{robot_id}] {next_wp} 점유됨(또는 다른 로봇이 접근 중)... 대기'
                )
                await asyncio.sleep(0.5)

            # ── 2. [핵심] 출발 직전 목적지 점유(예약) ──
            self.robot_target[robot_id] = next_wp

            # ── 3. 출발 (이때 current와 target 모두 점유 상태) ──
            await self.send_and_wait(client, *self.WAYPOINTS[next_wp], label=f'{robot_id}→{next_wp}')

            # ── 4. 도착 후 상태 갱신 (이전 자리 반납) ──
            self.robot_current[robot_id] = next_wp
            self.robot_target[robot_id] = None

            # ── [조건 1] 도착 직후 1초 대기 ──
            self.server_node.get_logger().info(f'[{robot_id}] {next_wp} 도착 — 1초 대기')
            await asyncio.sleep(1.0)

            # ── [조건 2] B 도착 시 C 또는 D 점유 확인 ──
            if next_wp == 'B':
                while self.is_waypoint_occupied('C', robot_id) or self.is_waypoint_occupied('D', robot_id):
                    self.server_node.get_logger().warn(
                        f'[{robot_id}] B 대기 — C 또는 D가 점유되어 있습니다.'
                    )
                    await asyncio.sleep(0.5)
                self.server_node.get_logger().info(f'[{robot_id}] C/D 확보 확인 — B 통과!')

            # ── [조건 3] A 또는 C 도착 시 OMX 정책 실행 + 완료 대기 ──
            if next_wp in ['A', 'C']:
                await self._run_omx_and_wait(robot_id, next_wp)

            # ── 스텝 증가 ──
            self.robot_step[robot_id] += 1

    # ──────────────────────────────────────────────
    # 진입점
    # ──────────────────────────────────────────────
    async def run(self):
        self._loop = asyncio.get_running_loop()

        await asyncio.gather(
            self.move_robot(self.robot1_client, 'robot1'),
            self.move_robot(self.robot2_client, 'robot2'),
        )

    # ──────────────────────────────────────────────
    # Nav2 헬퍼
    # ──────────────────────────────────────────────
    def make_goal(self, x, y, oz, ow):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = oz
        goal.pose.pose.orientation.w = ow
        return goal

    async def send_and_wait(self, client, x, y, oz, ow, label=''):
        self.server_node.get_logger().info(f'[{label}] 출발!')

        future = client.send_goal_async(self.make_goal(x, y, oz, ow))
        while not future.done():
            await asyncio.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.server_node.get_logger().error(f'[{label}] goal 거절됨!')
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            await asyncio.sleep(0.1)

        self.server_node.get_logger().info(f'[{label}] 도착!')
        return True

# ──────────────────────────────────────────────────────────
# start_mission: bridge.py의 threading.Thread에서 호출
# ──────────────────────────────────────────────────────────
def start_mission(mission: MissionManager):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(mission.run())