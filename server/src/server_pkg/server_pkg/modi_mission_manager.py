import asyncio
import threading
import queue
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import yaml
import os


class MissionManager:
    def __init__(self, server_node, robot1_node, robot2_node, shared_state, omx_connections=None):
        self.server_node = server_node
        self.shared_state = shared_state
        self.robot1_client = ActionClient(server_node, NavigateToPose, 'robot1/navigate_to_pose')
        self.robot2_client = ActionClient(server_node, NavigateToPose, 'robot2/navigate_to_pose')

        self.omx_connections = omx_connections or {}
        self.WP_TO_OMX = {'A': 'omx1', 'C': 'omx2'}
        # 정책 완료(cycle_done) 대기 전용 슬롯
        self._omx_pending = {'A': None, 'C': None}
        # 카메라 출발판정(depart_check) 응답 대기 전용 슬롯
        self._depart_pending = {'A': None, 'C': None}
        # 수동 해제(=OpenCV 판정 무시 강제출발) 전용 슬롯
        self._manual_pending = {'A': None, 'C': None}

        yaml_path = os.path.join(os.path.dirname(__file__), 'waypoints.yaml')
        with open(yaml_path, 'r') as f:
            raw = yaml.safe_load(f)

        self.WAYPOINTS = {k: tuple(v) for k, v in raw.items()}
        self.STEPS = ['A', 'B', 'C', 'D', 'E']

        self.robot_current = {'robot1': None, 'robot2': None}
        self.robot_target  = {'robot1': None, 'robot2': None}
        self.robot_step    = {'robot1': 0,    'robot2': 0}
        self.robot2_first_run = True
        self._loop = None
        self.log_queue = queue.Queue(maxsize=200)

    # ──────────────────────────────────────────────
    # 대시보드 로그 큐
    # ──────────────────────────────────────────────
    def push_log(self, msg: str, level: str = 'info'):
        try:
            self.log_queue.put_nowait({'msg': msg, 'level': level})
        except queue.Full:
            pass

    # ──────────────────────────────────────────────
    # 외부 공개 메서드
    # ──────────────────────────────────────────────
    def signal_resume(self, target_wp):
        """대시보드/키보드의 A·C 해제 버튼.
        = OpenCV 판정을 무시하고 그 자리에서 강제 출발시키는 오버라이드."""
        if self._loop is None:
            self.server_node.get_logger().error('[오류] 아직 미션 루프가 초기화되지 않았습니다.')
            return
        if target_wp not in self.WP_TO_OMX:
            self.server_node.get_logger().error(f'[오류] {target_wp}는 유효한 대기 지점이 아닙니다.')
            return
        if self._manual_pending.get(target_wp) is None:
            self.server_node.get_logger().warn(
                f'[수동신호] {target_wp}: 현재 대기 중인 작업이 없습니다 (로봇이 아직 도착 안 했거나 이미 출발).'
            )
            self.push_log(f'⚠ {target_wp} 신호: 현재 대기 중인 작업 없음', 'warn')
            return
        self._loop.call_soon_threadsafe(self._resolve_manual, target_wp)
        self.server_node.get_logger().info(f'[수동신호] {target_wp} 강제 출발 명령 수신!')
        self.push_log(f'🔓 {target_wp} 구역 수동 해제 (강제 출발)', 'success')

    def _omx_id_to_wp(self, omx_id):
        for w, oid in self.WP_TO_OMX.items():
            if oid == omx_id:
                return w
        return None

    def on_omx_done(self, omx_id, success):
        """OMX 정책 1회 완료(cycle_done) 콜백. OMX 수신 스레드에서 호출됨."""
        if self._loop is None:
            return
        wp = self._omx_id_to_wp(omx_id)
        if wp is None:
            return
        self._loop.call_soon_threadsafe(self._resolve_pending, wp, success)

    def on_omx_depart_check(self, omx_id, wp, depart_ok, counts, tray_found):
        """OMX 카메라 출발판정(depart_check) 응답 콜백. OMX 수신 스레드에서 호출됨."""
        if self._loop is None:
            return
        self._loop.call_soon_threadsafe(self._resolve_depart, wp, depart_ok)

    def _resolve_pending(self, target_wp, success):
        fut = self._omx_pending.get(target_wp)
        if fut is not None and not fut.done():
            fut.set_result(success)

    def _resolve_depart(self, target_wp, depart_ok):
        fut = self._depart_pending.get(target_wp)
        if fut is not None and not fut.done():
            fut.set_result(depart_ok)

    def _resolve_manual(self, target_wp):
        fut = self._manual_pending.get(target_wp)
        if fut is not None and not fut.done():
            fut.set_result(True)

    # ──────────────────────────────────────────────
    # 내부 헬퍼
    # ──────────────────────────────────────────────
    def is_waypoint_occupied(self, waypoint, robot_id):
        for rid in self.robot_current.keys():
            if rid != robot_id:
                if self.robot_current[rid] == waypoint or self.robot_target[rid] == waypoint:
                    return True
        return False

    async def wait_if_paused(self):
        while self.shared_state.get('paused', False):
            await asyncio.sleep(0.1)

    def _omx_available(self, wp_name) -> bool:
        """해당 지점 OMX가 실제로 연결되어 명령을 받을 수 있는 상태인지."""
        omx = self.omx_connections.get(self.WP_TO_OMX.get(wp_name))
        return omx is not None and getattr(omx, "connected", False)

    async def _check_departure(self, wp_name) -> bool:
        """OMX에 사진 촬영+카운트를 요청해 출발 여부를 판정받는다.
        - 카메라 OK  → True (자동 출발)
        - 미충족     → False (호출부가 다음 사이클 재추론)
        판정 주체는 OMX executor(config의 depart_target_count: A==3, C==0)."""
        if not self._omx_available(wp_name):
            # 연결 안 됨 → 판정 불가. 여기서 True를 주면 즉시 출발해버리므로
            # 판정은 하지 않고, 자동 통과 여부는 상위(_run_omx_and_wait)가 결정한다.
            return False

        omx = self.omx_connections[self.WP_TO_OMX[wp_name]]
        fut = self._loop.create_future()
        self._depart_pending[wp_name] = fut

        if not omx.check_departure(wp_name):
            self._depart_pending[wp_name] = None
            self.push_log(f'⚠ {wp_name} 출발검증 요청 전송 실패', 'warn')
            return False

        try:
            depart_ok = await asyncio.wait_for(fut, timeout=15.0)
        except asyncio.TimeoutError:
            self.push_log(f'⚠ {wp_name} 출발검증 응답 없음(timeout) — 재추론', 'warn')
            depart_ok = False
        finally:
            self._depart_pending[wp_name] = None

        return depart_ok

    async def _run_one_cycle(self, robot_id, wp_name) -> bool:
        """OMX 정책 1회 실행 요청 후 완료(cycle_done)까지 대기.
        연결이 안 돼 있으면 실행 자체가 불가하므로 None을 반환한다
        (상위에서 '자동 통과' 신호로 해석)."""
        omx_id = self.WP_TO_OMX[wp_name]
        omx = self.omx_connections.get(omx_id)

        if omx is None or not getattr(omx, "connected", False):
            self.server_node.get_logger().warn(
                f'[{robot_id}] {wp_name}: OMX({omx_id}) 연결 없음 — 이번 지점 건너뜀'
            )
            return None

        fut = self._loop.create_future()
        self._omx_pending[wp_name] = fut

        sent = omx.run_policy()
        if not sent:
            self.server_node.get_logger().error(
                f'[{robot_id}] {wp_name}: OMX({omx_id}) 요청 전송 실패 — 이번 지점 건너뜀'
            )
            self._omx_pending[wp_name] = None
            return None

        success = await fut
        self._omx_pending[wp_name] = None
        return success

    async def _run_omx_and_wait(self, robot_id, wp_name):
        rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'

        # ── OMX 미연결이면 이 지점 작업 전체를 자동으로 건너뛴다 ──
        if not self._omx_available(wp_name):
            omx_id = self.WP_TO_OMX.get(wp_name)
            self.push_log(f'⏭ {rname}: {wp_name} OMX({omx_id}) 미연결 — 작업 건너뛰고 통과', 'warn')
            self.server_node.get_logger().warn(
                f'[{robot_id}] {wp_name}: OMX({omx_id}) 미연결 — 자동 통과'
            )
            return

        self.push_log(f'🔧 {rname}: {wp_name} OMX 작업 시작 (카메라 출발판정)', 'info')
        self.server_node.get_logger().warn(
            f'[{robot_id}] {wp_name} 도착 — OMX 작업 시작 (카메라 출발판정)'
        )

        # ── 수동 해제(강제출발) 슬롯을 이 지점 작업 내내 열어둔다 ──
        # 검출이 계속 미충족이어도 사람이 대시보드 버튼으로 언제든 빼낼 수 있게.
        manual_fut = self._loop.create_future()
        self._manual_pending[wp_name] = manual_fut

        done_count = 0
        try:
            while True:
                # 강제출발(수동)이 이미 눌렸으면 즉시 출발
                if manual_fut.done():
                    self.push_log(f'🔓 {rname}: {wp_name} 수동 강제출발', 'success')
                    break

                success = await self._run_one_cycle(robot_id, wp_name)

                # 사이클 도중 미연결로 바뀌었으면(연결 끊김) 자동 통과
                if success is None:
                    self.push_log(f'⏭ {rname}: {wp_name} OMX 실행 불가 — 통과', 'warn')
                    break

                done_count += 1
                self.push_log(f'🔧 {rname}: {wp_name} 동작 {done_count}회 완료',
                              'success' if success else 'warn')
                self.server_node.get_logger().info(
                    f'[{robot_id}] {wp_name} 동작 {done_count}회 완료 (success={success})'
                )

                # 카메라 판정: 충족이면 자동 출발
                if await self._check_departure(wp_name):
                    self.push_log(f'📸 {rname}: {wp_name} 카메라 출발조건 충족 — 출발', 'success')
                    break

                # 미충족: 수동 강제출발이 그새 눌렸는지 확인, 아니면 다음 사이클 재추론
                if manual_fut.done():
                    self.push_log(f'🔓 {rname}: {wp_name} 수동 강제출발', 'success')
                    break

                self.push_log(
                    f'🔁 {rname}: {wp_name} 출발조건 미충족 — 재추론 (필요시 수동 해제 가능)', 'info'
                )
        finally:
            self._manual_pending[wp_name] = None

        self.server_node.get_logger().info(
            f'[{robot_id}] {wp_name} 출발 — ({done_count}회 완료)'
        )

    async def move_robot(self, client, robot_id):

        # ── robot2 전용: 최초 출발 시 E' → E 경유 ──
        if robot_id == 'robot2' and self.robot2_first_run:
            self.robot2_first_run = False

            self.robot_target[robot_id] = "E'"
            self.push_log("🤖 Waffle 2: Start → E' 이동 시작", 'info')
            self.server_node.get_logger().info(f'[{robot_id}] Start → E\'  이동 시작')
            while True:
                r = await self.send_and_wait(client, *self.WAYPOINTS["E'"], wp_name="E'", label=f"{robot_id}→E'")
                if r is None:
                    self.push_log("⏸ Waffle 2: E' 이동 취소 — 재개 대기", 'warn')
                    await self.wait_if_paused()
                    self.push_log("▶ Waffle 2: 재개 — E' 재이동", 'success')
                    continue
                break
            self.robot_current[robot_id] = "E'"
            self.robot_target[robot_id] = None
            self.push_log("✅ Waffle 2: E' 도착", 'success')
            self.server_node.get_logger().info(f'[{robot_id}] E\' 도착')

            self.robot_target[robot_id] = 'E'
            self.push_log("🤖 Waffle 2: E' → E 이동 시작", 'info')
            self.server_node.get_logger().info(f'[{robot_id}] E\' → E 이동 시작')
            while True:
                r = await self.send_and_wait(client, *self.WAYPOINTS['E'], wp_name='E', label=f'{robot_id}→E')
                if r is None:
                    self.push_log("⏸ Waffle 2: E 이동 취소 — 재개 대기", 'warn')
                    await self.wait_if_paused()
                    self.push_log("▶ Waffle 2: 재개 — E 재이동", 'success')
                    continue
                break
            self.robot_current[robot_id] = 'E'
            self.robot_target[robot_id] = None
            self.push_log("✅ Waffle 2: E 도착 — 순환 루프 시작", 'success')
            self.server_node.get_logger().info(f'[{robot_id}] E 도착 — 순환 루프 시작')

            self.robot_step[robot_id] = 0

        # ── 공통 순환 루프 ──
        while True:
            next_step_idx = self.robot_step[robot_id] % len(self.STEPS)
            next_wp = self.STEPS[next_step_idx]

            # ── 1. 목적지 점유 대기 ──
            while self.is_waypoint_occupied(next_wp, robot_id):
                await self.wait_if_paused()
                rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
                self.push_log(f'⏳ {rname}: {next_wp} 점유 대기 중...', 'warn')
                self.server_node.get_logger().warn(
                    f'[{robot_id}] {next_wp} 점유됨(또는 다른 로봇이 접근 중)... 대기'
                )
                await asyncio.sleep(5)

            # ── 2. 출발 직전 목적지 점유(예약) ──
            self.robot_target[robot_id] = next_wp

            await self.wait_if_paused()
            rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
            prev_wp = self.robot_current.get(robot_id) or 'Start'
            self.push_log(f'🤖 {rname}: {prev_wp} → {next_wp} 이동 시작', 'info')

            success = await self.send_and_wait(
                client,
                *self.WAYPOINTS[next_wp],
                wp_name=next_wp,          # ← 웨이포인트 이름 전달
                label=f'{robot_id}→{next_wp}'
            )

            if success is None:
                self.server_node.get_logger().info(
                    f'[{robot_id}] Pause로 인해 이동 중단 — 재개 대기 중'
                )
                await self.wait_if_paused()
                rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
                self.push_log(f'▶ {rname}: 재개 — {next_wp} 재이동', 'success')
                self.server_node.get_logger().info(f'[{robot_id}] 재개 — {next_wp} 재이동')
                continue

            if not success:
                self.server_node.get_logger().warn(f'[{robot_id}] 이동 실패 — 재시도')
                continue

            # ── 3. 도착 후 상태 갱신 ──
            self.robot_current[robot_id] = next_wp
            self.robot_target[robot_id] = None
            rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
            self.push_log(f'✅ {rname}: {next_wp} 도착', 'success')

            # ── 조건 1: 도착 직후 1초 대기 ──
            self.server_node.get_logger().info(f'[{robot_id}] {next_wp} 도착 — 1초 대기')
            for _ in range(10):
                await self.wait_if_paused()
                await asyncio.sleep(0.1)

            # ── 조건 2: B 도착 시 C 또는 D 점유 확인 ──
            if next_wp == 'B':
                while self.is_waypoint_occupied('C', robot_id) or self.is_waypoint_occupied('D', robot_id):
                    await self.wait_if_paused()
                    self.server_node.get_logger().warn(
                        f'[{robot_id}] B 대기 — C 또는 D가 점유되어 있습니다.'
                    )
                    await asyncio.sleep(0.5)
                self.server_node.get_logger().info(f'[{robot_id}] C/D 확보 확인 — B 통과!')

            # ── 조건 3: A 또는 C 도착 시 OMX 정책 실행 ──
            if next_wp in ['A', 'C']:
                await self._run_omx_and_wait(robot_id, next_wp)

            # ── 스텝 증가 ──
            self.robot_step[robot_id] += 1

    # ──────────────────────────────────────────────
    # 진입점
    # ──────────────────────────────────────────────
    async def run(self):
        self._loop = asyncio.get_running_loop()
        selected = self.shared_state.get('selected', [])

        tasks = []
        if 1 in selected:
            tasks.append(self.move_robot(self.robot1_client, 'robot1'))
        if 2 in selected:
            tasks.append(self.move_robot(self.robot2_client, 'robot2'))

        await asyncio.gather(*tasks)

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

    async def send_and_wait(self, client, x, y, oz, ow, wp_name='', label=''):
        """
        이동 명령을 보내고 결과를 기다린다.
        - wp_name: 현재 목표 웨이포인트 이름 → shared_state['current_wp']에 세팅
        - pause 감지 시 None 반환, 성공 True, 실패 False
        """
        # ── [핵심] 출발 전에 현재 목표 웨이포인트를 shared_state에 기록 ──
        self.shared_state['current_wp'] = wp_name

        future = client.send_goal_async(self.make_goal(x, y, oz, ow))
        while not future.done():
            await asyncio.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.server_node.get_logger().warn(f"{label} goal rejected")
            return False

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            if self.shared_state.get('paused', False):
                self.server_node.get_logger().info(f"{label} pause 감지 — 목표 취소 중")
                self.push_log(f'⏸ {label.split("→")[0]} 이동 취소 중...', 'warn')
                goal_handle.cancel_goal_async()
                while not result_future.done():
                    await asyncio.sleep(0.02)
                return None
            await asyncio.sleep(0.02)

        result = result_future.result()
        self.server_node.get_logger().info(f"{label} result status={result.status}")
        return result.status == GoalStatus.STATUS_SUCCEEDED


# ──────────────────────────────────────────────────────────
# start_mission: bridge.py의 threading.Thread에서 호출
# ──────────────────────────────────────────────────────────
def start_mission(mission: MissionManager):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(mission.run())