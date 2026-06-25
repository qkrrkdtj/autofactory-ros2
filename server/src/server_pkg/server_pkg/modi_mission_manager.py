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
        self.shared_state = shared_state # 상태 공유
        # 🚨 프록시 서버(server_node)를 가리키도록 변경
        self.robot1_client = ActionClient(server_node, NavigateToPose, 'robot1/navigate_to_pose')
        self.robot2_client = ActionClient(server_node, NavigateToPose, 'robot2/navigate_to_pose')
        
        self.omx_connections = omx_connections or {}
        self.WP_TO_OMX = {'A': 'omx1', 'C': 'omx2'}
        self._omx_pending = {'A': None, 'C': None}
        self.DEPART_TARGET = {'A': 3, 'C': 3}
        
        yaml_path = os.path.join(os.path.dirname(__file__), 'waypoints.yaml')
        with open(yaml_path, 'r') as f:
            raw = yaml.safe_load(f)
        
        self.WAYPOINTS = {k: tuple(v) for k, v in raw.items()}
        self.STEPS = ['A', 'B', 'C', 'D', 'E']
        
        self.robot_current = {'robot1': None, 'robot2': None}
        self.robot_target = {'robot1': None, 'robot2': None}
        self.robot_step = {'robot1': 0, 'robot2': 0}
        self.robot2_first_run = True
        self._loop = None
        self.log_queue = queue.Queue(maxsize=200)
        
    # ──────────────────────────────────────────────
    # 외부에서 호출하는 공개 메서드
    #   - OMX 완료 콜백 (omx_link 의 on_cycle_done 에서 호출)
    #   - Flask / 키보드 수동 오버라이드
    # 어느 쪽이든 "그 자리에서 대기 중인 Future를 완료시킨다"로 통일
    # ──────────────────────────────────────────────
    # ──────────────────────────────────────────────
    # 대시보드 로그 큐
    # ──────────────────────────────────────────────
    def push_log(self, msg: str, level: str = 'info'):
        """대시보드 SSE 스트림으로 보낼 로그를 큐에 넣는다."""
        try:
            self.log_queue.put_nowait({'msg': msg, 'level': level})
        except queue.Full:
            pass

    def signal_resume(self, target_wp):
        """수동 오버라이드: 사람이 Flask/키보드로 A 또는 C 대기를 강제 해제."""
        if self._loop is None:
            self.server_node.get_logger().error('[오류] 아직 미션 루프가 초기화되지 않았습니다.')
            return
        # ★ _omx_pending 키가 아니라 WP_TO_OMX 로 유효성 검사
        if target_wp not in self.WP_TO_OMX:
            self.server_node.get_logger().error(f'[오류] {target_wp}는 유효한 대기 지점이 아닙니다.')
            return
        # ★ 실제로 대기 중인 Future 가 없으면 경고만 내고 종료
        if self._omx_pending.get(target_wp) is None:
            self.server_node.get_logger().warn(
                f'[수동신호] {target_wp}: 현재 대기 중인 작업이 없습니다 (로봇이 아직 도착 안 했거나 이미 출발).'
            )
            self.push_log(f'⚠ {target_wp} 신호: 현재 대기 중인 작업 없음', 'warn')
            return
        self._loop.call_soon_threadsafe(self._resolve_pending, target_wp, True)
        self.server_node.get_logger().info(f'[수동신호] {target_wp} 대기 해제 명령 수신!')
        self.push_log(f'🔓 {target_wp} 구역 수동 해제', 'success')

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

    async def wait_if_paused(self):
        while self.shared_state.get(
            'paused',
            False
        ):
            await asyncio.sleep(0.1)
            
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
        rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
        self.push_log(f'🔧 {rname}: {wp_name} OMX 작업 시작 (목표 {self.DEPART_TARGET.get(wp_name)}회)', 'info')
        self.server_node.get_logger().warn(
            f'[{robot_id}] {wp_name} 도착 — OMX 작업 시작 (목표 {self.DEPART_TARGET.get(wp_name)}회)'
        )

        done_count = 0
        while True:
            success = await self._run_one_cycle(robot_id, wp_name)
            done_count += 1
            rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
            self.push_log(f'🔧 {rname}: {wp_name} 동작 {done_count}회 완료', 'success' if success else 'warn')
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
            self.push_log("🤖 Waffle 2: Start → E' 이동 시작", 'info')
            self.server_node.get_logger().info(f'[{robot_id}] Start → E\'  이동 시작')
            while True:
                r = await self.send_and_wait(client, *self.WAYPOINTS["E'"], label=f"{robot_id}→E'")
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

            # E 이동 예약 -> 출발 -> 도착 처리
            self.robot_target[robot_id] = 'E'
            self.push_log("🤖 Waffle 2: E' → E 이동 시작", 'info')
            self.server_node.get_logger().info(f'[{robot_id}] E\' → E 이동 시작')
            while True:
                r = await self.send_and_wait(client, *self.WAYPOINTS['E'], label=f'{robot_id}→E')
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
                f'[{robot_id}] {next_wp} 점유됨(또는 다른 로봇이 접근 중)... 대기')
                await asyncio.sleep(5)

            # ── 2. [핵심] 출발 직전 목적지 점유(예약) ──
            self.robot_target[robot_id] = next_wp

            await self.wait_if_paused()
            rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
            prev_wp = self.robot_current.get(robot_id) or 'Start'
            self.push_log(f'🤖 {rname}: {prev_wp} → {next_wp} 이동 시작', 'info')

            success = await self.send_and_wait(
                client,
                *self.WAYPOINTS[next_wp],
                label=f'{robot_id}→{next_wp}')

            if success is None:
                # ── pause로 취소됨: 재개될 때까지 기다린 뒤 같은 목적지 재시도 ──
                self.server_node.get_logger().info(
                    f'[{robot_id}] Pause로 인해 이동 중단 — 재개 대기 중'
                )
                await self.wait_if_paused()   # paused==False 될 때까지 블록
                rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
                self.push_log(f'▶ {rname}: 재개 — {next_wp} 재이동', 'success')
                self.server_node.get_logger().info(
                    f'[{robot_id}] 재개 — {next_wp} 재이동'
                )
                continue   # 같은 next_wp 로 재시도

            if not success:
                self.server_node.get_logger().warn(
                    f'[{robot_id}] 이동 실패 — 재시도'
                )
                continue
            
            # ── 3. 도착 후 상태 갱신 (이전 자리 반납) ──
            self.robot_current[robot_id] = next_wp
            self.robot_target[robot_id] = None
            rname = 'Waffle 1' if robot_id == 'robot1' else 'Waffle 2'
            self.push_log(f'✅ {rname}: {next_wp} 도착', 'success')

            # ── [조건 1] 도착 직후 1초 대기 ──
            self.server_node.get_logger().info(f'[{robot_id}] {next_wp} 도착 — 1초 대기')
            for _ in range(10):
                await self.wait_if_paused()
                await asyncio.sleep(0.1)

            # ── [조건 2] B 도착 시 C 또는 D 점유 확인 ──
            if next_wp == 'B':
                while self.is_waypoint_occupied('C', robot_id) or self.is_waypoint_occupied('D', robot_id):
                    await self.wait_if_paused()
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
        
        selected = self.shared_state.get('selected', [])

        tasks = []

        if 1 in selected:
            tasks.append(
                self.move_robot(
                    self.robot1_client,
                    'robot1'
                )
            )

        if 2 in selected:
            tasks.append(
                self.move_robot(
                    self.robot2_client,
                    'robot2'
                )
            )

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

    async def send_and_wait(self, client, x, y, oz, ow, label=''):
        """
        이동 명령을 보내고 결과를 기다린다.
        - 이동 중 pause 가 걸리면 취소(cancel)하고 'paused' 를 뜻하는 None 반환.
        - 성공이면 True, 실패(abort/취소)이면 False 반환.
        """
        future = client.send_goal_async(self.make_goal(x, y, oz, ow))
        while not future.done():
            await asyncio.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.server_node.get_logger().warn(f"{label} goal rejected")
            return False

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            # ── pause 감지: 취소 요청 후 결과 기다림 ──
            if self.shared_state.get('paused', False):
                self.server_node.get_logger().info(f"{label} pause 감지 — 목표 취소 중")
                self.push_log(f'⏸ {label.split("→")[0]} 이동 취소 중...', 'warn')
                goal_handle.cancel_goal_async()
                while not result_future.done():
                    await asyncio.sleep(0.02)
                return None   # ← None = "pause로 인한 취소"
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