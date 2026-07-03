import os
import sqlite3
from datetime import datetime
from contextlib import contextmanager

# DB 파일은 이 스크립트와 같은 폴더에 생성됨
DB_PATH = os.path.expanduser('~/teser/server/src/server_pkg/server_pkg/process_log.db')


@contextmanager
def _conn():
    """호출마다 새 커넥션 → Flask/ROS2 스레드 분리 환경에서도 안전."""
    conn = sqlite3.connect(DB_PATH, timeout=5.0)
    conn.row_factory = sqlite3.Row
    try:
        yield conn
        conn.commit()
    finally:
        conn.close()


def init_db():
    """앱 시작 시 1회 호출. 테이블 없으면 생성."""
    with _conn() as c:
        c.execute('''
            CREATE TABLE IF NOT EXISTS process_log (
                id           INTEGER PRIMARY KEY AUTOINCREMENT,
                completed_at TEXT    NOT NULL,   -- ISO 시각 (로컬)
                robot        TEXT    NOT NULL,   -- 'Waffle 1' / 'Waffle 2'
                path         TEXT    NOT NULL,   -- 'A구역 → B구역 (적재)'
                duration     REAL,               -- 소요 초 (실패 시 NULL)
                status       TEXT    NOT NULL    -- '성공' / '장애 정지'
            )
        ''')
        c.execute('CREATE INDEX IF NOT EXISTS idx_completed_at ON process_log(completed_at)')


def insert_cycle_log(robot, path, duration, status='성공'):
    """
    공정 1건 완료 시 호출.
      robot    : 'Waffle 1' | 'Waffle 2'
      path     : 'A구역 → B구역 (적재)'
      duration : 소요 초(float). 실패면 None
      status   : '성공' | '장애 정지'
    """
    with _conn() as c:
        c.execute(
            'INSERT INTO process_log (completed_at, robot, path, duration, status) '
            'VALUES (?, ?, ?, ?, ?)',
            (datetime.now().isoformat(timespec='seconds'), robot, path, duration, status)
        )


def get_summary():
    """금일(로컬 날짜) '성공' 사이클 기준 요약 통계."""
    today = "date(completed_at) = date('now', 'localtime')"
    with _conn() as c:
        total = c.execute(
            f"SELECT COUNT(*) FROM process_log WHERE status='성공' AND {today}"
        ).fetchone()[0]
        w1 = c.execute(
            f"SELECT COUNT(*) FROM process_log WHERE status='성공' AND robot='Waffle 1' AND {today}"
        ).fetchone()[0]
        w2 = c.execute(
            f"SELECT COUNT(*) FROM process_log WHERE status='성공' AND robot='Waffle 2' AND {today}"
        ).fetchone()[0]
        avg = c.execute(
            f"SELECT AVG(duration) FROM process_log "
            f"WHERE status='성공' AND duration IS NOT NULL AND {today}"
        ).fetchone()[0]
    return {
        'total':   total,
        'waffle1': w1,
        'waffle2': w2,
        'avg':     round(avg, 1) if avg is not None else 0,
    }


def get_logs(limit=50, robot=None, status=None):
    """최근 공정 이력. robot/status로 필터, 최신순."""
    q = 'SELECT id, completed_at, robot, path, duration, status FROM process_log'
    where, params = [], []
    if robot:
        where.append('robot = ?');  params.append(robot)
    if status:
        where.append('status = ?'); params.append(status)
    if where:
        q += ' WHERE ' + ' AND '.join(where)
    q += ' ORDER BY id DESC LIMIT ?'
    params.append(limit)

    with _conn() as c:
        rows = c.execute(q, params).fetchall()

    out = []
    for r in rows:
        ts = r['completed_at']
        
        # 'T'를 기준으로 날짜와 시간을 분리
        if 'T' in ts:
            date_val, time_val = ts.split('T')
        else:
            date_val = ts[:10] if len(ts) >= 10 else ''
            time_val = ts[11:19] if len(ts) >= 19 else ts

        out.append({
            'id':       r['id'],
            'date':     date_val,  # 분리한 날짜
            'time':     time_val,  # 분리한 시간
            'robot':    r['robot'],
            'path':     r['path'],
            'duration': r['duration'],
            'status':   r['status'],
        })
    return out