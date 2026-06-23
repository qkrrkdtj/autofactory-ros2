import paramiko
import threading
import time

DEVICES = [
    {
        "name": "turtlebot1",
        "host": "192.168.100.150",
        "user": "wapple",
        "script": "bash -i -c 'bringup'"
    },
    {
        "name": "turtlebot2",
        "host": "192.168.100.149",
        "user": "waffle",
        "script": "bash -i -c 'bringup'"
    },
    # {
    #     "name": "raspberry",
    #     "host": "192.168.100.101",
    #     "user": "rapi23",
    #     "script": "source ~/proj/venv/bin/activate && python3 ~/proj/GPIO/stepper.py"
    # },
]

sessions = []
_ready_count = 0
_ready_lock = threading.Lock()
_on_all_ready = None  # 모든 기기 준비 완료 콜백

def _check_ready():
    global _ready_count
    with _ready_lock:
        _ready_count += 1
        if _ready_count >= len(DEVICES):
            if _on_all_ready:
                _on_all_ready()

def ssh_run(device):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(device["host"], username=device["user"])
        print(f"[{device['name']}] SSH 접속 성공")

        stdin, stdout, stderr = ssh.exec_command(device["script"], get_pty=True)
        sessions.append((device["name"], ssh, stdin))
        print(f"[{device['name']}] 실행 시작!")

        # 첫 줄 출력이 오면 실행됐다고 판단
        first_line = stdout.readline()
        print(f"[{device['name']}] {first_line.strip()}")
        _check_ready()  # ← 첫 출력 후 카운트

        # 로그출력
        # for line in stdout:
        #     print(f"[{device['name']}] {line.strip()}")

    except Exception as e:
        print(f"[{device['name']}] 오류: {e}")
        
def launch_all(on_all_ready=None):
    global _on_all_ready, _ready_count
    _on_all_ready = on_all_ready
    _ready_count = 0

    for device in DEVICES:
        t = threading.Thread(target=ssh_run, args=(device,), daemon=True)
        t.start()

def kill_all():
    print("모든 기기 종료 중...")
    for name, ssh, stdin in sessions:
        try:
            stdin.channel.send('\x03')  # ← stdin.write 대신 이걸로
            time.sleep(0.5)
            ssh.close()
            print(f"[{name}] 종료 완료")
        except Exception as e:
            print(f"[{name}] 종료 오류: {e}")
    sessions.clear()