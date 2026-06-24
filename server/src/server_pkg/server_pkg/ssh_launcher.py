import paramiko
import threading
import time
import os
import subprocess

DEVICES = [
    {
        "name": "turtlebot1",
        "host": "192.168.100.150",
        "user": "wapple",
        "script": "bash -i -c 'bringup'",
        "kill_target": "robot.launch.py"
    },
    {
        "name": "turtlebot2",
        "host": "192.168.100.149",
        "user": "waffle",
        "script": "bash -i -c 'bringup'",
        "kill_target": "robot.launch.py"
    },
    {
        "name": "raspberry",
        "host": "192.168.100.101",
        "user": "rapi23",
        "script": "source ~/pro/bin/activate && python3 ~/proj/GPIO/stepper.py",
        "kill_target": "stepper.py" 
    },
]

sessions = []
_on_all_ready = None

def wait_for_topic(topic):
    """토픽이 뜰 때까지 무한 대기"""
    while True:
        for domain_id in [31, 32]:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = str(domain_id)
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True, text=True,
                env=env
            )
            if topic in result.stdout:
                return True
        time.sleep(1)

def ssh_run(device):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(device["host"], username=device["user"])
        print(f"[{device['name']}] SSH 접속 성공")

        stdin, stdout, stderr = ssh.exec_command(device["script"], get_pty=True)
        sessions.append((device, ssh, stdin))
        print(f"[{device['name']}] 실행 시작!")

        # 로그출력
        # for line in stdout:
        #     print(f"[{device['name']}] {line.strip()}")

    except Exception as e:
        print(f"[{device['name']}] SSH 접속 실패 - 기기가 꺼져있거나 네트워크 연결 안됨 ({device['host']})")
        
def launch_all(on_all_ready=None):
    global _on_all_ready
    _on_all_ready = on_all_ready

    for device in DEVICES:
        t = threading.Thread(target=ssh_run, args=(device,), daemon=True)
        t.start()
        
    def wait_and_notify():
        print("⏳ /keepout_filter_mask 토픽 대기 중...")
        wait_for_topic('/keepout_filter_mask')
        print("✅ keepout_filter 확인 — 모든 준비 완료!")
        if _on_all_ready:
            _on_all_ready()
    
    threading.Thread(target=wait_and_notify, daemon=True).start()

def kill_all():
    print("모든 기기 종료 중...")
    for device, ssh, stdin in sessions:
        name = device["name"]
        target = device["kill_target"]  # ← 기기별 종료 대상 사용
        try:
            # SIGINT 전송
            ssh.exec_command(f"pkill -SIGINT -f '{target}'")
            
            # 꺼졌는지 확인
            while True:
                time.sleep(5)
                _, stdout, _ = ssh.exec_command(f"pgrep -f '{target}'")
                stdout.channel.recv_exit_status()
                result = stdout.read().decode().strip()
                
                if not result:
                    print(f"[{name}] 종료 완료")
                    break
                else:
                    print(f"[{name}] 아직 실행 중... SIGINT 재전송")
                    ssh.exec_command(f"pkill -SIGINT -f '{target}'")
            
            ssh.close()
        except Exception as e:
            print(f"[{name}] 종료 오류: {e}")
    sessions.clear()
    
