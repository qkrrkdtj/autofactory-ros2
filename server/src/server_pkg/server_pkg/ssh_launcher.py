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

def wait_for_topic_by_domain(topic, domain_id):
    """특정 도메인의 토픽이 뜰 때까지 무한 대기"""
    while True:
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
        
def launch_all(on_robot1_ready=None, on_robot2_ready=None):
    """각 로봇의 콜백을 개별적으로 받아 처리합니다."""
    for device in DEVICES:
        t = threading.Thread(target=ssh_run, args=(device,), daemon=True)
        t.start()
        
    def wait_and_notify_1():
        print("⏳ [Waffle 1] /keepout_filter_mask 대기 중 (Domain 31)...")
        wait_for_topic_by_domain('/keepout_filter_mask', 31)
        
        if on_robot1_ready:
            on_robot1_ready()

    def wait_and_notify_2():
        print("⏳ [Waffle 2] /keepout_filter_mask 대기 중 (Domain 32)...")
        wait_for_topic_by_domain('/keepout_filter_mask', 32)
        
        if on_robot2_ready:
            on_robot2_ready()
    
    # 두 로봇의 토픽 대기를 각각 독립된 스레드로 실행
    threading.Thread(target=wait_and_notify_1, daemon=True).start()
    threading.Thread(target=wait_and_notify_2, daemon=True).start()

def kill_all():
    print("모든 기기 종료 중...")
    for device, ssh, stdin in sessions:
        name = device["name"]
        target = device["kill_target"]
        try:
            attempt = 0
            while True:
                attempt += 1
                ssh.exec_command(f"pkill -SIGINT -f '{target}'")
                print(f"[{name}] SIGINT 전송 ({attempt}회)")

                time.sleep(3)
                _, stdout, _ = ssh.exec_command(f"pgrep -f '{target}'")
                result = stdout.read().decode().strip()
                stdout.channel.recv_exit_status()

                if not result:
                    print(f"[{name}] 종료 완료")
                    break

                print(f"[{name}] 아직 실행 중... 재전송")

            ssh.close()
        except Exception as e:
            print(f"[{name}] 종료 오류: {e}")
    sessions.clear()