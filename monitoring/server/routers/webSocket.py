import asyncio
import threading
import sys
import time
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

# ---- OS 판별 및 라이브러리 설정 (이전과 동일) ----
try:
    import termios
    import tty
    import RPi.GPIO as GPIO
    PLATFORM = "LINUX"
except ImportError:
    import msvcrt
    PLATFORM = "WINDOWS"
    class MockGPIO:
        BCM = "BCM"; OUT = "OUT"
        def setmode(self, m): pass
        def setwarnings(self, f): pass
        def setup(self, p, m): pass
        def output(self, p, s): pass
        def cleanup(self): print("Mock GPIO Cleaned up")
        class PWM:
            def __init__(self, p, f): pass
            def start(self, d): pass
            def ChangeDutyCycle(self, d): pass
            def stop(self): pass
    GPIO = MockGPIO()

router = APIRouter(prefix="/api")

# ---- 글로벌 변수 ----
current_duty = 0.0
last_key_time = 0  # 웹 버튼이 눌려도 이 시간이 갱신됨
stop_threads = False

# ---- GPIO 초기화 ----
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pwm_pin = 13
in1_pin = 23
in2_pin = 24

GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)

pwm = GPIO.PWM(pwm_pin, 1000)
pwm.start(0)
GPIO.output(in1_pin, True)
GPIO.output(in2_pin, False)

# ---- 키 입력 리스너 (기존 유지) ----
def get_key_input():
    if PLATFORM == "LINUX":
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
    else:
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8').lower()
    return None

def key_listener():
    global last_key_time, stop_threads
    if PLATFORM == "LINUX":
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
    try:
        while not stop_threads:
            k = get_key_input()
            if k == 'w':
                last_key_time = time.time()
            elif k == 'q':
                stop_threads = True
            time.sleep(0.01) 
    except Exception as e:
        print(f"Key listener error: {e}")
    finally:
        if PLATFORM == "LINUX":
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ---- 모터 제어 루프 (기존 로직 유지) ----
def motor_control_loop():
    global current_duty, stop_threads, pwm
    
    speed = 0
    MAX_SPEED = 4095
    MIN_SPEED = 1500
    ACC = 200
    STOP_DELAY = 0.8 # 웹 지연 고려해서 넉넉히 유지
    
    while not stop_threads:
        time.sleep(0.05)
        
        time_diff = time.time() - last_key_time
        
        # 키보드 OR 웹 버튼 신호가 최근에 있었다면 가속
        if time_diff < STOP_DELAY:
            if time_diff < 0.2: 
                speed += ACC
                if speed > MAX_SPEED: speed = MAX_SPEED
            else:
                if speed > MIN_SPEED: speed -= 20 
        else:
            if speed > MIN_SPEED:
                speed -= 150
                if speed < MIN_SPEED: speed = MIN_SPEED
            else:
                speed = MIN_SPEED

        duty = (speed / MAX_SPEED) * 100
        pwm.ChangeDutyCycle(duty)
        current_duty = duty

    pwm.stop()
    GPIO.cleanup()

def start_hardware():
    t_motor = threading.Thread(target=motor_control_loop, daemon=True)
    t_motor.start()
    t_key = threading.Thread(target=key_listener, daemon=True)
    t_key.start()

# ---- [핵심 변경] 웹소켓 비동기 처리 ----
async def ws_sender(websocket: WebSocket):
    """모터 상태를 주기적으로 웹으로 전송"""
    try:
        while True:
            if stop_threads: break
            payload = { "duty": round(current_duty, 1) }
            await websocket.send_json(payload)
            await asyncio.sleep(0.05) # 20FPS 전송
    except WebSocketDisconnect:
        pass

async def ws_receiver(websocket: WebSocket):
    """웹에서 오는 버튼 신호를 받음"""
    global last_key_time
    try:
        while True:
            data = await websocket.receive_json()
            # 웹에서 {"command": "go"}를 보내면 가속으로 인식
            if data.get("command") == "go":
                last_key_time = time.time()
    except WebSocketDisconnect:
        print("Client disconnected")

@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    # 보내는 작업과 받는 작업을 동시에 실행 (병렬 처리)
    sender_task = asyncio.create_task(ws_sender(websocket))
    receiver_task = asyncio.create_task(ws_receiver(websocket))
    
    # 둘 중 하나라도 끝나면(연결 끊김 등) 종료
    done, pending = await asyncio.wait(
        [sender_task, receiver_task],
        return_when=asyncio.FIRST_COMPLETED,
    )
    for task in pending:
        task.cancel()