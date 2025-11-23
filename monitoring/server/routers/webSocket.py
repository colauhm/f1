import asyncio
import threading
import sys
import time
from fastapi import APIRouter, WebSocket

# ---- OS 판별 및 라이브러리 설정 ----
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
last_key_time = 0
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

# ---- 키 입력 ----
def get_key_input():
    if PLATFORM == "LINUX":
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            if sys.stdin in [sys.stdin]:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    else:
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8').lower()
    return None

def key_listener():
    global last_key_time, stop_threads
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

# ---- [핵심 수정] 모터 제어 루프 ----
def motor_control_loop():
    global current_duty, stop_threads, pwm
    
    speed = 0
    MAX_SPEED = 4095
    MIN_SPEED = 800
    
    ACC = 100   # 가속할 때 더해지는 값
    DEC = 50   # [추가] 감속할 때 빠지는 값 (부드러운 연결용)
    
    # [수정 1] 타임아웃을 0.5초로 넉넉하게 설정 (OS 딜레이 커버)
    KEY_TIMEOUT = .8 
    
    while not stop_threads:
        time.sleep(0.05) # 50ms 주기
        
        time_diff = time.time() - last_key_time
        
        # 1. 키를 누르고 있다고 판단될 때
        if time_diff < KEY_TIMEOUT:
            if speed < MIN_SPEED: speed = MIN_SPEED
            speed += ACC
            if speed > MAX_SPEED: speed = MAX_SPEED
            
        # 2. 키를 뗐다고 판단될 때
        else:
            # [수정 2] 확 떨어뜨리지 않고 서서히 줄임 (관성 효과 + 끊김 방지)
            if speed > MIN_SPEED:
                speed -= DEC
                # MIN_SPEED 밑으로는 안 떨어지게 방어
                if speed < MIN_SPEED:
                    speed = MIN_SPEED
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

@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if stop_threads: break
            payload = { "duty": round(current_duty, 1) }
            await websocket.send_json(payload)
            await asyncio.sleep(0.05)
    except Exception as e:
        print(f"WebSocket Disconnected: {e}")