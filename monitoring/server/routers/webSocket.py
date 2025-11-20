import asyncio
import threading
import sys
import time
from fastapi import APIRouter, WebSocket

# ---- OS 판별 및 라이브러리 불러오기 ----
try:
    # 리눅스/라즈베리파이 환경
    import termios
    import tty
    import RPi.GPIO as GPIO
    PLATFORM = "LINUX"
except ImportError:
    # 윈도우 환경 (테스트용 Mock 설정)
    import msvcrt  # 윈도우용 키 입력 라이브러리
    PLATFORM = "WINDOWS"
    
    # 가짜 GPIO 클래스 (에러 방지용)
    class MockGPIO:
        BCM = "BCM"
        OUT = "OUT"
        def setmode(self, mode): pass
        def setwarnings(self, flag): pass
        def setup(self, pin, mode): pass
        def output(self, pin, state): pass
        def cleanup(self): print("Mock GPIO Cleaned up")
        class PWM:
            def __init__(self, pin, freq): pass
            def start(self, duty): pass
            def ChangeDutyCycle(self, duty): pass
            def stop(self): pass
    
    GPIO = MockGPIO()
    print("⚠️ 윈도우 환경 감지됨: GPIO와 termios가 가상으로 작동합니다.")

# ---- 라우터 설정 ----
router = APIRouter(prefix="/api")

# ---- 글로벌 변수 ----
current_duty = 0.0
pressed_key = None
stop_threads = False

# ---- GPIO 설정 ----
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

# ---- 키 입력 함수 (OS별 분기) ----
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
        # 윈도우용 (msvcrt 사용)
        if msvcrt.kbhit():
            # 윈도우는 바이트로 들어오므로 디코딩
            return msvcrt.getch().decode('utf-8').lower()
    return None

# ---- 키 리스너 스레드 ----
def key_listener():
    global pressed_key, stop_threads
    try:
        while not stop_threads:
            k = get_key_input()
            if k:
                pressed_key = k
            time.sleep(0.01) # CPU 점유율 낮춤
    except Exception as e:
        print(f"Key listener error: {e}")

# ---- 모터 제어 스레드 ----
def motor_control_loop():
    global current_duty, pressed_key, stop_threads
    
    speed = 0
    MAX_SPEED = 4095
    MIN_SPEED = 1500
    ACC = 200
    
    while not stop_threads:
        time.sleep(0.05)
        
        key = pressed_key
        pressed_key = None 

        if key == 'q':
            stop_threads = True
            break
            
        if key == 'w':
            if speed < MIN_SPEED: speed = MIN_SPEED
            speed += ACC
            if speed > MAX_SPEED: speed = MAX_SPEED
        else:
            speed = MIN_SPEED

        duty = (speed / MAX_SPEED) * 100
        pwm.ChangeDutyCycle(duty)
        current_duty = duty

    pwm.stop()
    GPIO.cleanup()

# ---- 하드웨어 시작 함수 ----
def start_hardware():
    t_motor = threading.Thread(target=motor_control_loop, daemon=True)
    t_motor.start()

    t_key = threading.Thread(target=key_listener, daemon=True)
    t_key.start()

# ---- 웹소켓 엔드포인트 ----
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