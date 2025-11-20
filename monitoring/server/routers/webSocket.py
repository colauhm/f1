import asyncio
import threading
import sys
import time
from fastapi import APIRouter, WebSocket

# ---- OS 판별 및 라이브러리 설정 ----
try:
    # 리눅스/라즈베리파이 환경
    import termios
    import tty
    import RPi.GPIO as GPIO
    PLATFORM = "LINUX"
except ImportError:
    # 윈도우 환경 (테스트용 Mock 설정)
    import msvcrt
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
        # PWM 클래스 모킹
        class PWM:
            def __init__(self, pin, freq): pass
            def start(self, duty): pass
            def ChangeDutyCycle(self, duty): pass
            def stop(self): pass
    
    GPIO = MockGPIO()

# ---- 라우터 설정 ----
router = APIRouter(prefix="/api")

# ---- 글로벌 변수 ----
current_duty = 0.0
last_key_time = 0      # [중요] 마지막 키 입력 시간 저장
stop_threads = False

# ---- GPIO 초기화 (여기가 중요함) ----
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pwm_pin = 13
in1_pin = 23
in2_pin = 24

GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)

# [오류 해결 핵심] pwm 객체를 전역 변수로 생성
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
        # 윈도우용
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8').lower()
    return None

# ---- 키 리스너 스레드 (타임스탬프 방식) ----
def key_listener():
    global last_key_time, stop_threads
    try:
        while not stop_threads:
            k = get_key_input()
            
            if k == 'w':
                # 키가 감지되면 현재 시간을 기록
                last_key_time = time.time()
            elif k == 'q':
                stop_threads = True
                
            time.sleep(0.01) 
    except Exception as e:
        print(f"Key listener error: {e}")

# ---- 모터 제어 스레드 (부드러운 가속) ----
def motor_control_loop():
    global current_duty, stop_threads, pwm # pwm을 전역에서 가져옴
    
    speed = 0
    MAX_SPEED = 4095
    MIN_SPEED = 1500
    ACC = 200
    
    # 입력 유효 시간 (이 시간 안에 입력 없으면 뗀 것으로 간주)
    KEY_TIMEOUT = 0.15 
    
    while not stop_threads:
        time.sleep(0.05)
        
        # 현재 시간과 마지막 입력 시간의 차이 계산
        time_diff = time.time() - last_key_time
        
        # 타임아웃 이내라면 누르고 있는 중
        if time_diff < KEY_TIMEOUT:
            if speed < MIN_SPEED: speed = MIN_SPEED
            speed += ACC
            if speed > MAX_SPEED: speed = MAX_SPEED
            
        # 타임아웃 지났으면 키 뗀 것 (관성 주행)
        else:
            speed = MIN_SPEED

        duty = (speed / MAX_SPEED) * 100
        
        # 여기서 전역 변수 pwm을 사용함
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