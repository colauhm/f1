import asyncio
import threading
import sys
import tty
import termios
import time
import RPi.GPIO as GPIO
from fastapi import FastAPI, WebSocket, APIRouter

app = FastAPI()
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

# 정방향 설정
GPIO.output(in1_pin, True)
GPIO.output(in2_pin, False)

# ---- 키 입력 감지 (스레드) ----
def key_listener():
    global pressed_key, stop_threads
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while not stop_threads:
            # 넌블로킹 방식으로 키 하나 읽기
            if sys.stdin in [sys.stdin]: 
                 pressed_key = sys.stdin.read(1)
    except Exception:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ---- 모터 제어 로직 (스레드) ----
def motor_control_loop():
    global current_duty, pressed_key, stop_threads
    
    speed = 0
    MAX_SPEED = 4095
    MIN_SPEED = 1500  # 최소 유지 속도 (관성 주행 시)
    ACC = 200         # 가속량
    
    while not stop_threads:
        time.sleep(0.05) # 50ms 주기
        
        key = pressed_key
        pressed_key = None 

        if key == 'q':
            stop_threads = True
            break
            
        if key == 'w':
            # 가속
            if speed < MIN_SPEED: speed = MIN_SPEED
            speed += ACC
            if speed > MAX_SPEED: speed = MAX_SPEED
        else:
            # 키 뗌 -> 최소 속도 유지 (관성 주행 모드)
            speed = MIN_SPEED

        # Duty 계산
        duty = (speed / MAX_SPEED) * 100
        pwm.ChangeDutyCycle(duty)
        
        # 웹소켓 공유용 변수 업데이트
        current_duty = duty

    # 종료 처리
    pwm.stop()
    GPIO.cleanup()

# ---- 웹소켓 엔드포인트 ----
@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    
    try:
        while True:
            if stop_threads:
                break
            
            # 오직 Duty 값만 JSON으로 전송
            payload = {
                "duty": round(current_duty, 1)
            }
            
            await websocket.send_json(payload)

            # 갱신 주기 (0.05초 = 초당 20회 전송)
            await asyncio.sleep(0.05)

    except Exception as e:
        print(f"WebSocket Disconnected: {e}")

# ---- 메인 실행부 ----
app.include_router(router)

# 스레드 시작
t_motor = threading.Thread(target=motor_control_loop, daemon=True)
t_motor.start()

t_key = threading.Thread(target=key_listener, daemon=True)
t_key.start()