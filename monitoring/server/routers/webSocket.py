import asyncio
import threading
import time
import serial
from collections import deque
from fastapi import APIRouter, WebSocket

# ---- 1. 하드웨어 라이브러리 설정 ----
try:
    import RPi.GPIO as GPIO
    PLATFORM = "LINUX"
except ImportError:
    PLATFORM = "WINDOWS"
    class MockGPIO:
        BCM = "BCM"; OUT = "OUT"
        def setmode(self, m): pass
        def setwarnings(self, f): pass
        def setup(self, p, m): pass
        def output(self, p, s): pass
        def cleanup(self): print("GPIO Cleaned up")
        class PWM:
            def __init__(self, p, f): pass
            def start(self, d): pass
            def ChangeDutyCycle(self, d): pass
            def stop(self): pass
    GPIO = MockGPIO()

# [중요] 경로 설정 (403 오류 방지를 위해 /ws로 통일)
router = APIRouter(prefix="/ws")

# ---- 2. 핀 설정 (바퀴 2개) ----
# [Motor A - 왼쪽 바퀴]
PWM_A_PIN = 13
IN1_PIN = 23
IN2_PIN = 24

# [Motor B - 오른쪽 바퀴] (새로 추가됨)
PWM_B_PIN = 12
IN3_PIN = 5
IN4_PIN = 6

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# ---- 3. 주행 설정 ----
PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 310
RAPID_PRESS_COUNT = 3
RAPID_PRESS_WINDOW = 2.0
SAFETY_LOCK_DURATION = 5.0

SAFETY_SPEED = 15       # 위험 시 정지
IDLE_SPEED = 15        # 공회전 속도
IDLE_TIMEOUT = 5.0     # 자동 정지 시간

# ---- 전역 변수 ----
current_duty = 0.0
current_pedal_raw = 0
stop_threads = False

# ---- 4. 하드웨어 제어 루프 ----
def hardware_loop():
    global current_duty, current_pedal_raw, stop_threads

    # GPIO 초기화
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # 핀 모드 설정 (Motor A + Motor B)
    GPIO.setup(PWM_A_PIN, GPIO.OUT)
    GPIO.setup(IN1_PIN, GPIO.OUT)
    GPIO.setup(IN2_PIN, GPIO.OUT)
    
    GPIO.setup(PWM_B_PIN, GPIO.OUT)
    GPIO.setup(IN3_PIN, GPIO.OUT)
    GPIO.setup(IN4_PIN, GPIO.OUT)

    # PWM 시작 (두 바퀴 모두)
    pwm_a = GPIO.PWM(PWM_A_PIN, 1000)
    pwm_a.start(0)
    
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000)
    pwm_b.start(0)

    # 방향 설정 (전진)
    # 만약 바퀴가 거꾸로 돌면 True/False를 반대로 바꾸세요.
    GPIO.output(IN1_PIN, True)   # Motor A
    GPIO.output(IN2_PIN, False)
    
    GPIO.output(IN3_PIN, True)   # Motor B
    GPIO.output(IN4_PIN, False)

    # 로직 변수들
    press_timestamps = deque()
    override_end_time = 0
    last_pedal_value = 0
    last_time = time.time()
    prev_over_90 = False
    last_pedal_active_time = time.time()

    print(f"HW Loop: 포트 {SERIAL_PORT} 연결 시도...")
    
    ser = None
    try:
        if PLATFORM == "LINUX":
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            ser.flush()
            print("HW Loop: 아두이노 연결 성공! 듀얼 모터 제어 시작.")

        while not stop_threads:
            if ser is None and PLATFORM == "LINUX":
                # 연결 끊기면 재접속 시도 로직 (선택사항)
                time.sleep(1)
                continue
            
            # 윈도우 테스트용 가짜 데이터
            if ser is None: 
                time.sleep(1); continue

            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if not line.isdigit():
                        continue

                    # --- 데이터 처리 ---
                    current_pedal_value = int(line)
                    if current_pedal_value < 0: current_pedal_value = 0
                    if current_pedal_value > 100: current_pedal_value = 100
                    
                    current_pedal_raw = current_pedal_value
                    current_time = time.time()

                    # ================= [속도 계산 로직] =================
                    # (이전과 동일한 로직)
                    
                    if current_time < override_end_time:
                        target_speed = SAFETY_SPEED
                        last_pedal_active_time = current_time
                    else:
                        trigger_safety = False
                        dt = current_time - last_time
                        if dt > 0:
                            delta_percent = current_pedal_value - last_pedal_value
                            delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                            angular_velocity = delta_angle / dt
                            if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                                print(f"!!! 급가속 ({angular_velocity:.1f} deg/s)")
                                trigger_safety = True

                        is_over_90 = (current_pedal_value >= 90)
                        if is_over_90 and not prev_over_90:
                            press_timestamps.append(current_time)
                        
                        while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                            press_timestamps.popleft()

                        if len(press_timestamps) >= RAPID_PRESS_COUNT:
                            print(f"!!! 과속 연타 ({len(press_timestamps)}회)")
                            trigger_safety = True
                            press_timestamps.clear()
                        
                        prev_over_90 = is_over_90

                        if trigger_safety:
                            override_end_time = current_time + SAFETY_LOCK_DURATION
                            target_speed = SAFETY_SPEED
                        else:
                            if current_pedal_value > 0:
                                last_pedal_active_time = current_time
                                target_speed = max(current_pedal_value, IDLE_SPEED)
                            else:
                                idle_duration = current_time - last_pedal_active_time
                                if idle_duration >= IDLE_TIMEOUT:
                                    target_speed = 0
                                else:
                                    target_speed = IDLE_SPEED

                    # ================= [모터 2개 동시 제어] =================
                    pwm_a.ChangeDutyCycle(target_speed) # 왼쪽 바퀴
                    pwm_b.ChangeDutyCycle(target_speed) # 오른쪽 바퀴
                    
                    current_duty = target_speed

                    last_pedal_value = current_pedal_value
                    last_time = current_time

                except ValueError:
                    pass
                except Exception as e:
                    print(f"HW Loop Error: {e}")

            time.sleep(0.01)

    except serial.SerialException:
        print("아두이노 연결 실패! 케이블 확인 필요.")
    except Exception as e:
        print(f"치명적 오류: {e}")
    finally:
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        if ser and ser.is_open:
            ser.close()
        print("Hardware Thread Stopped.")

def start_hardware():
    t = threading.Thread(target=hardware_loop, daemon=True)
    t.start()

@router.websocket("")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if stop_threads: break
            payload = {
                "duty": round(current_duty, 1),
                "pedal": current_pedal_raw
            }
            await websocket.send_json(payload)
            await asyncio.sleep(0.05)
    except Exception as e:
        print(f"WebSocket Disconnected: {e}")