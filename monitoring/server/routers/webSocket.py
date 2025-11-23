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
    # 윈도우 등 테스트 환경을 위한 모의 객체
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

router = APIRouter(prefix="/ws")

# ---- 2. 사용자 설정 변수 ----
PWM_PIN = 13
IN1_PIN = 23
IN2_PIN = 24
SERIAL_PORT = '/dev/ttyUSB0'  # 실제 포트 확인 필요
BAUD_RATE = 9600

PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 310
RAPID_PRESS_COUNT = 3
RAPID_PRESS_WINDOW = 2.0
SAFETY_LOCK_DURATION = 5.0

SAFETY_SPEED = 15       # 위험 시 속도 (0 = 관성 주행)
IDLE_SPEED = 15        # 공회전 속도
IDLE_TIMEOUT = 5.0     # 자동 정지 대기 시간 (초)

# ---- 3. 전역 변수 (스레드 간 공유) ----
current_duty = 0.0          # 현재 모터 속도 (웹소켓 전송용)
current_pedal_raw = 0       # 현재 페달 값 (웹소켓 전송용)
stop_threads = False        # 프로그램 종료 플래그

# ---- 4. 하드웨어 제어 스레드 ----
def hardware_loop():
    global current_duty, current_pedal_raw, stop_threads

    # GPIO 초기화
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(PWM_PIN, GPIO.OUT)
    GPIO.setup(IN1_PIN, GPIO.OUT)
    GPIO.setup(IN2_PIN, GPIO.OUT)

    pwm = GPIO.PWM(PWM_PIN, 1000)
    pwm.start(0)
    GPIO.output(IN1_PIN, True)
    GPIO.output(IN2_PIN, False)

    # 로직용 변수 초기화
    press_timestamps = deque()
    override_end_time = 0
    last_pedal_value = 0
    last_time = time.time()
    prev_over_90 = False
    last_pedal_active_time = time.time()  # 마지막 활동 시간

    print(f"HW Loop: 포트 {SERIAL_PORT} 연결 시도...")
    
    ser = None
    try:
        if PLATFORM == "LINUX":
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            ser.flush()
            print("HW Loop: 아두이노 연결 성공! 모터 제어 시작.")

        while not stop_threads:
            # 윈도우 테스트용 (시리얼 없으면 0으로 처리)
            if ser is None:
                time.sleep(1)
                continue

            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if not line.isdigit():
                        continue

                    # --- 데이터 수집 ---
                    current_pedal_value = int(line)
                    # 범위 보정
                    if current_pedal_value < 0: current_pedal_value = 0
                    if current_pedal_value > 100: current_pedal_value = 100
                    
                    # 웹소켓 전송을 위해 전역 변수에 저장
                    current_pedal_raw = current_pedal_value 
                    current_time = time.time()

                    # ================= [모터 제어 로직 시작] =================

                    # [A] 급발진 안전 모드 (최우선)
                    if current_time < override_end_time:
                        target_speed = SAFETY_SPEED
                        last_pedal_active_time = current_time # 안전모드 중엔 타이머 갱신
                    
                    else:
                        # [B] 위험 감지 판단
                        trigger_safety = False
                        
                        # B-1. 각속도
                        dt = current_time - last_time
                        if dt > 0:
                            delta_percent = current_pedal_value - last_pedal_value
                            delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                            angular_velocity = delta_angle / dt
                            if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                                print(f"!!! 급가속 감지 ({angular_velocity:.1f} deg/s)")
                                trigger_safety = True

                        # B-2. 3회 연타
                        is_over_90 = (current_pedal_value >= 90)
                        if is_over_90 and not prev_over_90:
                            press_timestamps.append(current_time)
                        
                        while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                            press_timestamps.popleft()

                        if len(press_timestamps) >= RAPID_PRESS_COUNT:
                            print(f"!!! 과속 연타 감지 ({len(press_timestamps)}회)")
                            trigger_safety = True
                            press_timestamps.clear()
                        
                        prev_over_90 = is_over_90

                        # [C] 속도 결정
                        if trigger_safety:
                            override_end_time = current_time + SAFETY_LOCK_DURATION
                            target_speed = SAFETY_SPEED
                        else:
                            # --- [핵심] 자동 정지 + 공회전 로직 ---
                            if current_pedal_value > 0:
                                # 밟고 있음 -> 타이머 리셋, 속도는 최소 IDLE_SPEED 이상
                                last_pedal_active_time = current_time
                                target_speed = max(current_pedal_value, IDLE_SPEED)
                            else:
                                # 안 밟고 있음 (0%)
                                idle_duration = current_time - last_pedal_active_time
                                if idle_duration >= IDLE_TIMEOUT:
                                    target_speed = 0  # 5초 지남 -> 정지
                                else:
                                    target_speed = IDLE_SPEED # 아직 5초 안됨 -> 공회전

                    # ================= [모터 제어 로직 끝] =================

                    # 모터 구동 및 상태 업데이트
                    pwm.ChangeDutyCycle(target_speed)
                    current_duty = target_speed  # 웹소켓용 업데이트

                    last_pedal_value = current_pedal_value
                    last_time = current_time

                except ValueError:
                    pass
                except Exception as e:
                    print(f"HW Loop Error: {e}")

            time.sleep(0.01) # CPU 점유율 관리

    except serial.SerialException:
        print("아두이노 연결 실패! 케이블 확인 필요.")
    except Exception as e:
        print(f"치명적 오류: {e}")
    finally:
        pwm.stop()
        GPIO.cleanup()
        if ser and ser.is_open:
            ser.close()
        print("Hardware Thread Stopped.")

# ---- 5. 외부 호출용 함수 ----
def start_hardware():
    t = threading.Thread(target=hardware_loop, daemon=True)
    t.start()

# ---- 6. 웹소켓 라우터 ----
@router.websocket("")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if stop_threads: break
            
            # 프론트엔드로 현재 상태 전송
            payload = {
                "duty": round(current_duty, 1),
                "pedal": current_pedal_raw
            }
            await websocket.send_json(payload)
            await asyncio.sleep(0.05) # 20fps 전송
            
    except Exception as e:
        print(f"WebSocket Disconnected: {e}")