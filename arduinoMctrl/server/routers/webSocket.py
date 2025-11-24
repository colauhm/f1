import asyncio
import threading
import time
import serial
from collections import deque
from fastapi import APIRouter, WebSocket

# ---- 1. OS 판별 및 하드웨어 라이브러리 설정 ----
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

router = APIRouter(prefix="/ws")

# ---- 2. 핀 설정 ----
PWM_A_PIN = 13; IN1_PIN = 23; IN2_PIN = 24
PWM_B_PIN = 12; IN3_PIN = 5; IN4_PIN = 6

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# ---- 3. 주행 및 안전 설정 ----
PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 310
RAPID_PRESS_COUNT = 3
RAPID_PRESS_WINDOW = 2.0
SAFETY_LOCK_DURATION = 5.0

SAFETY_SPEED = 15
IDLE_SPEED = 15
IDLE_TIMEOUT = 5.0

# ---- 전역 변수 ----
current_duty = 0.0
current_pedal_raw = 0
current_safety_reason = None
current_remaining_time = 0  # [수정됨] 남은 시간 표시용 변수 추가
stop_threads = False

# ---- 4. 하드웨어 제어 루프 ----
def hardware_loop():
    global current_duty, current_pedal_raw, current_safety_reason, current_remaining_time, stop_threads

    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup(PWM_A_PIN, GPIO.OUT); GPIO.setup(IN1_PIN, GPIO.OUT); GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(PWM_B_PIN, GPIO.OUT); GPIO.setup(IN3_PIN, GPIO.OUT); GPIO.setup(IN4_PIN, GPIO.OUT)

    pwm_a = GPIO.PWM(PWM_A_PIN, 1000); pwm_a.start(0)
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000); pwm_b.start(0)
    
    GPIO.output(IN1_PIN, True); GPIO.output(IN2_PIN, False)
    GPIO.output(IN3_PIN, True); GPIO.output(IN4_PIN, False)

    # 로직 변수
    press_timestamps = deque()
    override_end_time = 0
    last_pedal_value = 0
    last_time = time.time()
    prev_over_90 = False
    last_pedal_active_time = time.time()
    
    # [수정됨] 안전 모드 상태 관리 변수
    safety_lock_active = False 
    safety_cause_msg = "" # 최초 차단 원인 저장

    print(f"HW Loop: 포트 {SERIAL_PORT} 연결 시도...")
    ser = None
    if PLATFORM == "LINUX":
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            ser.flush()
        except: pass

    try:
        while not stop_threads:
            if ser is None and PLATFORM == "LINUX":
                try: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1); ser.flush()
                except: time.sleep(1); continue
            elif ser is None and PLATFORM == "WINDOWS":
                time.sleep(0.1) # 윈도우 테스트용 더미 딜레이

            # 시리얼 읽기 (윈도우 테스트 시에는 임의 값 사용 필요)
            raw_line = ""
            if ser and ser.in_waiting > 0:
                raw_line = ser.readline().decode('utf-8').strip()
            
            # (테스트용) 윈도우나 시리얼 없을 때 로직 흐름 유지를 위해
            if not raw_line and PLATFORM == "WINDOWS": 
                pass # 실제 환경에선 주석 처리

            if raw_line.isdigit():
                current_pedal_value = int(raw_line)
                current_pedal_value = max(0, min(100, current_pedal_value))
                current_pedal_raw = current_pedal_value
                current_time = time.time()

                # ================= [수정된 안전 로직 시작] =================
                trigger_safety = False
                detected_reason = ""

                # 1. 안전 모드가 활성화된 상태라면?
                if safety_lock_active:
                    remaining = override_end_time - current_time
                    current_remaining_time = max(0, int(remaining)) # 남은 시간 업데이트

                    if remaining > 0:
                        # [상태 A] 아직 제한 시간이 남음
                        current_safety_reason = f"{safety_cause_msg}\n(해제까지 {current_remaining_time}초)"
                        target_speed = SAFETY_SPEED
                        last_pedal_active_time = current_time # 제한 중엔 공회전 카운트 리셋
                    
                    else:
                        # [상태 B] 시간은 지났지만 페달을 떼지 않음
                        if current_pedal_value > 0:
                            current_safety_reason = "⚠️ 엑셀에서 발을 떼세요!\n(안전 잠금 해제 대기중)"
                            target_speed = SAFETY_SPEED
                        else:
                            # [상태 C] 시간도 지났고, 페달도 0임 -> 해제!
                            safety_lock_active = False
                            current_safety_reason = None
                            current_remaining_time = 0
                            print(">>> 안전 잠금 해제됨")
                            # 바로 정상 주행 로직으로 넘어가도록 설정
                            target_speed = 0 

                # 2. 정상 주행 상태 (감지 로직 수행)
                else:
                    dt = current_time - last_time
                    if dt > 0:
                        # A. 급가속 감지
                        delta_percent = current_pedal_value - last_pedal_value
                        delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                        angular_velocity = delta_angle / dt
                        
                        if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                            print(f"!!! 급가속 감지 ({angular_velocity:.1f} deg/s)")
                            trigger_safety = True
                            detected_reason = "⚠️ 급발진 감지!"

                        # B. 과속 연타 감지
                        is_over_90 = (current_pedal_value >= 90)
                        if is_over_90 and not prev_over_90:
                            press_timestamps.append(current_time)
                        while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                            press_timestamps.popleft()
                        if len(press_timestamps) >= RAPID_PRESS_COUNT:
                            print(f"!!! 과속 연타 감지")
                            trigger_safety = True
                            detected_reason = "🚫 과속 페달 연타!"
                            press_timestamps.clear()
                        prev_over_90 = is_over_90

                    # 감지 결과 적용
                    if trigger_safety:
                        safety_lock_active = True
                        safety_cause_msg = detected_reason
                        override_end_time = current_time + SAFETY_LOCK_DURATION
                        target_speed = SAFETY_SPEED
                        current_remaining_time = int(SAFETY_LOCK_DURATION)
                    else:
                        # 평상시 주행 (공회전 로직)
                        if current_pedal_value > 0:
                            last_pedal_active_time = current_time
                            target_speed = max(current_pedal_value, IDLE_SPEED)
                        else:
                            if (current_time - last_pedal_active_time) >= IDLE_TIMEOUT:
                                target_speed = 0
                            else:
                                target_speed = IDLE_SPEED

                # ================= [모터 출력 적용] =================
                pwm_a.ChangeDutyCycle(target_speed)
                pwm_b.ChangeDutyCycle(target_speed)
                current_duty = target_speed
                
                last_pedal_value = current_pedal_value
                last_time = current_time

            time.sleep(0.01)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        pwm_a.stop(); pwm_b.stop(); GPIO.cleanup()
        if ser and ser.is_open: ser.close()

def start_hardware():
    t = threading.Thread(target=hardware_loop, daemon=True)
    t.start()

# ---- 5. 웹소켓 수정 ----
@router.websocket("")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if stop_threads: break
            payload = {
                "duty": round(current_duty, 1),
                "pedal": current_pedal_raw,
                "reason": current_safety_reason,
                "remaining_time": current_remaining_time # [수정됨] 남은 시간 전송
            }
            await websocket.send_json(payload)
            await asyncio.sleep(0.05)
    except Exception:
        pass