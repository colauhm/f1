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
    # 윈도우용 가짜 GPIO (에러 방지용)
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

# 라우터 설정 (경로: /ws)
router = APIRouter(prefix="/ws")

# ---- 2. 핀 설정 (듀얼 모터) ----
# [Motor A - 왼쪽]
PWM_A_PIN = 13
IN1_PIN = 23
IN2_PIN = 24

# [Motor B - 오른쪽]
PWM_B_PIN = 12
IN3_PIN = 5
IN4_PIN = 6

# 아두이노 시리얼 설정
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# ---- 3. 주행 및 안전 설정 ----
PEDAL_TOTAL_ANGLE = 45.0       # 페달 총 각도
CRITICAL_ANGULAR_VELOCITY = 310 # 급가속 판단 기준 (deg/s)
RAPID_PRESS_COUNT = 3          # 과속 연타 횟수 기준
RAPID_PRESS_WINDOW = 2.0       # 연타 감지 시간(초)
SAFETY_LOCK_DURATION = 5.0     # 안전 제한 지속 시간(초)

SAFETY_SPEED = 15       # 제한 걸렸을 때 속도
IDLE_SPEED = 15         # 기본 공회전 속도
IDLE_TIMEOUT = 5.0      # 자동 정지 대기 시간

# ---- 전역 변수 (스레드 간 공유) ----
current_duty = 0.0          # 현재 모터 출력 (%)
current_pedal_raw = 0       # 현재 페달 값 (0~100)
current_safety_reason = None # [핵심] 현재 제한 원인 (None이면 정상)
stop_threads = False        # 스레드 종료 플래그

# ---- 4. 하드웨어 제어 루프 (스레드) ----
def hardware_loop():
    global current_duty, current_pedal_raw, current_safety_reason, stop_threads

    # GPIO 초기화
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # 핀 모드 설정
    GPIO.setup(PWM_A_PIN, GPIO.OUT)
    GPIO.setup(IN1_PIN, GPIO.OUT)
    GPIO.setup(IN2_PIN, GPIO.OUT)
    
    GPIO.setup(PWM_B_PIN, GPIO.OUT)
    GPIO.setup(IN3_PIN, GPIO.OUT)
    GPIO.setup(IN4_PIN, GPIO.OUT)

    # PWM 시작
    pwm_a = GPIO.PWM(PWM_A_PIN, 1000)
    pwm_a.start(0)
    
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000)
    pwm_b.start(0)

    # 방향 설정 (전진)
    GPIO.output(IN1_PIN, True); GPIO.output(IN2_PIN, False)
    GPIO.output(IN3_PIN, True); GPIO.output(IN4_PIN, False)

    # 로직용 변수들
    press_timestamps = deque()
    override_end_time = 0
    last_pedal_value = 0
    last_time = time.time()
    prev_over_90 = False
    last_pedal_active_time = time.time()

    print(f"HW Loop: 포트 {SERIAL_PORT} 연결 시도...")
    
    ser = None
    try:
        # 리눅스일 때만 실제 시리얼 연결
        if PLATFORM == "LINUX":
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            ser.flush()
            print("HW Loop: 아두이노 연결 성공! 제어 시작.")

        while not stop_threads:
            # 윈도우 테스트용 (시리얼 없으면 대기)
            if ser is None:
                if PLATFORM == "WINDOWS": time.sleep(1)
                continue
            
            # 시리얼 데이터 읽기
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if not line.isdigit(): continue

                    # --- 데이터 처리 ---
                    current_pedal_value = int(line)
                    # 범위 제한 (0~100)
                    current_pedal_value = max(0, min(100, current_pedal_value))
                    
                    current_pedal_raw = current_pedal_value
                    current_time = time.time()

                    # ================= [안전 및 속도 계산 로직] =================
                    
                    # 1. 안전 제한 시간이 지났는지 확인 -> 지났으면 경고 해제
                    if current_time > override_end_time:
                        current_safety_reason = None 

                    # 2. 현재 안전 제한 상태인지 확인
                    if current_time < override_end_time:
                        # [제한 상태]
                        target_speed = SAFETY_SPEED
                        last_pedal_active_time = current_time # 제한 중에는 공회전 타이머 리셋
                    
                    else:
                        # [정상 감지 상태]
                        trigger_safety = False
                        detected_reason = "" # 감지된 원인 임시 저장

                        dt = current_time - last_time
                        
                        # A. 급가속 감지
                        if dt > 0:
                            delta_percent = current_pedal_value - last_pedal_value
                            delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                            angular_velocity = delta_angle / dt
                            
                            if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                                print(f"!!! 급가속 감지 ({angular_velocity:.1f} deg/s)")
                                trigger_safety = True
                                detected_reason = "⚠️ 급발진 감지! (Sudden Accel)"

                        # B. 과속 연타 감지
                        is_over_90 = (current_pedal_value >= 90)
                        if is_over_90 and not prev_over_90:
                            press_timestamps.append(current_time)
                        
                        # 오래된 기록 삭제
                        while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                            press_timestamps.popleft()

                        if len(press_timestamps) >= RAPID_PRESS_COUNT:
                            print(f"!!! 과속 연타 감지 ({len(press_timestamps)}회)")
                            trigger_safety = True
                            detected_reason = "🚫 과속 페달 연타! (Rapid Press)"
                            press_timestamps.clear()
                        
                        prev_over_90 = is_over_90

                        # [결과 적용]
                        if trigger_safety:
                            # 제한 발동!
                            override_end_time = current_time + SAFETY_LOCK_DURATION
                            target_speed = SAFETY_SPEED
                            current_safety_reason = detected_reason # 웹으로 보낼 원인 저장
                        else:
                            # 정상 주행
                            if current_pedal_value > 0:
                                last_pedal_active_time = current_time
                                target_speed = max(current_pedal_value, IDLE_SPEED)
                            else:
                                # 페달 뗀 상태 -> 일정 시간 후 완전 정지
                                idle_duration = current_time - last_pedal_active_time
                                if idle_duration >= IDLE_TIMEOUT:
                                    target_speed = 0
                                else:
                                    target_speed = IDLE_SPEED

                    # ================= [모터 출력 적용] =================
                    pwm_a.ChangeDutyCycle(target_speed)
                    pwm_b.ChangeDutyCycle(target_speed)
                    
                    current_duty = target_speed

                    last_pedal_value = current_pedal_value
                    last_time = current_time

                except ValueError:
                    pass
                except Exception as e:
                    print(f"Logic Error: {e}")

            # 루프 속도 조절
            time.sleep(0.01)

    except serial.SerialException:
        print("아두이노 연결 실패! 케이블을 확인하세요.")
    except Exception as e:
        print(f"Critical Error: {e}")
    finally:
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        if ser and ser.is_open:
            ser.close()
        print("Hardware Thread Stopped.")

# ---- 5. 외부에서 실행할 함수 ----
def start_hardware():
    t = threading.Thread(target=hardware_loop, daemon=True)
    t.start()

# ---- 6. 웹소켓 엔드포인트 ----
@router.websocket("")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if stop_threads: break
            
            # 웹으로 보낼 데이터 패키지
            payload = {
                "duty": round(current_duty, 1),
                "pedal": current_pedal_raw,
                "reason": current_safety_reason  # None이면 경고창 꺼짐, 값이 있으면 켜짐
            }
            
            await websocket.send_json(payload)
            
            # 전송 주기 (20 FPS)
            await asyncio.sleep(0.05)
            
    except Exception as e:
        print(f"WebSocket Disconnected: {e}")