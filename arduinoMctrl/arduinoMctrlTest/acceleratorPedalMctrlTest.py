import time
import serial
from collections import deque

# ---- 1. OS 판별 및 하드웨어 라이브러리 설정 ----
try:
    import RPi.GPIO as GPIO
    PLATFORM = "LINUX"
except ImportError:
    PLATFORM = "WINDOWS"
    # 윈도우/Mac 테스트용 가짜 GPIO
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

def main():
    # ---- GPIO 초기화 ----
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup(PWM_A_PIN, GPIO.OUT); GPIO.setup(IN1_PIN, GPIO.OUT); GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(PWM_B_PIN, GPIO.OUT); GPIO.setup(IN3_PIN, GPIO.OUT); GPIO.setup(IN4_PIN, GPIO.OUT)

    pwm_a = GPIO.PWM(PWM_A_PIN, 1000); pwm_a.start(0)
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000); pwm_b.start(0)
    
    # 전진 방향 설정
    GPIO.output(IN1_PIN, True); GPIO.output(IN2_PIN, False)
    GPIO.output(IN3_PIN, True); GPIO.output(IN4_PIN, False)

    # ---- 로직 변수 초기화 ----
    press_timestamps = deque()
    override_end_time = 0
    last_pedal_value = 0
    last_time = time.time()
    prev_over_90 = False
    last_pedal_active_time = time.time()
    
    # 안전 모드 상태 관리
    safety_lock_active = False 
    safety_cause_msg = ""
    
    current_pedal_raw = 0
    target_speed = 0

    print("="*50)
    print(f"모터 제어 테스트 시작 (Platform: {PLATFORM})")
    print(f"시리얼 포트: {SERIAL_PORT}")
    print("종료하려면 Ctrl + C를 누르세요.")
    print("="*50)

    # 시리얼 연결 시도
    ser = None
    if PLATFORM == "LINUX":
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            ser.flush()
            print("아두이노 연결 성공!")
        except Exception as e:
            print(f"아두이노 연결 실패: {e}")

    try:
        while True:
            # 1. 시리얼 재연결 로직 (Linux)
            if ser is None and PLATFORM == "LINUX":
                try: 
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                    ser.flush()
                    print("아두이노 재연결됨")
                except: 
                    time.sleep(1)
                    continue
            
            # 윈도우 테스트용 더미 데이터 (주석 해제시 사용 가능)
            # if PLATFORM == "WINDOWS": time.sleep(0.1); current_pedal_raw = 0 # 테스트값 입력

            # 2. 데이터 읽기
            raw_line = ""
            if ser and ser.in_waiting > 0:
                try:
                    raw_line = ser.readline().decode('utf-8').strip()
                except:
                    pass
            
            if raw_line.isdigit():
                current_pedal_value = int(raw_line)
                current_pedal_value = max(0, min(100, current_pedal_value))
                current_pedal_raw = current_pedal_value
                current_time = time.time()

                # ================= [안전 및 속도 제어 로직] =================
                trigger_safety = False
                status_log = "NORMAL"

                # [상황 1] 안전 모드 작동 중
                if safety_lock_active:
                    remaining = override_end_time - current_time
                    
                    if remaining > 0:
                        status_log = f"LOCKED ({int(remaining)}s left)"
                        target_speed = SAFETY_SPEED
                        last_pedal_active_time = current_time 
                    else:
                        # 시간 종료됨. 페달 체크
                        if current_pedal_value > 0:
                            status_log = "WAITING RELEASE (Pedal > 0)"
                            target_speed = SAFETY_SPEED
                        else:
                            # 해제 조건 충족
                            safety_lock_active = False
                            print("\n>>> [안전 모드 해제됨] 정상 주행 복귀 <<<\n")
                            target_speed = 0
                            status_log = "UNLOCKED"

                # [상황 2] 정상 주행
                else:
                    dt = current_time - last_time
                    if dt > 0:
                        # 급가속 감지
                        delta_percent = current_pedal_value - last_pedal_value
                        delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                        angular_velocity = delta_angle / dt
                        
                        if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                            print(f"\n!!! 급가속 감지 ({angular_velocity:.1f} deg/s) !!!")
                            trigger_safety = True
                            safety_cause_msg = "Sudden Accel"

                        # 과속 연타 감지
                        is_over_90 = (current_pedal_value >= 90)
                        if is_over_90 and not prev_over_90:
                            press_timestamps.append(current_time)
                        while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                            press_timestamps.popleft()
                        if len(press_timestamps) >= RAPID_PRESS_COUNT:
                            print(f"\n!!! 과속 연타 감지 ({len(press_timestamps)}회) !!!")
                            trigger_safety = True
                            safety_cause_msg = "Rapid Press"
                            press_timestamps.clear()
                        prev_over_90 = is_over_90

                    # 감지 결과 적용
                    if trigger_safety:
                        safety_lock_active = True
                        override_end_time = current_time + SAFETY_LOCK_DURATION
                        target_speed = SAFETY_SPEED
                        status_log = f"TRIGGERED: {safety_cause_msg}"
                    else:
                        # 공회전 및 일반 주행
                        if current_pedal_value > 0:
                            last_pedal_active_time = current_time
                            target_speed = max(current_pedal_value, IDLE_SPEED)
                        else:
                            if (current_time - last_pedal_active_time) >= IDLE_TIMEOUT:
                                target_speed = 0
                                status_log = "IDLE STOP"
                            else:
                                target_speed = IDLE_SPEED
                                status_log = "IDLE RUN"

                # ================= [모터 출력] =================
                pwm_a.ChangeDutyCycle(target_speed)
                pwm_b.ChangeDutyCycle(target_speed)
                
                last_pedal_value = current_pedal_value
                last_time = current_time

                # 터미널에 상태 출력 (캐리지 리턴 \r 사용하여 한 줄에서 갱신)
                print(f"\rPedal: {current_pedal_raw:3d} | Motor: {target_speed:3d}% | Status: {status_log:<30}", end="")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n테스트 종료 (사용자 중단)")
    except Exception as e:
        print(f"\n오류 발생: {e}")
    finally:
        pwm_a.stop(); pwm_b.stop()
        GPIO.cleanup()
        if ser and ser.is_open: ser.close()
        print("하드웨어 리소스 해제 완료.")

if __name__ == "__main__":
    main()