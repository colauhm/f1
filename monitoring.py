import RPi.GPIO as GPIO
import serial
import time
from collections import deque

# --- [사용자 설정 구간] ---
# 1. 하드웨어 핀 설정
PWM_PIN = 13
IN1_PIN = 23
IN2_PIN = 24
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# 2. 페달 물리적 특성 설정 (중요!)
# 페달을 0%에서 100%까지 밟았을 때 실제 움직이는 각도(도)
# 예: 45도라고 가정. 실제 페달 각도에 맞춰 수정하세요.
PEDAL_TOTAL_ANGLE = 45.0 

# 3. 감지 임계값 설정
CRITICAL_ANGULAR_VELOCITY = 310  # 감지할 각속도 (도/초)
RAPID_PRESS_COUNT = 3            # 몇 번 밟았는지
RAPID_PRESS_WINDOW = 2.0         # 몇 초 이내에 (초)
SAFETY_LOCK_DURATION = 5.0       # 속도 제한 걸리는 시간 (초)
SAFETY_SPEED = 30                # 제한 시 속도 (%)

# --- GPIO 초기화 ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(IN1_PIN, GPIO.OUT)
GPIO.setup(IN2_PIN, GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN, 1000)
pwm.start(0)
GPIO.output(IN1_PIN, True)
GPIO.output(IN2_PIN, False)

# --- 변수 초기화 ---
press_timestamps = deque()  # 90% 이상 밟은 시간 기록용 큐
override_end_time = 0       # 속도 제한이 끝나는 시간
last_pedal_value = 0        # 이전 루프의 페달 값
last_time = time.time()     # 이전 루프의 시간
prev_over_90 = False        # 이전 루프에서 90%를 넘었었는지 (엣지 검출용)

print(f"시스템 시작: 포트 {SERIAL_PORT} 연결 대기중...")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flush()
    print("연결 성공! 모니터링 시작.")

    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line.isdigit():
                    continue
                
                # 1. 현재 데이터 수집
                current_pedal_value = int(line)
                # 범위 보정
                if current_pedal_value < 0: current_pedal_value = 0
                if current_pedal_value > 100: current_pedal_value = 100
                
                current_time = time.time()
                
                # --- [로직 A] 5초간 속도 고정 모드인지 확인 ---
                if current_time < override_end_time:
                    # 안전 모드 작동 중
                    target_speed = SAFETY_SPEED
                    print(f"!!! 안전 모드 작동 중 ({override_end_time - current_time:.1f}초 남음) -> 속도 {SAFETY_SPEED}% 고정")
                
                else:
                    # --- [로직 B] 위험 감지 판단 ---
                    trigger_safety = False
                    
                    # B-1. 각속도 계산 (도/초)
                    dt = current_time - last_time
                    if dt > 0: # 0으로 나누기 방지
                        # (%) 변화량 -> (도) 변화량으로 변환
                        delta_percent = current_pedal_value - last_pedal_value
                        delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                        
                        angular_velocity = delta_angle / dt # (도/초)
                        
                        # 밟는 방향(+)이면서 속도가 기준치 이상일 때
                        if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                            print(f"[감지] 급발진 페달링! 각속도: {angular_velocity:.1f} deg/s")
                            trigger_safety = True

                    # B-2. 2초 이내 90% 이상 3회 밟기 (채터링 방지 포함)
                    is_over_90 = (current_pedal_value >= 90)
                    
                    # "방금 막" 90%를 넘었을 때만 카운트 (Rising Edge)
                    if is_over_90 and not prev_over_90:
                        press_timestamps.append(current_time)
                        
                    # 2초 지난 기록은 삭제
                    while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                        press_timestamps.popleft()
                    
                    # 횟수 확인
                    if len(press_timestamps) >= RAPID_PRESS_COUNT:
                        print(f"[감지] 2초 내 {len(press_timestamps)}회 과속 페달링!")
                        trigger_safety = True
                        press_timestamps.clear() # 트리거 후 기록 초기화

                    # B-3. 결과 적용
                    prev_over_90 = is_over_90 # 다음 루프를 위해 상태 저장

                    if trigger_safety:
                        override_end_time = current_time + SAFETY_LOCK_DURATION
                        target_speed = SAFETY_SPEED
                    else:
                        # 정상 주행
                        target_speed = current_pedal_value

                # --- 모터 구동 ---
                pwm.ChangeDutyCycle(target_speed)
                
                # --- 다음 루프를 위한 변수 업데이트 ---
                last_pedal_value = current_pedal_value
                last_time = current_time

            except ValueError:
                pass
            except Exception as e:
                print(f"에러 발생: {e}")
                
        # CPU 점유율 방지 (너무 길면 각속도 계산 정확도가 떨어지므로 짧게)
        time.sleep(0.01)

except serial.SerialException:
    print("아두이노 연결 끊김")
except KeyboardInterrupt:
    print("\n종료")
finally:
    pwm.stop()
    GPIO.cleanup()
    if 'ser' in locals() and ser.is_open:
        ser.close()