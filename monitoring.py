import RPi.GPIO as GPIO
import serial
import time
from collections import deque

# --- [사용자 설정 구간] ---
PWM_PIN = 13
IN1_PIN = 23
IN2_PIN = 24
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# 페달 물리적 특성
PEDAL_TOTAL_ANGLE = 45.0 

# 감지 임계값 설정
CRITICAL_ANGULAR_VELOCITY = 310  
RAPID_PRESS_COUNT = 3            
RAPID_PRESS_WINDOW = 2.0         
SAFETY_LOCK_DURATION = 5.0       

# 1. [안전 모드] 위험 감지 시 속도 (0 = 전력 차단/관성 주행)
SAFETY_SPEED = 15

# 2. [기본 주행 속도]
# 페달을 밟지 않아도(0%) 항상 유지되는 최소 속도
IDLE_SPEED = 15

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
press_timestamps = deque()  
override_end_time = 0       
last_pedal_value = 0        
last_time = time.time()     
prev_over_90 = False        

print(f"시스템 시작: 포트 {SERIAL_PORT} 연결 대기중...")
print(f"주의: 연결되면 페달을 밟지 않아도 모터가 {IDLE_SPEED}%로 회전합니다!")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flush()
    print("연결 성공! 주행 시작.")

    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line.isdigit():
                    continue
                
                # 1. 데이터 수집
                current_pedal_value = int(line)
                if current_pedal_value < 0: current_pedal_value = 0
                if current_pedal_value > 100: current_pedal_value = 100
                
                current_time = time.time()
                
                # --- [로직 A] 안전 모드 (최우선 순위) ---
                if current_time < override_end_time:
                    # 위험 감지 상태: 기본 속도(30)도 무시하고 0으로 만듦
                    target_speed = SAFETY_SPEED 
                    print(f"!!! 위험 감지! 동력 차단됨 ({override_end_time - current_time:.1f}초 남음)")
                
                else:
                    # --- [로직 B] 위험 감지 판단 ---
                    trigger_safety = False
                    
                    # B-1. 각속도 계산
                    dt = current_time - last_time
                    if dt > 0: 
                        delta_percent = current_pedal_value - last_pedal_value
                        delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                        angular_velocity = delta_angle / dt 
                        
                        if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                            print(f"[감지] 급가속! ({angular_velocity:.1f} deg/s)")
                            trigger_safety = True

                    # B-2. 3회 연타 감지
                    is_over_90 = (current_pedal_value >= 90)
                    if is_over_90 and not prev_over_90:
                        press_timestamps.append(current_time)
                        
                    while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                        press_timestamps.popleft()
                    
                    if len(press_timestamps) >= RAPID_PRESS_COUNT:
                        print(f"[감지] 과속 연타 ({len(press_timestamps)}회)!")
                        trigger_safety = True
                        press_timestamps.clear() 

                    prev_over_90 = is_over_90 

                    # B-3. 속도 결정 (정상 주행)
                    if trigger_safety:
                        override_end_time = current_time + SAFETY_LOCK_DURATION
                        target_speed = SAFETY_SPEED # 0% (동력 차단)
                    else:
                        # --- [핵심 수정] 항상 최소 30% 유지 ---
                        # 페달값(0~100)과 IDLE_SPEED(30) 중 큰 값을 선택
                        # 페달 0% -> 속도 30%
                        # 페달 20% -> 속도 30%
                        # 페달 50% -> 속도 50%
                        target_speed = max(current_pedal_value, IDLE_SPEED)

                # --- 모터 구동 ---
                pwm.ChangeDutyCycle(target_speed)
                
                last_pedal_value = current_pedal_value
                last_time = current_time

            except ValueError:
                pass
            except Exception as e:
                print(f"에러: {e}")
                
        time.sleep(0.01)

except serial.SerialException:
    print("통신 오류")
except KeyboardInterrupt:
    print("\n종료")
finally:
    pwm.stop()
    GPIO.cleanup()
    if 'ser' in locals() and ser.is_open:
        ser.close()