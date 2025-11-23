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

# 1. [안전 모드] 급발진 감지 시 속도 (관성 주행 = 0)
SAFETY_SPEED = 30

# 2. [추가] 정상 주행 시 최소 구동 속도 (30%)
# 페달을 살짝 밟아도 모터는 30% 힘으로 돌기 시작함 (마찰력 극복용)
MIN_START_SPEED = 30

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

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flush()
    print(f"연결 성공! (최소 속도 {MIN_START_SPEED}% 설정됨)")

    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line.isdigit():
                    continue
                
                # 1. 데이터 수집 및 전처리
                current_pedal_value = int(line)
                if current_pedal_value < 0: current_pedal_value = 0
                if current_pedal_value > 100: current_pedal_value = 100
                
                current_time = time.time()
                
                # --- [로직 A] 안전 모드 (관성 주행) ---
                if current_time < override_end_time:
                    target_speed = SAFETY_SPEED # 0% (전원 차단)
                    print(f"!!! 위험 감지! 관성 주행 중 ({override_end_time - current_time:.1f}초 남음)")
                
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

                    # B-3. 속도 결정
                    if trigger_safety:
                        override_end_time = current_time + SAFETY_LOCK_DURATION
                        target_speed = SAFETY_SPEED # 0%
                    else:
                        # --- [핵심 수정] 정상 주행 시 최소 속도 로직 ---
                        if current_pedal_value == 0:
                            # 발을 완전히 떼면 멈춤
                            target_speed = 30

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