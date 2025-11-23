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

# [수정] 관성 주행을 위해 속도를 0으로 설정
# 0%를 주면 모터에 전기가 끊겨 바퀴가 관성으로 굴러갑니다.
SAFETY_SPEED = 0                

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
    print("연결 성공! 모니터링 시작.")

    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line.isdigit():
                    continue
                
                # 1. 현재 데이터 수집
                current_pedal_value = int(line)
                if current_pedal_value < 0: current_pedal_value = 0
                if current_pedal_value > 100: current_pedal_value = 100
                
                current_time = time.time()
                
                # --- [로직 A] 5초간 전력 차단 (관성 주행) ---
                if current_time < override_end_time:
                    target_speed = SAFETY_SPEED # 0% (전력 차단)
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
                            print(f"[감지] 급발진 페달링! 각속도: {angular_velocity:.1f} deg/s")
                            trigger_safety = True

                    # B-2. 3회 연타 감지
                    is_over_90 = (current_pedal_value >= 90)
                    if is_over_90 and not prev_over_90:
                        press_timestamps.append(current_time)
                        
                    while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                        press_timestamps.popleft()
                    
                    if len(press_timestamps) >= RAPID_PRESS_COUNT:
                        print(f"[감지] 2초 내 {len(press_timestamps)}회 과속 페달링!")
                        trigger_safety = True
                        press_timestamps.clear() 

                    # B-3. 결과 적용
                    prev_over_90 = is_over_90 

                    if trigger_safety:
                        override_end_time = current_time + SAFETY_LOCK_DURATION
                        target_speed = SAFETY_SPEED # 0으로 설정됨
                    else:
                        target_speed = current_pedal_value

                # --- 모터 구동 ---
                pwm.ChangeDutyCycle(target_speed)
                
                last_pedal_value = current_pedal_value
                last_time = current_time

            except ValueError:
                pass
            except Exception as e:
                print(f"에러 발생: {e}")
                
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