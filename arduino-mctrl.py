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

# 1. [안전 모드] 위험 감지 시 속도 (0 = 전력 차단)
SAFETY_SPEED = 15       

# 2. [기본 주행 속도] (페달 안 밟았을 때)
IDLE_SPEED = 20

# 3. [추가] 자동 정지 시간 설정
# 페달을 밟지 않고 이 시간이 지나면 IDLE_SPEED를 끄고 0으로 만듦
IDLE_TIMEOUT = 5.0 

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

# [추가] 마지막으로 페달을 밟았던 시간 (초기화: 현재시간)
last_pedal_active_time = time.time()

print(f"시스템 시작: 포트 {SERIAL_PORT} 연결 대기중...")

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
                
                # --- [로직 A] 급발진 안전 모드 (최우선) ---
                if current_time < override_end_time:
                    target_speed = SAFETY_SPEED 
                    print(f"!!! 위험 감지! 동력 차단됨 ({override_end_time - current_time:.1f}초 남음)")
                    # 안전모드 중에는 '마지막 활동 시간'을 계속 갱신해줌 (안전모드 풀리자마자 바로 꺼지는 것 방지)
                    last_pedal_active_time = current_time 
                
                else:
                    # --- [로직 B] 위험 감지 판단 ---
                    trigger_safety = False
                    
                    # B-1. 각속도
                    dt = current_time - last_time
                    if dt > 0: 
                        delta_percent = current_pedal_value - last_pedal_value
                        delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                        angular_velocity = delta_angle / dt 
                        
                        if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                            print(f"[감지] 급가속! ({angular_velocity:.1f} deg/s)")
                            trigger_safety = True

                    # B-2. 3회 연타
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
                        target_speed = SAFETY_SPEED
                    else:
                        # === [핵심 수정 로직] 자동 정지 기능 ===
                        
                        if current_pedal_value > 0:
                            # 1. 페달을 밟고 있음 -> 타이머 리셋 및 정상 주행
                            last_pedal_active_time = current_time
                            # 30%보다 낮으면 30%로, 높으면 밟은 만큼
                            target_speed = max(current_pedal_value, IDLE_SPEED)
                        
                        else:
                            # 2. 페달을 안 밟고 있음 (0%)
                            # 얼마나 오래 안 밟았는지 계산
                            idle_duration = current_time - last_pedal_active_time
                            
                            if idle_duration >= IDLE_TIMEOUT:
                                # 5초 지남 -> 자동 정지 (0%)
                                target_speed = 0
                                # 너무 자주 출력되지 않게 1초에 한 번 정도만 출력하려면 조건 추가 가능
                                # print("자동 정지 모드 (대기 상태)") 
                            else:
                                # 아직 5초 안 지남 -> 공회전 주행 (30%)
                                target_speed = IDLE_SPEED

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