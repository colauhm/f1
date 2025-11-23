import RPi.GPIO as GPIO
import serial
import time

# --- 1. 설정 및 핀 초기화 (사용자 코드 기반) ---
# 모터 드라이버 핀 설정
PWM_PIN = 13
IN1_PIN = 23
IN2_PIN = 24

# 시리얼 포트 설정 (아두이노 연결)
SERIAL_PORT = '/dev/ttyACM0' # 포트 이름 확인 필요 (ls /dev/tty* 로 확인)
BAUD_RATE = 9600

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(IN1_PIN, GPIO.OUT)
GPIO.setup(IN2_PIN, GPIO.OUT)

# PWM 설정 (주파수 1000Hz)
pwm = GPIO.PWM(PWM_PIN, 1000)
pwm.start(0)

# 방향 설정 (전진 고정: 보여주신 코드 기준 True/False)
GPIO.output(IN1_PIN, True)
GPIO.output(IN2_PIN, False)

print(f"모터 제어 시작: {SERIAL_PORT} 연결 대기중...")

try:
    # --- 2. 아두이노 연결 ---
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flush()
    print("연결 성공! 페달을 밟아보세요.")

    # --- 3. 메인 루프 ---
    while True:
        if ser.in_waiting > 0:
            try:
                # 아두이노에서 값 읽기 (예: "50")
                line = ser.readline().decode('utf-8').strip()
                
                # 숫자인지 확인
                if line.isdigit():
                    pedal_value = int(line)
                    
                    # 안전장치: 0~100 범위 넘어가면 자르기
                    if pedal_value < 0: pedal_value = 0
                    if pedal_value > 100: pedal_value = 100
                    
                    # --- 핵심: 페달 값(%)을 듀티 사이클로 바로 적용 ---
                    # RPi.GPIO의 ChangeDutyCycle은 0~100 사이의 값을 받으므로 변환 불필요
                    pwm.ChangeDutyCycle(pedal_value)
                    
                    print(f"페달 입력: {pedal_value}% -> 모터 속도 적용 완료")
                    
            except UnicodeDecodeError:
                pass # 통신 에러 무시
            except ValueError:
                pass # 이상한 값 무시
                
        time.sleep(0.01) # CPU 점유율 낮추기 위한 짧은 대기

except serial.SerialException:
    print("오류: 아두이노가 연결되지 않았습니다.")

except KeyboardInterrupt:
    print("\n종료합니다.")

finally:
    # --- 4. 종료 시 안전하게 멈춤 ---
    pwm.stop()
    GPIO.cleanup()
    if 'ser' in locals() and ser.is_open:
        ser.close()