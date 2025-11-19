import RPi.GPIO as GPIO
import time
import sys
import tty
import termios
import threading

# ---- 글로벌 변수 선언 ----
pressed_key = None

# ---- 비차단 키 입력 (스레드용) ----
def key_listener():
    global pressed_key
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # 엔터 없이 입력 감지
    try:
        while True:
            pressed_key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# ---- GPIO 설정 ----
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pwm_pin = 13
in1_pin = 23
in2_pin = 24

GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)

pwm = GPIO.PWM(pwm_pin, 1000)  
pwm.start(0)

# ---- 속도 설정 ----
speed = 1000  # 기본 속도
ACC = 80     # 가속량
DEC = 40     # 감속량

# ---- 키 리스너 스레드 시작 ----
listener = threading.Thread(target=key_listener, daemon=True)
listener.start()

try:
    GPIO.output(in1_pin, True)
    GPIO.output(in2_pin, False)

    print("가속키: w / 종료: q")
    print("-------------------")

    while True:
        time.sleep(0.05)  # 50ms 루프

        key = pressed_key
        pressed_key = None  # 읽고 나면 초기화

        if key == 'q':
            break

        # 가속 키
        if key == 'w':
            speed += ACC
            if speed > 4095:
                speed = 4095
        else:
            # 자동 감속
            if speed > 100:
                speed -= DEC
                if speed < 100:
                    speed = 100

        # 듀티사이클 계산
        duty = (speed / 4095) * 100
        pwm.ChangeDutyCycle(duty)

        print(f"Speed: {speed}  Duty: {duty:.1f}%", end="\r")

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
