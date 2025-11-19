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

# 핀 번호 설정 (사용자 환경에 맞춤)
pwm_pin = 13
in1_pin = 23
in2_pin = 24

GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)

# PWM 설정 (주파수 1kHz)
pwm = GPIO.PWM(pwm_pin, 1000)  
pwm.start(0)

# ---- 변수 설정 ----
# speed: 현재 모터에 인가하려는 '전력 레벨' (0 ~ 4095)
speed = 0  
ACC = 200     # 가속 시 증가량

# ---- 키 리스너 스레드 시작 ----
listener = threading.Thread(target=key_listener, daemon=True)
listener.start()

try:
    # 정방향 설정 (모터 드라이버에 따라 True/False 방향 확인 필요)
    GPIO.output(in1_pin, True)
    GPIO.output(in2_pin, False)

    print("===========================================")
    print(" [제어 모드: 관성 주행]")
    print(" w : 가속 (누르고 있는 동안 전력 공급)")
    print(" 키 뗌 : 전력 차단 (관성으로 굴러감)")
    print(" q : 프로그램 종료")
    print("===========================================")

    while True:
        time.sleep(0.05)  # 50ms 루프 주기

        key = pressed_key
        pressed_key = None  # 키 입력 처리 후 초기화

        if key == 'q':
            print("\n프로그램을 종료한다.")
            break

        # ------------------------------------------------
        # [핵심 수정 부분]
        # ------------------------------------------------
        if key == 'w':
            # 가속: 키를 누르면 전력 레벨을 높임
            speed += ACC
            if speed > 4095:
                speed = 4095
        else:
            # 관성 주행: 키를 떼면 즉시 전력을 0으로 만듦
            # 전압을 서서히 낮추는 게 아니라 아예 끊어버림으로써
            # 모터가 물리적 관성에 의해서만 돌게 함
            speed = 0
        # ------------------------------------------------

        # 듀티사이클 계산 및 적용
        duty = (speed / 4095) * 100
        pwm.ChangeDutyCycle(duty)

        # 상태 출력 (speed가 0이면 Duty 0% -> 전력 차단 상태)
        status = "가속 중" if speed > 0 else "관성 주행(Free)"
        print(f"State: {status} | Power Level: {speed} | Duty: {duty:.1f}%   ", end="\r")

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
    print("\nGPIO 정리 완료.")