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
    tty.setcbreak(fd)
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

# ---- 속도 관련 설정 (이 부분을 조정하세요) ----
MAX_SPEED = 4095   # 최대 속도
MIN_SPEED = 800   # 최소 유지 속도 (모터가 간신히 도는 정도의 값으로 설정 추천)
ACC = 200          # 가속량

# 초기 속도를 0으로 시작할지, 바로 최소 속도로 켤지 결정
# 여기서는 안전을 위해 0으로 시작하되, 키를 한번이라도 누르거나 떼면 MIN_SPEED가 적용됨
current_speed = 0  

listener = threading.Thread(target=key_listener, daemon=True)
listener.start()

try:
    GPIO.output(in1_pin, True)
    GPIO.output(in2_pin, False)

    print("===========================================")
    print(f" [제어 모드: 최소 속도 유지 ({MIN_SPEED})]")
    print(" w : 가속 (누르는 동안 증가)")
    print(" 키 뗌 : 관성 감속 후 최소 속도 유지")
    print(" q : 종료")
    print("===========================================")

    while True:
        time.sleep(0.05)

        key = pressed_key
        pressed_key = None

        if key == 'q':
            break

        # ------------------------------------------------
        # [수정된 로직]
        # ------------------------------------------------
        if key == 'w':
            # 가속: 현재 속도에서 더함
            # (만약 정지 상태였다면 MIN_SPEED부터 시작하도록 보정해줄 수도 있음)
            if current_speed < MIN_SPEED:
                current_speed = MIN_SPEED
            
            current_speed += ACC
            if current_speed > MAX_SPEED:
                current_speed = MAX_SPEED
        else:
            # 키를 떼면: 강제로 속도를 줄이는 게 아니라
            # 목표 값을 '최소 속도'로 둠.
            # PWM 전압이 MAX에서 MIN으로 뚝 떨어지므로,
            # 모터는 물리적 관성으로 돌다가 마찰에 의해 MIN_SPEED 수준까지 자연스럽게 느려짐.
            current_speed = MIN_SPEED
        # ------------------------------------------------

        duty = (current_speed / MAX_SPEED) * 100
        pwm.ChangeDutyCycle(duty)

        status = "가속 중" if current_speed > MIN_SPEED else "최소 속도 유지"
        print(f"State: {status} | Speed: {current_speed} | Duty: {duty:.1f}%   ", end="\r")

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
    print("\n종료")