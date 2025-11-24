import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pwm_pin = 13
in1_pin = 23
in2_pin = 24

GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)

pwm = GPIO.PWM(pwm_pin, 1000)  # 1 kHz PWM
pwm.start(0)

try:
    # 모터 방향 설정 (앞으로)
    GPIO.output(in1_pin, True)
    GPIO.output(in2_pin, False)

    while True:
        # 사용자 입력 받기
        value = input("0~4095 입력 (종료: q): ")

        if value.lower() == "q":
            break
        
        # 정수로 변환
        try:
            v = int(value)
        except:
            print("숫자만 입력")
            continue

        # 범위 제한
        if v < 0: v = 0
        if v > 4095: v = 4095

        # 0~4095 → 0~100% 로 매핑
        duty = (v / 4095) * 100

        print(f"듀티사이클: {duty:.1f}%")
        pwm.ChangeDutyCycle(duty)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
