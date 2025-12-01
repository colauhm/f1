import RPi.GPIO as GPIO
import time

# 핀 설정 (작성해준 코드와 동일하게 설정)
TRIG = 27
ECHO = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

print("초음파 센서 테스트 시작 (Ctrl+C로 종료)")

try:
    while True:
        GPIO.output(TRIG, False)
        time.sleep(0.5)

        # 10us 펄스 발사
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_start = time.time()
        pulse_end = time.time()

        # Echo 핀이 1이 될 때까지 대기 (시작 시간)
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        # Echo 핀이 0이 될 때까지 대기 (종료 시간)
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)

        print(f"거리: {distance} cm")

except KeyboardInterrupt:
    print("테스트 종료")
    GPIO.cleanup()