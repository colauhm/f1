import RPi.GPIO as GPIO
import time

TRIG = 27
ECHO = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

print(f"--- 진단 시작 (Trig: {TRIG}, Echo: {ECHO}) ---")

try:
    while True:
        # 1. 트리거 발사 준비
        GPIO.output(TRIG, False)
        time.sleep(0.5)

        print("1. 신호 발사 시도...", end="")
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        print(" 완료. ", end="")

        # 2. 에코 대기 (High가 될 때까지)
        print("2. 에코 응답 대기중...", end="")
        timeout = time.time() + 0.1 # 0.1초 타임아웃
        
        pulse_start = time.time()
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                print(" [실패] 에코 신호가 안 옴 (연결 확인 필요)")
                break
        
        if pulse_start > timeout: continue # 다음 루프로

        # 3. 에코 수신 중 (Low가 될 때까지)
        pulse_end = time.time()
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                 print(" [실패] 에코 신호가 안 끊김 (센서 불량 의심)")
                 break

        print(" [성공] 수신 완료!")
        
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)

        print(f">>> 측정된 거리: {distance} cm")

except KeyboardInterrupt:
    print("종료")
    GPIO.cleanup()