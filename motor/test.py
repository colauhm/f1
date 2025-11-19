import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

pwm_pin = 13
in1_pin = 23
in2_pin = 24

GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)

pwm = GPIO.PWM(pwm_pin, 1000)  # 1 kHz frequency
pwm.start(0)

while True :
    GPIO.output(in1_pin, True)
    GPIO.output(in2_pin, False)