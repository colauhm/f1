import serial
import time

# 아두이노가 연결된 포트 설정 (보통 /dev/ttyACM0 또는 /dev/ttyUSB0)
# 터미널에서 'ls /dev/tty*' 명령어로 확인 가능합니다.
arduino_port = '/dev/ttyACM0' 
baud_rate = 9600

try:
    # 시리얼 통신 연결
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    ser.flush() # 버퍼 비우기

    print(f"아두이노와 연결되었습니다: {arduino_port}")
    
    while True:
        if ser.in_waiting > 0:
            # 아두이노가 보낸 줄을 읽어서 디코딩
            line = ser.readline().decode('utf-8').rstrip()
            
            # 숫자인지 확인 후 출력
            if line.isdigit():
                print(f"현재 엑셀 페달 밟음: {line}%")
            
except serial.SerialException:
    print("아두이노를 찾을 수 없습니다. 케이블 연결을 확인하세요.")
except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()