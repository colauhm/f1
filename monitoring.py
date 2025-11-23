import serial
import time

# 위에서 확인한 포트 이름을 적어주세요 (보통 /dev/ttyACM0)
PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

try:
    # 아두이노와 연결 시도
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    ser.flush() # 남아있는 쓰레기 데이터 비우기
    print(f"아두이노({PORT})와 연결되었습니다!")
    print("엑셀 페달 모니터링 시작... (Ctrl+C로 종료)")

    while True:
        # 들어온 데이터가 있는지 확인
        if ser.in_waiting > 0:
            # 1. 한 줄을 읽어옵니다 (바이트 형태)
            line = ser.readline()
            
            # 2. 문자로 변환하고 공백/줄바꿈 제거
            try:
                decoded_line = line.decode('utf-8').strip()
                
                # 3. 숫자인지 확인하고 출력
                if decoded_line.isdigit():
                    value = int(decoded_line)
                    # 보기 좋게 막대 그래프처럼 출력해보기
                    bar = "█" * (value // 5) # 5%당 네모 한 칸
                    print(f"밟음: {value:3d}%  |{bar}")
                    
            except UnicodeDecodeError:
                # 통신 도중 데이터가 깨지면 무시
                pass
                
except serial.SerialException:
    print(f"오류: {PORT} 포트를 찾을 수 없습니다. 아두이노가 잘 꽂혀있나요?")
except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()