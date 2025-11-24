import numpy as np
import sounddevice as sd
import time

def beep(frequency, duration, volume=0.5):
    """
    frequency: 주파수 (Hz)
    duration: 지속 시간 (초)
    volume: 소리 크기 (0.0 ~ 1.0)
    """
    sample_rate = 44100
    # 시간축 생성
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    # 사인파 생성 (소리 데이터)
    wave = np.sin(frequency * t * 2 * np.pi)
    # 볼륨 적용 및 데이터 타입 변환 (float32)
    wave = (wave * volume).astype(np.float32)
    
    # 재생 (blocking=True: 소리 끝날 때까지 대기)
    sd.play(wave, sample_rate, blocking=True)

# ---- 1. 장치 확인 (터미널 출력 확인용) ----
print("=== 현재 연결된 오디오 장치 목록 ===")
print(sd.query_devices())
print("=====================================")

# [중요] 아까 카드 3번이었으므로, 장치 번호를 설정해야 함.
# 목록을 보고 USB Audio의 번호를 적어주면 됨. (보통 3, 4번 쯤에 있음)
# 만약 ~/.asoundrc 설정을 했다면 이 줄을 지워도 됨.
try:
    # 일단 3번으로 시도해봄 (안되면 목록 보고 수정)
    sd.default.device = 3 
except:
    print("장치 설정 실패: 기본 장치로 시도합니다.")

# ---- 2. 소리 테스트 ----
print("테스트 시작...")

print("도 (261 Hz)")
beep(261.63, 0.5) 

print("미 (329 Hz)")
beep(329.63, 0.5)

print("솔 (392 Hz)")
beep(392.00, 0.5)

time.sleep(0.5)

print("!!! 경고음 테스트 (삐-삐-삐) !!!")
for _ in range(3):
    beep(1000, 0.1, volume=0.8) # 1000Hz 고음, 짧게
    time.sleep(0.1)

print("테스트 완료")