import numpy as np
import sounddevice as sd
import os
import time

def set_volume_max():
   
    # -c 3은 네 환경(카드 3번)에 맞춘 것
    os.system("amixer -c 3 set PCM 20% > /dev/null 2>&1")

def generate_square_wave(freq, duration, sample_rate=44100):
    """찢어지는 듯한 사각파(Square Wave) 생성"""
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    # np.sign을 사용하면 둥근 사인파가 네모난 사각파로 바뀌어 소리가 훨씬 거칠어짐
    wave = np.sign(np.sin(freq * t * 2 * np.pi))
    return wave.astype(np.float32)

def play_emergency_siren(duration=2.0):
    sample_rate = 44100
    
    # 1. 소리 패턴 만들기 (0.15초씩 고음-저음 반복)
    # 1200Hz(고음)과 600Hz(저음)을 섞으면 매우 긴박하게 들림
    high_tone = generate_square_wave(1200, 0.15, sample_rate)
    low_tone = generate_square_wave(600, 0.15, sample_rate)
    
    # 두 소리를 합쳐서 하나의 사이렌 주기(0.3초)를 만듦
    cycle = np.concatenate([high_tone, low_tone])
    
    # 2. 2초 동안 반복되도록 복사 붙여넣기
    # 필요한 반복 횟수 계산
    repeats = int(duration / 0.3) + 1
    # 소리 패턴을 반복해서 긴 배열로 만듦
    full_siren = np.tile(cycle, repeats)
    
    # 정확히 duration 길이만큼 자르기
    total_samples = int(sample_rate * duration)
    full_siren = full_siren[:total_samples]
    
    # 3. 재생 (Volume 0.8로 낮춰도 사각파라 충분히 시끄러움. 필요하면 1.0으로)
    sd.play(full_siren * 0.5, sample_rate, blocking=True)

# ---- 실행 ----

# 1. 장치 설정 (아까 확인한 3번으로 고정)
try:
    sd.default.device = 3
except:
    pass

print("🚨 재난 경보 발령! (2초간 재생)")

# 볼륨 최대로!
set_volume_max()

# 사이렌 울림
play_emergency_siren(2.0)

print("🚨 경보 종료")

# (선택) 귀가 아프다면 다시 볼륨을 줄여놓는 코드 추가
os.system("amixer -c 3 set PCM 70% > /dev/null 2>&1")