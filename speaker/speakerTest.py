import numpy as np
import sounddevice as sd
import os
import time

def set_volume_max():
 
    # -c 3은 네 환경(카드 3번)에 맞춘 것. 필요시 수정.
    os.system("amixer -c 3 set PCM 20% > /dev/null 2>&1")

def generate_siren_wave(duration, start_freq=400, end_freq=1500):
    """
    주파수가 올라갔다 내려가는 사각파 생성
    """
    sample_rate = 44100
    total_samples = int(sample_rate * duration)
    half_samples = total_samples // 2

    # 1. 주파수 배열 생성 (Frequency Sweep)
    # 전반부: 400Hz -> 1500Hz (상승)
    freq_up = np.linspace(start_freq, end_freq, half_samples)
    # 후반부: 1500Hz -> 400Hz (하강)
    freq_down = np.linspace(end_freq, start_freq, total_samples - half_samples)
    
    # 두 구간 합치기
    frequencies = np.concatenate([freq_up, freq_down])

    # 2. 위상(Phase) 계산 (핵심 로직)
    # 주파수가 계속 변하므로 단순 곱셈이 아니라 누적 합(적분)을 해야 소리가 안 깨짐
    phases = 2 * np.pi * np.cumsum(frequencies) / sample_rate

    # 3. 사각파 생성 (Square Wave)
    # np.sin으로 파형을 만들고 np.sign으로 사각파로 변환 (찢어지는 소리)
    wave = np.sign(np.sin(phases))
    
    return wave.astype(np.float32)

# ---- 실행 ----

# 1. 장치 설정 (아까 확인한 3번)
try:
    sd.default.device = 3
except:
    pass

print("🚨 공습 경보 발령! (상승-하강)")

# 볼륨 최대로
set_volume_max()

# 사이렌 생성 (총 3초: 1.5초 상승, 1.5초 하강)
siren_wave = generate_siren_wave(duration=3.0, start_freq=400, end_freq=1500)

# 재생 (볼륨 0.5로 설정, 사각파라 충분히 큼)
sd.play(siren_wave * 0.5, 44100, blocking=True)

print("🚨 종료")

# (선택) 볼륨 원복
os.system("amixer -c 3 set PCM 70% > /dev/null 2>&1")