import asyncio
import threading
import time
import serial
from collections import deque
from fastapi import APIRouter, WebSocket
import os

# ---- [추가] 사운드 관련 라이브러리 ----
import numpy as np
import sounddevice as sd

# ---- 1. OS 판별 및 하드웨어 라이브러리 설정 ----
try:
    import RPi.GPIO as GPIO
    PLATFORM = "LINUX"
except ImportError:
    PLATFORM = "WINDOWS"
    class MockGPIO:
        BCM = "BCM"; OUT = "OUT"
        def setmode(self, m): pass
        def setwarnings(self, f): pass
        def setup(self, p, m): pass
        def output(self, p, s): pass
        def cleanup(self): print("GPIO Cleaned up")
        class PWM:
            def __init__(self, p, f): pass
            def start(self, d): pass
            def ChangeDutyCycle(self, d): pass
            def stop(self): pass
        GPIO = MockGPIO()

router = APIRouter(prefix="/ws")

# ---- 2. 핀 설정 ----
PWM_A_PIN = 13; IN1_PIN = 23; IN2_PIN = 24
PWM_B_PIN = 12; IN3_PIN = 5; IN4_PIN = 6

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# ---- 3. 주행 및 안전 설정 ----
PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 310
RAPID_PRESS_COUNT = 3
RAPID_PRESS_WINDOW = 2.0
SAFETY_LOCK_DURATION = 5.0

SAFETY_SPEED = 15
IDLE_SPEED = 15
IDLE_TIMEOUT = 5.0

# ---- [추가] 오디오 설정 ----
AUDIO_CARD_ID = 3  # 사용자 환경에 맞춘 오디오 카드 번호

# ---- 전역 변수 ----
current_duty = 0.0
current_pedal_raw = 0
current_safety_reason = None
current_remaining_time = 0
stop_threads = False

# ---- [수정됨] 경고음 재생 함수 (쓰레드용) ----
def play_siren_thread():
    """
    모터 제어 루프를 방해하지 않기 위해 별도 쓰레드에서 소리를 재생합니다.
    기존 사이렌 대신 무거운 '삐- 삐- 삐-' 소리를 재생합니다.
    """
    def _run_siren():
        try:
            # 1. 장치 설정
            try:
                sd.default.device = AUDIO_CARD_ID
            except Exception as e:
                print(f"[Audio Error] Device setup failed: {e}")
                return

            print("🚨 경고음 발령! (소리 재생 시작)")
            
            # 2. 볼륨 설정 (사용자 요청: 20%)
            os.system(f"amixer -c {AUDIO_CARD_ID} set PCM 20% > /dev/null 2>&1")

            # 3. 파형 생성 (무거운 삐- 삐- 삐- 소리)
            sample_rate = 44100
            beep_freq = 500       # 주파수 (낮을수록 무거운 소리, 500Hz 설정)
            beep_duration = 0.5   # 삐- 지속 시간 (초)
            silence_duration = 0.5 # 멈춤 지속 시간 (초)
            repeats = 3           # 반복 횟수 (0.5초 삐 + 0.5초 멈춤 x 3회 = 총 3초)

            # 단일 '삐-' 소리 생성 (사각파로 무거운 느낌)
            # np.sign(np.sin(...))을 사용하여 사인파를 사각파로 변환합니다.
            t_beep = np.linspace(0, beep_duration, int(sample_rate * beep_duration), endpoint=False)
            beep_wave = np.sign(np.sin(2 * np.pi * beep_freq * t_beep)).astype(np.float32)

            # '무음' 구간 생성
            silence_wave = np.zeros(int(sample_rate * silence_duration), dtype=np.float32)

            # [삐, 무음] 패턴을 반복하여 전체 파형 완성
            full_wave = np.concatenate([beep_wave, silence_wave] * repeats)

            # 4. 재생 (blocking=True여도 이 함수는 메인 루프와 별개이므로 상관없음)
            # 사각파는 소리가 크므로 볼륨을 0.5배로 낮춰서 재생합니다.
            sd.play(full_wave * 0.5, sample_rate, blocking=True)
            
            # 5. 볼륨 원복 (선택사항)
            os.system(f"amixer -c {AUDIO_CARD_ID} set PCM 70% > /dev/null 2>&1")
            print("🚨 소리 재생 종료")

        except Exception as e:
            print(f"[Audio Error] Playback failed: {e}")

    # 별도 쓰레드로 실행하여 메인 루프 지연 방지
    threading.Thread(target=_run_siren, daemon=True).start()


# ---- 4. 하드웨어 제어 루프 ----
def hardware_loop():
    global current_duty, current_pedal_raw, current_safety_reason, current_remaining_time, stop_threads

    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup(PWM_A_PIN, GPIO.OUT); GPIO.setup(IN1_PIN, GPIO.OUT); GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(PWM_B_PIN, GPIO.OUT); GPIO.setup(IN3_PIN, GPIO.OUT); GPIO.setup(IN4_PIN, GPIO.OUT)

    pwm_a = GPIO.PWM(PWM_A_PIN, 1000); pwm_a.start(0)
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000); pwm_b.start(0)
    
    GPIO.output(IN1_PIN, True); GPIO.output(IN2_PIN, False)
    GPIO.output(IN3_PIN, True); GPIO.output(IN4_PIN, False)

    # 로직 변수
    press_timestamps = deque()
    override_end_time = 0
    last_pedal_value = 0
    last_time = time.time()
    prev_over_90 = False
    last_pedal_active_time = time.time()
    
    # 안전 모드 상태 관리 변수
    safety_lock_active = False 
    safety_cause_msg = "" 

    print(f"HW Loop: 포트 {SERIAL_PORT} 연결 시도...")
    ser = None
    if PLATFORM == "LINUX":
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            ser.flush()
        except: pass

    try:
        while not stop_threads:
            if ser is None and PLATFORM == "LINUX":
                try: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1); ser.flush()
                except: time.sleep(1); continue
            elif ser is None and PLATFORM == "WINDOWS":
                time.sleep(0.1)

            # 시리얼 읽기
            raw_line = ""
            if ser and ser.in_waiting > 0:
                raw_line = ser.readline().decode('utf-8').strip()
            
            if not raw_line and PLATFORM == "WINDOWS": 
                pass 

            if raw_line.isdigit():
                current_pedal_value = int(raw_line)
                current_pedal_value = max(0, min(100, current_pedal_value))
                current_pedal_raw = current_pedal_value
                current_time = time.time()

                # ================= [안전 로직] =================
                trigger_safety = False
                detected_reason = ""

                # 1. 안전 모드가 활성화된 상태라면?
                if safety_lock_active:
                    remaining = override_end_time - current_time
                    current_remaining_time = max(0, int(remaining))

                    if remaining > 0:
                        current_safety_reason = f"{safety_cause_msg}\n(해제까지 {current_remaining_time}초)"
                        target_speed = SAFETY_SPEED
                        last_pedal_active_time = current_time 
                    else:
                        if current_pedal_value > 0:
                            current_safety_reason = "⚠️ 엑셀에서 발을 떼세요!\n(안전 잠금 해제 대기중)"
                            target_speed = SAFETY_SPEED
                        else:
                            safety_lock_active = False
                            current_safety_reason = None
                            current_remaining_time = 0
                            print(">>> 안전 잠금 해제됨")
                            target_speed = 0 

                # 2. 정상 주행 상태 (감지 로직 수행)
                else:
                    dt = current_time - last_time
                    if dt > 0:
                        # A. 급가속 감지
                        delta_percent = current_pedal_value - last_pedal_value
                        delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                        angular_velocity = delta_angle / dt
                        
                        if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                            print(f"!!! 급가속 감지 ({angular_velocity:.1f} deg/s)")
                            trigger_safety = True
                            detected_reason = "⚠️ 급발진 감지!"

                        # B. 과속 연타 감지
                        is_over_90 = (current_pedal_value >= 90)
                        if is_over_90 and not prev_over_90:
                            press_timestamps.append(current_time)
                        while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                            press_timestamps.popleft()
                        if len(press_timestamps) >= RAPID_PRESS_COUNT:
                            print(f"!!! 과속 연타 감지")
                            trigger_safety = True
                            detected_reason = "🚫 과속 페달 연타!"
                            press_timestamps.clear()
                        prev_over_90 = is_over_90

                    # 감지 결과 적용
                    if trigger_safety:
                        safety_lock_active = True
                        safety_cause_msg = detected_reason
                        override_end_time = current_time + SAFETY_LOCK_DURATION
                        target_speed = SAFETY_SPEED
                        current_remaining_time = int(SAFETY_LOCK_DURATION)
                        
                        # [중요] 여기서 소리 재생 함수 호출!
                        play_siren_thread()
                    else:
                        if current_pedal_value > 0:
                            last_pedal_active_time = current_time
                            target_speed = max(current_pedal_value, IDLE_SPEED)
                        else:
                            if (current_time - last_pedal_active_time) >= IDLE_TIMEOUT:
                                target_speed = 0
                            else:
                                target_speed = IDLE_SPEED

                # ================= [모터 출력 적용] =================
                pwm_a.ChangeDutyCycle(target_speed)
                pwm_b.ChangeDutyCycle(target_speed)
                current_duty = target_speed
                
                last_pedal_value = current_pedal_value
                last_time = current_time

            time.sleep(0.01)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        pwm_a.stop(); pwm_b.stop(); GPIO.cleanup()
        if ser and ser.is_open: ser.close()

def start_hardware():
    t = threading.Thread(target=hardware_loop, daemon=True)
    t.start()

# ---- 5. 웹소켓 ----
@router.websocket("")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if stop_threads: break
            payload = {
                "duty": round(current_duty, 1),
                "pedal": current_pedal_raw,
                "reason": current_safety_reason,
                "remaining_time": current_remaining_time
            }
            await websocket.send_json(payload)
            
            # [수정] 0.05 -> 0.01로 변경 (초당 20회 -> 초당 100회 전송)
            # 하드웨어 루프(time.sleep(0.01))와 속도를 맞춰야 중간 데이터를 놓치지 않습니다.
            await asyncio.sleep(0.01) 
            
    except Exception:
        pass