import asyncio
import threading
import time
import serial
import queue
from collections import deque
from fastapi import APIRouter, FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
import os
import numpy as np
import sounddevice as sd

# ---- 1. 하드웨어 설정 ----
try:
    import RPi.GPIO as GPIO
    PLATFORM = "LINUX"
except ImportError:
    PLATFORM = "WINDOWS"
    class MockGPIO:
        BCM = "BCM"; OUT = "OUT"; IN = "IN"; PUD_UP = "PUD_UP"
        def setmode(self, m): pass
        def setwarnings(self, f): pass
        def setup(self, p, m, pull_up_down=None): pass
        def output(self, p, s): pass
        def input(self, p): return 1 
        def cleanup(self): print("GPIO Cleaned up")
        class PWM:
            def __init__(self, p, f): pass
            def start(self, d): pass
            def ChangeDutyCycle(self, d): pass
            def stop(self): pass
    GPIO = MockGPIO()

app = FastAPI()
app.mount("/static", StaticFiles(directory="."), name="static")
router = APIRouter(prefix="/ws")

# ---- 2. 핀 설정 ----
PWM_A_PIN = 13; IN1_PIN = 23; IN2_PIN = 24
PWM_B_PIN = 12; IN3_PIN = 5; IN4_PIN = 6
TRIG_PIN = 27; ECHO_PIN = 17 
BUTTON_PIN = 21 

SERIAL_PORT = '/dev/ttyUSB0'; BAUD_RATE = 115200

# ---- 3. 임계값 설정 ----
PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 420
RAPID_PRESS_COUNT = 3      
RAPID_PRESS_WINDOW = 2.0   
SAFETY_SPEED = 20
IDLE_SPEED = 20
IDLE_TIMEOUT = 5.0
COLLISION_DIST_LIMIT = 100.0 

# ---- 오디오 장치 ----
def get_usb_audio_id():
    try:
        devices = sd.query_devices()
        for i, dev in enumerate(devices):
            if 'USB' in dev['name'] and dev['max_output_channels'] > 0: return i
        return sd.default.device[1]
    except: return 0
AUDIO_CARD_ID = get_usb_audio_id()

# ---- 전역 변수 ----
current_duty = 0.0
current_pedal_raw = 0
current_safety_reason = None
current_remaining_time = 0
stop_threads = False

dist_history = deque(maxlen=10) 
data_queue = queue.Queue()

# ---- 사이렌 ----
def play_siren_thread():
    def _run_siren():
        try:
            sd.default.device = AUDIO_CARD_ID
            os.system(f"amixer -c {AUDIO_CARD_ID} set PCM 40% > /dev/null 2>&1")
            
            sample_rate = 48000; beep_freq = 600
            beep_duration = 0.3; silence_duration = 0.2; repeats = 4 
            
            t_beep = np.linspace(0, beep_duration, int(sample_rate * beep_duration), endpoint=False)
            beep_wave = np.sign(np.sin(2 * np.pi * beep_freq * t_beep)).astype(np.float32)
            silence_wave = np.zeros(int(sample_rate * silence_duration), dtype=np.float32)
            full_wave = np.concatenate([beep_wave, silence_wave] * repeats)
            
            sd.play(full_wave * 0.5, sample_rate, blocking=True)
            os.system(f"amixer -c {AUDIO_CARD_ID} set PCM 70% > /dev/null 2>&1")
        except: pass
    threading.Thread(target=_run_siren, daemon=True).start()

# ---- 거리 측정 ----
def read_distance():
    if PLATFORM == "WINDOWS": return 50 + 60 * np.sin(time.time()) + np.random.randint(-2, 2)
    try:
        GPIO.output(TRIG_PIN, False); time.sleep(0.000005)
        GPIO.output(TRIG_PIN, True); time.sleep(0.00001)
        GPIO.output(TRIG_PIN, False)
        start_time = time.time(); stop_time = time.time(); timeout = start_time + 0.04
        while GPIO.input(ECHO_PIN) == 0:
            start_time = time.time()
            if start_time > timeout: return None
        while GPIO.input(ECHO_PIN) == 1:
            stop_time = time.time()
            if stop_time > timeout: return None
        elapsed = stop_time - start_time
        distance = (elapsed * 34300) / 2
        if 2 < distance < 400: return distance
        else: return None
    except: return None

# ... (이전 import 및 설정 코드 동일)

# ---- 하드웨어 루프 ----
def hardware_loop():
    global current_duty, current_pedal_raw, current_safety_reason, current_remaining_time, stop_threads
    
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup(PWM_A_PIN, GPIO.OUT); GPIO.setup(IN1_PIN, GPIO.OUT); GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(PWM_B_PIN, GPIO.OUT); GPIO.setup(IN3_PIN, GPIO.OUT); GPIO.setup(IN4_PIN, GPIO.OUT)
    if PLATFORM == "LINUX": 
        GPIO.setup(TRIG_PIN, GPIO.OUT); GPIO.setup(ECHO_PIN, GPIO.IN)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    pwm_a = GPIO.PWM(PWM_A_PIN, 1000); pwm_a.start(0)
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000); pwm_b.start(0)
    GPIO.output(IN1_PIN, True); GPIO.output(IN2_PIN, False)
    GPIO.output(IN3_PIN, True); GPIO.output(IN4_PIN, False)
    
    press_timestamps = deque()
    
    last_pedal_value = 0; last_time = time.time()
    prev_over_90 = False; last_pedal_active_time = time.time()
    safety_lock_active = False; safety_cause_msg = ""
    prev_front_danger = False

    # [변수] 메시지 유지 타이머
    safety_msg_expiry = 0.0
    last_transient_msg = None

    ser = None
    if PLATFORM == "LINUX":
        try: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1); ser.flush()
        except: pass

    try:
        while not stop_threads:
            if ser is None and PLATFORM == "LINUX":
                try: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1); ser.flush()
                except: time.sleep(1); continue
            
            raw_line = ""
            if ser and ser.in_waiting > 0:
                try:
                    lines = ser.read_all().decode('utf-8').split('\n')
                    valid_lines = [l.strip() for l in lines if l.strip().isdigit()]
                    if valid_lines:
                        raw_line = valid_lines[-1]
                except: pass
            
            if raw_line.isdigit():
                current_pedal_value = int(raw_line)
                current_pedal_value = max(0, min(100, current_pedal_value))
                current_pedal_raw = current_pedal_value
                current_time = time.time()
                current_angular_velocity = 0.0

                # 1. 거리 측정
                raw_dist = read_distance()
                if raw_dist is not None: dist_history.append(raw_dist)
                final_dist = 0.0
                if len(dist_history) > 0: final_dist = sum(dist_history) / len(dist_history)

                # 2. 버튼
                is_btn_pressed = False
                if PLATFORM == "LINUX":
                    is_btn_pressed = (GPIO.input(BUTTON_PIN) == 0)

                # 3. 안전 로직 판단
                trigger_safety = False
                frame_reason = None  # 이번 프레임의 즉각적인 경고 원인
                
                front_danger = False
                if final_dist > 0 and final_dist <= COLLISION_DIST_LIMIT and current_pedal_value > 0:
                    front_danger = True

                # [우선순위 1] 잠금 상태 (즉시 표시 - 유지 안 함)
                if safety_lock_active:
                    target_speed = SAFETY_SPEED
                    current_remaining_time = 999 
                    
                    if current_pedal_value > 0:
                        frame_reason = "⚠️ 엑셀에서 발을 먼저 떼세요!"
                    else:
                        if is_btn_pressed:
                            safety_lock_active = False
                            frame_reason = None
                            current_remaining_time = 0
                            target_speed = 0
                            safety_msg_expiry = 0 # 잠금 해제 시 기존 경고도 클리어
                        else:
                            frame_reason = "🔵 푸쉬버튼을 눌러 제한을 해제하세요"

                # [우선순위 2] 전방 장애물 (즉시 표시 - 유지 안 함)
                elif front_danger:
                    frame_reason = "⚠️ 전방을 주의하세요!"
                    current_remaining_time = 0
                    target_speed = 0
                    if not prev_front_danger: play_siren_thread()
                
                # [우선순위 3] 이벤트 경고 (급발진/연타) -> 5초 유지 대상
                else:
                    dt = current_time - last_time
                    if dt > 0:
                        delta_percent = current_pedal_value - last_pedal_value
                        delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                        angular_velocity = delta_angle / dt
                        current_angular_velocity = angular_velocity
                        
                        # (A) 급발진 감지
                        if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                            trigger_safety = True
                            frame_reason = "⚠️ 급발진 감지!"
                            # [수정됨] 5초 유지 타이머 설정
                            safety_msg_expiry = current_time + 5.0
                            last_transient_msg = frame_reason
                        
                        # (B) 페달 연타 감지
                        is_over_90 = (current_pedal_value >= 90)
                        if is_over_90 and not prev_over_90: press_timestamps.append(current_time)
                        while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                            press_timestamps.popleft()
                        
                        if len(press_timestamps) >= RAPID_PRESS_COUNT:
                            trigger_safety = True
                            frame_reason = "🚫 과속 페달 연타!"
                            press_timestamps.clear()
                            # [수정됨] 5초 유지 타이머 설정
                            safety_msg_expiry = current_time + 5.0
                            last_transient_msg = frame_reason
                            
                        prev_over_90 = is_over_90

                    if trigger_safety:
                        safety_lock_active = True
                        safety_cause_msg = frame_reason
                        play_siren_thread()
                    else:
                        if current_pedal_value > 0:
                            last_pedal_active_time = current_time
                            target_speed = max(current_pedal_value, IDLE_SPEED)
                        else:
                            if (current_time - last_pedal_active_time) >= IDLE_TIMEOUT: target_speed = 0
                            else: target_speed = IDLE_SPEED
                
                prev_front_danger = front_danger

                # ---- [최종 메시지 결정 로직] ----
                # 1. 현재(이번 프레임)에 발생한 중요 경고(잠금, 장애물 등)가 있으면 우선 표시
                if frame_reason is not None:
                    current_safety_reason = frame_reason
                # 2. 현재 경고가 없고, 타이머가 살아있으면 '마지막 이벤트 경고'를 5초간 유지
                elif current_time < safety_msg_expiry and last_transient_msg is not None:
                    current_safety_reason = last_transient_msg
                # 3. 아무것도 없으면 끔
                else:
                    current_safety_reason = None
                # -----------------------------

                pwm_a.ChangeDutyCycle(target_speed)
                pwm_b.ChangeDutyCycle(target_speed)
                current_duty = target_speed
                last_pedal_value = current_pedal_value; last_time = current_time
                
                data_queue.put({
                    "t": current_time * 1000,
                    "p": current_pedal_value,
                    "d": current_duty,
                    "v": current_angular_velocity,
                    "dist": round(final_dist, 1),
                    "r": 1 if (safety_lock_active or front_danger) else 0,
                    "pc": len(press_timestamps)
                })
            
            time.sleep(0.01)

    except Exception as e: print(e)
    finally:
        pwm_a.stop(); pwm_b.stop(); GPIO.cleanup()
        if ser and ser.is_open: ser.close()

# ... (이후 코드 동일)

def start_hardware():
    t = threading.Thread(target=hardware_loop, daemon=True)
    t.start()

@router.websocket("")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if stop_threads: break
            history_batch = []
            while not data_queue.empty():
                try: history_batch.append(data_queue.get_nowait())
                except: break
            
            payload = {
                "type": "batch",
                "history": history_batch,
                "current": {
                    "duty": round(current_duty, 1),
                    "pedal": current_pedal_raw,
                    "reason": current_safety_reason,
                    "remaining_time": current_remaining_time
                }
            }
            await websocket.send_json(payload)
            # 웹소켓 전송 주기는 0.05초 유지 (화면 갱신은 20fps면 충분)
            await asyncio.sleep(0.05)
    except: pass
app.include_router(router)