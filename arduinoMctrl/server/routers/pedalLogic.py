import asyncio
import threading
import time
import serial
import queue
from collections import deque
from fastapi import APIRouter, FastAPI, WebSocket
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import os
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

app = FastAPI()
app.mount("/static", StaticFiles(directory="."), name="static")
router = APIRouter(prefix="/ws")

# ---- 2. 핀 설정 ----
PWM_A_PIN = 13; IN1_PIN = 23; IN2_PIN = 24
PWM_B_PIN = 12; IN3_PIN = 5; IN4_PIN = 6
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200 

# ---- 3. 설정 변수 ----
PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 310
RAPID_PRESS_COUNT = 3
RAPID_PRESS_WINDOW = 2.0
SAFETY_LOCK_DURATION = 5.0
SAFETY_SPEED = 15
IDLE_SPEED = 15
IDLE_TIMEOUT = 5.0
AUDIO_CARD_ID = 0 

# ---- 전역 변수 ----
current_duty = 0.0
current_pedal_raw = 0
current_safety_reason = None
current_remaining_time = 0
stop_threads = False

# [데이터 큐]
data_queue = queue.Queue()

# ---- 경고음 재생 함수 ----
def play_siren_thread():
    def _run_siren():
        try:
            sd.default.device = AUDIO_CARD_ID
            os.system(f"amixer -c {AUDIO_CARD_ID} set PCM 20% > /dev/null 2>&1")
            
            sample_rate = 44100
            beep_freq = 500
            beep_duration = 0.5
            silence_duration = 0.5
            repeats = 3

            t_beep = np.linspace(0, beep_duration, int(sample_rate * beep_duration), endpoint=False)
            beep_wave = np.sign(np.sin(2 * np.pi * beep_freq * t_beep)).astype(np.float32)
            silence_wave = np.zeros(int(sample_rate * silence_duration), dtype=np.float32)
            full_wave = np.concatenate([beep_wave, silence_wave] * repeats)

            sd.play(full_wave * 0.5, sample_rate, blocking=True)
            os.system(f"amixer -c {AUDIO_CARD_ID} set PCM 70% > /dev/null 2>&1")
        except Exception as e:
            print(f"[Audio Error] {e}")

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

    press_timestamps = deque()
    override_end_time = 0
    last_pedal_value = 0
    last_time = time.time()
    prev_over_90 = False
    last_pedal_active_time = time.time()
    
    safety_lock_active = False 
    safety_cause_msg = "" 

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
                try: raw_line = ser.readline().decode('utf-8').strip()
                except: pass
            
            if raw_line.isdigit():
                current_pedal_value = int(raw_line)
                current_pedal_value = max(0, min(100, current_pedal_value))
                current_pedal_raw = current_pedal_value
                current_time = time.time()

                # --- 안전 로직 (생략 없이 기존과 동일) ---
                trigger_safety = False
                detected_reason = ""

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
                            safety_lock_active = False; current_safety_reason = None
                            current_remaining_time = 0; target_speed = 0 
                else:
                    dt = current_time - last_time
                    if dt > 0:
                        delta_percent = current_pedal_value - last_pedal_value
                        delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
                        angular_velocity = delta_angle / dt
                        
                        if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                            trigger_safety = True; detected_reason = "⚠️ 급발진 감지!"

                        is_over_90 = (current_pedal_value >= 90)
                        if is_over_90 and not prev_over_90: press_timestamps.append(current_time)
                        while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                            press_timestamps.popleft()
                        if len(press_timestamps) >= RAPID_PRESS_COUNT:
                            trigger_safety = True; detected_reason = "🚫 과속 페달 연타!"; press_timestamps.clear()
                        prev_over_90 = is_over_90

                    if trigger_safety:
                        safety_lock_active = True; safety_cause_msg = detected_reason
                        override_end_time = current_time + SAFETY_LOCK_DURATION
                        target_speed = SAFETY_SPEED; current_remaining_time = int(SAFETY_LOCK_DURATION)
                        play_siren_thread()
                    else:
                        if current_pedal_value > 0:
                            last_pedal_active_time = current_time
                            target_speed = max(current_pedal_value, IDLE_SPEED)
                        else:
                            if (current_time - last_pedal_active_time) >= IDLE_TIMEOUT: target_speed = 0
                            else: target_speed = IDLE_SPEED

                # 모터 제어
                pwm_a.ChangeDutyCycle(target_speed)
                pwm_b.ChangeDutyCycle(target_speed)
                current_duty = target_speed
                
                last_pedal_value = current_pedal_value
                last_time = current_time

                # [핵심 수정] 큐에 페달(p)과 모터속도(d)를 같이 넣음
                data_queue.put({
                    "t": current_time * 1000, 
                    "p": current_pedal_value,  # 페달 입력
                    "d": current_duty,         # 모터 속도 (Duty)
                    "r": 1 if safety_lock_active else 0
                })

            time.sleep(0.01)

    except Exception as e: print(f"Error: {e}")
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
            
            history_batch = []
            while not data_queue.empty():
                try: history_batch.append(data_queue.get_nowait())
                except queue.Empty: break
            
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
            await asyncio.sleep(0.05) 
    except Exception: pass

app.include_router(router)

@app.get("/")
async def get_index():
    with open("index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read())

if __name__ == "__main__":
    start_hardware()
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)