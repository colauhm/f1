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
import pyttsx3

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

# ---- [수정됨] 오디오 장치 정보 가져오기 ----
def get_audio_device_info():
    """USB 오디오 장치의 ID와 지원하는 샘플링 레이트를 반환한다."""
    try:
        # 1. USB 장치 우선 검색
        devices = sd.query_devices()
        for i, dev in enumerate(devices):
            if 'USB' in dev['name'] and dev['max_output_channels'] > 0:
                # 장치의 기본 샘플링 레이트 가져오기 (없으면 48000)
                rate = int(dev.get('default_samplerate', 48000))
                return i, rate
        
        # 2. 없으면 시스템 기본 장치 사용
        default_device = sd.query_devices(kind='output')
        return default_device['index'], int(default_device.get('default_samplerate', 48000))
    except:
        return 0, 48000 # 완전 실패 시 안전값

# 전역 변수 초기화 시 정보를 미리 가져옴
AUDIO_DEVICE_ID, SYSTEM_SAMPLE_RATE = get_audio_device_info()
print(f"Detected Audio Config -> ID: {AUDIO_DEVICE_ID}, Rate: {SYSTEM_SAMPLE_RATE}Hz")


# ---- 전역 변수 ----
current_duty = 0.0
current_pedal_raw = 0
current_safety_reason = None
current_remaining_time = 0
stop_threads = False
is_audio_busy = False

dist_history = deque(maxlen=10) 
data_queue = queue.Queue()
audio_queue = queue.Queue()


# =========================================================
# [수정됨] 오디오 처리 스레드 (샘플링 레이트 오류 해결)
# =========================================================
def audio_processing_thread():
    global is_audio_busy
    
    # 1. TTS 엔진 초기화
    engine = None
    try:
        engine = pyttsx3.init(driverName='espeak')
        voices = engine.getProperty('voices')
        selected_voice = None
        for v in voices:
            if 'korea' in v.name.lower() or 'ko' in v.languages:
                selected_voice = v.id
                break
        if selected_voice is None and len(voices) > 0:
            selected_voice = voices[0].id
        if selected_voice:
            engine.setProperty('voice', selected_voice)
        
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate + 20) 
    except Exception as e:
        print(f"TTS Init Failed: {e}")
        engine = None

    # 2. 사이렌 소리 데이터 생성 (감지된 SYSTEM_SAMPLE_RATE 사용)
    # 하드웨어가 지원하는 레이트로 생성해야 오류가 안 남
    sample_rate = SYSTEM_SAMPLE_RATE 
    beep_freq = 600
    beep_duration = 0.3
    silence_duration = 0.2
    repeats = 3 
    
    t_beep = np.linspace(0, beep_duration, int(sample_rate * beep_duration), endpoint=False)
    beep_wave = np.sign(np.sin(2 * np.pi * beep_freq * t_beep)).astype(np.float32)
    silence_wave = np.zeros(int(sample_rate * silence_duration), dtype=np.float32)
    full_siren_wave = np.concatenate([beep_wave, silence_wave] * repeats) * 0.5

    while not stop_threads:
        try:
            task = audio_queue.get(timeout=1)
            is_audio_busy = True
            
            # 1. 사이렌 재생
            if task.get("siren", False):
                try:
                    # device=AUDIO_DEVICE_ID, samplerate=sample_rate 명시
                    sd.play(full_siren_wave, samplerate=sample_rate, device=AUDIO_DEVICE_ID, blocking=True)
                    time.sleep(0.3) # 재생 후 안정화 대기
                except Exception as e:
                    print(f"Siren Error: {e}")

            # 2. TTS 말하기
            msg = task.get("msg", "")
            if msg and engine:
                clean_msg = msg.replace("⚠️", "").replace("🚫", "").replace("🔵", "").strip()
                if clean_msg:
                    try:
                        engine.say(clean_msg)
                        engine.runAndWait()
                        time.sleep(0.2)
                    except Exception as e:
                        print(f"TTS Speak Error: {e}")

            is_audio_busy = False
            audio_queue.task_done()

        except queue.Empty:
            pass
        except Exception as e:
            print(f"Audio Thread Error: {e}")
            is_audio_busy = False


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


# =========================================================
# 안전 로직 및 모터 속도 계산 함수
# =========================================================
def process_safety_logic(
    current_time, current_pedal, last_pedal, last_time,
    final_dist, is_btn_pressed,
    lock_active, msg_expiry, last_transient_msg,
    press_timestamps, prev_over_90, prev_front_danger, last_pedal_active_time
):
    target_speed = 0
    trigger_siren = False
    frame_reason = None
    current_angular_velocity = 0.0
    
    front_danger = False
    if final_dist > 0 and final_dist <= COLLISION_DIST_LIMIT and current_pedal > 0:
        front_danger = True

    if lock_active:
        target_speed = SAFETY_SPEED
        if current_pedal > 0:
            frame_reason = "⚠️ 엑셀에서 발을 먼저 떼세요!"
        else:
            if is_btn_pressed:
                lock_active = False; msg_expiry = 0
                frame_reason = None; target_speed = 0
            else:
                frame_reason = "🔵 푸쉬버튼을 눌러 제한을 해제하세요"

    elif front_danger:
        frame_reason = "⚠️ 전방을 주의하세요!"
        target_speed = 0
        if not prev_front_danger: trigger_siren = True

    else:
        dt = current_time - last_time
        if dt > 0:
            delta_percent = current_pedal - last_pedal
            delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
            angular_velocity = delta_angle / dt
            current_angular_velocity = angular_velocity
            
            trigger_event = False; event_msg = ""

            if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                trigger_event = True; event_msg = "⚠️ 급발진 감지!"
            
            is_over_90 = (current_pedal >= 90)
            if is_over_90 and not prev_over_90: press_timestamps.append(current_time)
            while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                press_timestamps.popleft()
            
            if len(press_timestamps) >= RAPID_PRESS_COUNT:
                trigger_event = True; event_msg = "🚫 과속 페달 연타!"; press_timestamps.clear()
            
            prev_over_90 = is_over_90

            if trigger_event:
                lock_active = True
                trigger_siren = True
                msg_expiry = current_time + 5.0
                last_transient_msg = event_msg
            else:
                if current_pedal > 0:
                    last_pedal_active_time = current_time
                    target_speed = max(current_pedal, IDLE_SPEED)
                else:
                    if (current_time - last_pedal_active_time) >= IDLE_TIMEOUT: target_speed = 0
                    else: target_speed = IDLE_SPEED

    logical_reason = None
    if current_time < msg_expiry and last_transient_msg is not None:
        logical_reason = last_transient_msg
    elif frame_reason is not None:
        logical_reason = frame_reason

    return {
        "target_speed": target_speed, "logical_reason": logical_reason,
        "trigger_siren": trigger_siren, "angular_velocity": current_angular_velocity,
        "lock_active": lock_active, "msg_expiry": msg_expiry,
        "last_transient_msg": last_transient_msg, "prev_over_90": prev_over_90,
        "prev_front_danger": front_danger, "last_pedal_active_time": last_pedal_active_time
    }


# ---- 메인 하드웨어 루프 ----
def hardware_loop():
    global current_duty, current_pedal_raw, current_safety_reason, current_remaining_time, stop_threads, is_audio_busy
    
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
    
    state = {
        "lock_active": False, "msg_expiry": 0.0, "last_transient_msg": None,
        "prev_over_90": False, "prev_front_danger": False,
        "last_pedal_active_time": time.time()
    }

    last_enqueued_reason = None
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
                    if valid_lines: raw_line = valid_lines[-1]
                except: pass
            
            if raw_line.isdigit():
                current_pedal_value = int(raw_line)
                current_pedal_value = max(0, min(100, current_pedal_value))
                current_pedal_raw = current_pedal_value
                current_time = time.time()

                raw_dist = read_distance()
                if raw_dist is not None: dist_history.append(raw_dist)
                final_dist = 0.0
                if len(dist_history) > 0: final_dist = sum(dist_history) / len(dist_history)

                is_btn_pressed = False
                if PLATFORM == "LINUX":
                    is_btn_pressed = (GPIO.input(BUTTON_PIN) == 0)

                result = process_safety_logic(
                    current_time, current_pedal_value, last_pedal_value, last_time,
                    final_dist, is_btn_pressed,
                    state["lock_active"], state["msg_expiry"], state["last_transient_msg"],
                    press_timestamps, state["prev_over_90"], state["prev_front_danger"], state["last_pedal_active_time"]
                )
                
                new_reason = result["logical_reason"]
                
                should_speak = False
                if new_reason is not None:
                    if new_reason != last_enqueued_reason:
                        should_speak = True
                    if result["trigger_siren"]: 
                        should_speak = True
                else:
                    last_enqueued_reason = None

                if should_speak:
                    audio_task = {"msg": new_reason, "siren": result["trigger_siren"]}
                    audio_queue.put(audio_task)
                    last_enqueued_reason = new_reason

                if is_audio_busy:
                    if current_safety_reason is None and new_reason is not None:
                         current_safety_reason = new_reason
                else:
                    current_safety_reason = new_reason

                state.update({
                    "lock_active": result["lock_active"],
                    "msg_expiry": result["msg_expiry"],
                    "last_transient_msg": result["last_transient_msg"],
                    "prev_over_90": result["prev_over_90"],
                    "prev_front_danger": result["prev_front_danger"],
                    "last_pedal_active_time": result["last_pedal_active_time"]
                })
                
                target_speed = result["target_speed"]
                pwm_a.ChangeDutyCycle(target_speed)
                pwm_b.ChangeDutyCycle(target_speed)
                current_duty = target_speed

                data_queue.put({
                    "t": current_time * 1000,
                    "p": current_pedal_value,
                    "d": current_duty,
                    "v": result["angular_velocity"],
                    "dist": round(final_dist, 1),
                    "r": 1 if (state["lock_active"] or result["prev_front_danger"]) else 0,
                    "pc": len(press_timestamps)
                })

                last_pedal_value = current_pedal_value
                last_time = current_time
            
            time.sleep(0.01)

    except Exception as e: print(e)
    finally:
        pwm_a.stop(); pwm_b.stop(); GPIO.cleanup()
        if ser and ser.is_open: ser.close()

def start_hardware():
    t_audio = threading.Thread(target=audio_processing_thread, daemon=True)
    t_audio.start()
    t_hw = threading.Thread(target=hardware_loop, daemon=True)
    t_hw.start()

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
            await asyncio.sleep(0.05)
    except: pass
app.include_router(router)