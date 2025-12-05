import asyncio
import threading
import time
import serial
import subprocess
import numpy as np
import wave
import os
import queue
from collections import deque
from fastapi import APIRouter, FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles

# [DB 관련 코드 모두 제거됨]

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

# ---- 2. 핀 및 상수 설정 ----
PWM_A_PIN = 13; IN1_PIN = 24; IN2_PIN = 23
PWM_B_PIN = 12; IN3_PIN = 5; IN4_PIN = 6
TRIG_PIN = 27; ECHO_PIN = 17 
BUTTON_PIN = 21  # [해제 버튼]
BTN_DRIVE_PIN = 16; BTN_PARK_PIN = 20
BTN_SAFETY_PIN = 26 # [안전 모드 토글]

SERIAL_PORT = '/dev/ttyUSB0'; BAUD_RATE = 115200

# 임계값 설정
PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 420
RAPID_PRESS_COUNT = 3
RAPID_PRESS_WINDOW = 2.0   
SAFETY_SPEED = 20
COLLISION_DIST_LIMIT = 100.0 
IDLE_DUTY = 20.0
ACCEL_STEP = 1.5; DECEL_STEP = 0.5
MAX_RPM_REAL = 1350; SHIFT_POINT_1 = 35.0; SHIFT_POINT_2 = 70.0; SHIFT_DELAY_TIME = 0.2

# 전역 변수
stop_threads = False
is_warning_sound_active = False 
safety_mode_enabled = True 
drive_mode = 'N' 

# [핵심] 고속 데이터 전송을 위한 메모리 큐
data_queue = queue.Queue()

# 변속기 및 히스토리
virtual_gear = 1; virtual_rpm = 0; shift_pause_timer = 0.0
dist_history = deque(maxlen=10)

# 오디오 설정
def get_usb_card_number():
    try:
        result = subprocess.check_output("aplay -l", shell=True).decode()
        for line in result.split('\n'):
            if "USB" in line and "card" in line:
                return line.split(":")[0].replace("card", "").strip()
        return None 
    except: return None
USB_CARD_NUM = get_usb_card_number()

def generate_chime_file(filename="/tmp/chime.wav"):
    try:
        sample_rate = 44100; duration = 0.8
        t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
        freq = 880; decay = np.exp(-3 * t)
        wave_data = (0.98 * np.sin(2 * np.pi * freq * t) * decay * 32767).astype(np.int16)
        with wave.open(filename, 'w') as wf:
            wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(sample_rate)
            wf.writeframes(wave_data.tobytes())
    except: pass
generate_chime_file()

def set_system_volume():
    if USB_CARD_NUM:
        os.system(f"amixer -c {USB_CARD_NUM} set PCM 100% unmute")
set_system_volume()

def audio_processing_thread():
    global is_warning_sound_active, stop_threads, USB_CARD_NUM
    while not stop_threads:
        if is_warning_sound_active:
            device = f"-D plughw:{USB_CARD_NUM},0" if USB_CARD_NUM else ""
            os.system(f"aplay -q {device} /tmp/chime.wav")
            time.sleep(0.3) 
        else: time.sleep(0.1)

def read_distance():
    if PLATFORM == "WINDOWS": return 50 + 60 * np.sin(time.time())
    try:
        GPIO.output(TRIG_PIN, False); time.sleep(0.000005)
        GPIO.output(TRIG_PIN, True); time.sleep(0.00001); GPIO.output(TRIG_PIN, False)
        start = time.time(); timeout = start + 0.04
        while GPIO.input(ECHO_PIN) == 0: 
            if time.time() > timeout: return None
            start = time.time()
        stop = time.time()
        while GPIO.input(ECHO_PIN) == 1: 
            if time.time() > timeout: return None
            stop = time.time()
        dist = ((stop - start) * 34300) / 2
        return dist if 2 < dist < 400 else None
    except: return None

# [안전 로직] 3초 잠금 및 해제 절차 (강화된 버전 유지)
def process_safety_logic(t, pedal, last_p, last_t, dist, btn, lock, expiry, stamps, p_90, p_danger, last_act, mode):
    target=0; reason=None; sound=False; v_gear=mode; unlock=False; r_val=0; ang_vel=0
    
    # 1. P/N단일 때는 감시하지 않음
    if mode == 'P' or mode == 'N':
        return {"tgt":0, "msg":None, "snd":False, "vel":0, "lock":False, "exp":0, "p90":False, "pd":False, "lat":t, "vg":mode, "ul":False}

    # 2. 이미 잠금(Lock) 상태인 경우
    if lock:
        target = 0      # 속도 0 강제
        v_gear = 'N'    # 기어 중립 표시
        sound = True    # 경고음 지속
        r_val = 1       # 그래프 빨간색
        
        # [Phase 1] 3초 강제 유지 구간
        if t < expiry:
            lock = True 
            reason = "⛔ 위험 감지! (3초간 잠금)"
            
        # [Phase 2] 3초 경과 후 해제 조건 검사
        else:
            # 엑셀 발 떼기 확인
            if pedal > 0:
                reason = "🦶 엑셀에서 발을 완전히 떼세요!"
                lock = True # 잠금 유지
            else:
                # 버튼 확인
                if btn:
                    lock = False   # 해제 성공
                    reason = None
                    target = IDLE_DUTY # 크리핑 복귀
                    sound = False
                    unlock = True
                else:
                    reason = "🔵 해제버튼(21번)을 누르세요"
                    lock = True # 잠금 유지

        return {"tgt":target, "msg":reason, "snd":sound, "vel":0, "lock":lock, "exp":expiry, "p90":p_90, "pd":p_danger, "lat":last_act, "vg":v_gear, "ul":unlock}

    # 3. 잠금 아님 -> 위험 감지
    front_danger = (0 < dist <= COLLISION_DIST_LIMIT and pedal > 0)
    
    if front_danger:
        reason="⚠️ 전방 주의!"; target=0; sound=True; r_val=1
    else:
        # 급발진 감지
        dt = t - last_t
        if dt > 0:
            ang_vel = ((pedal - last_p)/100.0 * PEDAL_TOTAL_ANGLE) / dt
            trigger = (ang_vel >= CRITICAL_ANGULAR_VELOCITY)
            
            is_90 = (pedal >= 90)
            if is_90 and not p_90: stamps.append(t)
            while stamps and stamps[0] < t - RAPID_PRESS_WINDOW: stamps.popleft()
            if len(stamps) >= RAPID_PRESS_COUNT: trigger = True; stamps.clear()
            p_90 = is_90

            if trigger:
                lock = True; expiry = t + 3.0
                target = 0; v_gear = 'N'; sound = True; reason = "⛔ 위험 감지! (잠금 시작)"; r_val = 1
                print(f"!!! LOCK TRIGGERED at {t} !!!")
            else:
                target = max(pedal, IDLE_DUTY)

    return {"tgt":target, "msg":reason, "snd":sound, "vel":ang_vel, "lock":lock, "exp":expiry, "p90":p_90, "pd":front_danger, "lat":last_act, "vg":v_gear, "ul":unlock}

def simulate_transmission(duty, t):
    global virtual_gear, virtual_rpm, shift_pause_timer
    tgt_gear = 1
    if duty > SHIFT_POINT_2: tgt_gear = 3
    elif duty > SHIFT_POINT_1: tgt_gear = 2
    
    if tgt_gear != virtual_gear:
        if shift_pause_timer <= t: shift_pause_timer = t + SHIFT_DELAY_TIME
        virtual_gear = tgt_gear
        
    rpm = 0
    if virtual_gear == 1: rpm = (duty / SHIFT_POINT_1) * 1300
    elif virtual_gear == 2: rpm = 800 + ((duty - SHIFT_POINT_1)/(SHIFT_POINT_2-SHIFT_POINT_1))*(1300-800)
    elif virtual_gear == 3: rpm = 900 + ((duty - SHIFT_POINT_2)/(100-SHIFT_POINT_2))*(MAX_RPM_REAL-900)
    return virtual_gear, int(rpm)

# ---- 하드웨어 루프 (Original Queue Mode) ----
def hardware_loop():
    global stop_threads, is_warning_sound_active, drive_mode, safety_mode_enabled
    
    # GPIO 설정
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup([PWM_A_PIN, PWM_B_PIN, IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN], GPIO.OUT)
    if PLATFORM == "LINUX":
        GPIO.setup([TRIG_PIN], GPIO.OUT); GPIO.setup([ECHO_PIN, BUTTON_PIN, BTN_DRIVE_PIN, BTN_PARK_PIN, BTN_SAFETY_PIN], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    pwm_a = GPIO.PWM(PWM_A_PIN, 1000); pwm_a.start(0)
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000); pwm_b.start(0)
    GPIO.output([IN1_PIN, IN3_PIN], True); GPIO.output([IN2_PIN, IN4_PIN], False)
    
    ser = None; smoothed_duty = 0.0
    last_p = 0; last_t = time.time()
    stamps = deque()
    
    state = {"lock":False, "exp":0, "p90":False, "pd":False, "lat":time.time()}
    current_pedal_val = 0 
    
    # 버튼 노이즈 방지
    last_safety_btn_val = 1
    last_safety_toggle_time = 0 

    try:
        while not stop_threads:
            # 1. 시리얼
            if ser is None and PLATFORM == "LINUX":
                try: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1); ser.flush()
                except: pass
            
            t_now = time.time()
            
            # 2. 버튼 입력 (디바운싱 적용)
            if PLATFORM == "LINUX":
                if GPIO.input(BTN_DRIVE_PIN)==0: drive_mode='D'
                if GPIO.input(BTN_PARK_PIN)==0: drive_mode='P'
                
                curr_safe_val = GPIO.input(BTN_SAFETY_PIN)
                if curr_safe_val == 0 and last_safety_btn_val == 1:
                    # 0.5초 디바운싱
                    if t_now - last_safety_toggle_time > 0.5:
                        safety_mode_enabled = not safety_mode_enabled
                        last_safety_toggle_time = t_now
                        print(f"Safety Mode Toggled: {safety_mode_enabled}")
                        if not safety_mode_enabled: 
                            state["lock"] = False
                            is_warning_sound_active = False
                last_safety_btn_val = curr_safe_val

            # 3. 페달
            if ser and ser.in_waiting:
                try: 
                    lines = ser.read_all().decode().split('\n')
                    valid = [l for l in lines if l.strip().isdigit()]
                    if valid: current_pedal_val = max(0, min(100, int(valid[-1])))
                except: pass
            curr_pedal = current_pedal_val 

            # 4. 거리
            dist = read_distance() or 0
            if dist > 0: dist_history.append(dist)
            avg_dist = sum(dist_history)/len(dist_history) if dist_history else 0
            
            # 5. 해제 버튼
            btn_push = False
            if PLATFORM == "LINUX": btn_push = (GPIO.input(BUTTON_PIN)==0)

            # 6. 로직
            if safety_mode_enabled:
                res = process_safety_logic(t_now, curr_pedal, last_p, last_t, avg_dist, btn_push, state["lock"], state["exp"], stamps, state["p90"], state["pd"], state["lat"], drive_mode)
                
                target_d = res["tgt"]; msg = res["msg"]; is_warning_sound_active = res["snd"]
                
                # 상태 업데이트
                state["lock"]=res["lock"]
                state["exp"]=res["exp"]
                state["p90"]=res["p90"]
                state["pd"]=res["pd"]
                state["lat"]=res["lat"]
                
                if res["ul"]: drive_mode='D'
                if res["lock"] and drive_mode=='D': drive_mode='N'
                v_gear_char = res["vg"]; v_val = res["vel"]
                r_val = 1 if (res["lock"] or res["pd"]) else 0
            else:
                target_d = max(curr_pedal, IDLE_DUTY) if drive_mode == 'D' else 0
                msg = None; is_warning_sound_active = False; v_gear_char = drive_mode
                v_val = 0; r_val = 0
                if t_now - last_t > 0: v_val = ((curr_pedal - last_p)/100.0 * 45.0)/(t_now - last_t)

            # 7. 모터 & 변속
            is_shifting = (t_now < shift_pause_timer)
            if not is_shifting:
                if target_d > smoothed_duty: smoothed_duty = min(target_d, smoothed_duty + ACCEL_STEP)
                elif target_d < smoothed_duty: smoothed_duty = max(target_d, smoothed_duty - DECEL_STEP)
            
            sim_in = smoothed_duty if v_gear_char != 'N' else curr_pedal
            g_num, rpm = simulate_transmission(sim_in, t_now)
            if v_gear_char == 'P': rpm=0; g_num=1

            pwm_a.ChangeDutyCycle(smoothed_duty)
            pwm_b.ChangeDutyCycle(smoothed_duty)
            
            # [롤백] 큐에 데이터 넣기 (가장 빠름)
            log_data = {
                't': t_now, 'p': curr_pedal, 'd': smoothed_duty, 'v': v_val,
                'dist': round(avg_dist, 1), 'rpm': rpm, 'gear': g_num,
                'v_gear': v_gear_char, 'safety': safety_mode_enabled,
                'msg': msg, 'r': r_val
            }
            data_queue.put(log_data)

            last_p = curr_pedal; last_t = t_now
            time.sleep(0.01)

    except Exception as e: print(f"HW Loop Error: {e}")
    finally:
        pwm_a.stop(); pwm_b.stop(); GPIO.cleanup()
        if ser: ser.close()

def start_hardware():
    threading.Thread(target=audio_processing_thread, daemon=True).start()
    threading.Thread(target=hardware_loop, daemon=True).start()

# ---- 웹소켓 (Queue 배치 전송) ----
@router.websocket("")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    
    try:
        while True:
            if stop_threads: break
            
            # 큐에 쌓인 데이터를 한 번에 가져와서 전송 (그래프 부드러움 유지)
            history_batch = []
            while not data_queue.empty():
                try: history_batch.append(data_queue.get_nowait())
                except: break
            
            if history_batch:
                latest = history_batch[-1]
                
                payload = {
                    "type": "batch",
                    "history": history_batch,
                    "current": {
                        "duty": latest["d"],
                        "pedal": latest["p"],
                        "reason": latest["msg"], # 여기 msg로 통일
                        "remaining_time": 0,
                        "rpm": latest["rpm"],
                        "gear": latest["gear"],
                        "v_gear": latest["v_gear"],
                        "safety_mode": latest["safety"]
                    }
                }
                await websocket.send_json(payload)
            
            await asyncio.sleep(0.05) 
            
    except Exception as e:
        print(f"WS Error: {e}")
app.include_router(router)