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
import wave
import subprocess

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
PWM_A_PIN = 13; IN1_PIN = 24; IN2_PIN = 23
PWM_B_PIN = 12; IN3_PIN = 5; IN4_PIN = 6
TRIG_PIN = 27; ECHO_PIN = 17 
BUTTON_PIN = 21  # [안전 해제 버튼]

# [기어 변속 버튼]
BTN_DRIVE_PIN = 16  
BTN_PARK_PIN = 20
BTN_SAFETY_PIN = 26 # [안전 모드 토글]

SERIAL_PORT = '/dev/ttyUSB0'; BAUD_RATE = 115200

# ---- 3. 임계값 설정 ----
PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 420
RAPID_PRESS_COUNT = 3      
RAPID_PRESS_WINDOW = 2.0   
SAFETY_SPEED = 20     
COLLISION_DIST_LIMIT = 100.0 

# [D모드 최소 속도] 20%
IDLE_DUTY = 20.0      
IDLE_TIMEOUT = 5.0

# [가감속 반응성]
ACCEL_STEP = 1.5
DECEL_STEP = 0.5

# [변속기 시뮬레이션]
MAX_RPM_REAL = 1350
SHIFT_POINT_1 = 35.0
SHIFT_POINT_2 = 70.0
SHIFT_DELAY_TIME = 0.2

# ---- 전역 변수 ----
current_duty = 0.0          
target_duty_raw = 0.0       
current_pedal_raw = 0
current_safety_reason = None 
current_remaining_time = 0
stop_threads = False

# [상태 플래그]
is_warning_sound_active = False 
safety_mode_enabled = True # 기본값 ON

# [초기 상태 N]
drive_mode = 'N' 

# 변속기 상태
virtual_gear = 1
virtual_rpm = 0
shift_pause_timer = 0.0

dist_history = deque(maxlen=10) 
data_queue = queue.Queue()

# ---- USB 오디오 카드 찾기 ----
def get_usb_card_number():
    try:
        result = subprocess.check_output("aplay -l", shell=True).decode()
        for line in result.split('\n'):
            if "USB" in line and "card" in line:
                parts = line.split(":")
                card_part = parts[0] 
                card_num = card_part.replace("card", "").strip()
                return card_num
        return None 
    except:
        return None

USB_CARD_NUM = get_usb_card_number()
print(f"Detected USB Card Number: {USB_CARD_NUM}")

# ---- 자동차 경고음(Chime) 파일 생성 (최대 볼륨) ----
def generate_chime_file(filename="/tmp/chime.wav"):
    try:
        sample_rate = 44100
        duration = 0.8  
        t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
        
        freq = 880 
        decay = np.exp(-3 * t)
        
        wave_data = 0.98 * np.sin(2 * np.pi * freq * t) * decay
        
        wave_data = (wave_data * 32767).astype(np.int16)
        with wave.open(filename, 'w') as wf:
            wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(sample_rate)
            wf.writeframes(wave_data.tobytes())
    except Exception as e:
        print(f"Sound Gen Error: {e}")

generate_chime_file()

# ---- 시스템 볼륨 100% 설정 ----
def set_system_volume():
    if USB_CARD_NUM is not None:
        try:
            cmds = [
                f"amixer -c {USB_CARD_NUM} set PCM 100% unmute",
                f"amixer -c {USB_CARD_NUM} set Speaker 100% unmute",
                f"amixer -c {USB_CARD_NUM} set Master 100% unmute"
            ]
            for cmd in cmds: os.system(cmd)
        except: pass
set_system_volume()

# ---- 오디오 스레드 (오류 해결 버전) ----
def audio_processing_thread():
    global is_warning_sound_active, stop_threads, USB_CARD_NUM
    
    while not stop_threads:
        if is_warning_sound_active:
            # [수정] 재생할 때마다 카드 번호를 다시 확인 (연결 끊김 방지)
            current_card = get_usb_card_number()
            if current_card is not None:
                try:
                    cmd = f"aplay -q -D plughw:{current_card},0 /tmp/chime.wav"
                    ret = os.system(cmd)
                    if ret != 0: time.sleep(0.5)
                except: time.sleep(0.5)
            else:
                # 카드가 없으면 잠시 대기
                time.sleep(1.0)
            time.sleep(0.3) 
        else:
            time.sleep(0.1)

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
# 안전 로직 (Logic) - 3단계 엄격한 잠금 적용
# =========================================================
def process_safety_logic(
    current_time, current_pedal, last_pedal, last_time,
    final_dist, is_btn_pressed,
    lock_active, pedal_error_expiry,
    press_timestamps, prev_over_90, prev_front_danger, last_pedal_active_time,
    current_drive_mode
):
    target_speed = 0; frame_reason = None
    current_angular_velocity = 0.0
    trigger_sound = False
    visual_gear = current_drive_mode 
    unlock_success = False
    
    # [1] Park(P) 또는 Neutral(N)
    if current_drive_mode == 'P' or current_drive_mode == 'N':
        return {
            "target_speed": 0, "logical_reason": None,
            "trigger_sound": False, "angular_velocity": 0,
            "lock_active": False, "pedal_error_expiry": 0,
            "prev_over_90": False, "prev_front_danger": False, 
            "last_pedal_active_time": current_time, "visual_gear": current_drive_mode,
            "unlock_success": False
        }
    
    # [2] 안전 제한 (Lock Active) - [수정됨: 3단계 로직]
    if lock_active:
        target_speed = 0 # 제한 시 속도 0
        visual_gear = "N" 
        trigger_sound = True 
        
        # [단계 1] 3초 강제 대기
        if current_time < pedal_error_expiry:
            remaining = int(pedal_error_expiry - current_time) + 1
            frame_reason = f"⛔ 위험 감지! ({remaining}초 대기)"
            lock_active = True # 잠금 유지
            
        # [단계 2] 3초 경과 후 -> 해제 조건 검사
        else:
            # 2-1. 엑셀에서 발을 뗐는가?
            if current_pedal > 0:
                frame_reason = "🦶 엑셀에서 발을 완전히 떼세요!"
                lock_active = True # 잠금 유지
            # 2-2. 엑셀을 뗐다면 -> 버튼을 눌렀는가?
            else:
                if is_btn_pressed:
                    lock_active = False   # 해제 성공
                    pedal_error_expiry = 0
                    frame_reason = None
                    target_speed = IDLE_DUTY # 크리핑 복귀
                    trigger_sound = False
                    unlock_success = True
                else:
                    frame_reason = "🔵 해제버튼(21번)을 누르세요"
                    lock_active = True # 버튼 누르기 전까지 잠금 유지

        return {
            "target_speed": target_speed, "logical_reason": frame_reason,
            "trigger_sound": trigger_sound,
            "angular_velocity": current_angular_velocity,
            "lock_active": lock_active, "pedal_error_expiry": pedal_error_expiry,
            "prev_over_90": prev_over_90,
            "prev_front_danger": prev_front_danger, "last_pedal_active_time": last_pedal_active_time,
            "visual_gear": visual_gear,
            "unlock_success": unlock_success
        }

    # [3] Drive(D) - 위험 감지
    visual_gear = "D" 
    front_danger = False
    
    if final_dist > 0 and final_dist <= COLLISION_DIST_LIMIT and current_pedal > 0:
        front_danger = True

    if front_danger:
        frame_reason = "⚠️ 전방을 주의하세요!"
        target_speed = 0
        trigger_sound = True 
    else:
        dt = current_time - last_time
        if dt > 0:
            delta_percent = current_pedal - last_pedal
            delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
            angular_velocity = delta_angle / dt
            current_angular_velocity = angular_velocity
            
            trigger_event = False
            if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                trigger_event = True
            
            is_over_90 = (current_pedal >= 90)
            if is_over_90 and not prev_over_90: press_timestamps.append(current_time)
            while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                press_timestamps.popleft()
            if len(press_timestamps) >= RAPID_PRESS_COUNT:
                trigger_event = True; press_timestamps.clear()
            prev_over_90 = is_over_90

            if trigger_event:
                # [위험 감지 -> 잠금 시작]
                lock_active = True
                pedal_error_expiry = current_time + 3.0 # 3초 설정
                target_speed = 0 
                visual_gear = "N" 
                trigger_sound = True
                frame_reason = "⚠️ 페달 오조작 감지!"
                print(f"!!! LOCK TRIGGERED at {current_time} !!!")
            else:
                target_speed = max(current_pedal, IDLE_DUTY)

    return {
        "target_speed": target_speed, "logical_reason": frame_reason,
        "trigger_sound": trigger_sound,
        "angular_velocity": current_angular_velocity,
        "lock_active": lock_active, "pedal_error_expiry": pedal_error_expiry,
        "prev_over_90": prev_over_90,
        "prev_front_danger": prev_front_danger, "last_pedal_active_time": last_pedal_active_time,
        "visual_gear": visual_gear,
        "unlock_success": unlock_success
    }

def simulate_transmission(duty_val, current_time):
    global virtual_gear, virtual_rpm, shift_pause_timer
    
    target_gear = 1
    if duty_val > SHIFT_POINT_2: target_gear = 3
    elif duty_val > SHIFT_POINT_1: target_gear = 2
    
    if target_gear != virtual_gear:
        if shift_pause_timer <= current_time:
            shift_pause_timer = current_time + SHIFT_DELAY_TIME
        virtual_gear = target_gear
        
    if virtual_gear == 1:
        ratio = duty_val / SHIFT_POINT_1
        virtual_rpm = ratio * 1300
    elif virtual_gear == 2:
        ratio = (duty_val - SHIFT_POINT_1) / (SHIFT_POINT_2 - SHIFT_POINT_1)
        virtual_rpm = 800 + (ratio * (1300 - 800))
    elif virtual_gear == 3:
        ratio = (duty_val - SHIFT_POINT_2) / (100 - SHIFT_POINT_2)
        virtual_rpm = 900 + (ratio * (MAX_RPM_REAL - 900))
        
    return virtual_gear, int(virtual_rpm)

# ---- 메인 하드웨어 루프 ----
def hardware_loop():
    global current_duty, target_duty_raw, current_pedal_raw, current_safety_reason, current_remaining_time, stop_threads, is_warning_sound_active
    global shift_pause_timer, drive_mode, safety_mode_enabled
    
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup(PWM_A_PIN, GPIO.OUT); GPIO.setup(IN1_PIN, GPIO.OUT); GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(PWM_B_PIN, GPIO.OUT); GPIO.setup(IN3_PIN, GPIO.OUT); GPIO.setup(IN4_PIN, GPIO.OUT)
    if PLATFORM == "LINUX": 
        GPIO.setup(TRIG_PIN, GPIO.OUT); GPIO.setup(ECHO_PIN, GPIO.IN)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(BTN_DRIVE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(BTN_PARK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(BTN_SAFETY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    pwm_a = GPIO.PWM(PWM_A_PIN, 1000); pwm_a.start(0)
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000); pwm_b.start(0)
    GPIO.output(IN1_PIN, True); GPIO.output(IN2_PIN, False)
    GPIO.output(IN3_PIN, True); GPIO.output(IN4_PIN, False)
    
    press_timestamps = deque(); last_pedal_value = 0; last_time = time.time()
    
    state = { 
        "lock_active": False, 
        "pedal_error_expiry": 0.0,
        "prev_over_90": False, 
        "prev_front_danger": False, 
        "last_pedal_active_time": time.time() 
    }

    ser = None
    smoothed_duty = 0.0
    
    # 버튼 노이즈 방지용 변수
    last_safety_btn_val = 1 
    last_safety_toggle_time = 0

    try:
        while not stop_threads:
            if ser is None and PLATFORM == "LINUX":
                try: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1); ser.flush()
                except: pass

            t_now = time.time() # 현재 시간

            if PLATFORM == "LINUX":
                # [수정] 잠금(Lock) 상태가 아닐 때만 기어 변경 가능 (N/D 충돌 방지)
                if not state["lock_active"]:
                    if GPIO.input(BTN_DRIVE_PIN) == 0:
                        if drive_mode != 'D': drive_mode = 'D'
                    if GPIO.input(BTN_PARK_PIN) == 0:
                        if drive_mode != 'P': drive_mode = 'P'
                
                # [수정] 안전 모드 토글 (디바운싱 적용)
                curr_safety_btn = GPIO.input(BTN_SAFETY_PIN)
                if curr_safety_btn == 0 and last_safety_btn_val == 1: 
                    if t_now - last_safety_toggle_time > 0.5: # 0.5초 딜레이
                        safety_mode_enabled = not safety_mode_enabled
                        last_safety_toggle_time = t_now
                        print(f"Safety Mode: {safety_mode_enabled}")
                        if not safety_mode_enabled:
                            state["lock_active"] = False
                            is_warning_sound_active = False
                last_safety_btn_val = curr_safety_btn
            
            if ser and ser.in_waiting > 0:
                try:
                    lines = ser.read_all().decode('utf-8').split('\n')
                    valid_lines = [l.strip() for l in lines if l.strip().isdigit()]
                    if valid_lines: 
                        raw_val = int(valid_lines[-1])
                        current_pedal_raw = max(0, min(100, raw_val))
                except: pass
            
            raw_dist = read_distance()
            if raw_dist is not None: dist_history.append(raw_dist)
            final_dist = 0.0
            if len(dist_history) > 0: final_dist = sum(dist_history) / len(dist_history)

            is_btn_pressed = False
            if PLATFORM == "LINUX": is_btn_pressed = (GPIO.input(BUTTON_PIN) == 0)

            # ----------------------------------------------------
            if safety_mode_enabled:
                result = process_safety_logic(
                    t_now, current_pedal_raw, last_pedal_value, last_time,
                    final_dist, is_btn_pressed,
                    state["lock_active"], state["pedal_error_expiry"], 
                    press_timestamps, state["prev_over_90"], state["prev_front_danger"], state["last_pedal_active_time"],
                    drive_mode 
                )
                
                # [수정] 잠금 상태면 강제로 N단 유지
                if result["lock_active"] and drive_mode == 'D':
                    drive_mode = 'N'
                if result["unlock_success"]:
                    drive_mode = 'D'
                    result["visual_gear"] = 'D'

                target_raw = float(result["target_speed"])
                visual_gear = result["visual_gear"]
                is_warning_sound_active = result["trigger_sound"]
                
                current_safety_reason = result["logical_reason"]
                
                state.update({
                    "lock_active": result["lock_active"], 
                    "pedal_error_expiry": result["pedal_error_expiry"],
                    "prev_over_90": result["prev_over_90"],
                    "prev_front_danger": result["prev_front_danger"], 
                    "last_pedal_active_time": result["last_pedal_active_time"]
                })
                
                v_val = result["angular_velocity"]
                r_val = 1 if (result["lock_active"] or result["prev_front_danger"]) else 0

            else:
                target_raw = current_pedal_raw 
                if drive_mode == 'P' or drive_mode == 'N': target_raw = 0
                else: target_raw = max(current_pedal_raw, IDLE_DUTY) 
                
                visual_gear = drive_mode
                is_warning_sound_active = False
                current_safety_reason = None
                v_val = 0; r_val = 0
                
                dt = t_now - last_time
                if dt > 0:
                    delta_percent = current_pedal_raw - last_pedal_value
                    v_val = (delta_percent / 100.0 * PEDAL_TOTAL_ANGLE) / dt

            # ----------------------------------------------------
            
            is_shifting = (t_now < shift_pause_timer)
            if not is_shifting:
                if target_raw > smoothed_duty:
                    smoothed_duty += ACCEL_STEP
                    if smoothed_duty > target_raw: smoothed_duty = target_raw
                elif target_raw < smoothed_duty:
                    smoothed_duty -= DECEL_STEP
                    if smoothed_duty < target_raw: smoothed_duty = target_raw
            
            sim_duty_input = smoothed_duty
            if visual_gear == 'N': sim_duty_input = current_pedal_raw 

            gear_num, rpm = simulate_transmission(sim_duty_input, t_now)
            if visual_gear == 'P': rpm = 0; gear_num = 1
            
            current_duty = smoothed_duty
            pwm_a.ChangeDutyCycle(smoothed_duty)
            pwm_b.ChangeDutyCycle(smoothed_duty)

            # [수정] 큐에 데이터 넣기 (그래프 타임스탬프 ms 단위 수정)
            data_queue.put({
                "t": t_now * 1000, # 그래프를 위해 1000 곱함
                "p": current_pedal_raw, 
                "d": current_duty,
                "v": v_val, 
                "dist": round(final_dist, 1),
                "r": r_val, 
                "pc": len(press_timestamps),
                "rpm": rpm, 
                "gear": gear_num,
                "v_gear_char": visual_gear,
                "safety_mode": safety_mode_enabled,
                "msg": current_safety_reason # 메시지 통일
            })

            last_pedal_value = current_pedal_raw; last_time = t_now
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
            
            # 큐 배치 전송
            history_batch = []
            while not data_queue.empty():
                try: history_batch.append(data_queue.get_nowait())
                except: break
            
            if history_batch:
                latest = history_batch[-1]
                
                # reason 필드명을 msg와 일치시킴
                reason_val = latest.get("msg") 
                if reason_val is None: reason_val = current_safety_reason

                payload = {
                    "type": "batch", "history": history_batch,
                    "current": {
                        "duty": round(current_duty, 1), 
                        "pedal": current_pedal_raw,
                        "reason": reason_val, 
                        "remaining_time": current_remaining_time,
                        "rpm": latest.get("rpm", 0), 
                        "gear": latest.get("gear", 1), 
                        "v_gear": latest.get("v_gear_char", 'N'),
                        "safety_mode": latest.get("safety_mode", True)
                    }
                }
                await websocket.send_json(payload)
            await asyncio.sleep(0.05)
    except: pass
app.include_router(router)