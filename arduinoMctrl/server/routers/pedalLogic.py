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

# [기어 변속 버튼]
BTN_DRIVE_PIN = 16  
BTN_PARK_PIN = 20   

SERIAL_PORT = '/dev/ttyUSB0'; BAUD_RATE = 115200

# ---- 3. 임계값 설정 ----
PEDAL_TOTAL_ANGLE = 45.0
CRITICAL_ANGULAR_VELOCITY = 420
RAPID_PRESS_COUNT = 3      
RAPID_PRESS_WINDOW = 2.0   
SAFETY_SPEED = 20     
COLLISION_DIST_LIMIT = 100.0 

# [D모드 최소 속도] 30km/h ≈ Duty 19%
IDLE_DUTY = 19.0      
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

# [초기 상태 N]
drive_mode = 'N' 

# 변속기 상태
virtual_gear = 1
virtual_rpm = 0
shift_pause_timer = 0.0

dist_history = deque(maxlen=10) 
data_queue = queue.Queue()

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
# 안전 로직 (Logic)
# =========================================================
def process_safety_logic(
    current_time, current_pedal, last_pedal, last_time,
    final_dist, is_btn_pressed,
    lock_active, msg_expiry, last_transient_msg,
    press_timestamps, prev_over_90, prev_front_danger, last_pedal_active_time,
    current_drive_mode
):
    target_speed = 0; frame_reason = None
    current_angular_velocity = 0.0
    
    # [1] Park(P)
    if current_drive_mode == 'P':
        return {
            "target_speed": 0, "logical_reason": None,
            "angular_velocity": 0, "lock_active": False, "msg_expiry": 0,
            "last_transient_msg": None, "prev_over_90": False,
            "prev_front_danger": False, "last_pedal_active_time": current_time,
            "visual_gear": "P"
        }
    
    # [2] Neutral(N)
    if current_drive_mode == 'N':
        return {
            "target_speed": 0, "logical_reason": None,
            "angular_velocity": 0, "lock_active": False, "msg_expiry": 0,
            "last_transient_msg": None, "prev_over_90": False,
            "prev_front_danger": False, "last_pedal_active_time": current_time,
            "visual_gear": "N"
        }

    # [3] Drive(D)
    visual_gear = "D" 
    front_danger = False
    
    # 전방 감지
    if final_dist > 0 and final_dist <= COLLISION_DIST_LIMIT and current_pedal > 0:
        front_danger = True

    # ---- [중요] 안전 제한(Lock) 로직 ----
    if lock_active:
        target_speed = SAFETY_SPEED 
        visual_gear = "N" 
        
        # 안내 문구 설정
        if current_pedal > 0:
            frame_reason = "⚠️ 엑셀에서 발을 먼저 떼세요!"
        else:
            if is_btn_pressed:
                # 해제 성공
                lock_active = False; msg_expiry = 0
                frame_reason = None; target_speed = IDLE_DUTY 
            else:
                frame_reason = "🔵 푸쉬버튼을 눌러 제한을 해제하세요"
    
    # 위험 감지 (충돌)
    elif front_danger:
        frame_reason = "⚠️ 전방을 주의하세요!"
        target_speed = 0

    # 정상 주행 중 감지 (급발진/과속)
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
                msg_expiry = current_time + 5.0; last_transient_msg = event_msg
                target_speed = SAFETY_SPEED
                visual_gear = "N" 
                # 여기서 리턴하지 않고, 다음 루프부터 frame_reason이 채워지도록 함
            else:
                if current_pedal > 0:
                    last_pedal_active_time = current_time
                    target_speed = max(current_pedal, IDLE_DUTY)
                else:
                    target_speed = IDLE_DUTY 

    # [핵심 수정] 메시지 우선순위 변경
    # 안내 문구(frame_reason)가 있으면 그것을 최우선으로 표시
    # 그게 없을 때만 5초간 유지되는 이벤트 메시지(last_transient_msg) 표시
    logical_reason = None
    
    if frame_reason is not None:
        logical_reason = frame_reason
    elif current_time < msg_expiry and last_transient_msg is not None:
        logical_reason = last_transient_msg

    return {
        "target_speed": target_speed, "logical_reason": logical_reason,
        "angular_velocity": current_angular_velocity,
        "lock_active": lock_active, "msg_expiry": msg_expiry,
        "last_transient_msg": last_transient_msg, "prev_over_90": prev_over_90,
        "prev_front_danger": front_danger, "last_pedal_active_time": last_pedal_active_time,
        "visual_gear": visual_gear
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
    global current_duty, target_duty_raw, current_pedal_raw, current_safety_reason, current_remaining_time, stop_threads
    global shift_pause_timer, drive_mode
    
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup(PWM_A_PIN, GPIO.OUT); GPIO.setup(IN1_PIN, GPIO.OUT); GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(PWM_B_PIN, GPIO.OUT); GPIO.setup(IN3_PIN, GPIO.OUT); GPIO.setup(IN4_PIN, GPIO.OUT)
    if PLATFORM == "LINUX": 
        GPIO.setup(TRIG_PIN, GPIO.OUT); GPIO.setup(ECHO_PIN, GPIO.IN)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(BTN_DRIVE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(BTN_PARK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    pwm_a = GPIO.PWM(PWM_A_PIN, 1000); pwm_a.start(0)
    pwm_b = GPIO.PWM(PWM_B_PIN, 1000); pwm_b.start(0)
    GPIO.output(IN1_PIN, True); GPIO.output(IN2_PIN, False)
    GPIO.output(IN3_PIN, True); GPIO.output(IN4_PIN, False)
    
    press_timestamps = deque(); last_pedal_value = 0; last_time = time.time()
    state = { "lock_active": False, "msg_expiry": 0.0, "last_transient_msg": None,
              "prev_over_90": False, "prev_front_danger": False, "last_pedal_active_time": time.time() }

    ser = None
    smoothed_duty = 0.0

    try:
        while not stop_threads:
            if ser is None and PLATFORM == "LINUX":
                try: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1); ser.flush()
                except: pass

            if PLATFORM == "LINUX":
                if GPIO.input(BTN_DRIVE_PIN) == 0:
                    if drive_mode != 'D': 
                        print("👉 Drive Button Pressed!")
                        drive_mode = 'D'
                
                if GPIO.input(BTN_PARK_PIN) == 0:
                    if drive_mode != 'P':
                        print("👉 Park Button Pressed!")
                        drive_mode = 'P'
            
            if ser and ser.in_waiting > 0:
                try:
                    lines = ser.read_all().decode('utf-8').split('\n')
                    valid_lines = [l.strip() for l in lines if l.strip().isdigit()]
                    if valid_lines: 
                        raw_val = int(valid_lines[-1])
                        current_pedal_raw = max(0, min(100, raw_val))
                except: pass
            
            current_time = time.time()

            raw_dist = read_distance()
            if raw_dist is not None: dist_history.append(raw_dist)
            final_dist = 0.0
            if len(dist_history) > 0: final_dist = sum(dist_history) / len(dist_history)

            is_btn_pressed = False
            if PLATFORM == "LINUX": is_btn_pressed = (GPIO.input(BUTTON_PIN) == 0)

            result = process_safety_logic(
                current_time, current_pedal_raw, last_pedal_value, last_time,
                final_dist, is_btn_pressed,
                state["lock_active"], state["msg_expiry"], state["last_transient_msg"],
                press_timestamps, state["prev_over_90"], state["prev_front_danger"], state["last_pedal_active_time"],
                drive_mode 
            )
            
            if result["lock_active"] and drive_mode == 'D':
                drive_mode = 'N'

            target_raw = float(result["target_speed"])
            visual_gear = result["visual_gear"]
            
            is_shifting = (current_time < shift_pause_timer)
            
            if not is_shifting:
                if target_raw > smoothed_duty:
                    smoothed_duty += ACCEL_STEP
                    if smoothed_duty > target_raw: smoothed_duty = target_raw
                elif target_raw < smoothed_duty:
                    smoothed_duty -= DECEL_STEP
                    if smoothed_duty < target_raw: smoothed_duty = target_raw
            
            sim_duty_input = smoothed_duty
            if visual_gear == 'N':
                sim_duty_input = current_pedal_raw 

            gear_num, rpm = simulate_transmission(sim_duty_input, current_time)
            
            if visual_gear == 'P': 
                rpm = 0
                gear_num = 1
            
            current_duty = smoothed_duty
            pwm_a.ChangeDutyCycle(smoothed_duty)
            pwm_b.ChangeDutyCycle(smoothed_duty)

            new_reason = result["logical_reason"]
            current_safety_reason = new_reason

            state.update({
                "lock_active": result["lock_active"], "msg_expiry": result["msg_expiry"],
                "last_transient_msg": result["last_transient_msg"], "prev_over_90": result["prev_over_90"],
                "prev_front_danger": result["prev_front_danger"], "last_pedal_active_time": result["last_pedal_active_time"]
            })
            
            data_queue.put({
                "t": current_time * 1000, "p": current_pedal_raw, "d": current_duty,
                "v": result["angular_velocity"], "dist": round(final_dist, 1),
                "r": 1 if (state["lock_active"] or result["prev_front_danger"]) else 0, 
                "pc": len(press_timestamps),
                "rpm": rpm, 
                "gear": gear_num,
                "v_gear_char": visual_gear 
            })

            last_pedal_value = current_pedal_raw; last_time = current_time
            time.sleep(0.01)

    except Exception as e: print(e)
    finally:
        pwm_a.stop(); pwm_b.stop(); GPIO.cleanup()
        if ser and ser.is_open: ser.close()

def start_hardware():
    # 오디오 스레드 제거됨
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
            
            last_rpm = 0; last_gear = 1; last_v_gear = 'N'
            if history_batch:
                last_rpm = history_batch[-1].get("rpm", 0)
                last_gear = history_batch[-1].get("gear", 1)
                last_v_gear = history_batch[-1].get("v_gear_char", 'N')

            payload = {
                "type": "batch", "history": history_batch,
                "current": {
                    "duty": round(current_duty, 1), "pedal": current_pedal_raw,
                    "reason": current_safety_reason, "remaining_time": current_remaining_time,
                    "rpm": last_rpm, "gear": last_gear, "v_gear": last_v_gear
                }
            }
            await websocket.send_json(payload)
            await asyncio.sleep(0.05)
    except: pass
app.include_router(router)