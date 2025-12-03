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

# ... (상단 import 및 설정 변수들 동일) ...

# =========================================================
# [분리됨] 안전 로직 및 모터 속도 계산 함수
# =========================================================
def process_safety_logic(
    # [입력] 현재 상태 값들
    current_time, current_pedal, last_pedal, last_time,
    final_dist, is_btn_pressed,
    # [상태 변수] (이전 프레임의 상태를 받아와서 갱신)
    lock_active, msg_expiry, last_transient_msg,
    press_timestamps, prev_over_90, prev_front_danger, last_pedal_active_time
):
    """
    모터의 목표 속도, 경고 메시지, 안전 잠금 상태 등을 판단하는 순수 로직 함수
    """
    
    # 1. 초기화 및 기본 변수 계산
    target_speed = 0
    trigger_siren = False
    frame_reason = None     # 이번 프레임의 즉각적인 상태 메시지
    current_angular_velocity = 0.0
    
    # 거리 기반 위험 감지
    front_danger = False
    if final_dist > 0 and final_dist <= COLLISION_DIST_LIMIT and current_pedal > 0:
        front_danger = True

    # -----------------------------------------------------
    # [판단 로직 1] 안전 잠금(Lock) 상태일 때
    # -----------------------------------------------------
    if lock_active:
        target_speed = SAFETY_SPEED
        # 발을 밟고 있으면 경고, 떼고 버튼 누르면 해제
        if current_pedal > 0:
            frame_reason = "⚠️ 엑셀에서 발을 먼저 떼세요!"
        else:
            if is_btn_pressed:
                lock_active = False       # 잠금 해제
                msg_expiry = 0            # 기존 타이머 초기화
                frame_reason = None
                target_speed = 0
            else:
                frame_reason = "🔵 푸쉬버튼을 눌러 제한을 해제하세요"

    # -----------------------------------------------------
    # [판단 로직 2] 전방 장애물 위험 (Lock 아님)
    # -----------------------------------------------------
    elif front_danger:
        frame_reason = "⚠️ 전방을 주의하세요!"
        target_speed = 0
        # 이번에 새로 감지된 위험이면 사이렌 울림
        if not prev_front_danger:
            trigger_siren = True

    # -----------------------------------------------------
    # [판단 로직 3] 정상 주행 중 이벤트(급발진/과속) 감지
    # -----------------------------------------------------
    else:
        dt = current_time - last_time
        if dt > 0:
            # (1) 각속도 계산
            delta_percent = current_pedal - last_pedal
            delta_angle = (delta_percent / 100.0) * PEDAL_TOTAL_ANGLE
            angular_velocity = delta_angle / dt
            current_angular_velocity = angular_velocity
            
            trigger_event = False
            event_msg = ""

            # (A) 급발진 체크
            if angular_velocity >= CRITICAL_ANGULAR_VELOCITY:
                trigger_event = True
                event_msg = "⚠️ 급발진 감지!"
            
            # (B) 페달 연타 체크
            is_over_90 = (current_pedal >= 90)
            if is_over_90 and not prev_over_90: 
                press_timestamps.append(current_time)
            
            # 오래된 연타 기록 삭제
            while press_timestamps and press_timestamps[0] < current_time - RAPID_PRESS_WINDOW:
                press_timestamps.popleft()
            
            if len(press_timestamps) >= RAPID_PRESS_COUNT:
                trigger_event = True
                event_msg = "🚫 과속 페달 연타!"
                press_timestamps.clear()
            
            prev_over_90 = is_over_90

            # (C) 이벤트 발생 시 처리
            if trigger_event:
                lock_active = True
                trigger_siren = True
                # [핵심] 5초간 메시지 고정
                msg_expiry = current_time + 5.0
                last_transient_msg = event_msg
            else:
                # 정상 주행 속도 제어
                if current_pedal > 0:
                    last_pedal_active_time = current_time
                    target_speed = max(current_pedal, IDLE_SPEED)
                else:
                    if (current_time - last_pedal_active_time) >= IDLE_TIMEOUT:
                        target_speed = 0
                    else:
                        target_speed = IDLE_SPEED

    # -----------------------------------------------------
    # [최종 메시지 우선순위 결정]
    # 1. 5초 타이머가 살아있으면 -> 원인 메시지(급발진 등) 최우선
    # 2. 타이머 종료 후 -> 현재 상태 메시지(발 떼세요 등)
    # -----------------------------------------------------
    final_reason_text = None
    if current_time < msg_expiry and last_transient_msg is not None:
        final_reason_text = last_transient_msg
    elif frame_reason is not None:
        final_reason_text = frame_reason

    # 결과 반환 (갱신된 상태값들)
    return {
        "target_speed": target_speed,
        "final_reason": final_reason_text,
        "trigger_siren": trigger_siren,
        "angular_velocity": current_angular_velocity,
        # 갱신된 상태 변수들
        "lock_active": lock_active,
        "msg_expiry": msg_expiry,
        "last_transient_msg": last_transient_msg,
        "prev_over_90": prev_over_90,
        "prev_front_danger": front_danger,
        "last_pedal_active_time": last_pedal_active_time
    }


# ---- 메인 하드웨어 루프 (수정됨) ----
def hardware_loop():
    global current_duty, current_pedal_raw, current_safety_reason, current_remaining_time, stop_threads
    
    # 1. GPIO 설정
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
    
    # 2. 상태 변수 초기화
    press_timestamps = deque()  # deque는 mutable이라 함수 내부에서 수정 가능
    
    last_pedal_value = 0
    last_time = time.time()
    
    # [함수로 전달할 상태 변수들]
    state = {
        "lock_active": False,
        "msg_expiry": 0.0,
        "last_transient_msg": None,
        "prev_over_90": False,
        "prev_front_danger": False,
        "last_pedal_active_time": time.time()
    }

    ser = None
    if PLATFORM == "LINUX":
        try: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1); ser.flush()
        except: pass

    try:
        while not stop_threads:
            # 시리얼 읽기
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

                # 거리 측정
                raw_dist = read_distance()
                if raw_dist is not None: dist_history.append(raw_dist)
                final_dist = 0.0
                if len(dist_history) > 0: final_dist = sum(dist_history) / len(dist_history)

                # 버튼 입력 확인
                is_btn_pressed = False
                if PLATFORM == "LINUX":
                    is_btn_pressed = (GPIO.input(BUTTON_PIN) == 0)

                # ==========================================================
                # [함수 호출] 안전 로직 및 속도 계산 위임
                # ==========================================================
                result = process_safety_logic(
                    current_time, current_pedal_value, last_pedal_value, last_time,
                    final_dist, is_btn_pressed,
                    # 이전 상태값 전달
                    state["lock_active"], state["msg_expiry"], state["last_transient_msg"],
                    press_timestamps, state["prev_over_90"], state["prev_front_danger"], state["last_pedal_active_time"]
                )

                # ==========================================================
                # [결과 반영] 함수 반환값으로 시스템 상태 업데이트
                # ==========================================================
                
                # 1. 제어값 적용
                target_speed = result["target_speed"]
                current_safety_reason = result["final_reason"]
                if result["trigger_siren"]:
                    play_siren_thread()
                
                # 2. 다음 프레임을 위해 상태 변수 갱신
                state["lock_active"] = result["lock_active"]
                state["msg_expiry"] = result["msg_expiry"]
                state["last_transient_msg"] = result["last_transient_msg"]
                state["prev_over_90"] = result["prev_over_90"]
                state["prev_front_danger"] = result["prev_front_danger"]
                state["last_pedal_active_time"] = result["last_pedal_active_time"]
                
                # 3. 모터 구동
                pwm_a.ChangeDutyCycle(target_speed)
                pwm_b.ChangeDutyCycle(target_speed)
                current_duty = target_speed

                # 4. 데이터 큐 전송
                data_queue.put({
                    "t": current_time * 1000,
                    "p": current_pedal_value,
                    "d": current_duty,
                    "v": result["angular_velocity"],
                    "dist": round(final_dist, 1),
                    "r": 1 if (state["lock_active"] or result["prev_front_danger"]) else 0,
                    "pc": len(press_timestamps)
                })

                # 루프 변수 갱신
                last_pedal_value = current_pedal_value
                last_time = current_time
            
            time.sleep(0.01)

    except Exception as e: print(e)
    finally:
        pwm_a.stop(); pwm_b.stop(); GPIO.cleanup()
        if ser and ser.is_open: ser.close()

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