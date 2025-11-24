import subprocess
import time
import os

def set_volume_debug(level):
    # 실행할 명령어 문자열 생성
    cmd = f"amixer -c 3 sset PCM {level}%"
    print(f"\n[명령어 실행]: {cmd}")
    
    # 쉘 명령어를 실행하고 결과를 캡처 (에러가 나면 stderr에 담김)
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ 성공! 하드웨어 응답:")
        print(result.stdout.strip()) # 터미널에서 봤던 그 결과가 여기서 출력되어야 함
    else:
        print("❌ 실패! 에러 메시지:")
        print(result.stderr)

# ---- 테스트 시나리오 ----

print("1. 볼륨 40% 테스트")
set_volume_debug(40)
os.system('espeak "Volume check 40 percent"')

time.sleep(2)

print("\n2. 볼륨 90% 테스트")
set_volume_debug(90)
os.system('espeak "Volume check 90 percent"')