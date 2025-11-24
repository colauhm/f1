import os

def set_volume(level):
    """
    level: 0 ~ 100 사이의 정수
    """
    # 카드 번호(-c 1)와 컨트롤 이름(PCM)은 네 환경에 맞게 수정해야 함
    os.system(f"amixer -c 3 sset PCM {level}% > /dev/null")

# 사용 예시
print("안전 모드 발동! 볼륨 최대")
set_volume(100)
# 경고음 재생 (espeak 등)

print("정상 주행 복귀. 볼륨 조정")
set_volume(50)