import os

def set_volume(level):
    """
    level: 0 ~ 100 사이의 정수
    """
    # [수정됨] -c 3 : 3번 카드 선택
    # [주의] 네 장치의 컨트롤 이름이 Speaker라면 'PCM' 대신 'Speaker'라고 써야 함
    os.system(f"amixer -c 3 sset PCM {level}% > /dev/null")

# 테스트
set_volume(80)