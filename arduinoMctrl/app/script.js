// script.js 전체를 아래와 같이 수정하면 UX가 더 좋아집니다.

const wsUrl = "ws://" + window.location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const pedalText = document.getElementById("pedal-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");
// const warningIcon = document.querySelector(".warning-icon"); // 아이콘 제어 필요시 사용

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    // 1. 모터 듀티 게이지
    if (data.duty !== undefined) {
        const duty = parseFloat(data.duty);
        valText.innerText = duty.toFixed(0);
        
        let angle = (duty * 1.8) - 90;
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;
        needle.style.transform = `rotate(${angle}deg)`;
    }

    // 2. 페달 값
    if (data.pedal !== undefined && pedalText) {
        pedalText.innerText = data.pedal;
    }

    // 3. [수정됨] 경고 메시지 및 상태 처리
    if (data.reason) {
        warningBox.style.display = "block";
        warningText.innerText = data.reason; 

        // [추가 기능] 남은 시간이 0인데(시간은 지남) 경고가 떠 있다면 -> "발을 떼세요" 상황
        // 이때 배경색을 더 진하게 하거나 스타일을 변경할 수 있습니다.
        if (data.remaining_time !== undefined && data.remaining_time === 0) {
            warningBox.style.backgroundColor = "rgba(255, 0, 0, 1)"; // 완전 빨강
            warningBox.style.border = "3px solid #ffffff"; // 테두리 흰색 강조
        } else {
            // 카운트다운 중일 때 (기본 스타일)
            warningBox.style.backgroundColor = "rgba(255, 0, 0, 0.9)";
            warningBox.style.border = "3px solid #ff5555";
        }

    } else {
        warningBox.style.display = "none";
    }
};