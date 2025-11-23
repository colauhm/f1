const wsUrl = "ws://" + window.location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const pedalText = document.getElementById("pedal-val"); // 페달 값 표시용
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    // 1. 모터 듀티 게이지 업데이트
    if (data.duty !== undefined) {
        const duty = parseFloat(data.duty);
        valText.innerText = duty.toFixed(0);
        
        let angle = (duty * 1.8) - 90;
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;
        needle.style.transform = `rotate(${angle}deg)`;
    }

    // 2. 페달 값 업데이트 (선택사항)
    if (data.pedal !== undefined && pedalText) {
        pedalText.innerText = data.pedal;
    }

    // 3. [핵심] 경고 메시지 처리
    if (data.reason) {
        // 원인 데이터가 있으면 경고창 띄움
        warningBox.style.display = "block";
        warningText.innerText = data.reason; // 서버에서 온 메시지 표시
    } else {
        // 원인이 없으면(null) 숨김
        warningBox.style.display = "none";
    }
};