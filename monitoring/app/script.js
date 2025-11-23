const wsUrl = "ws://" + location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const statusText = document.getElementById("status-text");

ws.onopen = () => {
    console.log("Connected");
    statusText.innerText = "Connected (Monitoring)";
    statusText.style.color = "#00ff00";
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    if (data.duty !== undefined) {
        const duty = parseFloat(data.duty);

        // 텍스트 업데이트
        valText.innerText = duty.toFixed(0); // 소수점 제거

        // --- [핵심] 각도 계산 ---
        // Duty 0%   -> -90도 (왼쪽 끝)
        // Duty 50%  -> 0도 (가운데)
        // Duty 100% -> +90도 (오른쪽 끝)
        // 공식: (duty * 1.8) - 90
        
        let angle = (duty * 1.8) - 90;
        
        // 각도 제한 (바늘이 범위를 넘어가지 않게)
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;

        // CSS transform 적용
        needle.style.transform = `rotate(${angle}deg)`;
        
        // 고속일 때 숫자 색상 변경 (빨간색)
        if (duty > 80) {
            valText.style.color = "#ff4444";
        } else {
            valText.style.color = "#ffffff";
        }
    }
};

ws.onclose = () => {
    statusText.innerText = "Disconnected";
    statusText.style.color = "#ff4444";
    // 연결 끊기면 바늘 초기화
    needle.style.transform = `rotate(-90deg)`;
};