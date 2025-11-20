const wsUrl = "ws://" + window.location.host + "/api/ws";
const ws = new WebSocket(wsUrl);

const bar = document.getElementById("duty-bar");
const valText = document.getElementById("duty-val");
const statusMsg = document.getElementById("status-msg");

ws.onopen = () => {
    console.log("Connected");
    statusMsg.innerText = "Connected (Monitoring)";
    statusMsg.style.color = "#00f260";
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    if (data.duty !== undefined) {
        // 게이지 바 업데이트
        bar.style.width = data.duty + "%";
        // 숫자 텍스트 업데이트
        valText.innerText = data.duty;
    }
};

ws.onclose = () => {
    statusMsg.innerText = "Disconnected";
    statusMsg.style.color = "#ff4b1f";
    bar.style.width = "0%";
};