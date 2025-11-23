const wsUrl = "ws://" + window.location.host + "/api/ws";
const ws = new WebSocket(wsUrl);

const bar = document.getElementById("duty-bar");
const valText = document.getElementById("duty-val");
const statusMsg = document.getElementById("status-msg");
const btn = document.getElementById("acc-btn");

// 전송 타이머 ID
let intervalId = null;

ws.onopen = () => {
    console.log("Connected");
    statusMsg.innerText = "Connected (Ready)";
    statusMsg.style.color = "#00f260";
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.duty !== undefined) {
        bar.style.width = data.duty + "%";
        valText.innerText = data.duty;
    }
};

ws.onclose = () => {
    statusMsg.innerText = "Disconnected";
    statusMsg.style.color = "#ff4b1f";
    bar.style.width = "0%";
};

// ---- [핵심] 버튼 제어 로직 ----

function startSending() {
    if (intervalId) return; // 이미 누르고 있다면 중복 실행 방지
    
    // 누르자마자 한 번 전송
    sendGo();
    
    // 누르고 있는 동안 50ms마다 계속 전송 (심장박동처럼)
    intervalId = setInterval(() => {
        sendGo();
    }, 50);
    
    btn.style.backgroundColor = "#d63a15"; // 버튼 눌림 시각 효과
}

function stopSending() {
    if (intervalId) {
        clearInterval(intervalId);
        intervalId = null;
    }
    btn.style.backgroundColor = ""; // 원래 색 복귀
}

function sendGo() {
    if (ws.readyState === WebSocket.OPEN) {
        // 서버로 "나 지금 누르고 있어!" 신호 전송
        ws.send(JSON.stringify({ command: "go" }));
    }
}

// 마우스 이벤트 (PC)
btn.addEventListener("mousedown", startSending);
btn.addEventListener("mouseup", stopSending);
btn.addEventListener("mouseleave", stopSending); // 버튼 밖으로 마우스가 나가면 멈춤

// 터치 이벤트 (모바일) - 이게 없으면 폰에서 반응 안 함
btn.addEventListener("touchstart", (e) => {
    e.preventDefault(); // 터치 시 확대/스크롤 등 기본 동작 방지
    startSending();
});
btn.addEventListener("touchend", stopSending);