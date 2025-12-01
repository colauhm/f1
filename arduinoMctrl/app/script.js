// 현재 페이지가 https라면 wss를, 아니면 ws를 쓰도록 자동 설정
var protocol = (location.protocol === 'https:') ? 'wss://' : 'ws://';
var wsAddress = protocol + location.host + "/ws";

const ws = new WebSocket(wsAddress);
const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");

// [추가] 정보창 UI 요소
const valVelocity = document.getElementById("val-velocity");
const valDistance = document.getElementById("val-distance");
const valHistory = document.getElementById("val-history");

// 버퍼
let pedalBuffer = [], motorBuffer = [], velocityBuffer = [], distanceBuffer = [];
const MAX_STORE_MINUTES = 5; 
let viewSeconds = 60; 

function createChartConfig(buffer, label, color = '#39ff14') {
    return {
        type: 'line',
        data: {
            datasets: [{
                label: label, data: buffer, borderWidth: 1.5, fill: true,
                backgroundColor: color.startsWith('#') ? color + '1A' : color, borderColor: color,
                tension: 0.4, pointRadius: 0, spanGaps: true,
                segment: { borderColor: ctx => (ctx.p1.raw && ctx.p1.raw.restricted) ? '#ff0000' : color }
            }]
        },
        options: {
            responsive: true, maintainAspectRatio: false, animation: false, interaction: { intersect: false },
            scales: {
                x: { type: 'linear', display: true, ticks: { color: '#666', maxTicksLimit: 5, callback: val => new Date(val).toLocaleTimeString('ko-KR', { hour12: false }) }, grid: { display: false } },
                y: { grid: { color: '#333' }, ticks: { color: '#888' } }
            },
            plugins: { legend: { display: false }, tooltip: { enabled: false } }
        }
    };
}

const ctxPedal = document.getElementById('pedalChart').getContext('2d');
const pedalChart = new Chart(ctxPedal, createChartConfig(pedalBuffer, 'Pedal', '#39ff14'));
pedalChart.options.scales.y.min = 0; pedalChart.options.scales.y.max = 100;

const ctxMotor = document.getElementById('motorChart').getContext('2d');
const motorChart = new Chart(ctxMotor, createChartConfig(motorBuffer, 'Motor', '#39ff14'));
motorChart.options.scales.y.min = 0; motorChart.options.scales.y.max = 100;

const ctxVelocity = document.getElementById('velocityChart').getContext('2d');
const velocityChart = new Chart(ctxVelocity, createChartConfig(velocityBuffer, 'Velocity', '#ffff00'));

const ctxDistance = document.getElementById('distanceChart').getContext('2d');
const distanceChart = new Chart(ctxDistance, createChartConfig(distanceBuffer, 'Distance', '#00d2ff'));
distanceChart.options.scales.y.min = 0;

function initChartData() {
    const now = Date.now();
    for (let i = 2000; i > 0; i--) {
        const pt = { x: now - (i * 50), y: 0, restricted: false };
        pedalBuffer.push(pt); motorBuffer.push(pt); velocityBuffer.push(pt); distanceBuffer.push(pt);
    }
}

window.setTimeMode = function(seconds) {
    viewSeconds = seconds;
    document.querySelectorAll('.time-btn').forEach(btn => btn.classList.remove('active'));
    if (seconds === 10) document.getElementById('btn-10sec').classList.add('active');
    else if (seconds === 30) document.getElementById('btn-30sec').classList.add('active');
    else if (seconds === 60) document.getElementById('btn-1min').classList.add('active');
    requestAnimationFrame(updateChartScale);
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type === "batch") {
        const current = data.current;
        const history = data.history;

        // 1. 게이지 업데이트
        if (current.duty !== undefined) {
            const duty = parseFloat(current.duty);
            valText.innerText = duty.toFixed(0);
            let angle = (duty * 1.8) - 90;
            if (angle < -90) angle = -90; if (angle > 90) angle = 90;
            needle.style.transform = `rotate(${angle}deg)`;
        }

        // 2. 경고창
        if (current.reason) {
            warningBox.style.display = "block";
            warningText.innerText = current.reason;
            // 즉시 정지(0초)인 경우 빨간색 강조
            warningBox.style.backgroundColor = (current.remaining_time === 0) 
                ? "rgba(255, 0, 0, 1)" : "rgba(255, 0, 0, 0.9)";
        } else {
            warningBox.style.display = "none";
        }

        // 3. 차트 데이터 & 정보창 업데이트
        if (history && history.length > 0) {
            let lastPt = history[history.length - 1]; // 가장 최신 데이터

            // [정보창 업데이트]
            if (lastPt.v !== undefined) valVelocity.innerText = lastPt.v.toFixed(1);
            if (lastPt.dist !== undefined) valDistance.innerText = lastPt.dist.toFixed(1);
            if (lastPt.h && Array.isArray(lastPt.h)) {
                valHistory.innerText = lastPt.h.join(", ");
            }

            history.forEach(pt => {
                const isRestricted = (pt.r === 1);
                pedalBuffer.push({ x: pt.t, y: pt.p, restricted: isRestricted });
                motorBuffer.push({ x: pt.t, y: pt.d, restricted: isRestricted });
                velocityBuffer.push({ x: pt.t, y: pt.v, restricted: isRestricted });
                distanceBuffer.push({ x: pt.t, y: pt.dist, restricted: isRestricted });
            });
        }
    }
};

function startRenderLoop() {
    function render() {
        const now = Date.now();
        const limitTime = now - (MAX_STORE_MINUTES * 60 * 1000);
        function cleanBuffer(buffer) {
            let removeCount = 0;
            while(buffer.length > 0 && buffer[0].x < limitTime && removeCount < 50) {
                buffer.shift(); removeCount++;
            }
        }
        cleanBuffer(pedalBuffer); cleanBuffer(motorBuffer);
        cleanBuffer(velocityBuffer); cleanBuffer(distanceBuffer);
        updateChartScale(); 
        requestAnimationFrame(render);
    }
    requestAnimationFrame(render);
}

function updateChartScale() {
    const now = Date.now();
    const minTime = now - (viewSeconds * 1000);
    [pedalChart, motorChart, velocityChart, distanceChart].forEach(chart => {
        chart.options.scales.x.min = minTime; chart.options.scales.x.max = now;
        chart.update('none');
    });
}

initChartData();
startRenderLoop();