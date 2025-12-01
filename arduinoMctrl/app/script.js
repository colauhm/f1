// 현재 페이지가 https라면 wss를, 아니면 ws를 쓰도록 자동 설정
var protocol = (location.protocol === 'https:') ? 'wss://' : 'ws://';
var wsAddress = protocol + location.host + "/ws";

const ws = new WebSocket(wsAddress);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");
const valVelocity = document.getElementById("val-velocity");
const valDistance = document.getElementById("val-distance");
const valCount = document.getElementById("val-count");

const THRESHOLD_VELOCITY = 420; 
const THRESHOLD_DIST_MIN = 100; 
const THRESHOLD_COUNT = 3;      
const COLOR_GREEN = "#39ff14";
const COLOR_RED = "#ff0000";

let pedalBuffer = [], motorBuffer = [], velocityBuffer = [], distanceBuffer = [];
const MAX_STORE_MINUTES = 5; 
let viewSeconds = 60; 

// [추가] 인포메이션 아이콘 마우스 이벤트 로직
document.addEventListener("DOMContentLoaded", () => {
    const icons = document.querySelectorAll(".info-icon");
    const grid = document.getElementById("chartGrid");

    icons.forEach(icon => {
        // 마우스 올렸을 때
        icon.addEventListener("mouseenter", (e) => {
            const wrapper = e.target.closest(".chart-wrapper");
            
            // 1. 그리드 전체에 'dimming' 클래스 추가
            grid.classList.add("dimming");
            
            // 2. 해당 그래프 박스에 'active' 클래스 추가
            wrapper.classList.add("active");
        });

        // 마우스 뗐을 때
        icon.addEventListener("mouseleave", (e) => {
            const wrapper = e.target.closest(".chart-wrapper");
            
            // 클래스 제거 (원상복구)
            grid.classList.remove("dimming");
            wrapper.classList.remove("active");
        });
    });
});

// 차트 설정
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

        // 1. 게이지
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
            warningBox.style.backgroundColor = (current.remaining_time === 0) 
                ? "rgba(255, 0, 0, 1)" : "rgba(255, 0, 0, 0.9)";
        } else {
            warningBox.style.display = "none";
        }

        // 3. 데이터 업데이트
        if (history && history.length > 0) {
            let lastPt = history[history.length - 1];

            // 정보창 업데이트 (색상 로직 포함)
            const v = lastPt.v || 0;
            valVelocity.innerText = v.toFixed(1);
            valVelocity.style.color = (Math.abs(v) >= THRESHOLD_VELOCITY) ? COLOR_RED : COLOR_GREEN;

            const d = lastPt.dist || 0;
            valDistance.innerText = d.toFixed(1);
            if (d > 0 && d <= THRESHOLD_DIST_MIN) valDistance.style.color = COLOR_RED;
            else valDistance.style.color = COLOR_GREEN;

            const count = lastPt.pc || 0;
            valCount.innerText = count;
            valCount.style.color = (count >= THRESHOLD_COUNT) ? COLOR_RED : COLOR_GREEN;

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