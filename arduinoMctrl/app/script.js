// 현재 페이지가 https라면 wss를, 아니면 ws를 쓰도록 자동 설정
var protocol = (location.protocol === 'https:') ? 'wss://' : 'ws://';
var wsAddress = protocol + location.host + "/ws";

const ws = new WebSocket(wsAddress);

// [수정됨] HTML 요소 가져오기 (기존 duty-val 제거됨)
const needle = document.getElementById("needle");
const rpmText = document.getElementById("rpm-val");    // RPM 표시용
const speedText = document.getElementById("speed-val");  // 속도(km/h) 표시용

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

// [수정됨] RPM 및 속도 계산용 상수
const MAX_RPM = 1350;        // 모터 최대 RPM
const TIRE_CIRCUM = 2.0;     // 타이어 둘레 (미터)

let pedalBuffer = [], motorBuffer = [], velocityBuffer = [], distanceBuffer = [];

// [수정] 저장 시간을 1.5분으로 제한 (렉 방지)
const MAX_STORE_MINUTES = 1.5; 
// [추가] 데이터 개수 강제 제한 (안전장치: 최대 2500개까지만 저장)
const MAX_DATA_POINTS = 2500;

let viewSeconds = 60; 

// 마우스 이벤트 로직
document.addEventListener("DOMContentLoaded", () => {
    const icons = document.querySelectorAll(".info-icon");
    const chartGrid = document.getElementById("chartGrid");

    icons.forEach(icon => {
        icon.addEventListener("mouseenter", (e) => {
            const wrapper = e.target.closest(".chart-wrapper");
            if(wrapper) {
                wrapper.classList.add("active");
                if (wrapper.parentElement.id === "chartGrid") {
                    chartGrid.classList.add("dimming");
                }
            }
        });

        icon.addEventListener("mouseleave", (e) => {
            const wrapper = e.target.closest(".chart-wrapper");
            if(wrapper) {
                wrapper.classList.remove("active");
                if (wrapper.parentElement.id === "chartGrid") {
                    chartGrid.classList.remove("dimming");
                }
            }
        });
    });
});

// 차트 설정 함수
function createChartConfig(buffer, label, color = '#39ff14') {
    return {
        type: 'line',
        data: {
            datasets: [{
                label: label, data: buffer, borderWidth: 1.5, fill: true,
                backgroundColor: color.startsWith('#') ? color + '1A' : color, borderColor: color,
                tension: 0.4, pointRadius: 0, spanGaps: true,
                animation: false, 
                segment: { borderColor: ctx => (ctx.p1.raw && ctx.p1.raw.restricted) ? '#ff0000' : color }
            }]
        },
        options: {
            responsive: true, maintainAspectRatio: false, 
            animation: false, 
            parsing: false,
            interaction: { intersect: false },
            scales: {
                x: { 
                    type: 'linear', display: true, 
                    ticks: { color: '#666', maxTicksLimit: 5, callback: val => new Date(val).toLocaleTimeString('ko-KR', { hour12: false }) }, 
                    grid: { display: false } 
                },
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
    for (let i = 100; i > 0; i--) { 
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

// [핵심] WebSocket 데이터 수신부
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type === "batch") {
        const current = data.current;
        const history = data.history;

        // 1. 게이지 및 텍스트 업데이트 (수정된 로직)
        if (current.duty !== undefined) {
            const duty = parseFloat(current.duty);
            
            // 바늘 각도 (기존 로직 유지)
            let angle = (duty * 1.8) - 90;
            if (angle < -90) angle = -90; if (angle > 90) angle = 90;
            needle.style.transform = `rotate(${angle}deg)`;

            // RPM 및 속도 계산
            // RPM = (현재 듀티 / 100) * 최대 RPM
            const currentRpm = (duty / 100.0) * MAX_RPM;
            
            // Speed = RPM * 둘레(m) * 60분 / 1000m(km환산)
            const currentSpeed = currentRpm * TIRE_CIRCUM * 0.06;

            // 화면 표시 (RPM은 정수, 속도는 반올림 정수)
            rpmText.innerText = Math.round(currentRpm);
            speedText.innerText = Math.round(currentSpeed);
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

        // 3. 데이터 업데이트 (기존 동일)
        if (history && history.length > 0) {
            let lastPt = history[history.length - 1];

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
                const point = { x: pt.t, y: pt.p, restricted: isRestricted };
                const pointM = { x: pt.t, y: pt.d, restricted: isRestricted };
                const pointV = { x: pt.t, y: pt.v, restricted: isRestricted };
                const pointD = { x: pt.t, y: pt.dist, restricted: isRestricted };
                
                pedalBuffer.push(point);
                motorBuffer.push(pointM);
                velocityBuffer.push(pointV);
                distanceBuffer.push(pointD);
            });
            
            if (pedalBuffer.length > MAX_DATA_POINTS) {
                const excess = pedalBuffer.length - MAX_DATA_POINTS;
                pedalBuffer.splice(0, excess);
                motorBuffer.splice(0, excess);
                velocityBuffer.splice(0, excess);
                distanceBuffer.splice(0, excess);
            }
        }
    }
};

function startRenderLoop() {
    function render() {
        const now = Date.now();
        const limitTime = now - (MAX_STORE_MINUTES * 60 * 1000);
        
        if (pedalBuffer.length > 0 && pedalBuffer[0].x < limitTime) {
             let removeCount = 0;
             for(let i=0; i<pedalBuffer.length; i++) {
                 if(pedalBuffer[i].x < limitTime) removeCount++;
                 else break;
             }
             if (removeCount > 0) {
                 pedalBuffer.splice(0, removeCount);
                 motorBuffer.splice(0, removeCount);
                 velocityBuffer.splice(0, removeCount);
                 distanceBuffer.splice(0, removeCount);
             }
        }

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