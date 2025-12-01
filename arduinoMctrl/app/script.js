const wsUrl = "ws://" + window.location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const pedalText = document.getElementById("pedal-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");

// [추가] 거리 확인용 엘리먼트
const distValText = document.getElementById("dist-val");
const distStatusText = document.getElementById("dist-status");

// 버퍼 4개
let pedalBuffer = [];
let motorBuffer = [];
let velocityBuffer = [];
let distanceBuffer = [];

const MAX_STORE_MINUTES = 5; 
let viewSeconds = 60; 

// 차트 옵션 생성
function createChartConfig(buffer, label, color = '#39ff14') {
    return {
        type: 'line',
        data: {
            datasets: [{
                label: label,
                data: buffer,
                borderWidth: 1.5,
                fill: true,
                backgroundColor: color.startsWith('#') ? color + '1A' : color, 
                borderColor: color,
                tension: 0, 
                pointRadius: 0, 
                spanGaps: true,
                borderJoinStyle: 'round',
                segment: {
                    borderColor: ctx => {
                        if (ctx.p1.raw && ctx.p1.raw.restricted) return '#ff0000';
                        return color;
                    }
                }
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            interaction: { intersect: false },
            events: [],
            scales: {
                x: {
                    type: 'linear', 
                    display: true, 
                    position: 'bottom',
                    ticks: {
                        color: '#666',
                        maxTicksLimit: 5, // 2x2로 작아졌으니 틱 개수 감소
                        callback: val => new Date(val).toLocaleTimeString('ko-KR', { hour12: false })
                    },
                    grid: { display: false }
                },
                y: { 
                    grid: { color: '#333' }, 
                    ticks: { color: '#888' } 
                }
            },
            plugins: { 
                legend: { display: false },
                tooltip: { enabled: false }
            }
        }
    };
}

// ---- 차트 생성 ----
const ctxPedal = document.getElementById('pedalChart').getContext('2d');
const configPedal = createChartConfig(pedalBuffer, 'Pedal', '#39ff14');
configPedal.options.scales.y.min = 0;
configPedal.options.scales.y.max = 100;
const pedalChart = new Chart(ctxPedal, configPedal);

const ctxMotor = document.getElementById('motorChart').getContext('2d');
const configMotor = createChartConfig(motorBuffer, 'Motor', '#39ff14');
configMotor.options.scales.y.min = 0;
configMotor.options.scales.y.max = 100;
const motorChart = new Chart(ctxMotor, configMotor);

const ctxVelocity = document.getElementById('velocityChart').getContext('2d');
const configVelocity = createChartConfig(velocityBuffer, 'Velocity', '#ffff00');
const velocityChart = new Chart(ctxVelocity, configVelocity);

const ctxDistance = document.getElementById('distanceChart').getContext('2d');
const configDistance = createChartConfig(distanceBuffer, 'Distance', '#00d2ff');
configDistance.options.scales.y.min = 0;
const distanceChart = new Chart(ctxDistance, configDistance);

// 초기 데이터
function initChartData() {
    const now = Date.now();
    for (let i = 2000; i > 0; i--) {
        const pt = { x: now - (i * 50), y: 0, restricted: false };
        pedalBuffer.push(pt);
        motorBuffer.push(pt);
        velocityBuffer.push(pt);
        distanceBuffer.push(pt);
    }
}

// 시간 모드 변경
window.setTimeMode = function(seconds) {
    viewSeconds = seconds;
    document.querySelectorAll('.time-btn').forEach(btn => btn.classList.remove('active'));
    
    if (seconds === 10) document.getElementById('btn-10sec').classList.add('active');
    else if (seconds === 30) document.getElementById('btn-30sec').classList.add('active');
    else if (seconds === 60) document.getElementById('btn-1min').classList.add('active');
    
    requestAnimationFrame(updateChartScale);
};

// WebSocket 메시지 처리
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);

    if (data.type === "batch") {
        const current = data.current;
        const history = data.history;

        // 1. 게이지/텍스트 업데이트
        if (current.duty !== undefined) {
            const duty = parseFloat(current.duty);
            valText.innerText = duty.toFixed(0);
            
            let angle = (duty * 1.8) - 90;
            if (angle < -90) angle = -90;
            if (angle > 90) angle = 90;
            needle.style.transform = `rotate(${angle}deg)`;
        }
        if (current.pedal !== undefined && pedalText) pedalText.innerText = current.pedal;

        // 2. 경고창
        if (current.reason) {
            warningBox.style.display = "block";
            warningText.innerText = current.reason;
            warningBox.style.backgroundColor = (current.remaining_time === 0) 
                ? "rgba(255, 0, 0, 1)" : "rgba(255, 0, 0, 0.9)";
        } else {
            warningBox.style.display = "none";
        }

        // 3. 버퍼 저장
        let lastDist = 0;
        if (history && history.length > 0) {
            history.forEach(pt => {
                const isRestricted = (pt.r === 1);
                pedalBuffer.push({ x: pt.t, y: pt.p, restricted: isRestricted });
                motorBuffer.push({ x: pt.t, y: pt.d, restricted: isRestricted });
                velocityBuffer.push({ x: pt.t, y: pt.v, restricted: isRestricted });
                distanceBuffer.push({ x: pt.t, y: pt.dist, restricted: isRestricted });
                
                if(pt.dist !== undefined) lastDist = pt.dist;
            });
        }
        
        // 4. [추가] 거리 센서 값 UI 업데이트
        // 마지막 데이터 포인트의 거리를 표시
        if (distValText) {
            distValText.innerText = lastDist.toFixed(1);
            if (lastDist > 0 && lastDist < 300) {
                distStatusText.innerText = "● Operating Normally";
                distStatusText.style.color = "#00ff00";
            } else if (lastDist >= 300) {
                 distStatusText.innerText = "● Out of Range";
                 distStatusText.style.color = "#ffff00";
            } else {
                distStatusText.innerText = "○ No Signal";
                distStatusText.style.color = "#666";
            }
        }
    }
};

// 렌더링 루프
function startRenderLoop() {
    function render() {
        const now = Date.now();
        const limitTime = now - (MAX_STORE_MINUTES * 60 * 1000);
        
        function cleanBuffer(buffer) {
            let removeCount = 0;
            while(buffer.length > 0 && buffer[0].x < limitTime && removeCount < 50) {
                buffer.shift();
                removeCount++;
            }
        }

        cleanBuffer(pedalBuffer);
        cleanBuffer(motorBuffer);
        cleanBuffer(velocityBuffer);
        cleanBuffer(distanceBuffer);

        updateChartScale(); 
        requestAnimationFrame(render);
    }
    requestAnimationFrame(render);
}

function updateChartScale() {
    const now = Date.now();
    const minTime = now - (viewSeconds * 1000);

    const charts = [pedalChart, motorChart, velocityChart, distanceChart];
    
    charts.forEach(chart => {
        chart.options.scales.x.min = minTime;
        chart.options.scales.x.max = now;
        chart.update('none');
    });
}

initChartData();
startRenderLoop();