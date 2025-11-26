const wsUrl = "ws://" + window.location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const pedalText = document.getElementById("pedal-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");

// 버퍼 2개 생성
let pedalBuffer = [];
let motorBuffer = [];
const MAX_STORE_MINUTES = 5; 
let viewSeconds = 60; 

// 공통 차트 옵션 생성 함수
function createChartConfig(buffer, label) {
    return {
        type: 'line',
        data: {
            datasets: [{
                label: label,
                data: buffer,
                borderWidth: 1.5,
                fill: true,
                backgroundColor: 'rgba(57, 255, 20, 0.1)',
                tension: 0, 
                pointRadius: 0, 
                spanGaps: true,
                borderJoinStyle: 'round',
                segment: {
                    borderColor: ctx => {
                        if (ctx.p1.raw && ctx.p1.raw.restricted) return '#ff0000';
                        return '#39ff14';
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
                        maxTicksLimit: 6,
                        callback: val => new Date(val).toLocaleTimeString('ko-KR', { hour12: false })
                    },
                    grid: { display: false }
                },
                y: { 
                    min: 0, max: 100, 
                    grid: { color: '#333' }, 
                    ticks: { color: '#888' } 
                }
            },
            plugins: { 
                legend: { display: false },
                tooltip: { enabled: false },
                decimation: { enabled: false }
            }
        }
    };
}

// 차트 2개 생성
const ctxPedal = document.getElementById('pedalChart').getContext('2d');
const pedalChart = new Chart(ctxPedal, createChartConfig(pedalBuffer, 'Pedal'));

const ctxMotor = document.getElementById('motorChart').getContext('2d');
const motorChart = new Chart(ctxMotor, createChartConfig(motorBuffer, 'Motor'));

// 초기 데이터 채우기 (두 버퍼 모두)
function initChartData() {
    const now = Date.now();
    for (let i = 2000; i > 0; i--) {
        const pt = { x: now - (i * 50), y: 0, restricted: false };
        pedalBuffer.push(pt);
        motorBuffer.push(pt);
    }
}

// 시간 모드 변경 (버튼 하나로 두 차트 제어)
window.setTimeMode = function(seconds) {
    viewSeconds = seconds;
    document.querySelectorAll('.time-btn').forEach(btn => btn.classList.remove('active'));
    
    if (seconds === 10) document.getElementById('btn-10sec').classList.add('active');
    else if (seconds === 30) document.getElementById('btn-30sec').classList.add('active');
    else if (seconds === 60) document.getElementById('btn-1min').classList.add('active');
    
    // 즉시 갱신
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

        // 3. [핵심] 배치 데이터 분리하여 각각 저장
        if (history && history.length > 0) {
            history.forEach(pt => {
                const isRestricted = (pt.r === 1);
                
                // 페달 버퍼 (pt.p)
                pedalBuffer.push({
                    x: pt.t,
                    y: pt.p,
                    restricted: isRestricted
                });

                // 모터 버퍼 (pt.d)
                motorBuffer.push({
                    x: pt.t,
                    y: pt.d,
                    restricted: isRestricted
                });
            });
        }
    }
};

// 렌더링 루프 (두 차트 모두 갱신)
function startRenderLoop() {
    function render() {
        const now = Date.now();
        const limitTime = now - (MAX_STORE_MINUTES * 60 * 1000);
        let removeCount = 0;

        // 오래된 데이터 정리 (페달)
        while(pedalBuffer.length > 0 && pedalBuffer[0].x < limitTime && removeCount < 50) {
            pedalBuffer.shift();
            removeCount++;
        }
        
        // 오래된 데이터 정리 (모터) - 개수가 거의 같겠지만 따로 관리
        removeCount = 0;
        while(motorBuffer.length > 0 && motorBuffer[0].x < limitTime && removeCount < 50) {
            motorBuffer.shift();
            removeCount++;
        }

        updateChartScale(); 
        requestAnimationFrame(render);
    }
    requestAnimationFrame(render);
}

function updateChartScale() {
    const now = Date.now();
    const minTime = now - (viewSeconds * 1000);

    // 페달 차트 갱신
    pedalChart.options.scales.x.min = minTime;
    pedalChart.options.scales.x.max = now;
    pedalChart.update('none');

    // 모터 차트 갱신
    motorChart.options.scales.x.min = minTime;
    motorChart.options.scales.x.max = now;
    motorChart.update('none');
}

// 시작
initChartData();
startRenderLoop();