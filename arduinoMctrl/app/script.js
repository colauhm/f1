const wsUrl = "ws://" + window.location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const pedalText = document.getElementById("pedal-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");

// --- 차트 설정 ---
const ctx = document.getElementById('powerChart').getContext('2d');

let dataBuffer = [];
const MAX_STORE_MINUTES = 5; 
let viewSeconds = 60; // 기본 1분

const powerChart = new Chart(ctx, {
    type: 'line',
    data: {
        datasets: [{
            label: 'Pedal Value',
            data: dataBuffer, // 버퍼 직접 참조
            borderWidth: 1.5,
            fill: true,
            backgroundColor: 'rgba(57, 255, 20, 0.1)',
            
            // [중요] 스파이크 표현을 위해 곡선 끄기
            tension: 0, 
            pointRadius: 0, 
            spanGaps: true,
            borderJoinStyle: 'round',

            segment: {
                borderColor: ctx => {
                    // 제한 상태(restricted=true)면 빨간색
                    if (ctx.p1.raw && ctx.p1.raw.restricted) return '#ff0000';
                    return '#39ff14';
                }
            }
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false, // 애니메이션 꺼야 실시간성 확보
        interaction: { intersect: false },
        events: [], // 마우스 이벤트 끔 (성능 최적화)
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
            decimation: { enabled: false } // 데이터 압축 금지
        }
    }
});

// 초기 데이터 채우기
function initChartData() {
    const now = Date.now();
    for (let i = 2000; i > 0; i--) {
        dataBuffer.push({ x: now - (i * 50), y: 0, restricted: false });
    }
}

// 시간 모드 변경 (초 단위)
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

        // 1. 현재 상태 (게이지, 텍스트) 업데이트
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

        // 3. [핵심] 배치 데이터(History)를 차트 버퍼에 모두 추가
        if (history && history.length > 0) {
            history.forEach(pt => {
                dataBuffer.push({
                    x: pt.t,
                    y: pt.v,
                    restricted: pt.r === 1
                });
            });
        }
    }
};

// 렌더링 루프 (화면 그리기 전담)
function startRenderLoop() {
    function render() {
        const now = Date.now();

        // 오래된 데이터 정리 (성능 유지)
        const limitTime = now - (MAX_STORE_MINUTES * 60 * 1000);
        let removeCount = 0;
        // 한 번에 너무 많이 지우면 렉 걸리므로 50개씩만 처리
        while(dataBuffer.length > 0 && dataBuffer[0].x < limitTime && removeCount < 50) {
            dataBuffer.shift();
            removeCount++;
        }

        updateChartScale(); // 범위 업데이트 및 그리기
        requestAnimationFrame(render);
    }
    requestAnimationFrame(render);
}

function updateChartScale() {
    const now = Date.now();
    powerChart.options.scales.x.min = now - (viewSeconds * 1000);
    powerChart.options.scales.x.max = now;
    powerChart.update('none');
}

// 시작
initChartData();
startRenderLoop();