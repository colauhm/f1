// script.js

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
const MAX_STORE_MINUTES = 5; // 최대 5분치 데이터 보관 (필요시 늘려도 됨)
let viewSeconds = 60;        // [변경] 기본 보기 설정을 '1분(60초)'로 설정

const powerChart = new Chart(ctx, {
    type: 'line',
    data: {
        datasets: [{
            label: 'Motor Speed',
            data: [],
            borderWidth: 2,
            fill: true,
            backgroundColor: 'rgba(57, 255, 20, 0.1)',
            tension: 0.4,
            pointRadius: 0,
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
        scales: {
            x: {
                type: 'linear', 
                display: true,
                position: 'bottom',
                ticks: {
                    color: '#666',
                    maxTicksLimit: 6,
                    callback: function(value) {
                        return new Date(value).toLocaleTimeString('ko-KR', { hour12: false });
                    }
                },
                grid: { display: false }
            },
            y: { min: 0, max: 100, grid: { color: '#333' }, ticks: { color: '#888' } }
        },
        plugins: { legend: { display: false } }
    }
});

// [초기화] 5분치 0 데이터 채우기
function initChartData() {
    const now = Date.now();
    // 0.2초 간격으로 5분치 채움 (부드러운 스크롤 위해 촘촘하게)
    for (let i = 1500; i > 0; i--) {
        const pastTime = now - (i * 200); 
        dataBuffer.push({
            x: pastTime,
            y: 0,
            restricted: false
        });
    }
    updateChartDisplay();
}

// [핵심 변경] 초 단위(Seconds)로 모드 설정
window.setTimeMode = function(seconds) {
    viewSeconds = seconds;

    // 버튼 스타일 업데이트
    document.querySelectorAll('.time-btn').forEach(btn => btn.classList.remove('active'));
    
    if (seconds === 10) document.getElementById('btn-10sec').classList.add('active');
    else if (seconds === 30) document.getElementById('btn-30sec').classList.add('active');
    else if (seconds === 60) document.getElementById('btn-1min').classList.add('active');

    updateChartDisplay();
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    const isRestricted = !!data.reason; 

    if (data.duty !== undefined) {
        const duty = parseFloat(data.duty);
        valText.innerText = duty.toFixed(0);
        
        let angle = (duty * 1.8) - 90;
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;
        needle.style.transform = `rotate(${angle}deg)`;

        addDataToBuffer(duty, isRestricted);
    }

    if (data.pedal !== undefined && pedalText) pedalText.innerText = data.pedal;
    if (data.reason) {
        warningBox.style.display = "block";
        warningText.innerText = data.reason;
        if (data.remaining_time === 0) {
            warningBox.style.backgroundColor = "rgba(255, 0, 0, 1)";
            warningBox.style.border = "3px solid #ffffff";
        } else {
            warningBox.style.backgroundColor = "rgba(255, 0, 0, 0.9)";
            warningBox.style.border = "3px solid #ff5555";
        }
    } else {
        warningBox.style.display = "none";
    }
};

function addDataToBuffer(duty, isRestricted) {
    const now = Date.now();
    dataBuffer.push({ x: now, y: duty, restricted: isRestricted });

    // 버퍼 관리 (너무 오래된 데이터 삭제)
    const limitTime = now - (MAX_STORE_MINUTES * 60 * 1000);
    while(dataBuffer.length > 0 && dataBuffer[0].x < limitTime) {
        dataBuffer.shift();
    }
    updateChartDisplay();
}

function updateChartDisplay() {
    const now = Date.now();
    // [변경] 초 단위 변수를 사용하여 범위 계산
    const rangeMs = viewSeconds * 1000; 

    powerChart.options.scales.x.min = now - rangeMs;
    powerChart.options.scales.x.max = now;

    powerChart.data.datasets[0].data = dataBuffer;
    powerChart.update('none');
}

initChartData();