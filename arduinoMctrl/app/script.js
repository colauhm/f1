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
const MAX_STORE_MINUTES = 5; 
let viewSeconds = 60; 

const powerChart = new Chart(ctx, {
    type: 'line',
    data: {
        datasets: [{
            label: 'Motor Speed',
            data: [],
            
            // [수정 1] 선을 얇게 하여 조밀한 데이터를 잘 보이게 함
            borderWidth: 1.5, 
            
            fill: true,
            backgroundColor: 'rgba(57, 255, 20, 0.1)',
            
            // [수정 2] 핵심! 곡선(0.4)을 끄고 직선(0)으로 설정
            // 이렇게 해야 순간적으로 튀는 값(노이즈, 스파이크)이 뭉개지지 않고 그대로 보임
            tension: 0, 
            
            pointRadius: 0, // 점은 숨김 (데이터가 많으면 점 때문에 선이 안 보임)
            
            // [추가] 선이 끊기지 않고 이어지도록 설정 (데이터 누락 대비)
            spanGaps: true,

            segment: {
                borderColor: ctx => {
                    // 제한 상태면 빨강, 아니면 형광
                    if (ctx.p1.raw && ctx.p1.raw.restricted) return '#ff0000';
                    return '#39ff14';
                }
            }
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false, // 성능 최적화: 애니메이션 끔
        interaction: { intersect: false },
        
        // [추가] 성능 최적화 옵션 (데이터가 많을 때 버벅임 방지)
        elements: {
            line: {
                borderJoinStyle: 'round'
            }
        },
        
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
            y: { 
                min: 0, 
                max: 100, 
                grid: { color: '#333' }, 
                ticks: { color: '#888' } 
            }
        },
        plugins: { legend: { display: false } }
    }
});

// [수정 3] 초기화 데이터도 더 촘촘하게 (0.05초 간격)
function initChartData() {
    const now = Date.now();
    // 5분치 데이터를 아주 촘촘하게(50ms 간격) 채워서 시작
    // 데이터가 조밀해도 차트가 버티는지 확인 가능
    const totalPoints = (MAX_STORE_MINUTES * 60 * 1000) / 50; // 약 6000개 포인트
    
    for (let i = totalPoints; i > 0; i--) {
        const pastTime = now - (i * 50); 
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