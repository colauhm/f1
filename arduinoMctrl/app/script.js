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
const MAX_STORE_MINUTES = 10; 
let viewMinutes = 1; // 기본 1분

const powerChart = new Chart(ctx, {
    type: 'line',
    data: {
        datasets: [{
            label: 'Motor Speed',
            data: [], // 여기에 {x: timestamp, y: value} 형태가 들어감
            borderWidth: 2,
            fill: true,
            backgroundColor: 'rgba(57, 255, 20, 0.1)',
            tension: 0.4,
            pointRadius: 0,
            
            segment: {
                borderColor: ctx => {
                    if (ctx.p1.raw && ctx.p1.raw.restricted) {
                        return '#ff0000'; // 제한: 빨강
                    }
                    return '#39ff14'; // 정상: 형광
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
                // [핵심 변경] X축을 선형(Linear) 타입으로 변경하여 시간을 숫자로 처리
                type: 'linear', 
                display: true,
                position: 'bottom',
                
                // 시간 형식으로 라벨 표시 (예: 14:30:00)
                ticks: {
                    color: '#666',
                    maxTicksLimit: 6,
                    callback: function(value) {
                        // timestamp를 시간 문자열로 변환
                        const date = new Date(value);
                        return date.toLocaleTimeString('ko-KR', { hour12: false });
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

// [초기화] 10분치 빈 데이터 채우기 (그래프가 꽉 차서 시작하도록)
function initChartData() {
    const now = Date.now();
    // 0.5초 간격으로 10분치 데이터 생성 (총 1200개)
    // 간격을 촘촘하게 해서 더 자연스럽게 만듦
    for (let i = 1200; i > 0; i--) {
        const pastTime = now - (i * 500); 
        dataBuffer.push({
            x: pastTime,        // X축: 타임스탬프 (숫자)
            y: 0,               // Y축: 0
            restricted: false
        });
    }
    updateChartDisplay();
}

// [시간 모드 변경] 0.5 = 30초, 1 = 1분, 10 = 10분
window.setTimeMode = function(minutes) {
    viewMinutes = minutes;

    // 버튼 스타일 초기화 및 설정
    document.querySelectorAll('.time-btn').forEach(btn => btn.classList.remove('active'));
    
    if (minutes === 0.5) document.getElementById('btn-30sec').classList.add('active');
    else if (minutes === 1) document.getElementById('btn-1min').classList.add('active');
    else if (minutes === 10) document.getElementById('btn-10min').classList.add('active');

    // 즉시 반영
    updateChartDisplay();
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    const isRestricted = !!data.reason; 

    if (data.duty !== undefined) {
        const duty = parseFloat(data.duty);
        
        // 게이지 업데이트
        valText.innerText = duty.toFixed(0);
        let angle = (duty * 1.8) - 90;
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;
        needle.style.transform = `rotate(${angle}deg)`;

        // 차트 데이터 추가
        addDataToBuffer(duty, isRestricted);
    }

    // (페달, 경고창 처리 로직은 기존과 동일하므로 생략 가능, 혹은 그대로 두셔도 됩니다)
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
    
    // 데이터 추가 (x는 현재 타임스탬프)
    dataBuffer.push({
        x: now,
        y: duty,
        restricted: isRestricted
    });

    // 10분(600,000ms)보다 오래된 데이터 삭제
    const limitTime = now - (MAX_STORE_MINUTES * 60 * 1000);
    // 성능 최적화: 앞에서부터 오래된 것만 쳐냄
    while(dataBuffer.length > 0 && dataBuffer[0].x < limitTime) {
        dataBuffer.shift();
    }

    updateChartDisplay();
}

function updateChartDisplay() {
    const now = Date.now();
    const rangeMs = viewMinutes * 60 * 1000; // 보여줄 시간 범위 (ms)

    // 차트의 X축 범위를 강제로 설정 (이게 핵심!)
    // 데이터 개수와 상관없이 정확히 'rangeMs' 만큼의 시간을 보여줌
    powerChart.options.scales.x.min = now - rangeMs;
    powerChart.options.scales.x.max = now;

    // 데이터셋 업데이트
    // 성능을 위해 범위 밖의 너무 먼 데이터는 차트에 넣지 않을 수도 있지만, 
    // Chart.js가 알아서 잘라주므로 전체를 넘겨도 무방합니다.
    powerChart.data.datasets[0].data = dataBuffer;
    
    powerChart.update('none');
}

// 실행
initChartData();