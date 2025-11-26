// script.js

const wsUrl = "ws://" + window.location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const pedalText = document.getElementById("pedal-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");

// --- 차트 관련 변수 및 설정 ---
const ctx = document.getElementById('powerChart').getContext('2d');

// [핵심] 데이터 관리 변수
let dataBuffer = [];          // 최대 10분치 데이터를 모두 저장하는 버퍼
const MAX_STORE_MINUTES = 10; // 최대 저장 시간 (10분)
let viewMinutes = 1;          // 현재 보고 있는 시간 간격 (기본 1분)

const powerChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Motor Speed',
            data: [],
            borderWidth: 2, // 선 두께 살짝 얇게 조정
            fill: true,
            backgroundColor: 'rgba(57, 255, 20, 0.1)',
            tension: 0.4,
            pointRadius: 0,
            
            // 구간별 색상 변경 (기존 로직 유지)
            segment: {
                borderColor: ctx => {
                    if (ctx.p1.raw && ctx.p1.raw.restricted) {
                        return '#ff0000'; // 제한됨: 빨강
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
                display: true, // X축 시간 표시 (간격 확인용)
                ticks: {
                    maxTicksLimit: 6, // 라벨이 너무 많이 나오지 않게 제한
                    color: '#666',
                    font: { size: 10 }
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

// [추가] 시간 모드 변경 함수 (버튼 클릭 시 실행)
window.setTimeMode = function(minutes) {
    viewMinutes = minutes;

    // 버튼 스타일 업데이트
    document.getElementById('btn-1min').classList.remove('active');
    document.getElementById('btn-10min').classList.remove('active');
    
    if (minutes === 1) {
        document.getElementById('btn-1min').classList.add('active');
    } else {
        document.getElementById('btn-10min').classList.add('active');
    }

    // 모드가 바뀌면 즉시 차트 갱신 (데이터가 쌓여있다면 바로 보임)
    updateChartDisplay();
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    const isRestricted = !!data.reason; 

    // 1. 게이지 및 텍스트 업데이트
    if (data.duty !== undefined) {
        const duty = parseFloat(data.duty);
        valText.innerText = duty.toFixed(0);
        
        let angle = (duty * 1.8) - 90;
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;
        needle.style.transform = `rotate(${angle}deg)`;

        // 2. 차트 데이터 버퍼에 추가
        addDataToBuffer(duty, isRestricted);
    }

    // 3. 페달 값
    if (data.pedal !== undefined && pedalText) {
        pedalText.innerText = data.pedal;
    }

    // 4. 경고창 처리
    if (data.reason) {
        warningBox.style.display = "block";
        warningText.innerText = data.reason; 
        if (data.remaining_time !== undefined && data.remaining_time === 0) {
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

// [추가] 데이터를 버퍼에 저장하는 함수
function addDataToBuffer(duty, isRestricted) {
    const now = new Date(); // 현재 시간 객체
    const timeLabel = now.toLocaleTimeString('ko-KR', { hour12: false }); // "14:30:05" 형식
    const timestamp = now.getTime(); // 밀리초 타임스탬프 (계산용)

    // 버퍼에 새 데이터 추가
    dataBuffer.push({
        x: timeLabel,       // 차트 라벨용
        y: duty,            // 값
        timestamp: timestamp, // 시간 필터링용
        restricted: isRestricted // 색상용 상태
    });

    // 최대 저장 시간(10분)보다 오래된 데이터는 버퍼에서 삭제
    // 10분 = 600초 * 1000ms
    const limitTime = timestamp - (MAX_STORE_MINUTES * 60 * 1000);
    
    // 배열 앞부분(오래된 데이터) 제거 로직
    // 성능을 위해 while로 체크
    while(dataBuffer.length > 0 && dataBuffer[0].timestamp < limitTime) {
        dataBuffer.shift();
    }

    // 화면 갱신
    updateChartDisplay();
}

// [추가] 현재 모드에 맞춰 차트에 데이터를 반영하는 함수
function updateChartDisplay() {
    const now = Date.now();
    const rangeMs = viewMinutes * 60 * 1000; // 1분 또는 10분을 ms로 변환
    const cutOffTime = now - rangeMs;

    // 현재 설정된 시간 범위 안에 있는 데이터만 필터링
    // filter를 사용하면 원본 dataBuffer는 유지되면서 보여줄 것만 뽑음
    const visibleData = dataBuffer.filter(d => d.timestamp > cutOffTime);

    // 차트에 데이터 주입
    powerChart.data.labels = visibleData.map(d => d.x);
    powerChart.data.datasets[0].data = visibleData;
    
    // 차트 업데이트 ('none' 모드로 성능 최적화)
    powerChart.update('none');
}