const wsUrl = "ws://" + window.location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const pedalText = document.getElementById("pedal-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");

// --- 차트 설정 ---
const ctx = document.getElementById('powerChart').getContext('2d');

// [핵심 1] 대량의 데이터를 버틸 수 있는 버퍼 설정
let dataBuffer = [];
const MAX_STORE_MINUTES = 5; 
let viewSeconds = 60; // 기본 1분

const powerChart = new Chart(ctx, {
    type: 'line',
    data: {
        datasets: [{
            label: 'Motor Speed',
            data: dataBuffer, // 데이터 배열을 직접 참조하게 함 (성능 향상)
            borderWidth: 1.5, // 선을 얇게 유지
            fill: true,
            backgroundColor: 'rgba(57, 255, 20, 0.1)',
            
            // [핵심 2] 급격한 변화(각속도)를 뭉개지 않고 뾰족하게 표현
            tension: 0, 
            
            // 점을 그리는 연산을 생략하여 성능 확보 (선만 그림)
            pointRadius: 0, 
            pointHoverRadius: 0,
            
            // 데이터가 끊기지 않게 연결
            spanGaps: true,

            // [핵심 3] 성능 최적화: 픽셀 단위 미세 조정 끄기
            borderJoinStyle: 'round',
            borderCapStyle: 'round',

            segment: {
                borderColor: ctx => {
                    // 제한 조건(각속도 등) 발동 시 빨간색
                    if (ctx.p1.raw && ctx.p1.raw.restricted) return '#ff0000';
                    return '#39ff14';
                }
            }
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        
        // [핵심 4] 애니메이션 완전 제거 (실시간성 극대화)
        animation: false, 
        transitions: {
            active: {
                animation: { duration: 0 }
            }
        },
        
        // 마우스 호버 이벤트 등을 꺼서 그리기 부하 줄임
        events: [], 
        
        interaction: { intersect: false },
        
        scales: {
            x: {
                type: 'linear', 
                display: true,
                position: 'bottom',
                ticks: {
                    color: '#666',
                    maxTicksLimit: 6,
                    autoSkip: true, // 라벨 겹치면 자동 생략
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
        plugins: { 
            legend: { display: false },
            tooltip: { enabled: false }, // 툴팁 끔 (성능 최적화)
            decimation: {
                enabled: false // [중요] 데이터 압축 끄기 (모든 스파이크를 보기 위해)
            }
        }
    }
});

// 초기 데이터 채우기 (그래프 모양 잡기용)
function initChartData() {
    const now = Date.now();
    // 0.05초(50ms) 간격으로 촘촘하게 채움
    for (let i = 2000; i > 0; i--) {
        const pastTime = now - (i * 50); 
        dataBuffer.push({ x: pastTime, y: 0, restricted: false });
    }
}

// 시간 모드 설정
window.setTimeMode = function(seconds) {
    viewSeconds = seconds;
    document.querySelectorAll('.time-btn').forEach(btn => btn.classList.remove('active'));
    if (seconds === 10) document.getElementById('btn-10sec').classList.add('active');
    else if (seconds === 30) document.getElementById('btn-30sec').classList.add('active');
    else if (seconds === 60) document.getElementById('btn-1min').classList.add('active');
    
    // 버튼 누르면 즉시 한번 갱신
    updateChartScale(); 
};

// WebSocket 메시지 처리
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    const isRestricted = !!data.reason; 

    if (data.duty !== undefined) {
        const duty = parseFloat(data.duty);
        
        // 게이지는 바로바로 움직여야 하므로 즉시 반영 (CSS transform은 가벼움)
        valText.innerText = duty.toFixed(0);
        let angle = (duty * 1.8) - 90;
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;
        needle.style.transform = `rotate(${angle}deg)`;

        // [핵심 5] 차트 데이터는 배열에 넣기만 하고(Push), 여기서 차트를 그리지 않음(Render X)
        const now = Date.now();
        dataBuffer.push({ x: now, y: duty, restricted: isRestricted });
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

// [핵심 6] 렌더링 루프 (Render Loop)
// 데이터 수신과 무관하게 화면 주사율(60fps)에 맞춰서 차트를 갱신
function startRenderLoop() {
    
    function render() {
        const now = Date.now();

        // 1. 오래된 데이터 정리 (버퍼 관리)
        // 매 프레임마다 하면 비효율적일 수 있으나, 배열이 아주 길어지는 것을 방지
        const limitTime = now - (MAX_STORE_MINUTES * 60 * 1000);
        
        // 성능을 위해 한 번에 최대 50개씩만 지움 (렉 방지)
        let removeCount = 0;
        while(dataBuffer.length > 0 && dataBuffer[0].x < limitTime && removeCount < 50) {
            dataBuffer.shift();
            removeCount++;
        }

        // 2. X축 범위 갱신 (스크롤 효과)
        powerChart.options.scales.x.min = now - (viewSeconds * 1000);
        powerChart.options.scales.x.max = now;

        // 3. 차트 그리기
        powerChart.update('none'); // 'none' 모드가 가장 빠름

        // 다음 프레임 요청
        requestAnimationFrame(render);
    }
    
    requestAnimationFrame(render);
}

// 초기화 실행
initChartData();
startRenderLoop(); // 렌더링 시작

// 창 크기 변경 시 스케일 즉시 조정용
function updateChartScale() {
    const now = Date.now();
    powerChart.options.scales.x.min = now - (viewSeconds * 1000);
    powerChart.options.scales.x.max = now;
    powerChart.update('none');
}