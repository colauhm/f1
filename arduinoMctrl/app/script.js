const wsUrl = "ws://" + window.location.host + "/ws";
const ws = new WebSocket(wsUrl);

const needle = document.getElementById("needle");
const valText = document.getElementById("duty-val");
const pedalText = document.getElementById("pedal-val");
const warningBox = document.getElementById("warning-dialog");
const warningText = document.getElementById("warning-text");

// --- [추가] 차트 초기화 ---
const ctx = document.getElementById('powerChart').getContext('2d');
const MAX_DATA_POINTS = 50; // 그래프에 보여줄 최대 데이터 개수

const powerChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [], // 시간 축 (생략 가능하지만 배열 구조 유지)
        datasets: [{
            label: 'Motor Speed',
            data: [],
            borderWidth: 3,
            fill: true,
            backgroundColor: 'rgba(57, 255, 20, 0.1)', // 형광색 배경 (투명도)
            tension: 0.4, // 부드러운 곡선
            pointRadius: 0, // 점 숨기기 (선만 보임)
            
            // [핵심] 구간별 색상 변경 로직 (Segment Styling)
            segment: {
                borderColor: ctx => {
                    // 현재 구간의 끝지점 데이터가 'restricted' 상태면 빨간색
                    if (ctx.p1.raw && ctx.p1.raw.restricted) {
                        return '#ff0000'; // 제한됨: 빨강
                    }
                    return '#39ff14'; // 정상: 형광 연두 (Neon Green)
                }
            }
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false, // 실시간 성능 위해 애니메이션 끔
        interaction: {
            intersect: false
        },
        scales: {
            x: {
                display: false, // X축(시간) 라벨 숨김 (깔끔하게)
            },
            y: {
                min: 0,
                max: 100, // 모터 속도 0~100%
                grid: {
                    color: '#333'
                },
                ticks: {
                    color: '#888'
                }
            }
        },
        plugins: {
            legend: { display: false } // 범례 숨김
        }
    }
});

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    // 현재 제한 상태인지 확인 (reason이 있으면 제한 상태)
    const isRestricted = !!data.reason; 

    // 1. 모터 듀티 게이지
    if (data.duty !== undefined) {
        const duty = parseFloat(data.duty);
        valText.innerText = duty.toFixed(0);
        
        let angle = (duty * 1.8) - 90;
        if (angle < -90) angle = -90;
        if (angle > 90) angle = 90;
        needle.style.transform = `rotate(${angle}deg)`;

        // [추가] 차트 데이터 업데이트
        addDataToChart(duty, isRestricted);
    }

    // 2. 페달 값
    if (data.pedal !== undefined && pedalText) {
        pedalText.innerText = data.pedal;
    }

    // 3. 경고 메시지 처리
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

// [추가] 차트 데이터 추가 함수
function addDataToChart(duty, isRestricted) {
    const now = new Date().toLocaleTimeString();
    
    // 데이터셋에 객체 형태로 저장 {x: 시간, y: 값, restricted: 상태}
    // restricted 속성을 segment 스타일링에서 참조함
    powerChart.data.labels.push(now);
    powerChart.data.datasets[0].data.push({
        x: now,
        y: duty,
        restricted: isRestricted 
    });

    // 데이터 개수 제한 (오래된 데이터 삭제)
    if (powerChart.data.labels.length > MAX_DATA_POINTS) {
        powerChart.data.labels.shift();
        powerChart.data.datasets[0].data.shift();
    }

    powerChart.update('none'); // 'none' 모드로 업데이트 (성능 최적화)
}