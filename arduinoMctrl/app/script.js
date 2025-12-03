var protocol = (location.protocol === 'https:') ? 'wss://' : 'ws://';
var wsAddress = protocol + location.host + "/ws";

const ws = new WebSocket(wsAddress);

const needle = document.getElementById("needle");
const rpmText = document.getElementById("rpm-val");    
const speedText = document.getElementById("speed-val");  
const gearText = document.getElementById("gear-box"); 

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

const TIRE_CIRCUM = 2.0; 

let pedalBuffer = [], motorBuffer = [], velocityBuffer = [], distanceBuffer = [];
const MAX_STORE_MINUTES = 1.5; 
const MAX_DATA_POINTS = 2500;
let viewSeconds = 60; 

document.addEventListener("DOMContentLoaded", () => {
    const icons = document.querySelectorAll(".info-icon");
    const chartGrid = document.getElementById("chartGrid");

    icons.forEach(icon => {
        icon.addEventListener("mouseenter", (e) => {
            const wrapper = e.target.closest(".chart-wrapper");
            if(wrapper) {
                wrapper.classList.add("active");
                if (wrapper.parentElement.id === "chartGrid") chartGrid.classList.add("dimming");
            }
        });

        icon.addEventListener("mouseleave", (e) => {
            const wrapper = e.target.closest(".chart-wrapper");
            if(wrapper) {
                wrapper.classList.remove("active");
                if (wrapper.parentElement.id === "chartGrid") chartGrid.classList.remove("dimming");
            }
        });
    });
});

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
            responsive: true, maintainAspectRatio: false, animation: false, parsing: false,
            interaction: { intersect: false },
            scales: {
                x: { type: 'linear', display: true, ticks: { color: '#666', maxTicksLimit: 5, callback: val => new Date(val).toLocaleTimeString('ko-KR', { hour12: false }) }, grid: { display: false } },
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

// [함수 추가] 기어 스트립 불 켜기
function updateGearStrip(char) {
    // 1. 모든 불 끄기
    document.querySelectorAll('.gear-item').forEach(el => el.classList.remove('active'));
    
    // 2. 해당 문자 불 켜기
    const targetId = 'g-' + char.toLowerCase();
    const targetEl = document.getElementById(targetId);
    if(targetEl) targetEl.classList.add('active');
}

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type === "batch") {
        const current = data.current;
        const history = data.history;

        if (current.duty !== undefined) {
            const duty = parseFloat(current.duty);
            
            let angle = (duty * 1.8) - 90;
            if (angle < -90) angle = -90; if (angle > 90) angle = 90;
            needle.style.transform = `rotate(${angle}deg)`;

            const backendRpm = current.rpm || 0;
            const backendGear = current.gear || 1;
            const vGear = current.v_gear || 'P'; // 시각적 기어 (P, N, D)

            const currentSpeed = backendRpm * TIRE_CIRCUM * 0.06;

            rpmText.innerText = backendRpm; 
            speedText.innerText = Math.round(currentSpeed);
            
            // 하단 텍스트는 상세 정보 표시 (P, N, D1, D2...)
            if(vGear === 'D') gearText.innerText = "D" + backendGear;
            else gearText.innerText = vGear;

            // [추가] 세로 기어 스트립 업데이트
            updateGearStrip(vGear);
        }

        if (current.reason) {
            warningBox.style.display = "block";
            warningText.innerText = current.reason;
            warningBox.style.backgroundColor = (current.remaining_time === 0) 
                ? "rgba(255, 0, 0, 1)" : "rgba(255, 0, 0, 0.9)";
        } else {
            warningBox.style.display = "none";
        }

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