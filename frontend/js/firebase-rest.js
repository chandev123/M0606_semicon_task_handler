// Firebase REST API 클라이언트 (모듈 없음)
// Firebase Realtime Database REST 방식으로 접근

const FIREBASE_CONFIG = {
    databaseURL: "https://rokey-b-3-default-rtdb.firebaseio.com",
    apiKey: "AIzaSyCVaEaIp1lyqLlvKR7rBFDpLNyp3Iavx48"
};

// Chart 인스턴스
let forceChart = null;
let chartData = {
    labels: [],
    datasets: {
        x: [],
        y: [],
        z: []
    }
};


function initChart() {
    const ctx = document.getElementById('forceChart');
    if (!ctx) return;

    if (forceChart) forceChart.destroy();

    forceChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: chartData.labels,
            datasets: [
                {
                    label: 'Force X (N)',
                    data: chartData.datasets.x,
                    borderColor: '#ff5252', // Red
                    backgroundColor: 'rgba(255, 82, 82, 0.1)',
                    tension: 0.4,
                    borderWidth: 2,
                    pointRadius: 0 // 점을 숨겨서 깔끔하게 표현
                },
                {
                    label: 'Force Y (N)',
                    data: chartData.datasets.y,
                    borderColor: '#69f0ae', // Green
                    backgroundColor: 'rgba(105, 240, 174, 0.1)',
                    tension: 0.4,
                    borderWidth: 2,
                    pointRadius: 0
                },
                {
                    label: 'Force Z (N)',
                    data: chartData.datasets.z,
                    borderColor: '#448aff', // Blue
                    backgroundColor: 'rgba(68, 138, 255, 0.1)',
                    tension: 0.4,
                    borderWidth: 2,
                    pointRadius: 0
                }
            ]
        },
        options: {
            responsive: true,
            interaction: {
                mode: 'index', // 마우스 오버 시 3개 값 동시 확인
                intersect: false,
            },
            plugins: {
                legend: { labels: { color: '#e0e0e0' } },
                tooltip: {
                    mode: 'index',
                    intersect: false
                }
            },
            scales: {
                y: {
                    grid: { color: 'rgba(255, 255, 255, 0.1)' },
                    ticks: { color: '#888' }
                },
                x: {
                    grid: { display: false }, // X축 그리드는 숨겨서 깔끔하게
                    ticks: { display: false } // 라벨이 너무 많으면 지저분하므로 숨김 처리 고려
                }
            }
        }
    });
}
function updateDashboard(data) {
    // ... (기존 코드) ...

    // 힘 데이터 처리 (구조가 변경되었으므로 대응)
    // 백엔드에서 data.force = {x:..., y:..., z:...} 로 보낸다고 가정
    let fx = 0, fy = 0, fz = 0;

    if (data.force) {
        fx = data.force.x || 0;
        fy = data.force.y || 0;
        fz = data.force.z || 0;
    } else if (data.force_z) {
        // 하위 호환성 (구버전 데이터 대응)
        fz = data.force_z;
    }

    // 텍스트 업데이트
    document.getElementById('force-x').textContent = fx.toFixed(2);
    document.getElementById('force-y').textContent = fy.toFixed(2);
    document.getElementById('force-z').textContent = fz.toFixed(2);

    // [New] Call dashboard.js visual helper if available
    if (typeof updateForceColor === 'function') {
        updateForceColor(fz);
    }

    // [New] Update Peak Values
    updatePeakValues(fz, pos?.z, opTime);

    // 차트 데이터 추가
    addChartData(fx, fy, fz);

    // ... (이하 동일) ...
}

function addChartData(fx, fy, fz) {
    if (!forceChart) return;

    const now = new Date();
    const label = now.toLocaleTimeString('ko-KR', { hour12: false });

    chartData.labels.push(label);
    chartData.datasets.x.push(fx);
    chartData.datasets.y.push(fy);
    chartData.datasets.z.push(fz);

    // 데이터 60개 유지
    if (chartData.labels.length > 60) {
        chartData.labels.shift();
        chartData.datasets.x.shift();
        chartData.datasets.y.shift();
        chartData.datasets.z.shift();
    }

    forceChart.data.labels = chartData.labels;
    forceChart.data.datasets[0].data = chartData.datasets.x;
    forceChart.data.datasets[1].data = chartData.datasets.y;
    forceChart.data.datasets[2].data = chartData.datasets.z;

    forceChart.update('none'); // 'none' 모드로 업데이트 시 깜빡임 최소화
}

// [New] Peak Data Tracking
let peakData = {
    maxForceZ: 0,
    minPosZ: Infinity,
    maxPosZ: -Infinity,
    duration: 0
};

function updatePeakValues(fz, posZ, duration) {
    if (fz === undefined || posZ === undefined) return;

    fz = Math.abs(fz);
    if (fz > peakData.maxForceZ) peakData.maxForceZ = fz;
    if (posZ < peakData.minPosZ) peakData.minPosZ = posZ;
    if (posZ > peakData.maxPosZ) peakData.maxPosZ = posZ;

    // Duration is cumulative operational time
    if (duration > peakData.duration) peakData.duration = duration;

    // Update DOM
    const pfz = document.getElementById('peak-force-z');
    if (pfz) pfz.textContent = peakData.maxForceZ.toFixed(2);

    const pzmin = document.getElementById('peak-pos-z-min');
    if (pzmin) pzmin.textContent = (peakData.minPosZ === Infinity) ? '-' : peakData.minPosZ.toFixed(2);

    const pzmax = document.getElementById('peak-pos-z-max');
    if (pzmax) pzmax.textContent = (peakData.maxPosZ === -Infinity) ? '-' : peakData.maxPosZ.toFixed(2);

    const pdur = document.getElementById('peak-duration');
    if (pdur) pdur.textContent = peakData.duration.toFixed(2) + ' h';
}

// Firebase 폴링 타이머
let pollInterval = null;
let lastTimestamp = null;

function startFirebaseListener() {
    console.log("📡 Starting Firebase real-time listener (polling mode)...");

    // 기존 폴링 종료
    if (pollInterval) {
        clearInterval(pollInterval);
    }

    // 500ms마다 데이터 폴링
    pollInterval = setInterval(() => {
        pollFirebaseData();
    }, 500);

    // 즉시 한 번 실행
    pollFirebaseData();
}

function pollFirebaseData() {
    const url = `${FIREBASE_CONFIG.databaseURL}/one.json?auth=${FIREBASE_CONFIG.apiKey}`;

    fetch(url)
        .then(response => {
            if (response.status === 401) {
                console.error("❌ Firebase 401 Unauthorized - checking security rules...");
                updateConnectionStatus(false);
                return null;
            }
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            if (data === null) return;

            // 타임스탬프로 새로운 데이터인지 확인
            if (data && data.timestamp !== lastTimestamp) {
                lastTimestamp = data.timestamp;
                console.log("📥 Firebase data received (polling):", data);
                updateDashboard(data);
                updateConnectionStatus(true);
            }
        })
        .catch(error => {
            console.error("❌ Polling error:", error);
            updateConnectionStatus(false);
        });
}

function updateDashboard(data) {
    if (!data) return;

    // 연결 상태 업데이트
    updateConnectionStatus(true);

    // 상태 박스
    if (data.status) {
        const statusEl = document.getElementById('status-box');
        statusEl.textContent = data.status;
        statusEl.className = data.status;
    }

    // 운영 시간 (operational_time 또는 operationalTime)
    const opTime = data.operational_time || data.operationalTime || 0;
    if (opTime !== undefined) {
        document.getElementById('operational-time').textContent = opTime.toFixed(2);
    }

    // 로봇 위치 (pos 또는 position)
    const pos = data.pos || data.position;
    if (pos) {
        document.getElementById('pos-x').textContent = (pos.x || 0).toFixed(2);
        document.getElementById('pos-y').textContent = (pos.y || 0).toFixed(2);
        document.getElementById('pos-z').textContent = (pos.z || 0).toFixed(2);
    }

    // 힘 데이터 (force_z 또는 force.z)
    const forceZ = data.force_z || (data.force?.z) || 0;
    document.getElementById('force-z').textContent = forceZ.toFixed(2);

    // 차트에 데이터 추가
    addChartData(forceZ);

    // 시스템 정보
    if (data.system_info || data.systemInfo) {
        const sysInfo = data.system_info || data.systemInfo;
        document.getElementById('system-name').textContent = sysInfo.name || 'N/A';
        document.getElementById('system-version').textContent = sysInfo.version || 'N/A';
        document.getElementById('system-device').textContent = sysInfo.device_id || sysInfo.deviceId || 'N/A';
    }

    // 관절 데이터 (J1-J6 각도)
    if (data.joints && Array.isArray(data.joints)) {
        const jointNames = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
        for (let i = 0; i < 6; i++) {
            const angle = data.joints[i] || 0;
            // 게이지 값 업데이트
            const gaugeFill = document.getElementById(`joint-gauge-${i}`);
            const gaugeValue = document.getElementById(`joint-value-${i}`);
            if (gaugeFill) {
                // -180 ~ 180 범위를 0 ~ 100%로 정규화
                const percentage = ((angle + 180) / 360) * 100;
                gaugeFill.style.width = percentage + '%';
                // 색상: 중앙(0도)=청색, 극단값(±180도)=빨강
                const hue = 240 - (Math.abs(angle) / 180 * 180);
                gaugeFill.style.backgroundColor = `hsl(${hue}, 100%, 50%)`;
            }
            if (gaugeValue) {
                gaugeValue.textContent = angle.toFixed(1);
            }
        }
    }
    // [firebase-rest.js] updateDashboard 함수 내부

    function updateDashboard(data) {
        if (!data) return;

        // ... (기존 연결 상태 업데이트 코드) ...

        // [충돌 감지 로직 추가]
        const body = document.body;
        const statusBox = document.getElementById('status-box');

        // 백엔드에서 보낸 충돌 플래그 확인
        if (data.collision) {
            // 1. 대시보드 전체 혹은 상태 박스에 경고 효과 적용
            statusBox.classList.add('critical-alert');
            statusBox.textContent = "💥 COLLISION!";

            // (선택사항) 배경이나 헤더도 붉게 물들이고 싶다면:
            // document.querySelector('.header').style.backgroundColor = 'rgba(244, 67, 54, 0.2)';
        } else {
            // 2. 평상시 상태로 복구
            statusBox.classList.remove('critical-alert');

            // 기존 상태 텍스트 복구 (예: WAITING, RUNNING 등)
            if (data.status) {
                statusBox.textContent = data.status;
                statusBox.className = ''; // 기존 클래스 초기화
                statusBox.classList.add(data.status); // 상태별 색상 클래스 재적용
            }
        }

        // [힘 데이터 표시 업데이트] (지난번 코드와 연동)
        // 힘의 총량(Magnitude)도 보여주면 좋습니다.
        if (data.force_mag !== undefined) {
            // HTML에 id="force-mag" 요소가 있다면 업데이트
            // document.getElementById('force-mag').textContent = data.force_mag.toFixed(2);
        }

        // ... (나머지 차트 업데이트 등 기존 코드) ...
    }

    // 로봇 로그 메시지
    if (data.logs && Array.isArray(data.logs)) {
        updateRobotLogs(data.logs);
    }

    // 데이터 그리드
    if (data.timestamp) {
        document.getElementById('grid-status').textContent = data.status || '-';
        document.getElementById('grid-force').textContent = forceZ.toFixed(2) + ' N';
        document.getElementById('grid-pos-x').textContent = (pos?.x || 0).toFixed(2) + ' mm';
        document.getElementById('grid-pos-y').textContent = (pos?.y || 0).toFixed(2) + ' mm';
        document.getElementById('grid-pos-z').textContent = (pos?.z || 0).toFixed(2) + ' mm';
        document.getElementById('grid-optime').textContent = opTime.toFixed(2) + ' h';

        // timestamp는 unix timestamp (초 단위)
        const date = new Date(data.timestamp * 1000);
        document.getElementById('grid-timestamp').textContent = date.toLocaleString('ko-KR');
    }
}

function updateRobotLogs(logs) {
    const logConsole = document.getElementById('log-console');
    if (!logConsole) return;

    // 로그 콘솔에 메시지 추가
    logConsole.innerHTML = logs.map(log => `<div class="log-line">${log}</div>`).join('');

    // 최신 메시지로 스크롤
    logConsole.scrollTop = logConsole.scrollHeight;
}

function initChart() {
    const ctx = document.getElementById('forceChart');
    if (!ctx) {
        console.warn("⚠️ Chart canvas not found");
        return;
    }

    if (forceChart) {
        forceChart.destroy();
    }

    forceChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: chartData.labels,
            datasets: [{
                label: 'Z-Axis Force (N)',
                data: chartData.data,
                borderColor: '#00bcd4',
                backgroundColor: 'rgba(0, 188, 212, 0.1)',
                tension: 0.4,
                fill: true,
                pointRadius: 2,
                pointBackgroundColor: '#00bcd4'
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: true,
            plugins: {
                legend: {
                    labels: {
                        color: '#e0e0e0',
                        font: { size: 12 }
                    }
                }
            },
            scales: {
                y: {
                    beginAtZero: true,
                    ticks: { color: '#888' },
                    grid: { color: 'rgba(255, 255, 255, 0.1)' }
                },
                x: {
                    ticks: { color: '#888' },
                    grid: { color: 'rgba(255, 255, 255, 0.1)' }
                }
            }
        }
    });
}

function addChartData(value) {
    if (!forceChart) return;

    const now = new Date();
    const label = now.toLocaleTimeString('ko-KR', { hour12: false });

    chartData.labels.push(label);
    chartData.data.push(value);

    // 최근 60개만 유지
    if (chartData.labels.length > 60) {
        chartData.labels.shift();
        chartData.data.shift();
    }

    forceChart.data.labels = chartData.labels;
    forceChart.data.datasets[0].data = chartData.data;
    forceChart.update();
}

function updateConnectionStatus(connected) {
    const status = document.getElementById('connection-status');
    if (connected) {
        status.textContent = '🟢 Connected';
        status.style.color = '#4caf50';
    } else {
        status.textContent = '🔴 Disconnected';
        status.style.color = '#f44336';
    }
}

function initDashboard() {
    console.log("🚀 Dashboard Initialization Started");

    try {
        // 차트 초기화
        initChart();
        console.log("✅ Chart initialized");

        // Firebase 리스너 시작
        startFirebaseListener();
        console.log("✅ Firebase listener started");

        // 초기 데이터 로드
        // 초기 데이터 로드
        loadInitialData();
        console.log("✅ Initial data loaded");

        // 이벤트 리스너 초기화
        initEventListeners();
        console.log("✅ Event listeners initialized");

    } catch (error) {
        console.error("❌ Dashboard initialization error:", error);
        updateConnectionStatus(false);
    }
}

function initEventListeners() {
    const btnStop = document.getElementById('btn-emergency-stop');
    const btnRecover = document.getElementById('btn-emergency-recover');

    if (btnStop) {
        btnStop.addEventListener('click', () => sendCommand('STOP'));
    }
    if (btnRecover) {
        btnRecover.addEventListener('click', () => sendCommand('RECOVER'));
    }
}

function sendCommand(cmd) {
    console.log(`📤 Sending command: ${cmd}`);
    const url = `${FIREBASE_CONFIG.databaseURL}/one/emergencyStop.json?auth=${FIREBASE_CONFIG.apiKey}`;

    const payload = {
        command: cmd,
        timestamp: Date.now() / 1000
    };

    fetch(url, {
        method: 'PUT', // Use PUT to overwrite the command, or POST for list
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
    })
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            console.log("✅ Command sent successfully");
            alert(`Command '${cmd}' sent!`);
        })
        .catch(error => {
            console.error("❌ Error sending command:", error);
            alert(`Failed to send command: ${error}`);
        });
}

function loadInitialData() {
    console.log("📥 Loading initial data from Firebase...");

    const url = `${FIREBASE_CONFIG.databaseURL}/one.json?auth=${FIREBASE_CONFIG.apiKey}`;

    fetch(url)
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            console.log("✅ Initial data loaded:", data);
            if (data) {
                lastTimestamp = data.timestamp;
                updateDashboard(data);
            }
        })
        .catch(error => {
            console.error("❌ Error loading initial data:", error);
            updateConnectionStatus(false);
        });
}

// 페이지 로드 시 초기화
document.addEventListener('DOMContentLoaded', initDashboard);
