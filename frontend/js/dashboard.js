// dashboard.js

const FIREBASE_URL = "https://rokey-b-3-default-rtdb.firebaseio.com";
const API_KEY = "AIzaSyCVaEaIp1lyqLlvKR7rBFDpLNyp3Iavx48";
const DATA_PATH = "/task_runner";

let updateCount = 0;
let errorCount = 0;

// ========== Update Functions ==========
function updateStatus(status) {
    const statusBox = document.getElementById('status-box');
    if (statusBox) {
        statusBox.className = 'status-indicator ' + status;
        statusBox.textContent = status;
    }
}

function updateJointValues(joints) {
    if (!Array.isArray(joints) || joints.length < 6) return;

    for (let i = 0; i < 6; i++) {
        const angle = joints[i];

        const gaugeFill = document.getElementById(`joint-gauge-${i}`);
        if (gaugeFill) {
            const percentage = ((angle + 180) / 360) * 100;
            gaugeFill.style.width = Math.max(0, Math.min(100, percentage)) + '%';
        }

        const gaugeValue = document.getElementById(`joint-value-${i}`);
        if (gaugeValue) {
            gaugeValue.textContent = angle.toFixed(1) + '°';
        }
    }
}

// ========== Firebase Data Polling ==========
function pollRobotData() {
    const dataUrl = `${FIREBASE_URL}${DATA_PATH}.json?auth=${API_KEY}`;

    fetch(dataUrl, { method: 'GET' })
        .then(response => {
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            return response.json();
        })
        .then(data => {
            if (!data) return;

            updateCount++;

            // Update status
            if (data.status) updateStatus(data.status);

            // Update joints
            if (data.joints) updateJointValues(data.joints);

            // Update connection status
            const connStatus = document.getElementById('connection-status');
            const robotConn = document.getElementById('robot-connection');
            if (connStatus) {
                connStatus.textContent = '🟢 Connected';
                connStatus.className = 'status-connected';
            }
            if (robotConn) {
                robotConn.textContent = data.connected ? '✅ OK' : '❌ Disconnected';
                robotConn.style.color = data.connected ? '#4caf50' : '#f44336';
            }

            // Update error display
            const lastError = document.getElementById('last-error');
            if (lastError) {
                lastError.textContent = data.error || '-';
            }

            // Update timestamp
            const debugTimestamp = document.getElementById('debug-timestamp');
            if (debugTimestamp && data.timestamp) {
                const updateDate = new Date(data.timestamp * 1000).toLocaleTimeString('ko-KR');
                debugTimestamp.textContent = updateDate;
            }
        })
        .catch(error => {
            errorCount++;
            const connStatus = document.getElementById('connection-status');
            if (connStatus) {
                connStatus.textContent = '🔴 Disconnected';
                connStatus.className = 'status-disconnected';
            }
        });
}

// ========== Control Commands ==========
function sendCommand(command) {
    console.log('🎮 Sending command:', command);

    const commandUrl = `${FIREBASE_URL}${DATA_PATH}/command.json?auth=${API_KEY}`;

    const commandData = {
        cmd: command,
        timestamp: new Date().toISOString()
    };

    fetch(commandUrl, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(commandData)
    })
        .then(response => {
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            return response.json();
        })
        .then(data => {
            console.log('✅ Command sent:', command);
            updateStatus(command === 'START' ? 'RUNNING' : command);
        })
        .catch(error => {
            console.error('❌ Error sending command:', error);
            alert('명령 전송 실패: ' + error.message);
        });
}

// ========== Button Event Listeners ==========
document.addEventListener('DOMContentLoaded', function () {
    const buttons = {
        'btn-start': 'START',
        'btn-pause': 'PAUSE',
        'btn-home': 'HOME',
        'btn-reset': 'RESET',
        'btn-emergency-stop': 'EMERGENCY_STOP'
    };

    Object.entries(buttons).forEach(([id, cmd]) => {
        const btn = document.getElementById(id);
        if (btn) {
            btn.addEventListener('click', function () {
                if (cmd === 'EMERGENCY_STOP') {
                    if (confirm('정말로 긴급 정지를 실행하시겠습니까?')) {
                        sendCommand(cmd);
                    }
                } else if (cmd === 'RESET') {
                    if (confirm('Safe Stop 상태를 해제하시겠습니까?')) {
                        sendCommand(cmd);
                    }
                } else {
                    sendCommand(cmd);
                }
            });
        }
    });

    // Initialize
    console.log('🚀 Dashboard initializing...');
    const connStatus = document.getElementById('connection-status');
    if (connStatus) connStatus.textContent = '🟡 Connecting...';

    pollRobotData();
    setInterval(pollRobotData, 500);

    console.log('✅ Dashboard ready');
});
