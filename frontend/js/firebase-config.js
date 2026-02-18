/* ========================================
   Firebase Configuration Module
   ======================================== */

export const firebaseConfig = {
    apiKey: "AIzaSyCVaEaIp1lyqLlvKR7rBFDpLNyp3Iavx48",
    authDomain: "rokey-b-3.firebaseapp.com",
    databaseURL: "https://rokey-b-3-default-rtdb.firebaseio.com",
    projectId: "rokey-b-3",
    storageBucket: "rokey-b-3.firebasestorage.app",
    messagingSenderId: "845506185932",
    appId: "1:845506185932:web:6a4a0b2c44992633151069",
    measurementId: "G-1HKVQ6P0Y2"
};

export const databasePaths = {
    current: '/one',
    logs: '/one/logs',
    emergencyStop: '/one/emergencyStop',
    command: '/one/command'
};

export function validateConfig() {
    console.log("🔍 Validating Firebase config...");
    const required = ['apiKey', 'databaseURL', 'projectId'];
    for (let key of required) {
        if (!firebaseConfig[key]) {
            console.error(`❌ Missing Firebase config: ${key}`);
            return false;
        }
        console.log(`   ✓ ${key}: ${firebaseConfig[key].substring(0, 20)}...`);
    }
    console.log("✅ Firebase config validated successfully");
    return true;
}

// Validate on module load
console.log("📦 Firebase-config module loaded");
validateConfig();
