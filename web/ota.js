/**
 * GPS Laptimer — BLE OTA Updater
 *
 * Web Bluetooth API로 ESP32-S3 NimBLE GATT 서비스에 연결하여
 * 펌웨어를 전송하는 프론트엔드 로직.
 *
 * GATT Service: 0xFFE0
 *   - Control (0xFFE1): Write — START/END/ABORT/VERSION
 *   - Data    (0xFFE2): Write Without Response — firmware chunks
 *   - Status  (0xFFE3): Read + Notify — [state, progress, error, version(16B)]
 */

// BLE UUIDs
const SVC_UUID  = 0xFFE0;
const CTRL_UUID = 0xFFE1;
const DATA_UUID = 0xFFE2;
const STAT_UUID = 0xFFE3;

// Control commands
const CMD_START   = 0x01;
const CMD_END     = 0x02;
const CMD_ABORT   = 0x03;
const CMD_VERSION = 0x04;

// OTA states
const STATE_IDLE       = 0;
const STATE_RECEIVING  = 1;
const STATE_VALIDATING = 2;
const STATE_COMPLETE   = 3;
const STATE_ERROR      = 4;

// Chunk size for BLE transfer
const CHUNK_SIZE = 512;

// GitHub repo for release downloads
const GITHUB_REPO = 'korellas/gps-laptimer';

// --- DOM refs ---
const $connectBtn    = document.getElementById('connectBtn');
const $disconnectBtn = document.getElementById('disconnectBtn');
const $statusDot     = document.getElementById('statusDot');
const $statusText    = document.getElementById('statusText');
const $devVersion    = document.getElementById('devVersion');
const $releaseCard   = document.getElementById('releaseCard');
const $releaseVer    = document.getElementById('releaseVer');
const $releaseDate   = document.getElementById('releaseDate');
const $releaseSize   = document.getElementById('releaseSize');
const $releaseStatus = document.getElementById('releaseStatus');
const $fileInput     = document.getElementById('fileInput');
const $fileInfo      = document.getElementById('fileInfo');
const $startBtn      = document.getElementById('startBtn');
const $progressCard  = document.getElementById('progressCard');
const $progressFill  = document.getElementById('progressFill');
const $progressPct   = document.getElementById('progressPct');
const $transferred   = document.getElementById('transferred');
const $elapsed       = document.getElementById('elapsed');
const $speed         = document.getElementById('speed');
const $eta           = document.getElementById('eta');
const $resultCard    = document.getElementById('resultCard');
const $resultMsg     = document.getElementById('resultMsg');
const $log           = document.getElementById('log');
const $abortBtn      = document.getElementById('abortBtn');

// --- State ---
let bleDevice = null;
let ctrlChar  = null;
let dataChar  = null;
let statChar  = null;
let firmware  = null;   // ArrayBuffer
let firmwareName = '';
let releaseVersion = '';
let deviceVersion = '';
let transferring = false;
let startTime = 0;

// --- Logging ---
function log(msg) {
    const ts = new Date().toLocaleTimeString();
    $log.textContent += '[' + ts + '] ' + msg + '\n';
    $log.scrollTop = $log.scrollHeight;
}

// --- BLE Connection ---
async function connect() {
    if (!navigator.bluetooth) {
        alert('Web Bluetooth is not supported in this browser.\nUse Chrome or Edge.');
        return;
    }
    try {
        log('Scanning for LAPTIMER-OTA...');
        bleDevice = await navigator.bluetooth.requestDevice({
            filters: [{ namePrefix: 'LAPTIMER' }],
            optionalServices: [SVC_UUID]
        });
        bleDevice.addEventListener('gattserverdisconnected', onDisconnected);

        log('Connecting to ' + bleDevice.name + '...');
        const server = await bleDevice.gatt.connect();

        log('Getting OTA service...');
        const svc = await server.getPrimaryService(SVC_UUID);

        ctrlChar = await svc.getCharacteristic(CTRL_UUID);
        dataChar = await svc.getCharacteristic(DATA_UUID);
        statChar = await svc.getCharacteristic(STAT_UUID);

        // Subscribe to status notifications
        await statChar.startNotifications();
        statChar.addEventListener('characteristicvaluechanged', onStatusNotify);

        setConnected(true);
        log('Connected! Requesting version...');

        // Request version
        await ctrlChar.writeValue(new Uint8Array([CMD_VERSION]));

    } catch (e) {
        log('Error: ' + e.message);
        setConnected(false);
    }
}

function disconnect() {
    if (bleDevice && bleDevice.gatt.connected) {
        bleDevice.gatt.disconnect();
    }
}

function onDisconnected() {
    log('Disconnected');
    setConnected(false);
    ctrlChar = null;
    dataChar = null;
    statChar = null;
    deviceVersion = '';
    if (transferring) {
        transferring = false;
        showResult('Connection lost during transfer', true);
    }
}

function setConnected(connected) {
    $statusDot.className = 'status-dot ' + (connected ? 'connected' : 'disconnected');
    $statusText.textContent = connected ? 'Connected' : 'Disconnected';
    $connectBtn.classList.toggle('hidden', connected);
    $disconnectBtn.classList.toggle('hidden', !connected);
    updateStartButton();
}

function updateStartButton() {
    var ready = ctrlChar && firmware;
    $startBtn.disabled = !ready;
    if (firmware && releaseVersion) {
        if (deviceVersion && deviceVersion === releaseVersion) {
            $startBtn.textContent = 'Already up to date (' + deviceVersion + ')';
            $startBtn.disabled = true;
        } else if (deviceVersion) {
            $startBtn.textContent = 'Update ' + deviceVersion + ' → ' + releaseVersion;
        } else {
            $startBtn.textContent = 'Update to ' + releaseVersion;
        }
    } else if (firmware) {
        $startBtn.textContent = 'Start Update';
    } else {
        $startBtn.textContent = 'Loading firmware...';
    }
}

// --- Status Notification ---
function onStatusNotify(event) {
    const data = new Uint8Array(event.target.value.buffer);
    if (data.length < 3) return;

    const state = data[0];
    const progress = data[1];
    const errorCode = data[2];

    // Extract version string (bytes 3-18)
    let version = '';
    if (data.length >= 19) {
        const verBytes = data.slice(3, 19);
        const nullIdx = verBytes.indexOf(0);
        version = new TextDecoder().decode(verBytes.slice(0, nullIdx >= 0 ? nullIdx : 16));
    }

    if (version && state === STATE_IDLE) {
        deviceVersion = version;
        $devVersion.textContent = version;
        log('Device version: ' + version);
        updateStartButton();
    }

    if (state === STATE_RECEIVING) {
        updateProgress(progress);
    } else if (state === STATE_VALIDATING) {
        updateProgress(100);
        $progressPct.textContent = 'Validating...';
        log('Validating firmware...');
    } else if (state === STATE_COMPLETE) {
        transferring = false;
        log('OTA complete! Device is rebooting...');
        showResult('Update successful! Device is rebooting.', false);
    } else if (state === STATE_ERROR) {
        transferring = false;
        const errors = ['Unknown', 'Image too large', 'Validation failed', 'Low battery', 'Write error'];
        const msg = errors[errorCode] || ('Error code: ' + errorCode);
        log('OTA error: ' + msg);
        showResult(msg, true);
    }
}

// --- Progress ---
function updateProgress(pct) {
    $progressFill.style.width = pct + '%';
    $progressPct.textContent = pct + '%';

    if (startTime > 0) {
        const elapsed = (Date.now() - startTime) / 1000;
        const bytesTotal = firmware ? firmware.byteLength : 0;
        const bytesSent = Math.round(bytesTotal * pct / 100);
        const kbps = elapsed > 0 ? (bytesSent / 1024 / elapsed).toFixed(1) : '0';
        const etaSec = pct > 0 ? Math.round(elapsed * (100 - pct) / pct) : 0;

        $transferred.textContent = formatBytes(bytesSent) + ' / ' + formatBytes(bytesTotal);
        $elapsed.textContent = formatTime(elapsed);
        $speed.textContent = kbps + ' KB/s';
        $eta.textContent = etaSec > 0 ? formatTime(etaSec) : '--';
    }
}

function formatBytes(b) {
    if (b < 1024) return b + ' B';
    return (b / 1024 / 1024).toFixed(2) + ' MB';
}

function formatTime(sec) {
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return m + ':' + (s < 10 ? '0' : '') + s;
}

// --- Firmware source ---
function onFileSelect() {
    const file = $fileInput.files[0];
    if (!file) return;
    $fileInfo.textContent = file.name + ' (' + formatBytes(file.size) + ')';
    const reader = new FileReader();
    reader.onload = function () {
        firmware = reader.result;
        firmwareName = file.name;
        releaseVersion = '';  // manual file, no version info
        updateStartButton();
        log('Loaded: ' + file.name + ' (' + formatBytes(firmware.byteLength) + ')');
    };
    reader.readAsArrayBuffer(file);
}

async function checkRelease() {
    try {
        log('Checking latest release...');
        $releaseStatus.textContent = 'Checking...';
        const resp = await fetch('https://api.github.com/repos/' + GITHUB_REPO + '/releases/latest');
        if (!resp.ok) {
            log('No releases found');
            $releaseStatus.textContent = 'No releases available';
            return;
        }
        const rel = await resp.json();
        // Find gps_laptimer.bin specifically (not bootloader or partition table)
        const asset = rel.assets.find(function (a) { return a.name === 'gps_laptimer.bin'; })
                   || rel.assets.find(function (a) { return a.name.endsWith('.bin'); });
        if (!asset) {
            log('No .bin asset in release');
            $releaseStatus.textContent = 'No firmware in release';
            return;
        }

        releaseVersion = rel.tag_name;
        $releaseVer.textContent = rel.tag_name;
        $releaseDate.textContent = new Date(rel.published_at).toLocaleDateString();
        $releaseSize.textContent = formatBytes(asset.size);
        $releaseCard.classList.remove('hidden');
        log('Latest release: ' + rel.tag_name + ' (' + formatBytes(asset.size) + ')');

        // Auto-download firmware
        $releaseStatus.textContent = 'Downloading firmware...';
        log('Downloading ' + asset.name + '...');
        const dlResp = await fetch(asset.browser_download_url);
        firmware = await dlResp.arrayBuffer();
        firmwareName = asset.name;
        $releaseStatus.textContent = 'Ready — ' + formatBytes(firmware.byteLength);
        $fileInfo.textContent = firmwareName + ' (' + formatBytes(firmware.byteLength) + ')';
        updateStartButton();
        log('Firmware ready: ' + formatBytes(firmware.byteLength));

    } catch (e) {
        log('Release check failed: ' + e.message);
        $releaseStatus.textContent = 'Failed to load release';
    }
}

// --- OTA Transfer ---
async function startOta() {
    if (!ctrlChar || !dataChar || !firmware) return;

    transferring = true;
    startTime = Date.now();
    $progressCard.classList.remove('hidden');
    $resultCard.classList.add('hidden');
    $startBtn.disabled = true;
    $abortBtn.classList.remove('hidden');
    updateProgress(0);

    const size = firmware.byteLength;
    log('Starting OTA: ' + formatBytes(size));

    try {
        // Send START command: [0x01, size(4B little-endian)]
        const startCmd = new Uint8Array(5);
        startCmd[0] = CMD_START;
        startCmd[1] = size & 0xFF;
        startCmd[2] = (size >> 8) & 0xFF;
        startCmd[3] = (size >> 16) & 0xFF;
        startCmd[4] = (size >> 24) & 0xFF;
        await ctrlChar.writeValue(startCmd);
        log('START sent, size=' + size);

        // Send firmware in chunks
        const data = new Uint8Array(firmware);
        let offset = 0;
        while (offset < size && transferring) {
            const end = Math.min(offset + CHUNK_SIZE, size);
            const chunk = data.slice(offset, end);
            await dataChar.writeValueWithoutResponse(chunk);
            offset = end;

            // Local progress update (server notifies every 5%)
            const localPct = Math.round(offset / size * 100);
            updateProgress(localPct);
        }

        if (!transferring) {
            log('Transfer aborted by user');
            return;
        }

        // Send END command
        log('All chunks sent, sending END...');
        await ctrlChar.writeValue(new Uint8Array([CMD_END]));

    } catch (e) {
        transferring = false;
        log('Transfer error: ' + e.message);
        showResult('Transfer failed: ' + e.message, true);
    }
}

async function abortOta() {
    transferring = false;
    if (ctrlChar) {
        try {
            await ctrlChar.writeValue(new Uint8Array([CMD_ABORT]));
            log('Abort sent');
        } catch (e) {
            log('Abort send failed: ' + e.message);
        }
    }
    $abortBtn.classList.add('hidden');
    updateStartButton();
    showResult('Update aborted', true);
}

function showResult(msg, isError) {
    $progressCard.classList.add('hidden');
    $resultCard.classList.remove('hidden');
    $abortBtn.classList.add('hidden');
    $resultMsg.className = isError ? 'err' : 'ok';
    $resultMsg.textContent = msg;
    updateStartButton();
}

// --- Init ---
$connectBtn.addEventListener('click', connect);
$disconnectBtn.addEventListener('click', disconnect);
$fileInput.addEventListener('change', onFileSelect);
$startBtn.addEventListener('click', startOta);
$abortBtn.addEventListener('click', abortOta);

// Auto-fetch latest release on page load
checkRelease();
