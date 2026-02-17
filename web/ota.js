/**
 * GPS Laptimer BLE OTA Updater
 *
 * GATT Service: 0xFFE0
 *   - Control (0xFFE1): START/END/ABORT/VERSION
 *   - Data    (0xFFE2): firmware chunks
 *   - Status  (0xFFE3): [state, progress, error, version(16B)]
 */

// BLE UUIDs
const SVC_UUID = 0xFFE0;
const CTRL_UUID = 0xFFE1;
const DATA_UUID = 0xFFE2;
const STAT_UUID = 0xFFE3;

// Control commands
const CMD_START = 0x01;
const CMD_END = 0x02;
const CMD_ABORT = 0x03;
const CMD_VERSION = 0x04;

// OTA states
const STATE_IDLE = 0;
const STATE_RECEIVING = 1;
const STATE_VALIDATING = 2;
const STATE_COMPLETE = 3;
const STATE_ERROR = 4;

// Chunk size for BLE transfer.
// 244-byte payload is conservative and generally more stable on Web Bluetooth stacks.
const CHUNK_SIZE = 244;
const WRITE_RETRY_MAX = 5;
const WRITE_RETRY_DELAY_MS = 20;
const WRITE_YIELD_EVERY = 20;
const CONTROL_RETRY_MAX = 3;

// GitHub repo for release downloads
const GITHUB_REPO = "korellas/gps-laptimer";

// --- DOM refs ---
const $connectBtn = document.getElementById("connectBtn");
const $disconnectBtn = document.getElementById("disconnectBtn");
const $statusDot = document.getElementById("statusDot");
const $statusText = document.getElementById("statusText");
const $devVersion = document.getElementById("devVersion");
const $releaseCard = document.getElementById("releaseCard");
const $releaseVer = document.getElementById("releaseVer");
const $releaseDate = document.getElementById("releaseDate");
const $releaseSize = document.getElementById("releaseSize");
const $releaseStatus = document.getElementById("releaseStatus");
const $retryReleaseBtn = document.getElementById("retryReleaseBtn");
const $fileInput = document.getElementById("fileInput");
const $fileInfo = document.getElementById("fileInfo");
const $confirmUpdate = document.getElementById("confirmUpdate");
const $startBtn = document.getElementById("startBtn");
const $progressCard = document.getElementById("progressCard");
const $progressState = document.getElementById("progressState");
const $progressFill = document.getElementById("progressFill");
const $progressPct = document.getElementById("progressPct");
const $transferred = document.getElementById("transferred");
const $elapsed = document.getElementById("elapsed");
const $speed = document.getElementById("speed");
const $eta = document.getElementById("eta");
const $resultCard = document.getElementById("resultCard");
const $resultMsg = document.getElementById("resultMsg");
const $log = document.getElementById("log");
const $abortBtn = document.getElementById("abortBtn");

// --- State ---
let bleDevice = null;
let ctrlChar = null;
let dataChar = null;
let statChar = null;
let firmware = null; // ArrayBuffer
let firmwareName = "";
let firmwareSource = "";
let releaseVersion = "";
let deviceVersion = "";
let transferring = false;
let startTime = 0;
let releaseAutoState = "loading"; // loading | ready | unavailable | failed
let latestOtaState = STATE_IDLE;
let latestOtaErrorCode = 0;
let lastVersionNote = "";

// --- Logging ---
function log(msg) {
    const ts = new Date().toLocaleTimeString();
    $log.textContent += "[" + ts + "] " + msg + "\n";
    $log.scrollTop = $log.scrollHeight;
}

function formatError(err) {
    if (!err) {
        return "unknown error";
    }
    if (err.name === "AbortError") {
        return "request timeout";
    }
    if (err.message) {
        return err.message;
    }
    return String(err);
}

function sleep(ms) {
    return new Promise(function (resolve) {
        setTimeout(resolve, ms);
    });
}

async function writeControlWithRetry(payload, label) {
    let lastErr = null;
    for (let attempt = 1; attempt <= CONTROL_RETRY_MAX; attempt += 1) {
        try {
            await ctrlChar.writeValue(payload);
            return;
        } catch (err) {
            lastErr = err;
            if (attempt === CONTROL_RETRY_MAX) {
                break;
            }
            log(label + " retry " + attempt + "/" + CONTROL_RETRY_MAX + ": " + formatError(err));
            await sleep(WRITE_RETRY_DELAY_MS * attempt);
        }
    }
    throw lastErr || new Error(label + " failed");
}

async function writeDataChunk(chunk) {
    if (dataChar && dataChar.properties && dataChar.properties.write) {
        await dataChar.writeValue(chunk);
    } else {
        await dataChar.writeValueWithoutResponse(chunk);
    }
}

function decodeOtaError(code) {
    const map = {
        0: "Unknown",
        1: "Image too large",
        2: "Validation failed",
        3: "Low battery",
        4: "Write failed",
        5: "No OTA partition",
        6: "OTA begin failed",
        7: "Set boot partition failed",
        8: "Internal stack required",
        9: "OTA busy",
        10: "OTA timeout",
        11: "OTA aborted"
    };
    return map[code] || ("Error code: " + code);
}

async function fetchWithTimeout(url, options, timeoutMs) {
    const controller = new AbortController();
    const timeoutId = setTimeout(function () {
        controller.abort();
    }, timeoutMs || 30000);
    try {
        const opts = Object.assign({}, options || {});
        opts.signal = controller.signal;
        return await fetch(url, opts);
    } finally {
        clearTimeout(timeoutId);
    }
}

function setProgressState(text, isError) {
    const error = !!isError;
    const ok = !error && text.toLowerCase().includes("complete");
    $progressState.textContent = text;
    $progressState.className = "progress-state" + (error ? " error" : ok ? " ok" : "");
}

function resetProgressView() {
    $progressFill.style.width = "0%";
    $progressPct.textContent = "0%";
    $transferred.textContent = "0 B / 0 B";
    $elapsed.textContent = "0:00";
    $speed.textContent = "-- KB/s";
    $eta.textContent = "--";
    setProgressState("Ready", false);
}

function setTransferringUi(active) {
    transferring = active;
    $abortBtn.classList.toggle("hidden", !active);
    $fileInput.disabled = active;
    $confirmUpdate.disabled = active;
    updateStartButton();
}

function resetConfirmation() {
    $confirmUpdate.checked = false;
    updateStartButton();
}

function getTargetLabel() {
    if (releaseVersion) {
        return releaseVersion;
    }
    if (firmwareName) {
        return firmwareName;
    }
    return "selected firmware";
}

function parseVersion(input) {
    if (!input) {
        return null;
    }
    const m = String(input).match(/(\d+)\.(\d+)\.(\d+)/);
    if (!m) {
        return null;
    }
    return [Number(m[1]), Number(m[2]), Number(m[3])];
}

function compareVersions(a, b) {
    const va = parseVersion(a);
    const vb = parseVersion(b);
    if (!va || !vb) {
        return null;
    }
    for (let i = 0; i < 3; i += 1) {
        if (va[i] > vb[i]) {
            return 1;
        }
        if (va[i] < vb[i]) {
            return -1;
        }
    }
    return 0;
}

function buildVersionNote() {
    if (!deviceVersion || !releaseVersion) {
        return "";
    }
    const cmp = compareVersions(releaseVersion, deviceVersion);
    if (cmp === 0) {
        return "same version";
    }
    if (cmp === null) {
        return "version compare unavailable";
    }
    if (cmp < 0) {
        return "target is lower (downgrade)";
    }
    return "target is newer";
}

function setFirmwareLoaded(buffer, name, source) {
    firmware = buffer;
    firmwareName = name || "gps_laptimer.bin";
    firmwareSource = source || "unknown";
    $fileInfo.textContent = firmwareName + " (" + formatBytes(firmware.byteLength) + "), source: " + firmwareSource;
}

function updateConnectionDot(connected) {
    if (!connected) {
        $statusDot.className = "status-dot disconnected";
        return;
    }
    $statusDot.className = transferring ? "status-dot updating" : "status-dot connected";
}

// --- BLE Connection ---
async function connect() {
    if (!navigator.bluetooth) {
        alert("Web Bluetooth is not supported in this browser. Use Chrome or Edge.");
        return;
    }

    try {
        log("Scanning for LAPTIMER-OTA...");
        bleDevice = await navigator.bluetooth.requestDevice({
            filters: [{ namePrefix: "LAPTIMER" }],
            optionalServices: [SVC_UUID]
        });
        bleDevice.addEventListener("gattserverdisconnected", onDisconnected);

        log("Connecting to " + bleDevice.name + "...");
        const server = await bleDevice.gatt.connect();

        log("Getting OTA service...");
        const svc = await server.getPrimaryService(SVC_UUID);

        ctrlChar = await svc.getCharacteristic(CTRL_UUID);
        dataChar = await svc.getCharacteristic(DATA_UUID);
        statChar = await svc.getCharacteristic(STAT_UUID);

        await statChar.startNotifications();
        statChar.addEventListener("characteristicvaluechanged", onStatusNotify);

        setConnected(true);
        log("Connected. Requesting device version...");
        await writeControlWithRetry(new Uint8Array([CMD_VERSION]), "VERSION");
    } catch (e) {
        log("Error: " + e.message);
        setConnected(false);
    }
}

function disconnect() {
    if (bleDevice && bleDevice.gatt.connected) {
        bleDevice.gatt.disconnect();
    }
}

function onDisconnected() {
    log("Disconnected");
    ctrlChar = null;
    dataChar = null;
    statChar = null;
    deviceVersion = "";
    latestOtaState = STATE_IDLE;
    latestOtaErrorCode = 0;
    $devVersion.textContent = "--";

    if (transferring) {
        setTransferringUi(false);
        setProgressState("Disconnected during update", true);
        showResult("Connection lost during transfer", true);
    }

    setConnected(false);
    resetConfirmation();
}

function setConnected(connected) {
    updateConnectionDot(connected);
    $statusText.textContent = connected ? "Connected" : "Disconnected";
    $connectBtn.classList.toggle("hidden", connected);
    $disconnectBtn.classList.toggle("hidden", !connected);
    updateStartButton();
}

function updateStartButton() {
    const connected = !!ctrlChar;
    const hasFirmware = !!firmware;
    const confirmed = !!$confirmUpdate.checked;

    if (transferring) {
        $startBtn.disabled = true;
        $startBtn.textContent = "Updating...";
        return;
    }

    if (!hasFirmware) {
        $startBtn.disabled = true;
        if (releaseAutoState === "loading") {
            $startBtn.textContent = "Loading firmware...";
        } else {
            $startBtn.textContent = "Select local .bin file";
        }
        return;
    }

    if (!connected) {
        $startBtn.disabled = true;
        $startBtn.textContent = "Connect device first";
        return;
    }

    if (!confirmed) {
        $startBtn.disabled = true;
        $startBtn.textContent = "Check confirmation to enable update";
        return;
    }

    $startBtn.disabled = false;
    const note = buildVersionNote();
    if (releaseVersion && deviceVersion) {
        $startBtn.textContent =
            "Start Update " + deviceVersion + " -> " + releaseVersion + (note ? " (" + note + ")" : "");
    } else if (releaseVersion) {
        $startBtn.textContent = "Start Update to " + releaseVersion;
    } else {
        $startBtn.textContent = "Start Update";
    }
}

// --- Status Notification ---
function onStatusNotify(event) {
    const data = new Uint8Array(event.target.value.buffer);
    if (data.length < 3) {
        return;
    }

    const state = data[0];
    const progress = data[1];
    const errorCode = data[2];
    latestOtaState = state;
    latestOtaErrorCode = errorCode;

    let version = "";
    if (data.length >= 19) {
        const verBytes = data.slice(3, 19);
        const nullIdx = verBytes.indexOf(0);
        version = new TextDecoder().decode(verBytes.slice(0, nullIdx >= 0 ? nullIdx : 16));
    }

    if (version && state === STATE_IDLE) {
        deviceVersion = version;
        $devVersion.textContent = version;
        log("Device version: " + version);
        const note = buildVersionNote();
        if (note && note !== lastVersionNote) {
            log("Version note: " + note + " (update is still allowed)");
            lastVersionNote = note;
        }
        updateStartButton();
    }

    if (state === STATE_RECEIVING) {
        $progressCard.classList.remove("hidden");
        setProgressState("Step 3/4: Uploading browser -> device over BLE...", false);
        updateProgress(progress);
        return;
    }

    if (state === STATE_VALIDATING) {
        $progressCard.classList.remove("hidden");
        updateProgress(100);
        setProgressState("Step 4/4: Device validating firmware...", false);
        log("[STEP 4/4] Device validating firmware...");
        return;
    }

    if (state === STATE_COMPLETE) {
        setTransferringUi(false);
        updateConnectionDot(true);
        updateProgress(100);
        setProgressState("Complete: Device accepted firmware and will reboot.", false);
        log("OTA complete. Device accepted firmware and is rebooting.");
        showResult("Update successful! Device is rebooting.", false);
        resetConfirmation();
        return;
    }

    if (state === STATE_ERROR) {
        const msg = decodeOtaError(errorCode);
        setTransferringUi(false);
        setProgressState("Error: " + msg, true);
        log("OTA error: " + msg);
        showResult(msg, true);
        return;
    }

    if (state === STATE_IDLE && !transferring) {
        setProgressState("Ready", false);
    }
}

// --- Progress ---
function updateProgress(pct) {
    $progressFill.style.width = pct + "%";
    $progressPct.textContent = pct + "%";

    if (startTime <= 0) {
        return;
    }

    const elapsedSec = (Date.now() - startTime) / 1000;
    const bytesTotal = firmware ? firmware.byteLength : 0;
    const bytesSent = Math.round(bytesTotal * pct / 100);
    const kbps = elapsedSec > 0 ? (bytesSent / 1024 / elapsedSec).toFixed(1) : "0";
    const etaSec = pct > 0 ? Math.round(elapsedSec * (100 - pct) / pct) : 0;

    $transferred.textContent = formatBytes(bytesSent) + " / " + formatBytes(bytesTotal);
    $elapsed.textContent = formatTime(elapsedSec);
    $speed.textContent = kbps + " KB/s";
    $eta.textContent = etaSec > 0 ? formatTime(etaSec) : "--";
}

function formatBytes(bytes) {
    if (bytes < 1024) {
        return bytes + " B";
    }
    return (bytes / 1024 / 1024).toFixed(2) + " MB";
}

function formatTime(sec) {
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return m + ":" + (s < 10 ? "0" : "") + s;
}

// --- Firmware source ---
function onFileSelect() {
    const file = $fileInput.files[0];
    if (!file) {
        return;
    }

    $fileInfo.textContent = file.name + " (" + formatBytes(file.size) + ")";
    const reader = new FileReader();
    reader.onload = function () {
        firmware = reader.result;
        firmwareName = file.name;
        firmwareSource = "local file";
        releaseVersion = ""; // Manual file, version unknown.
        setFirmwareLoaded(firmware, firmwareName, firmwareSource);
        resetConfirmation();
        log("Loaded local file: " + file.name + " (" + formatBytes(firmware.byteLength) + ")");
    };
    reader.readAsArrayBuffer(file);
}

async function tryDownloadFirmware(url, sourceName, expectedSize) {
    log("[FW] Download attempt: " + sourceName + " (" + url + ")");
    const resp = await fetchWithTimeout(url, { cache: "no-store" }, 30000);
    if (!resp.ok) {
        throw new Error("HTTP " + resp.status + " from " + sourceName);
    }
    const buffer = await resp.arrayBuffer();
    if (expectedSize && buffer.byteLength !== expectedSize) {
        throw new Error(
            sourceName + " size mismatch (expected " + expectedSize + ", got " + buffer.byteLength + ")"
        );
    }
    return buffer;
}

async function checkRelease() {
    if ($retryReleaseBtn) {
        $retryReleaseBtn.disabled = true;
    }
    try {
        releaseAutoState = "loading";
        updateStartButton();
        log("Origin: " + window.location.origin);
        log("Checking latest release...");
        $releaseStatus.textContent = "Checking...";

        const resp = await fetchWithTimeout(
            "https://api.github.com/repos/" + GITHUB_REPO + "/releases/latest",
            { cache: "no-store" },
            15000
        );
        if (!resp.ok) {
            log("No releases found");
            releaseAutoState = "unavailable";
            $releaseStatus.textContent = "No releases available (HTTP " + resp.status + ")";
            $releaseCard.classList.remove("hidden");
            updateStartButton();
            return;
        }

        const rel = await resp.json();
        const asset = rel.assets.find(function (a) { return a.name === "gps_laptimer.bin"; })
            || rel.assets.find(function (a) { return a.name.endsWith(".bin"); });
        if (!asset) {
            log("No .bin asset in release");
            releaseAutoState = "failed";
            $releaseStatus.textContent = "No firmware in release";
            $releaseCard.classList.remove("hidden");
            updateStartButton();
            return;
        }

        releaseVersion = rel.tag_name;
        releaseAutoState = "ready";
        $releaseVer.textContent = rel.tag_name;
        $releaseDate.textContent = new Date(rel.published_at).toLocaleDateString();
        $releaseSize.textContent = formatBytes(asset.size);
        $releaseCard.classList.remove("hidden");

        log("Latest release: " + rel.tag_name + " (" + formatBytes(asset.size) + ")");
        $releaseStatus.textContent = "Step 1/4: Downloading firmware package to browser...";

        const attemptErrors = [];
        let fwBuffer = null;
        const sameOriginUrl = "./firmware/latest/gps_laptimer.bin";
        try {
            fwBuffer = await tryDownloadFirmware(sameOriginUrl, "same-origin cache", asset.size);
            setFirmwareLoaded(fwBuffer, asset.name, "same-origin cache");
            log("[FW] Downloaded from same-origin cache.");
        } catch (e1) {
            const err1 = formatError(e1);
            attemptErrors.push("same-origin: " + err1);
            log("[FW] same-origin cache failed: " + err1);
            try {
                fwBuffer = await tryDownloadFirmware(asset.browser_download_url, "github release", asset.size);
                setFirmwareLoaded(fwBuffer, asset.name, "github release");
                log("[FW] Downloaded from github release URL.");
            } catch (e2) {
                const err2 = formatError(e2);
                attemptErrors.push("github release: " + err2);
                throw new Error(attemptErrors.join(" | "));
            }
        }

        $releaseStatus.textContent =
            "Step 1/4 complete: firmware downloaded to browser (" + firmwareSource + "). "
            + "Next: connect device, confirm, then Start to upload browser -> device.";
        resetConfirmation();
        log("[STEP 1/4] Firmware downloaded to browser: " + formatBytes(firmware.byteLength));
    } catch (e) {
        releaseAutoState = "failed";
        log("Release check failed: " + formatError(e));
        $releaseStatus.textContent = "Failed to load release package. Select a local .bin file or retry.";
        $releaseCard.classList.remove("hidden");
        updateStartButton();
    } finally {
        if ($retryReleaseBtn) {
            $retryReleaseBtn.disabled = false;
        }
    }
}

// --- OTA Transfer ---
async function startOta() {
    if (!ctrlChar || !dataChar || !firmware || transferring) {
        return;
    }

    if (!$confirmUpdate.checked) {
        alert("Please check the confirmation box first.");
        return;
    }

    const size = firmware.byteLength;
    const summary = [
        "Start OTA update?",
        "",
        "Device: " + (deviceVersion || "(unknown)"),
        "Target: " + getTargetLabel(),
        "Version note: " + (buildVersionNote() || "not available"),
        "Size: " + formatBytes(size),
        "",
        "Do not power off during update.",
        "Flow: browser download -> BLE upload to device flash"
    ].join("\n");

    if (!window.confirm(summary)) {
        log("Update canceled by user before START command.");
        return;
    }

    startTime = Date.now();
    $progressCard.classList.remove("hidden");
    $resultCard.classList.add("hidden");
    resetProgressView();
    setProgressState("Step 2/4: Sending START command to device...", false);
    setTransferringUi(true);
    updateConnectionDot(true);
    log("[STEP 2/4] Starting BLE OTA upload to device: " + formatBytes(size));

    try {
        const startCmd = new Uint8Array(5);
        startCmd[0] = CMD_START;
        startCmd[1] = size & 0xFF;
        startCmd[2] = (size >> 8) & 0xFF;
        startCmd[3] = (size >> 16) & 0xFF;
        startCmd[4] = (size >> 24) & 0xFF;
        await writeControlWithRetry(startCmd, "START");
        log("[STEP 2/4] START sent to device, size=" + size);
        setProgressState("Step 2/4: Device preparing OTA partition...", false);

        const startWaitBegin = Date.now();
        while (Date.now() - startWaitBegin < 30000) {
            if (!ctrlChar) {
                throw new Error("Device disconnected before OTA start");
            }
            if (latestOtaState === STATE_RECEIVING) {
                break;
            }
            if (latestOtaState === STATE_ERROR) {
                throw new Error("START rejected: " + decodeOtaError(latestOtaErrorCode));
            }
            await new Promise(function (resolve) { setTimeout(resolve, 100); });
        }
        if (latestOtaState !== STATE_RECEIVING) {
            throw new Error("Timeout waiting for device to enter RECEIVING state");
        }
        log("[STEP 3/4] Device accepted START. Uploading browser -> device over BLE.");
        const dataMode = (dataChar.properties && dataChar.properties.write)
            ? "write-with-response"
            : "write-without-response";
        log("Data channel mode: " + dataMode + ", chunk=" + CHUNK_SIZE + " bytes");

        setProgressState("Step 3/4: Uploading browser -> device over BLE...", false);
        const bytes = new Uint8Array(firmware);
        let offset = 0;
        let sentChunks = 0;

        while (offset < size && transferring) {
            const end = Math.min(offset + CHUNK_SIZE, size);
            const chunk = bytes.slice(offset, end);
            let written = false;
            for (let attempt = 1; attempt <= WRITE_RETRY_MAX; attempt += 1) {
                try {
                    await writeDataChunk(chunk);
                    written = true;
                    break;
                } catch (err) {
                    if (attempt === WRITE_RETRY_MAX) {
                        throw err;
                    }
                    log("Chunk write retry " + attempt + "/" + WRITE_RETRY_MAX + ": " + formatError(err));
                    await sleep(WRITE_RETRY_DELAY_MS * attempt);
                }
            }
            if (!written) {
                throw new Error("Failed to write chunk");
            }

            offset = end;
            sentChunks += 1;

            if ((sentChunks % WRITE_YIELD_EVERY) === 0) {
                await sleep(0);
            }

            const localPct = Math.round(offset / size * 100);
            updateProgress(localPct);
        }

        if (!transferring) {
            log("Transfer aborted by user.");
            return;
        }

        log("[STEP 3/4] All chunks sent. Sending END...");
        setProgressState("Step 4/4: Upload complete, waiting device validation...", false);
        await writeControlWithRetry(new Uint8Array([CMD_END]), "END");
        log("[STEP 4/4] END sent. Waiting for device validation/complete notify...");
    } catch (e) {
        setTransferringUi(false);
        updateConnectionDot(true);
        setProgressState("Transfer failed", true);
        log("Transfer error: " + e.message);
        showResult("Transfer failed: " + e.message, true);
    }
}

async function abortOta() {
    setTransferringUi(false);
    updateConnectionDot(!!ctrlChar);
    setProgressState("Aborted by user", true);

    if (ctrlChar) {
        try {
            await writeControlWithRetry(new Uint8Array([CMD_ABORT]), "ABORT");
            log("Abort sent");
        } catch (e) {
            log("Abort send failed: " + e.message);
        }
    }

    showResult("Update aborted", true);
}

function showResult(msg, isError) {
    $resultCard.classList.remove("hidden");
    $resultMsg.className = isError ? "err" : "ok";
    $resultMsg.textContent = msg;
    updateStartButton();
}

// --- Init ---
$connectBtn.addEventListener("click", connect);
$disconnectBtn.addEventListener("click", disconnect);
$fileInput.addEventListener("change", onFileSelect);
if ($retryReleaseBtn) {
    $retryReleaseBtn.addEventListener("click", checkRelease);
}
$confirmUpdate.addEventListener("change", updateStartButton);
$startBtn.addEventListener("click", startOta);
$abortBtn.addEventListener("click", abortOta);

resetProgressView();
checkRelease();
