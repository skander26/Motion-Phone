// PhoneMotion - Phase 2 Client (Android Chrome)
// Generic Sensor API (60 Hz) + DeviceMotion fallback, Joy-Con touch controls, and live settings sync.

const TARGET_FREQUENCY_HZ = 60;
const SEND_INTERVAL_MS = 1000 / TARGET_FREQUENCY_HZ;
const PING_INTERVAL_MS = 500;
const GENERIC_WATCHDOG_MS = 700;
const SCROLL_REPEAT_MS = 80;
const DEG_TO_RAD = Math.PI / 180;

const connectBtn = document.getElementById("connect-btn");
const stopBtn = document.getElementById("stop-btn");
const pauseBtn = document.getElementById("pause-btn");
const calibrateBtn = document.getElementById("calibrate-btn");
const btnLeft = document.getElementById("btn-left");
const btnRight = document.getElementById("btn-right");
const btnScrollUp = document.getElementById("btn-scroll-up");
const btnScrollDown = document.getElementById("btn-scroll-down");

const stateBadge = document.getElementById("state-badge");
const sampleRateEl = document.getElementById("sample-rate");
const rttLatencyEl = document.getElementById("rtt-latency");
const sensorSourceEl = document.getElementById("sensor-source");
const gyroUnitEl = document.getElementById("gyro-unit");
const errorMsg = document.getElementById("error-msg");
const calibBanner = document.getElementById("calib-banner");

const sliderSensitivity = document.getElementById("slider-sensitivity");
const sliderSmoothing = document.getElementById("slider-smoothing");
const sliderDeadzone = document.getElementById("slider-deadzone");
const selectXAxis = document.getElementById("select-x-axis");
const selectYAxis = document.getElementById("select-y-axis");
const toggleInvertX = document.getElementById("toggle-invert-x");
const toggleInvertY = document.getElementById("toggle-invert-y");

const valSensitivity = document.getElementById("val-sensitivity");
const valSmoothing = document.getElementById("val-smoothing");
const valDeadzone = document.getElementById("val-deadzone");

const valueEls = {
  gx: document.getElementById("val-gx"),
  gy: document.getElementById("val-gy"),
  gz: document.getElementById("val-gz"),
  ax: document.getElementById("val-ax"),
  ay: document.getElementById("val-ay"),
  az: document.getElementById("val-az"),
  roll: document.getElementById("val-roll"),
  pitch: document.getElementById("val-pitch"),
  yaw: document.getElementById("val-yaw"),
};

const sensorData = {
  type: "motion",
  t: 0,
  gx: 0,
  gy: 0,
  gz: 0,
  ax: 0,
  ay: 0,
  az: 0,
};

let ws = null;
let isConnected = false;
let controlState = "active";

let gyroSensor = null;
let accelSensor = null;
let hasGyroReadings = false;
let hasGenericReading = false;
let hasAnyReading = false;
let fallbackActive = false;
let hasRotationRate = false;
let hasMotionEvents = false;
let prevOri = null;

let sampleCount = 0;
let rateIntervalId = null;
let motionLoopId = null;
let pingLoopId = null;
let calibHideTimeoutId = null;
let genericWatchdogId = null;
let sensorCheckTimeoutId = null;
let diagTimeoutId = null;
let uiLoopId = null;
let scrollIntervalId = null;

function round3(value) {
  return typeof value === "number" && Number.isFinite(value)
    ? Math.round(value * 1000) / 1000
    : 0;
}

function pickAxis(primary, secondary, axis) {
  if (primary && typeof primary[axis] === "number" && Number.isFinite(primary[axis])) {
    return primary[axis];
  }
  if (secondary && typeof secondary[axis] === "number" && Number.isFinite(secondary[axis])) {
    return secondary[axis];
  }
  return 0;
}

function sendJson(payload) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return false;
  ws.send(JSON.stringify(payload));
  return true;
}

function showError(message) {
  errorMsg.textContent = message;
  errorMsg.classList.remove("hidden");
}

function clearError() {
  errorMsg.textContent = "";
  errorMsg.classList.add("hidden");
}

function showInsecureContextHint() {
  const origin = window.location.origin;
  showError(
    `Chrome blocks motion sensors on plain HTTP. Open chrome://flags/#unsafely-treat-insecure-origin-as-secure on your phone, enable it, add ${origin}, and relaunch Chrome (or run the server with --https).`
  );
}

function setControlPadDisabled(disabled) {
  pauseBtn.disabled = disabled;
  calibrateBtn.disabled = disabled;
  btnLeft.disabled = disabled;
  btnRight.disabled = disabled;
  btnScrollUp.disabled = disabled;
  btnScrollDown.disabled = disabled;
}

function showCalibrationBanner(message, activeCalibrating, hideAfterMs = 0) {
  if (calibHideTimeoutId !== null) {
    clearTimeout(calibHideTimeoutId);
    calibHideTimeoutId = null;
  }
  calibBanner.textContent = message;
  calibBanner.classList.remove("hidden");
  calibrateBtn.classList.toggle("is-calibrating", activeCalibrating);

  if (hideAfterMs > 0) {
    calibHideTimeoutId = setTimeout(() => {
      calibBanner.classList.add("hidden");
      calibHideTimeoutId = null;
    }, hideAfterMs);
  }
}

function hideCalibrationBanner() {
  if (calibHideTimeoutId !== null) {
    clearTimeout(calibHideTimeoutId);
    calibHideTimeoutId = null;
  }
  calibBanner.classList.add("hidden");
  calibrateBtn.classList.remove("is-calibrating");
}

function renderStateUi() {
  stateBadge.classList.remove(
    "badge-disconnected",
    "badge-connecting",
    "badge-active",
    "badge-paused",
    "badge-stopped"
  );
  pauseBtn.classList.remove("is-paused");
  stopBtn.classList.remove("is-stopped");

  if (!isConnected) {
    if (stateBadge.textContent === "Connecting...") {
      stateBadge.classList.add("badge-connecting");
    } else {
      stateBadge.textContent = "Disconnected";
      stateBadge.classList.add("badge-disconnected");
    }
    stopBtn.disabled = true;
    stopBtn.textContent = "EMERGENCY STOP";
    pauseBtn.textContent = "RECENTER / PAUSE";
    setControlPadDisabled(true);
    return;
  }

  stopBtn.disabled = false;

  if (controlState === "stopped") {
    stateBadge.textContent = "STOPPED";
    stateBadge.classList.add("badge-stopped");
    stopBtn.textContent = "RESUME FROM STOP";
    stopBtn.classList.add("is-stopped");
    pauseBtn.textContent = "STOPPED";
    setControlPadDisabled(true);
    stopHoldScroll();
  } else if (controlState === "paused") {
    stateBadge.textContent = "Paused";
    stateBadge.classList.add("badge-paused");
    stopBtn.textContent = "EMERGENCY STOP";
    pauseBtn.textContent = "RESUME CURSOR";
    pauseBtn.classList.add("is-paused");
    setControlPadDisabled(false);
  } else {
    stateBadge.textContent = "Active";
    stateBadge.classList.add("badge-active");
    stopBtn.textContent = "EMERGENCY STOP";
    pauseBtn.textContent = "RECENTER / PAUSE";
    setControlPadDisabled(false);
  }
}

function setConnectionUi(state) {
  if (state === "connected") {
    isConnected = true;
    connectBtn.textContent = "Disconnect";
    connectBtn.classList.add("is-connected");
    connectBtn.disabled = false;
    renderStateUi();
  } else if (state === "connecting") {
    isConnected = false;
    stateBadge.textContent = "Connecting...";
    connectBtn.textContent = "Connecting...";
    connectBtn.classList.remove("is-connected");
    connectBtn.disabled = true;
    renderStateUi();
  } else {
    isConnected = false;
    stateBadge.textContent = "Disconnected";
    connectBtn.textContent = "Connect";
    connectBtn.classList.remove("is-connected");
    connectBtn.disabled = false;
    sampleRateEl.textContent = "0.0 Hz";
    rttLatencyEl.textContent = "RTT: -- ms";
    sensorSourceEl.textContent = "Idle";
    valueEls.roll.textContent = "0.0";
    valueEls.pitch.textContent = "0.0";
    valueEls.yaw.textContent = "0.0";
    hideCalibrationBanner();
    renderStateUi();
  }
}

function updateDisplay() {
  valueEls.gx.textContent = sensorData.gx.toFixed(2);
  valueEls.gy.textContent = sensorData.gy.toFixed(2);
  valueEls.gz.textContent = sensorData.gz.toFixed(2);
  valueEls.ax.textContent = sensorData.ax.toFixed(2);
  valueEls.ay.textContent = sensorData.ay.toFixed(2);
  valueEls.az.textContent = sensorData.az.toFixed(2);
}

function startUiLoop() {
  if (uiLoopId !== null) return;
  const render = () => {
    updateDisplay();
    if (isConnected) {
      uiLoopId = requestAnimationFrame(render);
    } else {
      uiLoopId = null;
    }
  };
  uiLoopId = requestAnimationFrame(render);
}

function stopUiLoop() {
  if (uiLoopId !== null) {
    cancelAnimationFrame(uiLoopId);
    uiLoopId = null;
  }
}

function emitMotionSample() {
  hasAnyReading = true;
}

function startMotionLoop() {
  if (motionLoopId !== null) return;
  motionLoopId = setInterval(() => {
    if (!hasAnyReading || !ws || ws.readyState !== WebSocket.OPEN) return;
    // Never queue/batch frames in the socket buffer if the link stalls.
    if (ws.bufferedAmount > 0) return;

    sensorData.t = Date.now();
    ws.send(JSON.stringify(sensorData));
    sampleCount += 1;
  }, SEND_INTERVAL_MS);
}

function stopMotionLoop() {
  if (motionLoopId !== null) {
    clearInterval(motionLoopId);
    motionLoopId = null;
  }
}

function startPingLoop() {
  stopPingLoop();
  const sendPing = () => {
    sendJson({ type: "ping", client_t: performance.now() });
  };
  sendPing();
  pingLoopId = setInterval(sendPing, PING_INTERVAL_MS);
}

function stopPingLoop() {
  if (pingLoopId !== null) {
    clearInterval(pingLoopId);
    pingLoopId = null;
  }
}

function clearGenericWatchdog() {
  if (genericWatchdogId !== null) {
    clearTimeout(genericWatchdogId);
    genericWatchdogId = null;
  }
}

function stopGenericSensors() {
  clearGenericWatchdog();

  if (gyroSensor) {
    gyroSensor.onreading = null;
    gyroSensor.onerror = null;
    try {
      gyroSensor.stop();
    } catch (_) {
      // Ignore if sensor never started.
    }
    gyroSensor = null;
  }

  if (accelSensor) {
    accelSensor.onreading = null;
    accelSensor.onerror = null;
    try {
      accelSensor.stop();
    } catch (_) {
      // Ignore if sensor never started.
    }
    accelSensor = null;
  }
}

function wrapAngleDelta(delta) {
  if (delta > 180) return delta - 360;
  if (delta < -180) return delta + 360;
  return delta;
}

function onDeviceOrientation(event) {
  if (event.alpha === null && event.beta === null && event.gamma === null) return;

  const alpha = typeof event.alpha === "number" ? event.alpha : 0;
  const beta = typeof event.beta === "number" ? event.beta : 0;
  const gamma = typeof event.gamma === "number" ? event.gamma : 0;
  const now = performance.now();

  if (prevOri !== null && !hasRotationRate) {
    const dt = (now - prevOri.t) / 1000;
    if (dt > 0.004 && dt < 0.2) {
      sensorData.gx = round3((wrapAngleDelta(beta - prevOri.beta) * DEG_TO_RAD) / dt);
      sensorData.gy = round3((wrapAngleDelta(gamma - prevOri.gamma) * DEG_TO_RAD) / dt);
      sensorData.gz = round3((wrapAngleDelta(alpha - prevOri.alpha) * DEG_TO_RAD) / dt);
    }
  }

  prevOri = { t: now, alpha, beta, gamma };

  if (!hasMotionEvents) {
    emitMotionSample();
  }
}

function onDeviceMotion(event) {
  hasMotionEvents = true;

  // Convert DeviceMotion deg/s to rad/s when hardware rotationRate is available.
  const rot = event.rotationRate;
  const hasRotNums =
    rot &&
    (typeof rot.beta === "number" || typeof rot.gamma === "number" || typeof rot.alpha === "number");
  const hasNonZeroRot =
    hasRotNums &&
    (Math.abs(rot.beta || 0) > 0.05 || Math.abs(rot.gamma || 0) > 0.05 || Math.abs(rot.alpha || 0) > 0.05);

  if (hasNonZeroRot) {
    hasRotationRate = true;
  }

  if (hasRotationRate || (hasRotNums && prevOri === null)) {
    sensorData.gx = round3((rot && typeof rot.beta === "number" ? rot.beta : 0) * DEG_TO_RAD);
    sensorData.gy = round3((rot && typeof rot.gamma === "number" ? rot.gamma : 0) * DEG_TO_RAD);
    sensorData.gz = round3((rot && typeof rot.alpha === "number" ? rot.alpha : 0) * DEG_TO_RAD);
  }

  const accG = event.accelerationIncludingGravity;
  const acc = event.acceleration;
  sensorData.ax = round3(pickAxis(accG, acc, "x"));
  sensorData.ay = round3(pickAxis(accG, acc, "y"));
  sensorData.az = round3(pickAxis(accG, acc, "z"));

  emitMotionSample();
}

function startDeviceMotionFallback() {
  if (fallbackActive) return;
  fallbackActive = true;
  stopGenericSensors();

  const hasDm = typeof window.DeviceMotionEvent !== "undefined";
  const hasDo = typeof window.DeviceOrientationEvent !== "undefined";
  if (!hasDm && !hasDo) {
    sensorSourceEl.textContent = "Unavailable";
    showError("Neither Generic Sensor API nor DeviceMotion/Orientation is available in this browser.");
    return;
  }

  hasRotationRate = false;
  hasMotionEvents = false;
  prevOri = null;

  sensorSourceEl.textContent = "DeviceMotion";
  gyroUnitEl.textContent = "rad/s (conv)";

  if (hasDm) {
    window.addEventListener("devicemotion", onDeviceMotion);
  }
  if (hasDo) {
    window.addEventListener("deviceorientation", onDeviceOrientation);
  }
  sendDiagnostics("fallback_started");
}

function stopDeviceMotionFallback() {
  fallbackActive = false;
  hasRotationRate = false;
  hasMotionEvents = false;
  prevOri = null;
  window.removeEventListener("devicemotion", onDeviceMotion);
  window.removeEventListener("deviceorientation", onDeviceOrientation);
}

function startGenericSensors() {
  stopSensors();
  hasGyroReadings = false;
  hasGenericReading = false;
  hasAnyReading = false;
  fallbackActive = false;

  const hasGyroApi = "Gyroscope" in window;
  const hasAccelApi = "Accelerometer" in window;

  if (!hasGyroApi && !hasAccelApi) {
    startDeviceMotionFallback();
    return;
  }

  sensorSourceEl.textContent = "Generic Sensor";
  gyroUnitEl.textContent = "rad/s";

  let activeSensors = 0;
  let failedSensors = 0;

  const handleSensorError = (event) => {
    failedSensors += 1;
    const errName = event?.error?.name || "SensorError";
    if (errName === "SecurityError" || errName === "NotAllowedError" || failedSensors >= activeSensors) {
      startDeviceMotionFallback();
    }
  };

  try {
    if (hasGyroApi) {
      gyroSensor = new Gyroscope({ frequency: TARGET_FREQUENCY_HZ });
      gyroSensor.onreading = () => {
        hasGenericReading = true;
        hasGyroReadings = true;
        clearGenericWatchdog();
        sensorData.gx = round3(gyroSensor.x);
        sensorData.gy = round3(gyroSensor.y);
        sensorData.gz = round3(gyroSensor.z);
        emitMotionSample();
      };
      gyroSensor.onerror = handleSensorError;
      activeSensors += 1;
    }

    if (hasAccelApi) {
      accelSensor = new Accelerometer({ frequency: TARGET_FREQUENCY_HZ });
      accelSensor.onreading = () => {
        hasGenericReading = true;
        clearGenericWatchdog();
        sensorData.ax = round3(accelSensor.x);
        sensorData.ay = round3(accelSensor.y);
        sensorData.az = round3(accelSensor.z);
        if (!hasGyroReadings) {
          emitMotionSample();
        }
      };
      accelSensor.onerror = handleSensorError;
      activeSensors += 1;
    }

    if (gyroSensor) gyroSensor.start();
    if (accelSensor) accelSensor.start();

    // Fall back automatically if Generic Sensor API stays silent for 700 ms.
    genericWatchdogId = setTimeout(() => {
      if (!hasGenericReading) {
        startDeviceMotionFallback();
      }
    }, GENERIC_WATCHDOG_MS);
  } catch (_) {
    startDeviceMotionFallback();
  }
}

function stopSensors() {
  stopGenericSensors();
  stopDeviceMotionFallback();
}

function startRateCounter() {
  sampleCount = 0;
  let lastTick = performance.now();

  rateIntervalId = setInterval(() => {
    const now = performance.now();
    const elapsedSec = (now - lastTick) / 1000;
    const hz = elapsedSec > 0 ? sampleCount / elapsedSec : 0;
    sampleRateEl.textContent = `${hz.toFixed(1)} Hz`;
    sampleCount = 0;
    lastTick = now;
  }, 1000);
}

function stopRateCounter() {
  if (rateIntervalId !== null) {
    clearInterval(rateIntervalId);
    rateIntervalId = null;
  }
}

function readSettingsPayload() {
  return {
    type: "settings",
    sensitivity: Number(sliderSensitivity.value),
    smoothing: Number(sliderSmoothing.value),
    deadzone: Number(sliderDeadzone.value),
    xAxis: selectXAxis.value,
    yAxis: selectYAxis.value,
    invertX: toggleInvertX.checked,
    invertY: toggleInvertY.checked,
  };
}

function updateSettingsLabels() {
  valSensitivity.textContent = String(Number(sliderSensitivity.value));
  valSmoothing.textContent = Number(sliderSmoothing.value).toFixed(2);
  valDeadzone.textContent = Number(sliderDeadzone.value).toFixed(3);
}

function syncSettings() {
  updateSettingsLabels();
  sendJson(readSettingsPayload());
}

function applySyncedSettings(msg) {
  if (typeof msg.sensitivity === "number") sliderSensitivity.value = String(msg.sensitivity);
  if (typeof msg.smoothing === "number") sliderSmoothing.value = String(msg.smoothing);
  if (typeof msg.deadzone === "number") sliderDeadzone.value = String(msg.deadzone);
  if (msg.xAxis) selectXAxis.value = msg.xAxis;
  if (msg.yAxis) selectYAxis.value = msg.yAxis;
  if (typeof msg.invertX === "boolean") toggleInvertX.checked = msg.invertX;
  if (typeof msg.invertY === "boolean") toggleInvertY.checked = msg.invertY;
  updateSettingsLabels();
}

function stopHoldScroll() {
  if (scrollIntervalId !== null) {
    clearInterval(scrollIntervalId);
    scrollIntervalId = null;
  }
  btnScrollUp.classList.remove("is-pressed");
  btnScrollDown.classList.remove("is-pressed");
}

function bindClickButton(element, buttonName) {
  let pressed = false;

  const release = () => {
    if (!pressed) return;
    pressed = false;
    element.classList.remove("is-pressed");
    sendJson({ type: "button", button: buttonName, action: "up" });
  };

  element.addEventListener("pointerdown", (e) => {
    if (element.disabled) return;
    e.preventDefault();
    if (pressed) return;
    pressed = true;
    element.classList.add("is-pressed");
    if (element.setPointerCapture) {
      try {
        element.setPointerCapture(e.pointerId);
      } catch (_) {
        // Ignore if pointer capture is unsupported.
      }
    }
    sendJson({ type: "button", button: buttonName, action: "down" });
  });

  element.addEventListener("pointerup", release);
  element.addEventListener("pointercancel", release);
  element.addEventListener("lostpointercapture", release);
  element.addEventListener("contextmenu", (e) => e.preventDefault());
}

function bindScrollButton(element, buttonName) {
  const startScroll = (e) => {
    if (element.disabled) return;
    e.preventDefault();
    stopHoldScroll();
    element.classList.add("is-pressed");
    if (element.setPointerCapture) {
      try {
        element.setPointerCapture(e.pointerId);
      } catch (_) {
        // Ignore if pointer capture is unsupported.
      }
    }
    sendJson({ type: "button", button: buttonName, action: "scroll" });
    scrollIntervalId = setInterval(() => {
      if (element.disabled) {
        stopHoldScroll();
        return;
      }
      sendJson({ type: "button", button: buttonName, action: "scroll" });
    }, SCROLL_REPEAT_MS);
  };

  element.addEventListener("pointerdown", startScroll);
  element.addEventListener("pointerup", stopHoldScroll);
  element.addEventListener("pointercancel", stopHoldScroll);
  element.addEventListener("lostpointercapture", stopHoldScroll);
  element.addEventListener("contextmenu", (e) => e.preventDefault());
}

function sendDiagnostics(phase) {
  sendJson({
    type: "diag",
    phase,
    secureContext: Boolean(window.isSecureContext),
    protocol: window.location.protocol,
    sensorSource: sensorSourceEl.textContent,
    hasAnyReading,
    hasGyroApi: "Gyroscope" in window,
    hasAccelApi: "Accelerometer" in window,
    hasDeviceMotion: "DeviceMotionEvent" in window,
    hasDeviceOrientation: "DeviceOrientationEvent" in window,
    sampleCount,
  });
}

function disconnect() {
  stopHoldScroll();
  stopMotionLoop();
  stopPingLoop();
  stopSensors();
  stopRateCounter();
  stopUiLoop();

  if (sensorCheckTimeoutId !== null) {
    clearTimeout(sensorCheckTimeoutId);
    sensorCheckTimeoutId = null;
  }

  if (diagTimeoutId !== null) {
    clearTimeout(diagTimeoutId);
    diagTimeoutId = null;
  }

  if (ws) {
    ws.onopen = null;
    ws.onclose = null;
    ws.onerror = null;
    ws.onmessage = null;
    if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) {
      ws.close();
    }
    ws = null;
  }

  setConnectionUi("disconnected");
}

function openWebSocket() {
  const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${wsProtocol}//${window.location.host}/ws`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    controlState = "active";
    setConnectionUi("connected");
    startMotionLoop();
    startPingLoop();
    startRateCounter();
    startUiLoop();
    sendDiagnostics("connect");

    diagTimeoutId = setTimeout(() => {
      if (isConnected) {
        sendDiagnostics("after_1.5s");
      }
    }, 1500);

    sensorCheckTimeoutId = setTimeout(() => {
      if (isConnected && !hasAnyReading) {
        if (!window.isSecureContext) {
          showInsecureContextHint();
        } else {
          showError("Connected, but no sensor readings arrived. Check Chrome Motion Sensors site permissions.");
        }
      }
    }, 2000);
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (!msg || !msg.type) return;

      if (msg.type === "pong") {
        if (typeof msg.client_t === "number") {
          const rttMs = Math.max(0, performance.now() - msg.client_t);
          rttLatencyEl.textContent = `RTT: ${rttMs.toFixed(1)} ms`;
        }
        if (typeof msg.roll === "number") valueEls.roll.textContent = msg.roll.toFixed(1);
        if (typeof msg.pitch === "number") valueEls.pitch.textContent = msg.pitch.toFixed(1);
        if (typeof msg.yaw === "number") valueEls.yaw.textContent = msg.yaw.toFixed(1);
      } else if (msg.type === "settings_sync") {
        applySyncedSettings(msg);
      } else if (msg.type === "calibration") {
        if (msg.status === "calibrating") {
          const dur = Number(msg.duration || 2).toFixed(0);
          showCalibrationBanner(`Hold phone still (${dur}s): calibrating gyro bias...`, true);
        } else if (msg.status === "done") {
          showCalibrationBanner("Gyro bias calibrated and centered.", false, 1500);
        } else if (msg.status === "recentered") {
          showCalibrationBanner("Orientation recentered.", false, 1000);
        }
      } else if (msg.type === "state" && msg.state) {
        controlState = msg.state;
        renderStateUi();
        if (controlState === "stopped" && msg.reason) {
          showError(`Control STOPPED: ${msg.reason}. Press RESUME FROM STOP to re-enable.`);
        } else if (controlState === "active" && msg.reason) {
          clearError();
        }
      }
    } catch (_) {
      // Ignore malformed server messages.
    }
  };

  ws.onerror = () => {
    showError("WebSocket connection error. Verify both devices are on the same Wi-Fi and the server is running.");
  };

  ws.onclose = () => {
    if (isConnected || stateBadge.textContent === "Connecting...") {
      disconnect();
    }
  };
}

async function handleConnectClick() {
  if (isConnected) {
    disconnect();
    return;
  }

  clearError();

  // Request iOS-style permission only in a secure context, without blocking connection/fallback.
  let permissionWarning = "";
  if (
    window.isSecureContext &&
    typeof DeviceMotionEvent !== "undefined" &&
    typeof DeviceMotionEvent.requestPermission === "function"
  ) {
    try {
      const state = await DeviceMotionEvent.requestPermission();
      if (state !== "granted") {
        permissionWarning =
          "Browser did not grant Motion access. Allow Motion & Orientation in site settings (or install/trust /cert.pem over HTTPS).";
      }
    } catch (err) {
      permissionWarning = `Sensor permission note: ${err.message || err}`;
    }
  }

  const isLocalhost = location.hostname === "localhost" || location.hostname === "127.0.0.1";
  if (!window.isSecureContext && !isLocalhost) {
    showInsecureContextHint();
  } else if (permissionWarning) {
    showError(permissionWarning);
  }

  // Start sensors directly inside the click gesture so browsers allow sensor access immediately.
  startGenericSensors();
  setConnectionUi("connecting");
  openWebSocket();
}

connectBtn.addEventListener("click", handleConnectClick);

pauseBtn.addEventListener("click", () => {
  sendJson({ type: "control", command: "toggle_pause" });
});

calibrateBtn.addEventListener("click", () => {
  sendJson({ type: "control", command: "calibrate" });
});

stopBtn.addEventListener("click", () => {
  sendJson({ type: "control", command: "toggle_stop" });
});

bindClickButton(btnLeft, "left");
bindClickButton(btnRight, "right");
bindScrollButton(btnScrollUp, "scroll_up");
bindScrollButton(btnScrollDown, "scroll_down");

[sliderSensitivity, sliderSmoothing, sliderDeadzone].forEach((el) => {
  el.addEventListener("input", syncSettings);
});

[selectXAxis, selectYAxis, toggleInvertX, toggleInvertY].forEach((el) => {
  el.addEventListener("change", syncSettings);
});

updateSettingsLabels();
setConnectionUi("disconnected");
