"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const number = (value, fallback = 0) => Number.isFinite(Number(value)) ? Number(value) : fallback;
  const clamp = (value, low, high) => Math.min(high, Math.max(low, value));
  const fixed = (value, digits = 1) => Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "—";
  const scenarioNames = { shell: "Shell contact", polar: "Polar belt", transition: "Transition" };
  const scenarioNotes = { shell: "Shell resting on the ground", polar: "Polar belt resting on the ground", transition: "Near the shell–belt contact boundary" };
  const pathNames = { hold: "Hold position", line: "Forward path", sideways: "Lateral path", curve: "Curved path", transition: "Contact transition" };
  const actuators = [ ["Shaft", "N·m"], ["Shell A", "N·m"], ["Shell B", "N·m"], ["Belt A", "m/s"], ["Belt B", "m/s"] ];
  const cameras = [
    { label: "Perspective", azimuth: 135, elevation: -23, distance: 0.90 },
    { label: "Front view", azimuth: 0, elevation: -12, distance: 0.90 },
    { label: "Side view", azimuth: 90, elevation: -12, distance: 0.90 },
    { label: "Top view", azimuth: 90, elevation: -85, distance: 1.10 },
  ];
  let state = null;
  let connected = false;
  let pending = false;
  let selectedPath = "hold";
  let selectedScenario = "shell";
  let cameraIndex = 0;
  let pollTimer = null;
  let streamTimer = null;
  let toastTimer = null;
  let commandError = "";
  let connectionError = "";
  let drag = null;
  let streamLoaded = false;
  let chartsNeedDraw = true;
  let lastChartTime = 0;
  const actuatorNodes = actuators.map(([name, unit]) => {
    const row = document.createElement("div");
    row.className = "actuator-row";
    const label = document.createElement("span");
    label.className = "actuator-name";
    label.textContent = name;
    const bar = document.createElement("div");
    bar.className = "actuator-bar";
    const fill = document.createElement("span");
    bar.append(fill);
    const value = document.createElement("span");
    value.className = "actuator-value";
    const amount = document.createElement("span");
    amount.textContent = "—";
    const suffix = document.createElement("span");
    suffix.textContent = unit;
    value.append(amount, suffix);
    row.append(label, bar, value);
    $("actuatorList").append(row);
    return { fill, amount };
  });

  function showToast(message) {
    clearTimeout(toastTimer);
    $("toast").textContent = message;
    $("toast").hidden = false;
    toastTimer = setTimeout(() => { $("toast").hidden = true; }, 3000);
  }

  function isFallen() {
    return !!state && (state.failed || /fall|fail|crash|terminated/i.test(state.status || ""));
  }

  function updateControls() {
    const unavailable = pending || !connected;
    document.querySelectorAll("button").forEach((button) => {
      if (["helpButton", "helpLink", "closeHelp", "helpDone", "retryButton"].includes(button.id)) return;
      button.disabled = unavailable;
    });
    ["controllerToggle", "beltToggle", "controllerMode"].forEach((id) => { $(id).disabled = unavailable; });
    $("synchronizationToggle").disabled = unavailable || state?.controller_mode !== "transition_aware" || state?.controller_enabled === false;
    if (isFallen()) {
      $("playButton").disabled = true;
      $("trajectoryButton").disabled = true;
      document.querySelectorAll("[data-push]").forEach((button) => { button.disabled = true; });
    }
    $("pathDistance").disabled = selectedPath === "hold";
    $("pathDuration").disabled = selectedPath === "hold";
  }

  function showErrors() {
    const renderError = state?.server?.render_error;
    const physicsError = state?.server?.physics_error;
    const error = commandError || connectionError || physicsError || renderError || state?.last_error;
    $("errorBanner").hidden = !error;
    $("errorText").textContent = error ? String(error) : "";
    $("retryButton").hidden = !connectionError && !renderError;
  }

  function updateStreamStatus() {
    const image = $("simulationImage");
    const server = state?.server || {};
    const stale = number(server.frame_age_s, 0) > 4;
    const ready = connected && (streamLoaded || image.naturalWidth > 0) && !server.render_error && !stale;
    $("streamPlaceholder").hidden = ready;
    if (!connected) {
      $("streamMessage").textContent = state ? "Connection interrupted" : "Connecting to the simulator";
      $("streamHint").textContent = state ? "Waiting to reconnect to the live simulation." : "The live view will appear here.";
    } else if (server.render_error || stale) {
      $("streamMessage").textContent = "Live view unavailable";
      $("streamHint").textContent = server.render_error ? "Check the renderer message above." : "Waiting for a fresh frame from the simulator.";
    } else {
      $("streamMessage").textContent = "Loading the live view";
      $("streamHint").textContent = "Rendering the current simulation state.";
    }
  }

  async function fetchJSON(url, options = {}) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 6000);
    try {
      const response = await fetch(url, { ...options, cache: "no-store", signal: controller.signal });
      let payload;
      try { payload = await response.json(); } catch (_) { throw new Error("The simulator returned an unreadable response."); }
      if (!response.ok || payload?.ok === false) {
        throw new Error(payload?.error || payload?.detail || `Simulator request failed (${response.status}).`);
      }
      return payload;
    } finally { clearTimeout(timeout); }
  }

  async function command(action, values = {}) {
    if (pending || !connected) return null;
    pending = true;
    commandError = "";
    updateControls();
    try {
      const result = await fetchJSON("/api/command", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action, ...values }),
      });
      if (result.state) applyState(result.state);
      else if (typeof result.time_s === "number") applyState(result);
      await pollOnce();
      return result;
    } catch (error) {
      commandError = error.name === "AbortError" ? "The command timed out. Check the simulator connection." : error.message;
      return null;
    } finally {
      pending = false;
      updateControls();
      showErrors();
    }
  }

  async function pollOnce() {
    try {
      const response = await fetchJSON("/api/state");
      const freshState = response.state || response;
      if (typeof freshState.time_s !== "number") throw new Error("Waiting for a valid simulation state.");
      connected = true;
      connectionError = "";
      applyState(freshState);
    } catch (error) {
      connected = false;
      connectionError = error.name === "AbortError" ? "Simulator connection timed out. Retrying…" : `Cannot reach the simulator. ${error.message}`;
      $("connection").className = "connection offline";
      $("connection").lastElementChild.textContent = "Disconnected";
      $("runStatus").textContent = "Connection interrupted";
      $("viewState").className = "view-state";
      $("viewState").replaceChildren(document.createElement("i"), document.createTextNode("DISCONNECTED"));
      $("liveMark").className = "live-mark";
      $("runningDot").className = "small-dot";
      updateControls();
      updateStreamStatus();
      showErrors();
    }
  }

  async function pollLoop() {
    await pollOnce();
    clearTimeout(pollTimer);
    pollTimer = setTimeout(pollLoop, connected ? 150 : 1200);
  }

  function contactLabel(name) {
    const match = String(name || "").match(/(shell|belt)[_ ]?([ab])/i);
    if (match) return `${match[1].toLowerCase() === "shell" ? "Shell" : "Belt"} ${match[2].toUpperCase()}`;
    return String(name || "Ground contact")
      .replace(/contact/g, "").replace(/[_:]/g, " ").replace(/\s+/g, " ").trim()
      .replace(/\bshell\b/gi, "Shell").replace(/\bbelt\b/gi, "Belt");
  }

  function applyState(nextState) {
    const firstState = state === null;
    const newRun = firstState || nextState.run_id !== state.run_id;
    state = nextState;
    connected = true;
    const fallen = isFallen();
    const running = !!state.running;
    const tilt = number(state.tilt_deg);
    const motionSettled = typeof state.motion_settled === "boolean" ? state.motion_settled : tilt < 0.5 && number(state.angular_speed_rad_s) < 0.08 && number(state.tracking_velocity_error_m_s) < 0.03;
    const following = state.trajectory?.kind && state.trajectory.kind !== "hold" && number(state.trajectory.elapsed_s) < number(state.trajectory.duration_s);
    const xy = state.position_xy_m || [];
    $("connection").className = "connection online";
    $("connection").lastElementChild.textContent = "Simulator connected";
    $("tiltValue").textContent = fixed(state.tilt_deg, 2);
    $("tiltDetail").textContent = `Roll ${fixed(state.roll_deg, 2)}° · Pitch ${fixed(state.pitch_deg, 2)}°`;
    $("tiltStatus").textContent = fallen ? "Balance lost" : !state.controller_enabled ? "Control off" : !running ? "Paused" : !motionSettled ? "Recovering" : following ? "Tracking" : "Balanced";
    $("tiltStatus").className = `metric-status ${fallen ? "fallen" : !motionSettled || !state.controller_enabled ? "warning" : "balanced"}`;
    $("trackingValue").textContent = fixed(number(state.tracking_error_m) * 1000, 1);
    $("positionDetail").textContent = `X ${fixed(number(xy[0]) * 1000, 1)} · Y ${fixed(number(xy[1]) * 1000, 1)} mm`;
    $("timeValue").textContent = fixed(state.time_s, 2);
    $("runStatus").textContent = fallen ? "Reset to start a new run" : running ? !motionSettled && tilt < 0.5 ? "Correcting motion" : "Simulation running" : "Simulation paused";
    $("runningDot").className = `small-dot ${running ? "active" : "paused"}`;
    $("liveMark").className = `live-mark${running ? " active" : ""}`;
    const visibleStatus = fallen ? "BALANCE LOST" : running ? "LIVE SIMULATION" : "PAUSED";
    $("viewState").className = `view-state ${fallen ? "fallen" : running ? "running" : "paused"}`;
    $("viewState").replaceChildren(document.createElement("i"), document.createTextNode(visibleStatus));
    $("playButton").innerHTML = running ? "Ⅱ <span>Pause</span>" : "▶ <span>Run</span>";
    $("playButton").setAttribute("aria-label", running ? "Pause simulation" : "Run simulation");
    $("failureBanner").hidden = !fallen;
    $("failureText").textContent = state.last_error || "Reset the simulation to try again.";
    const contacts = (state.contacts || []).filter((item) => number(item.normal_force_N) > 0.001);
    $("contactValue").textContent = contacts.length ? [...new Set(contacts.map((item) => contactLabel(item.name)))].join(" + ") : "No ground contact";
    $("contactDetail").textContent = contacts.length ? `${fixed(contacts.reduce((sum, item) => sum + number(item.normal_force_N), 0), 2)} N total normal force` : "No supporting contact force";
    $("controllerToggle").checked = state.controller_enabled !== false;
    $("beltToggle").checked = state.belt_enabled !== false;
    const advanced = state.controller_mode === "transition_aware";
    $("controllerMode").value = advanced ? "transition_aware" : "local_lqr";
    $("modeDescription").textContent = advanced ? "For contact transitions and repeated pushes." : "Baseline feedback around a local balance point.";
    $("synchronizationToggle").checked = state.synchronization_enabled !== false;
    const preparing = (state.synchronization?.targets || []).filter((target) => number(target.blend_weight) > 0.01);
    let synchronizationText = !state.controller_enabled ? "Incoming preparation paused: controller off." : !advanced ? "Original local LQR · incoming preparation inactive." : state.synchronization_enabled === false ? "Incoming drive preparation off." : "Ready for the next contact change.";
    if (state.controller_enabled && advanced && state.synchronization_enabled !== false && preparing.length) {
      const mismatch = Math.max(...preparing.map((target) => Math.hypot(...(target.actual_mismatch_to_outgoing_m_s || []).map((value) => number(value)))));
      synchronizationText = `${running ? "Preparing" : "Preview"}: ${preparing.map((target) => contactLabel(target.name)).join(" + ")} · mismatch ${fixed(mismatch, 3)} m/s`;
    }
    $("synchronizationStatus").textContent = synchronizationText;
    $("synchronizationStatus").className = `sync-status${preparing.length && state.controller_enabled && advanced && state.synchronization_enabled !== false ? " preparing" : ""}`;
    $("controllerNote").textContent = !state.controller_enabled ? "All drive commands are zero. Belt speed is held at zero with frictional contact." : !state.belt_enabled ? "Belt speeds held at zero; the belts still make frictional contact." : "Belt drives support balance when their surfaces contact the ground.";
    selectedScenario = state.scenario || selectedScenario;
    $("scenarioTag").textContent = scenarioNames[selectedScenario] || selectedScenario;
    $("scenarioNote").textContent = scenarioNotes[selectedScenario] || "";
    if (newRun && state.trajectory) {
      const trajectory = state.trajectory;
      const kind = trajectory.kind === "transition" ? "curve" : trajectory.kind;
      setPath(["hold", "line", "sideways", "curve"].includes(kind) ? kind : "hold");
      $("pathDistance").value = String(Number((number(trajectory.distance_m, 0.04) * 1000).toFixed(3)));
      $("pathDuration").value = String(number(trajectory.duration_s, 10));
      $("pathLateral").value = String(Number((number(trajectory.lateral_m, 0.02) * 1000).toFixed(3)));
    }
    document.querySelectorAll("[data-scenario]").forEach((button) => {
      button.classList.toggle("selected", button.dataset.scenario === selectedScenario);
      button.setAttribute("aria-pressed", button.dataset.scenario === selectedScenario ? "true" : "false");
    });
    if (state.camera) {
      const camera = state.camera;
      const match = cameras.findIndex((preset) =>
        Math.abs(number(camera.azimuth, Infinity) - preset.azimuth) < 0.5 &&
        Math.abs(number(camera.elevation, Infinity) - preset.elevation) < 0.5 &&
        Math.abs(number(camera.distance, Infinity) - preset.distance) < 0.025);
      if (match >= 0) {
        cameraIndex = match;
        $("cameraLabel").textContent = cameras[match].label;
      } else {
        cameraIndex = -1;
        $("cameraLabel").textContent = "Custom view";
      }
    }
    actuatorNodes.forEach((node, index) => {
      const value = number(state.inputs?.[index]);
      const bound = state.input_bounds?.[index];
      const limit = Array.isArray(bound) ? Math.max(...bound.map((v) => Math.abs(number(v)))) : Math.abs(number(bound, index < 3 ? 1 : 0.1));
      const ratio = clamp(value / (limit || 1), -1, 1);
      node.fill.style.left = `${ratio < 0 ? 50 + ratio * 50 : 50}%`;
      node.fill.style.width = `${Math.abs(ratio) * 50}%`;
      node.amount.textContent = fixed(value, 3);
    });
    const force = state.active_force_N || [];
    const netForce = Math.hypot(...force.map((value) => number(value)));
    const pushCount = Math.max(0, Math.round(number(state.active_push_count)));
    $("forceBadge").hidden = !running || netForce < 0.0001 && pushCount === 0;
    $("forceBadge").textContent = `${fixed(netForce, 2)} N NET${pushCount > 1 ? ` · ${pushCount} PUSHES` : ""}`;
    $("forceBadge").title = `World force: X ${fixed(force[0], 2)}, Y ${fixed(force[1], 2)}, Z ${fixed(force[2], 2)} N`;
    if (state.last_push?.force_N) {
      const push = state.last_push;
      const magnitude = Math.hypot(...push.force_N.map((value) => number(value)));
      $("lastPush").textContent = `Last: ${fixed(magnitude, 2)} N · ${fixed(push.duration_s, 2)} s`;
    } else { $("lastPush").textContent = "No push applied yet"; }
    const trajectory = state.trajectory || {};
    const elapsed = number(trajectory.elapsed_s);
    const duration = number(trajectory.duration_s);
    const progress = duration > 0 ? clamp(elapsed / duration, 0, 1) : 0;
    const holding = !trajectory.kind || trajectory.kind === "hold";
    $("trajectoryStatus").textContent = holding ? "Holding current position" : progress >= 1 ? `${pathNames[trajectory.kind] || "Path"} complete · holding end` : `${pathNames[trajectory.kind] || "Path"} in progress`;
    $("trajectoryElapsed").textContent = holding ? "" : `${fixed(elapsed, 1)} / ${fixed(duration, 0)} s`;
    $("trajectoryProgress").style.width = holding ? "0%" : `${progress * 100}%`;
    updateControls();
    updateStreamStatus();
    showErrors();
    chartsNeedDraw = true;
    if (firstState) {
      drawTilt();
      drawPath();
      chartsNeedDraw = false;
      lastChartTime = performance.now();
    }
  }

  function reset(scenario = selectedScenario) {
    const roll = clamp(number($("initialRoll").value), -10, 10);
    const pitch = clamp(number($("initialPitch").value), -10, 10);
    return command("reset", { scenario, initial_tilt_deg: [roll, pitch] });
  }

  async function push(direction, multiplier = 1) {
    if (pending || !connected || isFallen()) return;
    if (!state?.running) { showToast("Resume the simulation to apply a push."); return; }
    const strength = clamp(number($("pushStrength").value, 0.5), 0.1, 5) * multiplier;
    const duration = clamp(number($("pushDuration").value, 0.1), 0.05, 0.5);
    const vectors = { forward: [1, 0, 0], back: [-1, 0, 0], left: [0, -1, 0], right: [0, 1, 0] };
    const vector = Array.isArray(direction) ? direction : vectors[direction];
    const button = typeof direction === "string" ? document.querySelector(`[data-push="${direction}"]`) : null;
    button?.classList.add("is-pushing");
    await command("push", { force_N: vector.map((value) => value * strength), duration_s: duration });
    setTimeout(() => button?.classList.remove("is-pushing"), 150);
  }

  function setPath(kind) {
    selectedPath = kind;
    document.querySelectorAll("[data-path]").forEach((button) => {
      button.classList.toggle("selected", button.dataset.path === kind);
      button.setAttribute("aria-pressed", button.dataset.path === kind ? "true" : "false");
    });
    $("lateralField").hidden = kind !== "curve";
    $("trajectoryButton").innerHTML = `<span>▷</span> ${kind === "hold" ? "Hold position" : "Start trajectory"}`;
    updateControls();
  }

  function refreshStream() {
    clearTimeout(streamTimer);
    streamLoaded = false;
    $("simulationImage").src = `/stream.mjpg?t=${Date.now()}`;
  }

  $("simulationImage").addEventListener("load", () => { streamLoaded = true; updateStreamStatus(); });
  $("simulationImage").addEventListener("error", () => {
    streamLoaded = false;
    updateStreamStatus();
    clearTimeout(streamTimer);
    streamTimer = setTimeout(refreshStream, 1500);
  });
  $("resetButton").addEventListener("click", () => reset());
  $("playButton").addEventListener("click", () => command(state?.running ? "pause" : "resume"));
  $("controllerMode").addEventListener("change", async (event) => {
    const result = await command("control_mode", { mode: event.target.value });
    if (!result && state) $("controllerMode").value = state.controller_mode === "transition_aware" ? "transition_aware" : "local_lqr";
  });
  $("synchronizationToggle").addEventListener("change", async (event) => {
    const result = await command("synchronization", { enabled: event.target.checked });
    if (!result && state) $("synchronizationToggle").checked = state.synchronization_enabled !== false;
  });
  $("controllerToggle").addEventListener("change", async (event) => {
    const result = await command("controller", { enabled: event.target.checked });
    if (!result && state) event.target.checked = state.controller_enabled !== false;
  });
  $("beltToggle").addEventListener("change", async (event) => {
    const result = await command("belt", { enabled: event.target.checked });
    if (!result && state) event.target.checked = state.belt_enabled !== false;
  });
  $("pushStrength").addEventListener("input", () => {
    const value = number($("pushStrength").value);
    $("pushStrengthValue").innerHTML = `${fixed(value, 1)} <span>N</span>`;
    const percent = (value - 0.1) / 4.9 * 100;
    $("pushStrength").style.background = `linear-gradient(to right, #ceaa66 ${percent}%, #eaeceb ${percent}%)`;
  });
  document.querySelectorAll("[data-push]").forEach((button) => button.addEventListener("click", () => push(button.dataset.push)));
  document.querySelectorAll("[data-scenario]").forEach((button) => button.addEventListener("click", () => reset(button.dataset.scenario)));
  document.querySelectorAll("[data-path]").forEach((button) => button.addEventListener("click", () => setPath(button.dataset.path)));
  $("trajectoryButton").addEventListener("click", async () => {
    const result = await command("trajectory", {
      kind: selectedPath,
      duration_s: clamp(number($("pathDuration").value, 10), 5, 60),
      distance_m: clamp(number($("pathDistance").value, 40), 1, 100) / 1000,
      lateral_m: clamp(number($("pathLateral").value, 20), 1, 50) / 1000,
    });
    if (result) {
      if (selectedPath !== "hold" && !state?.running) await command("resume");
      showToast(selectedPath === "hold" ? "Holding the current position." : "Reference trajectory started.");
    }
  });
  $("cameraButton").addEventListener("click", async () => {
    const next = (cameraIndex + 1) % cameras.length;
    const { label, ...settings } = cameras[next];
    const result = await command("camera", settings);
    if (result) { cameraIndex = next; $("cameraLabel").textContent = label; }
  });
  $("saveButton").addEventListener("click", async () => {
    const result = await command("save");
    if (result) {
      const link = document.createElement("a");
      link.href = result.download_url || "/api/export";
      link.download = "sphere-lab-run.csv";
      document.body.append(link);
      link.click();
      link.remove();
      showToast("Downloading this run as CSV.");
    }
  });
  $("retryButton").addEventListener("click", () => {
    commandError = "";
    clearTimeout(pollTimer);
    refreshStream();
    pollLoop();
  });
  ["helpButton", "helpLink"].forEach((id) => $(id).addEventListener("click", () => $("helpDialog").showModal()));
  ["closeHelp", "helpDone"].forEach((id) => $(id).addEventListener("click", () => $("helpDialog").close()));
  $("helpDialog").addEventListener("click", (event) => {
    const rect = $("helpDialog").getBoundingClientRect();
    if (event.clientX < rect.left || event.clientX > rect.right || event.clientY < rect.top || event.clientY > rect.bottom) $("helpDialog").close();
  });

  document.addEventListener("keydown", (event) => {
    if (event.repeat || event.ctrlKey || event.altKey || event.metaKey || event.isComposing || $("helpDialog").open) return;
    const target = event.target;
    if (target.matches("input, select, textarea, [contenteditable='true']")) return;
    const keys = { w: "forward", s: "back", a: "left", d: "right" };
    const key = event.key.toLowerCase();
    if (keys[key]) { event.preventDefault(); push(keys[key]); }
    else if (event.code === "Space" && !target.matches("button")) { event.preventDefault(); if (!isFallen()) command(state?.running ? "pause" : "resume"); }
    else if (key === "r") { event.preventDefault(); reset(); }
  });

  const viewport = $("viewport");
  viewport.addEventListener("pointerdown", (event) => {
    if (event.button !== 0 || !connected || pending || isFallen() || !state?.running) return;
    viewport.focus({ preventScroll: true });
    viewport.setPointerCapture(event.pointerId);
    const rect = viewport.getBoundingClientRect();
    drag = { id: event.pointerId, x: event.clientX, y: event.clientY, left: event.clientX - rect.left, top: event.clientY - rect.top };
    event.preventDefault();
  });
  viewport.addEventListener("pointermove", (event) => {
    if (!drag || event.pointerId !== drag.id) return;
    const dx = event.clientX - drag.x;
    const dy = event.clientY - drag.y;
    const length = Math.hypot(dx, dy);
    $("dragAnnotation").hidden = length < 5;
    const line = $("dragLine");
    line.style.left = `${drag.left}px`;
    line.style.top = `${drag.top}px`;
    line.style.width = `${length}px`;
    line.style.transform = `rotate(${Math.atan2(dy, dx)}rad)`;
    $("dragTip").style.left = `${drag.left + dx}px`;
    $("dragTip").style.top = `${drag.top + dy}px`;
  });
  viewport.addEventListener("pointerup", (event) => {
    if (!drag || event.pointerId !== drag.id) return;
    const dx = event.clientX - drag.x;
    const dy = event.clientY - drag.y;
    const length = Math.hypot(dx, dy);
    drag = null;
    $("dragAnnotation").hidden = true;
    if (length >= 8) push([-dy / length, dx / length, 0], clamp(length / 100, 0.1, 1));
  });
  viewport.addEventListener("pointercancel", () => { drag = null; $("dragAnnotation").hidden = true; });

  function prepareCanvas(canvas) {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const targetWidth = Math.round(width * ratio);
    const targetHeight = Math.round(height * ratio);
    if (canvas.width !== targetWidth || canvas.height !== targetHeight) { canvas.width = targetWidth; canvas.height = targetHeight; }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.font = "9px -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif";
    return { ctx, width, height };
  }

  function drawGrid(ctx, bounds, xLabel, yLabel) {
    const { left, top, width, height } = bounds;
    ctx.strokeStyle = "#edf1ef";
    ctx.lineWidth = 1;
    ctx.fillStyle = "#a6b1b4";
    for (let i = 0; i <= 4; i++) {
      const fraction = i / 4;
      const x = left + width * fraction;
      const y = top + height * (1 - fraction);
      ctx.beginPath();ctx.moveTo(x, top);ctx.lineTo(x, top + height);ctx.stroke();
      ctx.beginPath();ctx.moveTo(left, y);ctx.lineTo(left + width, y);ctx.stroke();
      ctx.textAlign = "center";
      ctx.fillText(xLabel(fraction), x, top + height + 16);
      ctx.textAlign = "right";
      ctx.fillText(yLabel(fraction), left - 7, y + 3);
    }
  }

  function drawTilt() {
    const { ctx, width, height } = prepareCanvas($("tiltChart"));
    const bounds = { left: 30, top: 9, width: Math.max(width - 43, 20), height: Math.max(height - 31, 20) };
    const history = state?.history || {};
    const times = history.time_s || [];
    const tilts = history.tilt_deg || [];
    const end = Math.max(number(times[times.length - 1]), 1);
    const start = Math.max(0, end - 20);
    const indices = times.map((time, i) => [time, i]).filter(([time]) => time >= start);
    const maxTilt = Math.max(2, ...indices.map(([, i]) => number(tilts[i])));
    const max = maxTilt <= 2 ? 2 : Math.ceil(maxTilt * 1.15 / 2) * 2;
    drawGrid(ctx, bounds, (v) => fixed(start + (end - start) * v, 0), (v) => fixed(max * v, max < 4 ? 1 : 0));
    $("tiltEmpty").hidden = times.length > 1;
    if (times.length < 2) return;
    const x = (time) => bounds.left + (time - start) / Math.max(end - start, 0.001) * bounds.width;
    const y = (value) => bounds.top + bounds.height * (1 - value / max);
    ctx.save();ctx.beginPath();ctx.rect(bounds.left, bounds.top, bounds.width, bounds.height);ctx.clip();
    const forces = history.force_N || [];
    ctx.fillStyle = "#d8a6522b";
    indices.forEach(([time, i]) => {
      const force = forces[i];
      const active = Array.isArray(force) ? force.some((value) => Math.abs(number(value)) > 0.0001) : Math.abs(number(force)) > 0.0001;
      if (active) { const nextTime = number(times[i + 1], time + 0.04);ctx.fillRect(x(time), bounds.top, Math.max(2, x(nextTime) - x(time)), bounds.height); }
    });
    ctx.beginPath();
    indices.forEach(([time, i], index) => { if (index === 0) ctx.moveTo(x(time), y(number(tilts[i]))); else ctx.lineTo(x(time), y(number(tilts[i]))); });
    ctx.strokeStyle = "#287e6d";ctx.lineWidth = 1.8;ctx.lineJoin = "round";ctx.stroke();
    ctx.lineTo(x(indices[indices.length - 1][0]), bounds.top + bounds.height);
    ctx.lineTo(x(indices[0][0]), bounds.top + bounds.height);ctx.closePath();
    const gradient = ctx.createLinearGradient(0, bounds.top, 0, bounds.top + bounds.height);
    gradient.addColorStop(0, "#3c9b7b20");gradient.addColorStop(1, "#3c9b7b01");ctx.fillStyle = gradient;ctx.fill();ctx.restore();
  }

  function drawPath() {
    const { ctx, width, height } = prepareCanvas($("pathChart"));
    const bounds = { left: 34, top: 9, width: Math.max(width - 49, 20), height: Math.max(height - 31, 20) };
    const history = state?.history || {};
    const actual = (history.xy || []).filter((point) => Array.isArray(point) && point.length >= 2);
    const reference = (history.reference_xy || []).filter((point) => Array.isArray(point) && point.length >= 2);
    const all = actual.concat(reference);
    let minX = -10, maxX = 10, minY = -10, maxY = 10;
    if (all.length) {
      minX = Math.min(...all.map((point) => number(point[0]) * 1000));
      maxX = Math.max(...all.map((point) => number(point[0]) * 1000));
      minY = Math.min(...all.map((point) => number(point[1]) * 1000));
      maxY = Math.max(...all.map((point) => number(point[1]) * 1000));
      const centerX = (minX + maxX) / 2, centerY = (minY + maxY) / 2;
      const spanX = Math.max(20, maxX - minX) * 1.3, spanY = Math.max(20, maxY - minY) * 1.3;
      const unitsPerPixel = Math.max(spanX / bounds.width, spanY / bounds.height);
      minX = centerX - unitsPerPixel * bounds.width / 2;maxX = centerX + unitsPerPixel * bounds.width / 2;
      minY = centerY - unitsPerPixel * bounds.height / 2;maxY = centerY + unitsPerPixel * bounds.height / 2;
    }
    // X runs horizontally, Y vertically; both axes share the same physical scale.
    drawGrid(ctx, bounds, (v) => fixed(minX + (maxX - minX) * v, 0), (v) => fixed(minY + (maxY - minY) * v, 0));
    $("pathEmpty").hidden = actual.length > 1;
    const x = (value) => bounds.left + (value * 1000 - minX) / (maxX - minX) * bounds.width;
    const y = (value) => bounds.top + bounds.height * (1 - (value * 1000 - minY) / (maxY - minY));
    function path(points, color, dashed) {
      if (!points.length) return;
      ctx.beginPath();ctx.strokeStyle = color;ctx.lineWidth = dashed ? 1.5 : 1.9;ctx.setLineDash(dashed ? [4, 4] : []);
      points.forEach((point, i) => { if (i === 0) ctx.moveTo(x(number(point[0])), y(number(point[1])));else ctx.lineTo(x(number(point[0])), y(number(point[1]))); });
      ctx.stroke();ctx.setLineDash([]);
    }
    ctx.save();ctx.beginPath();ctx.rect(bounds.left, bounds.top, bounds.width, bounds.height);ctx.clip();
    path(reference, "#acb9bb", true);
    path(actual, "#287e6d", false);
    if (actual.length) {
      const first = actual[0], last = actual[actual.length - 1];
      ctx.beginPath();ctx.arc(x(number(first[0])), y(number(first[1])), 3, 0, Math.PI * 2);ctx.fillStyle = "#fff";ctx.fill();ctx.strokeStyle = "#85b1a1";ctx.lineWidth = 1.4;ctx.stroke();
      ctx.beginPath();ctx.arc(x(number(last[0])), y(number(last[1])), 3.2, 0, Math.PI * 2);ctx.fillStyle = "#287e6d";ctx.fill();
    }
    ctx.restore();
    ctx.fillStyle = "#9aabad";ctx.font = "8px -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif";
    ctx.textAlign = "right";ctx.fillText("X", width - 1, height - 3);ctx.textAlign = "left";ctx.fillText("Y", 2, 9);
  }

  function paintCharts(timestamp) {
    if (chartsNeedDraw && timestamp - lastChartTime > 180) {
      drawTilt();drawPath();chartsNeedDraw = false;lastChartTime = timestamp;
    }
    requestAnimationFrame(paintCharts);
  }
  if (typeof ResizeObserver !== "undefined") new ResizeObserver(() => { chartsNeedDraw = true; }).observe($("tiltChart").parentElement);
  window.addEventListener("resize", () => { chartsNeedDraw = true; });
  document.addEventListener("visibilitychange", () => { if (!document.hidden) { chartsNeedDraw = true; updateStreamStatus(); } });
  setPath("hold");
  updateControls();
  requestAnimationFrame(paintCharts);
  pollLoop();
})();
