let ws = null;
let limits = null;

const GAUGE_MIN = 0;
const GAUGE_MAX = 180;
const TOLERANCE = 5;

const jointDirty = Array(6).fill(false);
const jointTarget = Array(6).fill(null);

const editingJoints = new Set();
const jointEditUntil = Array(6).fill(0);

const PITCH_MIN = -90;
const PITCH_MAX = 90;
const PITCH_DEFAULT = -53;
const XYZ_AXES = ["x", "y", "z"];
const MEAS_X_FIXED_Z = 90;

let measurementSeries = [];
let measurementIndex = -1;

function lockJoint(jid, ms = 800) {
  jointEditUntil[jid] = Date.now() + ms;
}
function isLocked(jid) {
  return Date.now() < jointEditUntil[jid];
}

function wsUrl() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws`;
}

function clamp(v, lo, hi) {
  v = Number(v);
  lo = Number(lo);
  hi = Number(hi);
  if (Number.isNaN(v)) v = lo;
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

function isGripper(jid) {
  return Number(jid) === 5;
}

function getJointLimit(jid) {
  if (!limits) return null;
  return limits.joints.find((j) => Number(j.id) === Number(jid)) || null;
}

function jointRange(jid) {
  if (isGripper(jid)) return { lo: 0, hi: 90 };
  const lim = getJointLimit(jid);
  if (lim) return { lo: Number(lim.min), hi: Number(lim.max) };
  return { lo: 0, hi: 180 };
}

function clearDirty(jid) {
  jointDirty[jid] = false;
  jointTarget[jid] = null;
}

function clearAllDirty() {
  for (let i = 0; i < 6; i++) clearDirty(i);
}

function setWifiUI(online) {
  const dot = document.getElementById("wifi-dot");
  const txt = document.getElementById("wifi-state");
  if (!dot || !txt) return;
  dot.style.background = online ? "#22c55e" : "#ef4444";
  txt.textContent = online ? "Connected" : "Disconnected";
}

function setRobotUI(state) {
  const elText = document.getElementById("robot-state");
  const elDot = document.getElementById("robot-dot");
  if (!elText || !elDot) return;

  const label = state === "UNREFERENCED" ? "UNREF" : state;
  elText.textContent = label || "Unknown";

  if (state === "OPERATING" || state === "RUNNING") elDot.style.background = "#22c55e";
  else if (state === "STOPPING") elDot.style.background = "#f59e0b";
  else if (state === "READY" || state === "IDLE") elDot.style.background = "#fbbf24";
  else if (state === "READY_FOR_SYNC") elDot.style.background = "#38bdf8";
  else if (state === "UNREFERENCED" || state === "POSE_UNKNOWN") elDot.style.background = "#f59e0b";
  else if (state === "DISARMED") elDot.style.background = "#ef4444";
  else if (state === "ALARM" || state === "ERROR" || state === "STOPPED") elDot.style.background = "#ef4444";
  else elDot.style.background = "#888";
}

function setCanNodeUI(id) {
  const el = document.getElementById("can-node-id");
  if (!el) return;
  const n = Number(id);
  el.textContent = Number.isFinite(n) ? String(n) : "-";
}

async function loadControlLimits() {
  try {
    const r = await fetch("/api/limits");
    if (!r.ok) throw new Error("HTTP " + r.status);
    const data = await r.json();
    if (!data || !Array.isArray(data.joints) || data.joints.length < 6) throw new Error("Bad limits JSON");
    limits = data;
    applyLimitsToSliders();
  } catch (e) {
    limits = null;
    applyLimitsToSliders();
  }
}

function applyLimitsToSliders() {
  for (let jid = 0; jid < 6; jid++) {
    const input = document.getElementById(`joint-${jid}`);
    const targetEl = document.getElementById(`joint-value-${jid}`);
    if (!input || !targetEl) continue;

    const r = jointRange(jid);
    input.min = r.lo;
    input.max = r.hi;
    input.step = "1";

    const lo = r.lo;
    const hi = r.hi;

    if (jointDirty[jid] && jointTarget[jid] != null) {
      jointTarget[jid] = clamp(jointTarget[jid], lo, hi);
      input.value = jointTarget[jid];
      targetEl.textContent = `${jointTarget[jid]}°`;
    } else {
      const v = clamp(input.value, lo, hi);
      input.value = v;
      targetEl.textContent = `${Number(v).toFixed(0)}°`;
    }
  }
}

function installJointLocks() {
  for (let jid = 0; jid < 6; jid++) {
    const el = document.getElementById(`joint-${jid}`);
    if (!el) continue;

    el.addEventListener("pointerdown", () => {
      editingJoints.add(jid);
      lockJoint(jid, 2000);
    });
    el.addEventListener("pointerup", () => {
      editingJoints.delete(jid);
      lockJoint(jid, 1200);
    });
    el.addEventListener("pointercancel", () => {
      editingJoints.delete(jid);
      lockJoint(jid, 1200);
    });
    el.addEventListener("blur", () => {
      editingJoints.delete(jid);
      lockJoint(jid, 1200);
    });
    el.addEventListener("mouseleave", () => {
      editingJoints.delete(jid);
      lockJoint(jid, 1200);
    });

    el.addEventListener(
      "touchstart",
      () => {
        editingJoints.add(jid);
        lockJoint(jid, 2000);
      },
      { passive: true },
    );
    el.addEventListener(
      "touchend",
      () => {
        editingJoints.delete(jid);
        lockJoint(jid, 1200);
      },
      { passive: true },
    );
  }
}

function initControlWS() {
  ws = new WebSocket(wsUrl());

  ws.onopen = () => {
    setWifiUI(true);
    ws.send(JSON.stringify({ cmd: "sensors" }));
  };

  ws.onmessage = (event) => {
    let data;
    try {
      data = JSON.parse(event.data);
    } catch (e) {
      return;
    }

    if (data.state) setRobotUI(data.state);
    if (Object.prototype.hasOwnProperty.call(data, "can_node_id")) setCanNodeUI(data.can_node_id);

    if (data.sensors && Array.isArray(data.sensors)) {
      data.sensors.forEach((s) => {
        const jid = Number(s.id);
        const ang = Number(s.angle);
        if (Number.isNaN(jid) || Number.isNaN(ang)) return;

        const g = document.querySelector(`.gauge[data-id="${jid}"]`);
        if (g) updateGauge(g, ang);

        const input = document.getElementById(`joint-${jid}`);
        const targetEl = document.getElementById(`joint-value-${jid}`);
        const actEl = document.getElementById(`joint-act-${jid}`);
        if (!input || !targetEl || !actEl) return;

        const r = jointRange(jid);
        const lo = r.lo;
        const hi = r.hi;

        const actual = clamp(ang, lo, hi);
        actEl.textContent = `${actual.toFixed(0)}°`;

        const focused = document.activeElement === input;
        const blocked = isLocked(jid) || editingJoints.has(jid) || focused;

        if (!jointDirty[jid] && !blocked) {
          input.value = actual;
          targetEl.textContent = `${actual.toFixed(0)}°`;
          return;
        }

        const t = jointTarget[jid] != null ? jointTarget[jid] : Number(input.value);
        const tc = clamp(t, lo, hi);
        targetEl.textContent = `${tc.toFixed(0)}°`;

        if (jointDirty[jid] && jointTarget[jid] != null && !editingJoints.has(jid) && !focused) {
          if (Math.abs(actual - jointTarget[jid]) <= 1) clearDirty(jid);
        }
      });
    }
  };

  ws.onclose = () => {
    setWifiUI(false);
    setRobotUI("Offline");
    setCanNodeUI(null);
    setTimeout(initControlWS, 2000);
  };

  ws.onerror = () => {};
}

function setJointUI(jointId) {
  lockJoint(jointId, 2000);

  const input = document.getElementById(`joint-${jointId}`);
  const targetEl = document.getElementById(`joint-value-${jointId}`);
  if (!input || !targetEl) return;

  const r = jointRange(jointId);
  const lo = r.lo;
  const hi = r.hi;

  const v = clamp(input.value, lo, hi);
  input.value = v;

  jointDirty[jointId] = true;
  jointTarget[jointId] = v;

  targetEl.textContent = `${Number(v).toFixed(0)}°`;
}

function setJoint(jointId) {
  if (!ws || ws.readyState !== 1) {
    alert("WebSocket not connected.");
    return;
  }

  const input = document.getElementById(`joint-${jointId}`);
  const targetEl = document.getElementById(`joint-value-${jointId}`);
  if (!input || !targetEl) return;

  const r = jointRange(jointId);
  const lo = r.lo;
  const hi = r.hi;

  const v = clamp(input.value, lo, hi);
  input.value = v;

  jointDirty[jointId] = true;
  jointTarget[jointId] = v;

  targetEl.textContent = `${Number(v).toFixed(0)}°`;

  lockJoint(jointId, 2000);
  ws.send(JSON.stringify({ joint: jointId, angle: Number(v) }));
}

function goHome() {
  if (!ws || ws.readyState !== 1) {
    alert("WebSocket not connected.");
    return;
  }

  clearAllDirty();
  for (let i = 0; i < 6; i++) lockJoint(i, 1500);

  ws.send(JSON.stringify({ cmd: "home" }));
  ws.send(JSON.stringify({ cmd: "sensors" }));
  setTimeout(() => {
    if (ws && ws.readyState === 1) ws.send(JSON.stringify({ cmd: "sensors" }));
  }, 350);
  setTimeout(() => {
    if (ws && ws.readyState === 1) ws.send(JSON.stringify({ cmd: "sensors" }));
  }, 1400);
}

function getPitchDeg() {
  const el = document.getElementById("pitch");
  const v = el ? Number(el.value) : PITCH_DEFAULT;
  return clamp(v, PITCH_MIN, PITCH_MAX);
}

function setPitchDeg(v) {
  const pv = clamp(v, PITCH_MIN, PITCH_MAX);
  const num = document.getElementById("pitch");
  const rng = document.getElementById("pitch_range");
  if (num) num.value = pv;
  if (rng) rng.value = pv;
  localStorage.setItem("pitch_deg", String(pv));
}

function formatCmdNumber(v) {
  const n = Number(v);
  if (!Number.isFinite(n)) return "0";
  if (Number.isInteger(n)) return String(n);
  return n.toFixed(3).replace(/\.?0+$/, "");
}

function updateJogReadouts() {
  XYZ_AXES.forEach((axis) => {
    const input = document.getElementById(axis);
    const out = document.getElementById(`jog-${axis}-value`);
    if (!input || !out) return;
    out.textContent = formatCmdNumber(input.value);
  });
}

function setXYZValues(x, y, z) {
  const xEl = document.getElementById("x");
  const yEl = document.getElementById("y");
  const zEl = document.getElementById("z");
  if (!xEl || !yEl || !zEl) return;
  xEl.value = x;
  yEl.value = y;
  zEl.value = z;
  updateJogReadouts();
}

function jogAxis(axis, delta) {
  const input = document.getElementById(axis);
  if (!input) return;

  const current = Number(input.value);
  const next = (Number.isFinite(current) ? current : 0) + Number(delta);
  input.value = next;
  updateJogReadouts();

  const auto = document.getElementById("jog_auto_send");
  if (auto && auto.checked) moveXYZ();
}

function getMeasurementAxis() {
  const axis = String(document.getElementById("meas_axis")?.value || "z").toLowerCase();
  return axis === "x" || axis === "y" || axis === "z" ? axis : "z";
}

function axisLabel(axis) {
  return String(axis || "").toUpperCase();
}

function updateMeasurementAxisUI() {
  const axis = getMeasurementAxis();
  const up = axisLabel(axis);

  const axisValueLabel = document.getElementById("meas_axis_value_label");
  const startLabel = document.getElementById("meas_sweep_start_label");
  const stopLabel = document.getElementById("meas_sweep_stop_label");
  const stepLabel = document.getElementById("meas_sweep_step_label");
  const zValueEl = document.getElementById("meas_z");

  if (axisValueLabel) axisValueLabel.textContent = axis === "z" ? `Z sweep [mm]` : `Z fixed [mm]`;
  if (startLabel) startLabel.textContent = `${up} start [mm]`;
  if (stopLabel) stopLabel.textContent = `${up} stop [mm]`;
  if (stepLabel) stepLabel.textContent = `${up} step [mm]`;
  if (zValueEl) {
    if (axis === "x") {
      zValueEl.value = MEAS_X_FIXED_Z;
      zValueEl.disabled = true;
    } else {
      zValueEl.disabled = false;
    }
  }

  updateMeasurementStatus();
}

function buildMeasurementSeries(startValue, stopValue, stepValue) {
  const step = Math.abs(Number(stepValue));
  const start = Number(startValue);
  const stop = Number(stopValue);

  if (!Number.isFinite(start) || !Number.isFinite(stop) || !Number.isFinite(step) || step <= 0) {
    return [];
  }

  if (start === stop) return [start];

  const dir = stop >= start ? 1 : -1;
  const values = [];
  let value = start;
  while ((dir > 0 && value <= stop) || (dir < 0 && value >= stop)) {
    values.push(Number(value.toFixed(6)));
    value += dir * step;
  }

  if (values.length === 0 || Math.abs(values[values.length - 1] - stop) > 1e-9) {
    values.push(stop);
  }

  return values;
}

function invalidateMeasurementSeries() {
  measurementSeries = [];
  measurementIndex = -1;
  updateMeasurementStatus("Series needs prepare");
}

function currentMeasurementPoint() {
  if (!measurementSeries.length) return null;
  const axis = getMeasurementAxis();
  const xBase = Number(document.getElementById("meas_x")?.value);
  const yBase = Number(document.getElementById("meas_y")?.value);
  const zBase = axis === "x" ? MEAS_X_FIXED_Z : Number(document.getElementById("meas_z")?.value);
  const sweepValue = measurementSeries[measurementIndex];

  if (!Number.isFinite(xBase) || !Number.isFinite(yBase) || !Number.isFinite(zBase) || !Number.isFinite(sweepValue)) {
    return null;
  }

  const x = axis === "x" ? sweepValue : xBase;
  const y = axis === "y" ? sweepValue : yBase;
  const z = axis === "z" ? sweepValue : zBase;
  return { x, y, z };
}

function updateMeasurementStatus(message = "") {
  const statusEl = document.getElementById("measurement-status");
  const previewEl = document.getElementById("measurement-preview");

  const axis = getMeasurementAxis();
  const axisUp = axisLabel(axis);
  const xBase = Number(document.getElementById("meas_x")?.value);
  const yBase = Number(document.getElementById("meas_y")?.value);
  const zBase = axis === "x" ? MEAS_X_FIXED_Z : Number(document.getElementById("meas_z")?.value);
  if (previewEl) {
    if (!measurementSeries.length || !Number.isFinite(xBase) || !Number.isFinite(yBase) || !Number.isFinite(zBase)) {
      previewEl.textContent = "-";
    } else {
      previewEl.textContent = measurementSeries
        .map((sweepValue) => {
          const x = axis === "x" ? sweepValue : xBase;
          const y = axis === "y" ? sweepValue : yBase;
          const z = axis === "z" ? sweepValue : zBase;
          return `${formatCmdNumber(x)} ${formatCmdNumber(y)} ${formatCmdNumber(z)}`;
        })
        .join(" | ");
    }
  }

  if (!statusEl) return;

  if (!measurementSeries.length) {
    statusEl.textContent = message || "Series not prepared";
    return;
  }

  const idx = clamp(measurementIndex, 0, measurementSeries.length - 1);
  const point = currentMeasurementPoint();
  if (!point) {
    statusEl.textContent = message || "Series prepared but current point invalid";
    return;
  }
  const sweepValue = measurementSeries[idx];
  const detail = `Point ${idx + 1}/${measurementSeries.length} ready: X ${formatCmdNumber(point.x)}, Y ${formatCmdNumber(point.y)}, Z ${formatCmdNumber(point.z)} (${axisUp}=${formatCmdNumber(sweepValue)})`;
  statusEl.textContent = message ? `${message}. ${detail}` : detail;
}

function prepareMeasurementSeries() {
  const axis = getMeasurementAxis();
  const xBase = Number(document.getElementById("meas_x")?.value);
  const yBase = Number(document.getElementById("meas_y")?.value);
  const zBase = axis === "x" ? MEAS_X_FIXED_Z : Number(document.getElementById("meas_z")?.value);
  const startValue = Number(document.getElementById("meas_start")?.value);
  const stopValue = Number(document.getElementById("meas_stop")?.value);
  const stepValue = Number(document.getElementById("meas_step")?.value);

  if (!Number.isFinite(xBase) || !Number.isFinite(yBase) || !Number.isFinite(zBase) ||
      !Number.isFinite(startValue) || !Number.isFinite(stopValue) || !Number.isFinite(stepValue)) {
    alert("Measurement mode: enter valid numbers");
    return false;
  }

  measurementSeries = buildMeasurementSeries(startValue, stopValue, stepValue);
  if (!measurementSeries.length) {
    alert("Measurement mode: step must be non-zero");
    return false;
  }

  measurementIndex = 0;
  const startPoint = currentMeasurementPoint();
  if (!startPoint) {
    alert("Measurement mode: invalid start point");
    return false;
  }
  setXYZValues(startPoint.x, startPoint.y, startPoint.z);
  updateMeasurementStatus(`Series prepared (${axisLabel(axis)} sweep)`);
  return true;
}

function loadMeasurementStart() {
  if (!prepareMeasurementSeries()) return;
  updateMeasurementStatus("Start point loaded");
}

function sendMeasurementPoint() {
  if (!measurementSeries.length && !prepareMeasurementSeries()) return;
  const point = currentMeasurementPoint();
  if (!point) {
    alert("Measurement mode: invalid current point");
    return;
  }
  setXYZValues(point.x, point.y, point.z);
  if (moveXYZ()) {
    updateMeasurementStatus("Point sent");
  } else {
    updateMeasurementStatus("Point not sent");
  }
}

function nextMeasurementPoint() {
  if (!measurementSeries.length && !prepareMeasurementSeries()) return;
  const axisUp = axisLabel(getMeasurementAxis());
  if (measurementIndex >= measurementSeries.length - 1) {
    updateMeasurementStatus("End of series");
    alert(`Measurement mode: end of ${axisUp} series`);
    return;
  }

  const previousIndex = measurementIndex;
  measurementIndex += 1;
  const point = currentMeasurementPoint();
  if (!point) {
    alert("Measurement mode: invalid next point");
    return;
  }

  setXYZValues(point.x, point.y, point.z);
  if (moveXYZ()) {
    updateMeasurementStatus("Next point sent");
  } else {
    measurementIndex = previousIndex;
    const previousPoint = currentMeasurementPoint();
    if (previousPoint) setXYZValues(previousPoint.x, previousPoint.y, previousPoint.z);
    updateMeasurementStatus("Next point not sent");
  }
}

function reverseMeasurementSeries() {
  if (!measurementSeries.length && !prepareMeasurementSeries()) return;

  const len = measurementSeries.length;
  const prevIndex = clamp(measurementIndex, 0, len - 1);

  measurementSeries.reverse();
  measurementIndex = len - 1 - prevIndex;

  const point = currentMeasurementPoint();
  if (point) {
    setXYZValues(point.x, point.y, point.z);
  }

  updateMeasurementStatus("Series reversed");
}

function moveXYZ() {
  if (!ws || ws.readyState !== 1) {
    alert("WebSocket not connected.");
    return false;
  }

  const x = Number(document.getElementById("x")?.value);
  const y = Number(document.getElementById("y")?.value);
  const z = Number(document.getElementById("z")?.value);
  const pitch = getPitchDeg();

  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    alert("Please enter valid numbers");
    return false;
  }

  setPitchDeg(pitch);
  ws.send(JSON.stringify({ cmd: "move_xyz", x: x, y: y, z: z, pitch: pitch }));
  updateJogReadouts();
  return true;
}

function sendGcodeLine() {
  if (!ws || ws.readyState !== 1) {
    alert("WebSocket not connected.");
    return;
  }

  const input = document.getElementById("gcode_input");
  const line = input.value.trim();
  if (!line) {
    alert("Please enter a command");
    return;
  }

  ws.send(JSON.stringify({ cmd: "gcode_line", line: line }));
  input.value = "";
}

function stopGcode() {
  if (!ws || ws.readyState !== 1) return;
  ws.send(JSON.stringify({ cmd: "gcode_stop" }));
  ws.send(JSON.stringify({ cmd: "sensors" }));
}

function runFile(filename) {
  if (!ws || ws.readyState !== 1) {
    alert("WebSocket not connected.");
    return;
  }
  if (!confirm(`Run program "${filename}"?`)) return;
  ws.send(JSON.stringify({ cmd: "run_gcode", filename: filename }));
}

function disarmRobot() {
  if (!ws || ws.readyState !== 1) return;
  ws.send(JSON.stringify({ cmd: "disarm" }));
  ws.send(JSON.stringify({ cmd: "sensors" }));
  setRobotUI("DISARMED");
}

function armRobot() {
  if (!ws || ws.readyState !== 1) return;
  ws.send(JSON.stringify({ cmd: "arm" }));
  ws.send(JSON.stringify({ cmd: "sensors" }));
}

async function uploadFile() {
  const fileInput = document.getElementById("fileInput");
  const file = fileInput.files[0];
  if (!file) {
    alert("Please select a file first.");
    return;
  }
  if (!/\.(txt|gcode)$/i.test(file.name)) {
    alert("Only .txt or .gcode are allowed.");
    return;
  }

  const res = await fetch("/file/" + encodeURIComponent(file.name), {
    method: "PUT",
    headers: { "Content-Type": "text/plain" },
    body: await file.text(),
  });

  if (!res.ok) {
    alert("Error uploading file");
    return;
  }

  fileInput.value = "";
  loadFiles();
}

async function loadFiles() {
  const list = document.getElementById("files-body");
  if (!list) return;

  list.innerHTML = `<tr><td colspan="5" style="padding:8px;">Loading…</td></tr>`;

  try {
    const res = await fetch("/files");
    if (!res.ok) throw new Error("HTTP " + res.status);
    const files = await res.json();

    if (!files.length) {
      list.innerHTML = `<tr><td colspan="5" style="padding:8px;">No files found</td></tr>`;
      return;
    }

    list.innerHTML = "";
    for (const f of files) {
      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td style="border-bottom:1px solid #eee; padding:6px 8px;">
          <input class="file-chk" type="checkbox" value="${f.name}">
        </td>
        <td style="border-bottom:1px solid #eee; padding:6px 8px;">
          <a href="/file/${encodeURIComponent(f.name)}" target="_blank">${f.name}</a>
        </td>
        <td style="border-bottom:1px solid #eee; padding:6px 8px; text-align:right;">${f.size} B</td>
        <td style="border-bottom:1px solid #eee; padding:6px 8px; text-align:center;">
          <button style="background:#22c55e; width:auto; padding:5px 10px;" onclick="runFile('${f.name}')">▶ Run</button>
        </td>
        <td style="border-bottom:1px solid #eee; padding:6px 8px;">
          <button style="background:#ef4444; width:auto; padding:5px 10px;" data-del="${f.name}">Delete</button>
        </td>
      `;
      list.appendChild(tr);
    }

    document.querySelectorAll("button[data-del]").forEach((btn) => {
      btn.onclick = async () => {
        const name = btn.getAttribute("data-del");
        if (!confirm(`Delete "${name}"?`)) return;
        const r = await fetch("/file/" + encodeURIComponent(name), { method: "DELETE" });
        if (!r.ok) {
          alert("Delete failed");
          return;
        }
        loadFiles();
      };
    });
  } catch (e) {
    list.innerHTML = `<tr><td colspan="5" style="padding:8px;">Error: ${e}</td></tr>`;
  }
}

async function deleteSelected() {
  const checks = Array.from(document.querySelectorAll(".file-chk:checked"));
  if (!checks.length) {
    alert("Nothing selected.");
    return;
  }
  if (!confirm(`Delete ${checks.length} file(s)?`)) return;

  for (const c of checks) {
    await fetch("/file/" + encodeURIComponent(c.value), { method: "DELETE" });
  }
  loadFiles();
}

function gaugeColor(angle) {
  if (angle <= GAUGE_MIN + TOLERANCE || angle >= GAUGE_MAX - TOLERANCE) return "#ef4444";
  if (angle < 20 || angle > 160) return "#f59e0b";
  return "#22c55e";
}

function createGauge(el, id) {
  el.innerHTML = `
  <svg viewBox="0 0 100 50">
    <path d="M10 50 A40 40 0 0 1 90 50"
          fill="none"
          stroke="#334155"
          stroke-width="8"/>
    <path class="gauge-fill"
          d="M10 50 A40 40 0 0 1 90 50"
          fill="none"
          stroke="#22c55e"
          stroke-width="8"
          stroke-dasharray="126"
          stroke-dashoffset="126"/>
  </svg>
  <div class="gauge-value">--°</div>
  <div class="gauge-label">J${id}</div>
`;
}

function updateGauge(el, angle) {
  const fill = el.querySelector(".gauge-fill");
  const value = el.querySelector(".gauge-value");
  if (!fill || !value) return;

  const jid = Number(el.dataset.id);
  const gMax = isGripper(jid) ? 90 : GAUGE_MAX;

  const clamped = Math.max(GAUGE_MIN, Math.min(gMax, Number(angle)));
  const percent = clamped / gMax;
  const dash = 126 * (1 - percent);

  fill.style.strokeDashoffset = dash;
  fill.style.stroke = gaugeColor(clamped);
  value.textContent = clamped.toFixed(1) + "°";
}

function installMoveTools() {
  XYZ_AXES.forEach((axis) => {
    const el = document.getElementById(axis);
    if (!el) return;
    el.addEventListener("input", updateJogReadouts);
    el.addEventListener("change", updateJogReadouts);
  });

  ["meas_x", "meas_y", "meas_z", "meas_start", "meas_stop", "meas_step", "meas_axis"].forEach((id) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener("input", invalidateMeasurementSeries);
    el.addEventListener("change", invalidateMeasurementSeries);
  });

  const axisEl = document.getElementById("meas_axis");
  if (axisEl) {
    axisEl.addEventListener("input", updateMeasurementAxisUI);
    axisEl.addEventListener("change", updateMeasurementAxisUI);
  }
}

function bootControlPage() {
  const gauges = document.querySelectorAll(".gauge");
  gauges.forEach((g) => createGauge(g, g.dataset.id));

  setWifiUI(false);
  setRobotUI("Unknown");
  setCanNodeUI(null);

  // pitch init
  const savedPitch = Number(localStorage.getItem("pitch_deg"));
  setPitchDeg(Number.isFinite(savedPitch) ? savedPitch : PITCH_DEFAULT);

  const pitchNum = document.getElementById("pitch");
  const pitchRng = document.getElementById("pitch_range");
  if (pitchNum) {
    pitchNum.addEventListener("input", () => setPitchDeg(pitchNum.value));
    pitchNum.addEventListener("change", () => setPitchDeg(pitchNum.value));
  }
  if (pitchRng) {
    pitchRng.addEventListener("input", () => setPitchDeg(pitchRng.value));
    pitchRng.addEventListener("change", () => setPitchDeg(pitchRng.value));
  }

  installMoveTools();
  updateMeasurementAxisUI();
  updateJogReadouts();
  updateMeasurementStatus("Series not prepared");

  initControlWS();
  loadFiles();
  loadControlLimits();
  installJointLocks();
}

function initSettingsWS() {
  try {
    ws = new WebSocket(wsUrl());

    ws.onopen = () => {
      setWifiUI(true);
      ws.send(JSON.stringify({ cmd: "sensors" }));
    };

    ws.onmessage = (event) => {
      let data;
      try {
        data = JSON.parse(event.data);
      } catch (_) {
        return;
      }
      if (data.state) setRobotUI(data.state);
      if (Object.prototype.hasOwnProperty.call(data, "can_node_id")) setCanNodeUI(data.can_node_id);
    };

    ws.onclose = () => {
      setWifiUI(false);
      setRobotUI("Offline");
      setCanNodeUI(null);
      setTimeout(initSettingsWS, 2000);
    };

    ws.onerror = () => {
      setWifiUI(false);
    };
  } catch (_) {
    setWifiUI(false);
  }
}

function stopRobot() {
  if (!ws || ws.readyState !== 1) {
    alert("WebSocket not connected.");
    return;
  }
  ws.send(JSON.stringify({ cmd: "gcode_stop" }));
  setRobotUI("STOPPING");
}

function saveWiFi() {
  const ssid = document.getElementById("ssid")?.value?.trim();
  const pass = document.getElementById("pass")?.value?.trim();
  if (!ssid || !pass) {
    alert("SSID and password must be filled!");
    return;
  }

  fetch("/wifi_config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ ssid: ssid, password: pass }),
  })
    .then((r) => r.text())
    .then(() => alert("WiFi settings sent! A restart of the AP will be needed."))
    .catch((err) => alert("Error: " + err));
}

function resetWiFi() {
  if (!confirm("Do you really want to reset WiFi settings?")) return;

  fetch("/wifi_reset", { method: "POST" })
    .then((r) => r.text())
    .then(() => alert("WiFi settings reset. ESP32 will restart."))
    .catch((err) => alert("Error: " + err));
}

async function loadSettingsLimitsTable() {
  const body = document.getElementById("limitsTableBody");
  if (!body) return;

  body.innerHTML = `<tr><td colspan="5">Loading...</td></tr>`;

  try {
    const r = await fetch("/api/limits");
    if (!r.ok) throw new Error("GET /api/limits failed (HTTP " + r.status + ")");
    const data = await r.json();

    if (!data || !Array.isArray(data.joints) || data.joints.length < 1) {
      throw new Error("Bad limits JSON");
    }

    body.innerHTML = "";

    data.joints.forEach((j, idx) => {
      const name = j.name ?? `J${j.id ?? idx}`;
      const id = j.id ?? idx;
      const min = j.min ?? "";
      const max = j.max ?? "";
      const speed = j.speed ?? j.v ?? "";

      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td>${name}</td>
        <td>${id}</td>
        <td>${min}</td>
        <td>${max}</td>
        <td>${speed}</td>
      `;
      body.appendChild(tr);
    });
  } catch (e) {
    body.innerHTML = `<tr><td colspan="5">Error: ${String(e.message || e)}</td></tr>`;
  }
}

function bootSettingsPage() {
  setWifiUI(false);
  setRobotUI("Unknown");
  setCanNodeUI(null);
  initSettingsWS();
  loadSettingsLimitsTable();
}

function isControlPage() {
  return !!document.getElementById("joint-controls");
}

function isSettingsPage() {
  return !!document.getElementById("limitsTableBody") && !!document.getElementById("ssid");
}

document.addEventListener("DOMContentLoaded", () => {
  if (isControlPage()) {
    bootControlPage();
    return;
  }

  if (isSettingsPage()) {
    bootSettingsPage();
  }
});
