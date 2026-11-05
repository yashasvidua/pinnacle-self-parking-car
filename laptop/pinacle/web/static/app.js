// Pinacle dashboard (vanilla JS, no build step).
// The server (modes.py via AppContext) owns all mode rules; this file only forwards
// input and displays /api/status.
"use strict";

const $ = (id) => document.getElementById(id);
const CFG = { repeat_ms: 100, default_speed: 160, max_pwm: 255, min_pwm: 90 };

// ------------------------------------------------------------------ helpers
async function post(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
    keepalive: true,
  });
  let data = {};
  try { data = await res.json(); } catch (e) { /* ignore */ }
  return { status: res.status, data };
}

let toastTimer = null;
function toast(msg, ms = 2500) {
  const el = $("toast");
  el.textContent = msg;
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, ms);
}

// ------------------------------------------------------------------ manual drive
// Two input sources (pointer on the pad, keyboard). The pointer wins if both are active.
const drive = { pointer: null, keys: null, active: null, timer: null };
const speedEl = $("speed");

function speed() { return parseInt(speedEl.value, 10) || CFG.default_speed; }

function sendDrive(cmd) {
  post("/api/manual", { cmd, speed: speed() }).then(({ data }) => {
    if (data && data.aborted_auto) toast("Manual takeover: FIND & PARK aborted");
  }).catch(() => {});
}

function updateDrive() {
  const want = drive.pointer || drive.keys;
  if (want === drive.active) return;
  clearInterval(drive.timer);
  drive.timer = null;
  const wasDriving = drive.active !== null;
  drive.active = want;
  document.querySelectorAll(".pad .drive").forEach((b) =>
    b.classList.toggle("held", b.dataset.cmd === want));
  $("cmdIndicator").textContent = want ? want.replace("_", " ").toUpperCase() : "STOP";
  if (want) {
    sendDrive(want);
    // repeat while held: the server's dead-man stops the car if these stop arriving
    drive.timer = setInterval(() => sendDrive(want), CFG.repeat_ms);
  } else if (wasDriving) {
    post("/api/manual", { cmd: "stop" }).catch(() => {});
  }
}

function releaseAll() {
  drive.pointer = null;
  drive.keys = null;
  keysDown.clear();
  updateDrive();
}

// Pad buttons: pointer events cover mouse, pen and touch. Leaving the button counts as release.
document.querySelectorAll(".pad .drive").forEach((btn) => {
  const press = (e) => {
    e.preventDefault();
    drive.pointer = btn.dataset.cmd;
    updateDrive();
  };
  const release = (e) => {
    if (drive.pointer === btn.dataset.cmd) {
      drive.pointer = null;
      updateDrive();
    }
  };
  if (window.PointerEvent) {
    btn.addEventListener("pointerdown", press);
    ["pointerup", "pointerleave", "pointercancel", "lostpointercapture"].forEach((ev) =>
      btn.addEventListener(ev, release));
  } else {
    btn.addEventListener("touchstart", press, { passive: false });
    btn.addEventListener("mousedown", press);
    ["touchend", "touchcancel", "mouseup", "mouseleave"].forEach((ev) => btn.addEventListener(ev, release));
  }
  // extra safety on touch devices: a finished touch always releases
  btn.addEventListener("touchend", release);
  btn.addEventListener("touchcancel", release);
  btn.addEventListener("contextmenu", (e) => e.preventDefault());
});

// Keyboard: WASD + Q/E, several keys at once (e.g. W+A = forward-left).
const keysDown = new Set();
const DRIVE_KEYS = new Set(["w", "a", "s", "d", "q", "e"]);

function keysToCmd() {
  const k = (c) => keysDown.has(c);
  if (k("q") && !k("e")) return "rotate_left";
  if (k("e") && !k("q")) return "rotate_right";
  const fwd = k("w") && !k("s");
  const back = k("s") && !k("w");
  const left = k("a") && !k("d");
  const right = k("d") && !k("a");
  if (back) return left ? "back_left" : right ? "back_right" : "back";
  if (fwd) return left ? "left" : right ? "right" : "forward";
  if (left) return "left";
  if (right) return "right";
  return null;
}

function typingInField(e) {
  const t = e.target;
  return t && (t.tagName === "INPUT" && t.type !== "range" || t.tagName === "TEXTAREA");
}

window.addEventListener("keydown", (e) => {
  if (typingInField(e)) return;
  const key = e.key.toLowerCase();
  if (key === " " || e.code === "Space") {
    e.preventDefault();
    if (!e.repeat) estop();
    return;
  }
  if (!DRIVE_KEYS.has(key)) return;
  e.preventDefault();
  keysDown.add(key);
  drive.keys = keysToCmd();
  updateDrive();
});

window.addEventListener("keyup", (e) => {
  const key = e.key.toLowerCase();
  if (!DRIVE_KEYS.has(key)) return;
  keysDown.delete(key);
  drive.keys = keysToCmd();
  updateDrive();
});

window.addEventListener("blur", releaseAll);
document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });

speedEl.addEventListener("input", () => { $("speedVal").textContent = speedEl.value; });

// ------------------------------------------------------------------ e-stop / find & park
function estop() {
  drive.pointer = null; drive.keys = null; keysDown.clear();
  clearInterval(drive.timer); drive.timer = null; drive.active = null;
  document.querySelectorAll(".pad .drive.held").forEach((b) => b.classList.remove("held"));
  $("cmdIndicator").textContent = "STOP";
  post("/api/estop").then(() => toast("E-STOP sent")).catch(() => toast("E-STOP: server unreachable!"));
}
$("estop").addEventListener("pointerdown", (e) => { e.preventDefault(); estop(); });
$("estop").addEventListener("keydown", (e) => { if (e.key === "Enter") estop(); });

$("findPark").addEventListener("click", async () => {
  releaseAll();
  const { status, data } = await post("/api/find_park");
  if (status === 409) toast("Cannot start: " + (data.reason || "not ready"));
  else if (status === 200) toast("FIND & PARK started");
});

// ------------------------------------------------------------------ heartbeat
setInterval(() => { post("/api/heartbeat").catch(() => {}); }, 300);

// ------------------------------------------------------------------ status
function distColor(d) {
  if (d === null || d === undefined || d < 0) return "#7b8794";
  if (d < 8) return "#e5484d";
  if (d < 20) return "#f2a531";
  if (d < 50) return "#f5c542";
  return "#2fbf71";
}
const SENSORS = ["fl", "fr", "bl", "br", "l", "r"];

function setText(id, txt) { const el = $(id); if (el.textContent !== txt) el.textContent = txt; }
function setHTML(id, html) { const el = $(id); if (el.innerHTML !== html) el.innerHTML = html; }
const dot = (cls) => `<span class="dot ${cls}"></span>`;

function render(s) {
  // badges
  const mode = $("modeBadge");
  mode.textContent = s.mode;
  mode.className = "badge" + (s.mode === "AUTO" ? " auto" : "");
  const st = $("stateBadge");
  st.textContent = s.fsm.state;
  st.className = "badge" + (s.fsm.state === "PARKED" ? " parked" : s.fsm.state === "ABORT" ? " abort"
    : s.fsm.active ? " active" : "");
  $("simTag").hidden = !s.sim;

  // find & park
  const fp = $("findPark");
  const running = s.mode === "AUTO" && s.fsm.active;
  fp.disabled = !s.find_park.enabled || running;
  fp.textContent = running ? "PARKING…" : "FIND & PARK";
  setText("parkReason", running ? "Running. Any drive input or E-STOP takes over." : s.find_park.reason);

  // sensors
  const d = s.telemetry.d;
  SENSORS.forEach((name, i) => {
    const g = $("s-" + name);
    const v = d ? d[i] : null;
    g.querySelector(".pill").setAttribute("fill", d ? distColor(v) : "#555");
    g.querySelector("text").textContent = (v === null || v === undefined || v < 0) ? "—" : Math.round(v);
  });

  // key/values
  setText("kMode", s.mode);
  setText("kState", s.fsm.state);
  const step = s.fsm.step_index !== null && s.fsm.step_count
    ? `${s.fsm.step_index + 1}/${s.fsm.step_count} · ${s.fsm.step_desc}` : s.fsm.step_desc;
  setText("kStep", step || "—");
  setText("kReason", s.fsm.reason || "—");
  const t = s.telemetry;
  setHTML("kDevkit", t.live
    ? `${dot(t.state === "OK" ? "ok" : "bad")}live · ${t.state} · ${(t.age_s ?? 0).toFixed(2)} s`
    : `${dot("bad")}offline${t.age_s !== null ? ` · last ${t.age_s.toFixed(1)} s ago` : ""}`);
  setHTML("kCamera", s.camera.live ? `${dot("ok")}live · ${s.camera.fps} fps` : `${dot("bad")}offline`);
  setHTML("kCalib", s.homography_loaded ? `${dot("ok")}loaded` : `${dot("bad")}missing · <a href="/calibrate">calibrate</a>`);
  setText("kYaw", t.yaw === null || t.yaw === undefined ? (t.live ? "no gyro" : "—") : `${t.yaw.toFixed(1)}°${t.busy ? " (rotating)" : ""}`);
  const det = s.detection;
  setText("kDet", det.confidence === null ? "no bay" :
    `conf ${det.confidence.toFixed(2)} · x ${det.pose.x_cm} y ${det.pose.y_cm} cm · ${det.pose.angle_deg}°` +
    (det.length_cm ? ` · ${Math.round(det.length_cm)}×${Math.round(det.width_cm || 0)} cm` : ""));
  const gap = s.gap;
  setText("kGap", gap.length_cm ? `${Math.round(gap.length_cm)} cm (${gap.source}) · need ${gap.required_cm}` +
    (gap.ok ? " ✓" : " ✗") : (gap.sonar_cm ? `sonar ${Math.round(gap.sonar_cm)} cm` : "—"));
  setText("kCmd", s.last_cmd);

  const hist = s.fsm.history.slice().reverse()
    .map((h) => `<li><b>${h.from} → ${h.to}</b>${h.reason ? " · " + escapeHtml(h.reason) : ""}</li>`).join("");
  setHTML("history", hist);
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

let failures = 0;
async function pollStatus() {
  try {
    const res = await fetch("/api/status", { cache: "no-store" });
    render(await res.json());
    failures = 0;
    $("offline").hidden = true;
  } catch (e) {
    failures += 1;
    if (failures > 3) $("offline").hidden = false;
  }
  setTimeout(pollStatus, 150);
}

// Reload the MJPEG stream if it breaks (server restart, network hiccup).
$("video").addEventListener("error", () => {
  setTimeout(() => { $("video").src = "/video?t=" + Date.now(); }, 1000);
});

// ------------------------------------------------------------------ start
fetch("/api/config").then((r) => r.json()).then((c) => {
  Object.assign(CFG, c);
  speedEl.min = String(c.min_pwm);
  speedEl.max = String(c.max_pwm);
  speedEl.value = String(c.default_speed);
  $("speedVal").textContent = speedEl.value;
}).catch(() => {});
pollStatus();
