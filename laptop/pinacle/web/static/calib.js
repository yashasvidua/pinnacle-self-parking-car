// Calibration page: freeze a frame, click 4 floor marks, enter car-frame cm, save.
"use strict";

const $ = (id) => document.getElementById(id);
const canvas = $("calibCanvas");
const ctx2d = canvas.getContext("2d");
const img = new Image();
let points = [];               // [{u, v}] in native image pixels
const world = [["", ""], ["", ""], ["", ""], ["", ""]];

function draw() {
  ctx2d.clearRect(0, 0, canvas.width, canvas.height);
  if (img.complete && img.naturalWidth) ctx2d.drawImage(img, 0, 0);
  const r = Math.max(8, canvas.width / 60);
  points.forEach((p, i) => {
    ctx2d.beginPath();
    ctx2d.arc(p.u, p.v, r, 0, 2 * Math.PI);
    ctx2d.fillStyle = "rgba(245,197,66,.9)";
    ctx2d.fill();
    ctx2d.lineWidth = 2;
    ctx2d.strokeStyle = "#222";
    ctx2d.stroke();
    ctx2d.fillStyle = "#222";
    ctx2d.font = `bold ${Math.round(r * 1.3)}px system-ui`;
    ctx2d.textAlign = "center";
    ctx2d.textBaseline = "middle";
    ctx2d.fillText(String(i + 1), p.u, p.v + 1);
  });
  if (points.length >= 2) {
    ctx2d.beginPath();
    points.forEach((p, i) => (i ? ctx2d.lineTo(p.u, p.v) : ctx2d.moveTo(p.u, p.v)));
    if (points.length === 4) ctx2d.closePath();
    ctx2d.strokeStyle = "rgba(245,197,66,.7)";
    ctx2d.lineWidth = 1.5;
    ctx2d.stroke();
  }
  renderTable();
}

function renderTable() {
  const rows = [];
  for (let i = 0; i < 4; i++) {
    const p = points[i];
    rows.push(`<tr><td><span class="num">${i + 1}</span></td>
      <td>${p ? `${p.u.toFixed(1)}, ${p.v.toFixed(1)}` : "<i>click</i>"}</td>
      <td><input inputmode="decimal" data-i="${i}" data-k="0" value="${world[i][0]}" aria-label="x cm point ${i + 1}"></td>
      <td><input inputmode="decimal" data-i="${i}" data-k="1" value="${world[i][1]}" aria-label="y cm point ${i + 1}"></td></tr>`);
  }
  $("ptsBody").innerHTML = rows.join("");
  $("ptsBody").querySelectorAll("input").forEach((inp) => {
    inp.addEventListener("input", () => {
      world[+inp.dataset.i][+inp.dataset.k] = inp.value;
      updateSave();
    });
  });
  updateSave();
}

function worldNumbers() {
  return world.map(([x, y]) => [parseFloat(x), parseFloat(y)]);
}

function updateSave() {
  const ok = points.length === 4 && worldNumbers().every(([x, y]) => isFinite(x) && isFinite(y));
  $("save").disabled = !ok;
}

function showResult(html, cls) {
  const el = $("result");
  el.innerHTML = html;
  el.className = "result " + (cls || "");
  el.hidden = false;
}

async function freeze() {
  const res = await fetch("/api/calib/frame?t=" + Date.now(), { cache: "no-store" });
  if (!res.ok) { showResult("No camera frame yet. Is the stream live?", "err"); return; }
  const blob = await res.blob();
  img.onload = () => {
    canvas.width = img.naturalWidth;
    canvas.height = img.naturalHeight;
    draw();
  };
  img.src = URL.createObjectURL(blob);
}

canvas.addEventListener("click", (e) => {
  if (points.length >= 4) return;
  const rect = canvas.getBoundingClientRect();
  const u = (e.clientX - rect.left) * canvas.width / rect.width;
  const v = (e.clientY - rect.top) * canvas.height / rect.height;
  points.push({ u, v });
  draw();
});

$("freeze").addEventListener("click", freeze);
$("undo").addEventListener("click", () => { points.pop(); draw(); });
$("clear").addEventListener("click", () => { points = []; draw(); });

$("loadCurrent").addEventListener("click", async () => {
  const info = await (await fetch("/api/calib/info", { cache: "no-store" })).json();
  if (!info.image_points || !info.world_points_cm) { showResult("No calibration saved yet.", "err"); return; }
  points = info.image_points.slice(0, 4).map(([u, v]) => ({ u, v }));
  info.world_points_cm.slice(0, 4).forEach(([x, y], i) => { world[i] = [String(x), String(y)]; });
  draw();
});

$("save").addEventListener("click", async () => {
  const body = { image_points: points.map((p) => [p.u, p.v]), world_points_cm: worldNumbers() };
  const res = await fetch("/api/calib/save", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok || !data.ok) { showResult("Save failed: " + (data.error || res.status), "err"); return; }
  const good = data.max_err_cm < 2;
  showResult(`Saved and reloaded live.<br>Reprojection error: mean <b>${data.mean_err_cm} cm</b>,
    max <b>${data.max_err_cm} cm</b> (${data.mean_err_px} px mean).
    ${good ? "" : "<br>Error is large: re-check the order and the cm values."}`, good ? "good" : "err");
  refreshBadge();
});

async function refreshBadge() {
  try {
    const info = await (await fetch("/api/calib/info", { cache: "no-store" })).json();
    const b = $("calibBadge");
    b.textContent = info.loaded ? "calibrated" : "not calibrated";
    b.className = "badge " + (info.loaded ? "parked" : "abort");
  } catch (e) { /* ignore */ }
}

renderTable();
refreshBadge();
freeze();
