"use strict";

const $ = (id) => document.getElementById(id);
const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

const STATE_TEXT = { idle: "대기", moving: "주행 중", arrived: "도착", picking: "집는 중", returning: "복귀 중",
  done: "완료", failed: "실패", canceled: "멈춤", rejected: "거절" };
const COLOR_TEXT = { green: "초록", red: "빨강", blue: "파랑" };
const LAYERS = [
  { key: "scan", label: "/scan", color: "--scan" },
  { key: "particles", label: "파티클", color: "--accent" },
  { key: "plan", label: "Nav2 경로", color: "--plan" },
  { key: "trail", label: "odom 궤적", color: "--trail" },
];
const visible = { scan: true, particles: true, plan: true, trail: true };

let meta = null;
let mapImg = null;
let last = null;
let lastEventAt = 0;

function fmtAge(a) {
  if (a == null) return "받은 적 없음";
  return `${a < 10 ? a.toFixed(1) : Math.round(a)}초 전`;
}

function fmtSec(s) {
  if (s == null) return "–";
  return `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
}

// 정지/해제 요청 실패 문구. SSE 가 0.2 s 마다 warn-banner 를 다시 그리므로 성공할 때까지 붙잡아 둔다.
let requestError = "";

function showWarnings(list) {
  const all = requestError ? [requestError, ...list] : list;
  $("warn-banner").hidden = all.length === 0;
  $("warn-banner").textContent = all.join(" · ");
}

async function postAction(url, message) {
  try {
    const r = await fetch(url, { method: "POST" });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    requestError = "";
  } catch (err) {
    requestError = message(err.message || String(err));
  }
  showWarnings(last ? last.warnings : []);
}

async function init() {
  // Wire stop/release buttons first (safety-critical)
  $("stop-btn").addEventListener("click", () =>
    postAction("/api/stop", (why) => `정지 요청 실패: ${why} — 로봇 전원 스위치를 사용하세요`));
  $("release-btn").addEventListener("click", () =>
    postAction("/api/stop/release", (why) => `해제 요청 실패: ${why}`));

  // Build legend and set up layer visibility
  buildLegend();
  new ResizeObserver(() => draw()).observe($("map").parentElement);

  // Connect EventSource for real-time updates
  const es = new EventSource("/api/events");
  es.onmessage = (ev) => {
    last = JSON.parse(ev.data);
    lastEventAt = performance.now();
    render(last);
  };

  // Monitor connection health
  setInterval(checkConnection, 1000);

  // Load map info asynchronously (may fail, but buttons remain functional)
  try {
    meta = await (await fetch("/api/map_info")).json();
    mapImg = new Image();
    mapImg.onload = () => draw();
    mapImg.src = "/map.png";
  } catch (err) {
    $("warn-banner").hidden = false;
    $("warn-banner").textContent = `지도 정보를 불러오지 못했어요: ${err}`;
  }
}

function checkConnection() {
  const ok = performance.now() - lastEventAt < 3000;
  const c = $("conn");
  c.querySelector(".dot").className = "dot " + (ok ? "ok" : "bad");
  c.querySelector("span").textContent = ok ? "대시보드 연결됨" : "대시보드 서버 연결 끊김";
}

function buildLegend() {
  $("legend").innerHTML = LAYERS.map((l) =>
    `<label><input type="checkbox" id="layer-${l.key}" checked><i style="background:var(${l.color})"></i>` +
    `${l.label} <span class="age" data-age="${l.key}"></span></label>`).join("") +
    `<span><i style="background:var(--ink)"></i>로봇</span>`;
  for (const l of LAYERS) {
    $(`layer-${l.key}`).addEventListener("change", (e) => { visible[l.key] = e.target.checked; draw(); });
  }
}

function render(s) {
  const m = s.mission;
  const chip = $("mission-chip");
  chip.textContent = m.state ? (STATE_TEXT[m.state] || m.state) : "대기";
  chip.className = "chip state" + (m.state === "done" ? " ok"
    : ["failed", "canceled", "rejected"].includes(m.state) ? " bad" : "");
  $("target").innerHTML = m.target
    ? `${m.color ? `<i class="pill ${esc(m.color)}"></i>` : ""}${esc(m.target)} ${esc(COLOR_TEXT[m.color] || "")}`
    : "목표 없음";
  $("elapsed").textContent = fmtSec(m.elapsed);
  $("status-text").textContent = m.status || "";
  $("stages").innerHTML = m.stages.length
    ? m.stages.map((st) => `<li class="${st.state}"><span class="ic"></span><span class="label">${esc(st.label)}</span>` +
        `<span class="t">${st.sec == null ? "" : fmtSec(st.sec)}</span></li>`).join("")
    : `<li class="todo"><span class="ic"></span><span class="label">미션 없음 — fetch 명령을 기다리는 중</span><span></span></li>`;

  $("latch-banner").hidden = !s.stop.latched;
  $("latch-info").textContent = s.stop.latched ? `stop ${s.stop.sent}회 · ${s.stop.last_result || ""}` : "";
  showWarnings(s.warnings);

  const p = s.pose;
  $("pose-info").textContent = p
    ? `x ${p.x.toFixed(2)} · y ${p.y.toFixed(2)} · ${(p.yaw * 180 / Math.PI).toFixed(0)}°` +
      (p.age > 1 ? ` · TF 끊김 ${p.age.toFixed(1)}초` : "")
    : "위치 없음 (TF 없음)";

  for (const l of LAYERS) {
    const el = document.querySelector(`[data-age="${l.key}"]`);
    if (l.key === "trail") { el.textContent = `${s.trail.length}점`; continue; }
    const age = s[l.key].age;
    if (l.key === "particles" && age != null && age > 2) {
      el.textContent = `마지막 갱신 ${Math.round(age)}초 전 (정지 중엔 정상)`;
      el.classList.remove("stale");
    } else {
      el.textContent = fmtAge(age);
      el.classList.toggle("stale", age != null && age > 2);
    }
  }

  renderCams(s.pick);
  renderTopics(s.topics);
  draw();
}

function renderCams(pick) {
  for (const view of ["front", "wrist"]) {
    const fig = $(`cam-${view}`);
    const img = fig.querySelector("img");
    if (!pick.reachable) {
      if (img.dataset.src) { img.removeAttribute("src"); delete img.dataset.src; }   // MJPEG 연결을 끊는다
      fig.classList.remove("live", "frozen");
      fig.querySelector(".cam-meta").textContent = "";
      fig.querySelector(".cam-bar").textContent = "";
      continue;
    }
    const url = `http://${location.hostname}:8000/stream/${view}`;
    if (img.dataset.src !== url) { img.src = url; img.dataset.src = url; }
    fig.classList.add("live");
    const st = pick.status || {};
    const age = (st.frame_age_s || {})[view];
    const frozen = age != null && age > 2;
    fig.classList.toggle("frozen", frozen);
    fig.querySelector(".cam-meta").textContent =
      `${st.state || ""} · ${st.hz ?? "–"} Hz` + (frozen ? ` · 카메라 멈춤 ${age.toFixed(1)}초` : "");
    const bits = [];
    const pu = (st.purple || {})[view];
    if (pu) bits.push(`보라 ${pu.ratio.toFixed(2)} / ${pu.thr.toFixed(2)}${pu.ratio >= pu.thr ? " OK" : ""}`);
    if (view === "front" && st.pick_attempts != null) bits.push(`시도 ${st.pick_attempts}/${st.max_pick_attempts}`);
    if (view === "wrist" && st.retry_depth) bits.push(`재시도 깊이 ${st.retry_depth.toFixed(2)}`);
    fig.querySelector(".cam-bar").textContent = bits.join(" · ");
  }
}

function renderTopics(topics) {
  $("topics").innerHTML = Object.entries(topics).map(([name, t]) => {
    const cls = t.age == null ? "never" : (t.rate > 0 && t.age > 2 ? "stale" : "");
    return `<div class="io-row ${cls}"><span class="dir ${t.dir}">${t.dir === "out" ? "OUT" : "IN"}</span>` +
      `<code>${esc(name)}</code><span class="last">${esc(t.last)}</span>` +
      `<span class="age">${t.rate ? `${t.rate.toFixed(1)} Hz · ` : ""}${fmtAge(t.age)}</span></div>`;
  }).join("");
}

function draw() {
  const canvas = $("map");
  if (!meta || !mapImg || !mapImg.complete || !mapImg.naturalWidth) return;
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.parentElement.clientWidth;
  const cssH = cssW * meta.height / meta.width;
  if (canvas.width !== Math.round(cssW * dpr)) {
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
    canvas.style.height = `${cssH}px`;
  }
  const ctx = canvas.getContext("2d");
  const k = canvas.width / meta.width;                                     // 지도 한 칸 = k 캔버스 픽셀
  const P = (x, y) => [(x - meta.origin_x) / meta.resolution * k,
                       (meta.height - (y - meta.origin_y) / meta.resolution) * k];

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.filter = css("--map-filter") || "none";
  ctx.drawImage(mapImg, 0, 0, canvas.width, canvas.height);
  ctx.filter = "none";

  ctx.font = `${Math.round(12 * dpr)}px ${css("--sans")}`;
  for (const [name, w] of Object.entries(meta.waypoints)) {
    const [px, py] = P(w.x, w.y);
    ctx.strokeStyle = css("--muted");
    ctx.fillStyle = css("--muted");
    ctx.lineWidth = dpr;
    ctx.beginPath(); ctx.arc(px, py, 6 * dpr, 0, Math.PI * 2); ctx.stroke();
    ctx.fillText(name, px + 8 * dpr, py - 8 * dpr);
  }

  const s = last;
  if (!s) return;
  const line = (pts, color, width, alpha) => {
    if (pts.length < 2) return;
    ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width * dpr;
    ctx.beginPath();
    pts.forEach(([x, y], i) => { const [px, py] = P(x, y); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
    ctx.stroke(); ctx.globalAlpha = 1;
  };
  const dots = (pts, color, size, alpha) => {
    ctx.globalAlpha = alpha; ctx.fillStyle = color;
    for (const [x, y] of pts) { const [px, py] = P(x, y); ctx.fillRect(px - size / 2, py - size / 2, size, size); }
    ctx.globalAlpha = 1;
  };

  if (visible.trail) line(s.trail, css("--trail"), 2, 0.5);
  const driving = s.mission.stages.some((st) => st.state === "active" && (st.key === "drive" || st.key === "return"));
  if (visible.plan && driving) line(s.plan.pts, css("--plan"), 2.5, 0.9);
  if (visible.particles) dots(s.particles.pts, css("--accent"), 2 * dpr, s.particles.age > 2 ? 0.25 : 0.6);
  if (visible.scan && s.scan.age != null && s.scan.age <= 2) dots(s.scan.pts, css("--scan"), 3 * dpr, 0.9);

  if (s.pose) {
    const [px, py] = P(s.pose.x, s.pose.y);
    const R = Math.max(7 * dpr, 0.12 / meta.resolution * k);              // 로봇 반경 약 12 cm
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-s.pose.yaw);                                               // 캔버스는 y 가 아래 → 부호 반대
    ctx.fillStyle = s.pose.age > 1 ? css("--muted") : css("--ink");
    ctx.beginPath(); ctx.moveTo(R, 0); ctx.lineTo(-R * 0.7, R * 0.6); ctx.lineTo(-R * 0.7, -R * 0.6); ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

init().catch((err) => {
  $("warn-banner").hidden = false;
  $("warn-banner").textContent = `지도 정보를 불러오지 못했어요: ${err}`;
});
