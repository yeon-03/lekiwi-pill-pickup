"use strict";

const $ = (id) => document.getElementById(id);
const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

const STATE_TEXT = { idle: "대기", moving: "주행 중", arrived: "도착", picking: "집는 중", returning: "복귀 중",
  done: "완료", failed: "실패", canceled: "멈춤", rejected: "거절" };
// 약통 색 — 에이보 웹앱과 같은 색·이름 (색 이름 글자를 늘 같이 보여준다)
const PILL = { red: { hex: "#D93636", kr: "빨간색" }, green: { hex: "#2E9E4F", kr: "초록색" },
  blue: { hex: "#2F6BD8", kr: "파란색" } };
const NO_TARGET_HEX = "#7A8190";
const STAGE_BADGE = { todo: "대기", active: "진행 중", done: "완료", failed: "실패" };
const STAGE_ICON = { todo: "○", active: "◉", done: "✔", failed: "✖" };
const SUMMARY_TOPICS = ["/scan", "/tf", "/odom", "/particle_cloud", "/abo/state", "/abo/pick_done"];
const STREAM_TOPICS = new Set(["/scan", "/tf", "/odom"]);    // 끊기면 안 되는 주기 토픽 — 2초 넘게 안 오면 경고색
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

function fmtClock(epoch) {
  if (epoch == null) return "";
  const d = new Date(epoch * 1000);
  return [d.getHours(), d.getMinutes(), d.getSeconds()].map((v) => String(v).padStart(2, "0")).join(":");
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

const UI_SCALE_KEY = "lekiwi-dashboard-ui-scale";
let uiScale = 1;

function loadUiScale() {
  try {
    const v = parseFloat(localStorage.getItem(UI_SCALE_KEY));
    return Number.isFinite(v) ? v : 1;
  } catch (_) {
    return 1;
  }
}

function setUiScale(v, save = true) {
  uiScale = Math.min(2, Math.max(0.8, Math.round(v * 10) / 10));
  document.documentElement.style.setProperty("--ui-scale", String(uiScale));
  if (save) {
    try { localStorage.setItem(UI_SCALE_KEY, String(uiScale)); } catch (_) { /* 저장 못 해도 화면은 바뀐다 */ }
  }
  draw();
}

async function init() {
  // Wire stop/release buttons first (safety-critical)
  $("stop-btn").addEventListener("click", () =>
    postAction("/api/stop", (why) => `정지 요청 실패: ${why} — 로봇 전원 스위치를 사용하세요`));
  $("release-btn").addEventListener("click", () =>
    postAction("/api/stop/release", (why) => `해제 요청 실패: ${why}`));

  // 글자 크기 조절 (큰 모니터에서 운영자가 맞춘다)
  $("font-down").addEventListener("click", () => setUiScale(uiScale - 0.1));
  $("font-up").addEventListener("click", () => setUiScale(uiScale + 0.1));
  setUiScale(loadUiScale(), false);

  // 토픽 자세히 — 전체 표를 화면 위 서랍으로
  const drawer = $("topics-drawer");
  const setDrawer = (open) => {
    drawer.hidden = !open;
    $("topics-toggle").setAttribute("aria-expanded", String(open));
    if (open) $("topics-close").focus();
  };
  $("topics-toggle").addEventListener("click", () => setDrawer(drawer.hidden));
  $("topics-close").addEventListener("click", () => { setDrawer(false); $("topics-toggle").focus(); });
  drawer.addEventListener("click", (e) => { if (e.target === drawer) setDrawer(false); });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape" && !drawer.hidden) setDrawer(false); });

  // Build legend and set up layer visibility
  buildLegend();
  new ResizeObserver(() => draw()).observe($("map").parentElement);

  // Connect EventSource for real-time updates
  const es = new EventSource("/api/events");
  es.onmessage = (ev) => {
    last = JSON.parse(ev.data);
    const first = lastEventAt === 0;
    lastEventAt = performance.now();
    if (first) checkConnection();
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
  const pill = PILL[m.color];
  const badge = $("target-badge");
  badge.textContent = m.target ? targetText(m) : "목표 없음";
  badge.className = "target-badge" + (m.target ? "" : " none");
  badge.style.setProperty("--target", m.target && pill ? pill.hex : NO_TARGET_HEX);
  $("mission-pane").style.setProperty("--target", m.target && pill ? pill.hex : NO_TARGET_HEX);
  $("elapsed").textContent = fmtSec(m.elapsed);
  $("headline").textContent = m.headline || "";
  $("status-text").textContent = m.status || "";
  renderFlow(m, s.pick);

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

function targetText(m) {
  const pill = PILL[m.color];
  return `● ${pill ? `${pill.kr} 약` : "약"} · ${m.target}`;
}

function renderFlow(m, pick) {
  const stages = m.stages.length ? m.stages : [
    { key: "drive", label: "Nav2 주행", state: "todo" }, { key: "pick", label: "집기", state: "todo" },
    { key: "return", label: "복귀", state: "todo" }];
  const pill = PILL[m.color];
  const rows = [];
  const got = m.received_at != null && m.stages.length > 0;
  rows.push(`<li class="node ${got ? "done" : "todo"}"><span class="ic">${got ? "●" : "○"}</span><span class="label">명령 받음</span>` +
    `<span class="side"><span class="t">${got ? fmtClock(m.received_at) : ""}</span></span>` +
    (got && m.target ? `<span class="extra target-badge small" style="--target:${pill ? pill.hex : NO_TARGET_HEX}">${esc(targetText(m))}</span>` : "") +
    `</li>`);
  for (const st of stages) {
    rows.push(`<li class="link ${st.state === "active" ? "active" : st.state === "todo" ? "" : "done"}" aria-hidden="true">▼</li>`);
    const time = st.sec == null ? "" : fmtSec(st.sec);
    const subs = [];
    const meta = [];
    if (st.start_at != null) meta.push(`${fmtClock(st.start_at)} ${st.key === "pick" ? "시작" : "출발"}`);
    if (st.key === "pick" && st.state === "active" && pick && pick.reachable && pick.status) {
      const ps = pick.status;
      if (ps.pick_attempts != null) meta.push(`시도 ${ps.pick_attempts}/${ps.max_pick_attempts}`);
      const pu = (ps.purple || {}).front;
      if (pu) meta.push(`보라 ${pu.ratio.toFixed(2)}/${pu.thr.toFixed(2)}`);
    }
    if (meta.length) subs.push(`<span class="sub">${esc(meta.join(" · "))}</span>`);
    if (st.note) subs.push(`<span class="sub note">“${esc(st.note)}”</span>`);
    rows.push(`<li class="node ${st.state}"><span class="ic">${STAGE_ICON[st.state] || "○"}</span>` +
      `<span class="label">${esc(st.label)}</span>` +
      `<span class="side"><span class="badge">${STAGE_BADGE[st.state] || esc(st.state)}</span><span class="t">${time}</span></span>` +
      subs.join("") + `</li>`);
  }
  const outcome = m.outcome;
  const pickFailed = m.stages.some((st) => st.key === "pick" && st.state === "failed");
  rows.push(`<li class="link ${outcome ? "done" : ""}" aria-hidden="true">▼</li>`);
  const resultText = outcome === "done" ? `✔ 완료 · 전체 ${fmtSec(m.elapsed)}`
    : outcome === "failed" ? `✖ ${pickFailed ? "물건은 집지 못했어요" : "미션 실패"} · 전체 ${fmtSec(m.elapsed)}` : "—";
  rows.push(`<li class="node result ${outcome === "done" ? "done" : outcome === "failed" ? "failed" : "todo"}">` +
    `<span class="ic">${outcome ? "" : "○"}</span><span class="label">${outcome ? resultText : "결과"}</span>` +
    `<span class="side"><span class="t">${outcome ? "" : "—"}</span></span></li>`);
  $("stages").innerHTML = rows.join("");

  const camTarget = $("cam-title-target");
  const picking = m.stages.some((st) => st.key === "pick" && st.state === "active");
  camTarget.innerHTML = picking && pill ? `타겟 <i style="background:${pill.hex}"></i>${pill.kr}` : "";
}

function renderCams(pick) {
  for (const view of ["front", "wrist"]) {
    const fig = $(`cam-${view}`);
    const img = fig.querySelector("img");
    if (!img.dataset.wired) {
      img.dataset.wired = "1";
      img.addEventListener("error", () => {
        if (!img.dataset.src) return;
        fig.classList.add("noimg");
        fig.querySelector(".cam-empty").textContent = "영상 연결 안 됨 — 다시 시도 중";
        img.dataset.failedAt = String(performance.now());
      });
    }
    if (!pick.reachable) {
      if (img.dataset.src) { img.removeAttribute("src"); delete img.dataset.src; }   // MJPEG 연결을 끊는다
      fig.classList.remove("live", "frozen", "noimg");
      fig.querySelector(".cam-empty").textContent = "집기 단계에서 켜져요";
      fig.querySelector(".cam-meta").textContent = "";
      fig.querySelector(".cam-bar").textContent = "";
      continue;
    }
    const url = `http://${location.hostname}:8000/stream/${view}`;
    const retry = fig.classList.contains("noimg") && performance.now() - Number(img.dataset.failedAt || 0) > 3000;
    if (img.dataset.src !== url || retry) {
      fig.classList.remove("noimg");
      img.src = retry ? `${url}?t=${Date.now()}` : url;
      img.dataset.src = url;
    }
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

function renderTopicSummary(topics) {
  $("topics-summary").innerHTML = SUMMARY_TOPICS.map((name) => {
    const t = topics[name];
    if (!t) return "";
    let cls = "";
    let v;
    if (t.age == null) { cls = "never"; v = "받은 적 없음"; }
    else if (STREAM_TOPICS.has(name) && t.age > 2) { cls = "stale"; v = fmtAge(t.age); }
    else if (t.rate > 0 && t.age <= 2) v = `${t.rate.toFixed(1)} Hz`;
    else v = fmtAge(t.age);
    return `<span class="tchip ${cls}"><code>${esc(name)}</code><span class="v">${esc(v)}</span></span>`;
  }).join("");
}

function renderTopics(topics) {
  renderTopicSummary(topics);
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
  const ui = Math.max(1, (parseFloat(getComputedStyle(document.body).fontSize) || 19) / 19);   // 글자 크기에 맞춰 지도 표식도 키운다
  // 칸의 폭과 높이 둘 다에 맞춘다 (넓은 화면은 칸 높이가 고정, 좁은 화면은 폭 기준으로 쌓인다)
  const box = canvas.parentElement;
  const wide = window.matchMedia("(min-width: 1280px)").matches;
  const ratio = meta.width / meta.height;
  const cssW = Math.max(1, Math.floor(Math.min(box.clientWidth, wide ? box.clientHeight * ratio : Infinity)));
  const cssH = Math.floor(cssW / ratio);
  if (canvas.width !== Math.round(cssW * dpr) || canvas.height !== Math.round(cssH * dpr)) {
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
    canvas.style.width = `${cssW}px`;
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

  ctx.font = `${Math.round(15 * ui * dpr)}px ${css("--sans")}`;
  for (const [name, w] of Object.entries(meta.waypoints)) {
    const [px, py] = P(w.x, w.y);
    ctx.strokeStyle = css("--muted");
    ctx.fillStyle = css("--muted");
    ctx.lineWidth = dpr * ui;
    ctx.beginPath(); ctx.arc(px, py, 8 * ui * dpr, 0, Math.PI * 2); ctx.stroke();
    ctx.fillText(name, px + 10 * ui * dpr, py - 10 * ui * dpr);
  }

  const s = last;
  if (!s) return;
  const line = (pts, color, width, alpha) => {
    if (pts.length < 2) return;
    ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width * ui * dpr;
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
  if (visible.particles) dots(s.particles.pts, css("--accent"), 3 * ui * dpr, s.particles.age > 2 ? 0.25 : 0.6);
  if (visible.scan && s.scan.age != null && s.scan.age <= 2) dots(s.scan.pts, css("--scan"), 4 * ui * dpr, 0.9);

  if (s.pose) {
    const [px, py] = P(s.pose.x, s.pose.y);
    const R = Math.max(9 * ui * dpr, 0.12 / meta.resolution * k);              // 로봇 반경 약 12 cm
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
