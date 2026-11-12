"use strict";

// ==== 定数 ====
const SEND_INTERVAL_MS = 100;
const RECONNECT_MS = 1000;
const ZERO_REPEAT = 3; // 離したときに v=0,w=0 を送る回数（100ms 間隔）
const TOAST_MS = 3000;
const GOAL_TAP_PX = 15; // これより短いドラッグは「ロボット→指定点」の向きを使う
const PAN_THRESHOLD_PX = 4;
const MIN_SCALE = 2; // px/m
const MAX_SCALE = 600;
const STORE_V = "cugo_web.maxV";
const STORE_W = "cugo_web.maxW";

// ESP32 のモード → [表示, バッジの色]
const MODES = {
  idle: ["待機", "off"],
  auto: ["自律可", "auto"],
  manual: ["スマホ手動", "warn"],
  test: ["テスト", "warn"],
  disconnected: ["ESP32未接続", "err"],
};
// Nav2 の状態 → [表示, バッジの色]
const NAVS = {
  idle: ["待機", "off"],
  active: ["移動中", "auto"],
  succeeded: ["到着", "ok"],
  aborted: ["失敗", "err"],
  canceled: ["中止", "warn"],
};

const $ = (id) => document.getElementById(id);

// ==== 状態 ====
let ws = null;
let sending = !document.hidden;
let zeroLeft = 0;
const keys = new Set();
const joy = { active: false, pointerId: null, fwd: 0, turn: 0 };

const world = {
  map: null, // {img, w, h, res, ox, oy}
  pose: null, // {x, y, yaw}
  goal: null, // {x, y, yaw}
  scan: [],
  plan: [],
};
const view = { cx: 0, cy: 0, scale: 40, w: 0, h: 0, dpr: 1 };
let follow = true;
let goalMode = false;
let goalDrag = null; // {sx, sy, x, y, curSx, curSy}

// ==== 設定（速度スライダー）====
function loadNum(key, def, min, max) {
  try {
    const v = parseFloat(localStorage.getItem(key));
    if (Number.isFinite(v) && v >= min && v <= max) return v;
  } catch (_) { /* 無視 */ }
  return def;
}
function saveNum(key, v) {
  try { localStorage.setItem(key, String(v)); } catch (_) { /* 無視 */ }
}

let maxV = loadNum(STORE_V, 0.25, 0.05, 0.40);
let maxW = loadNum(STORE_W, 0.8, 0.2, 1.5);

function setupSlider(id, unit, digits, get, set, storeKey) {
  const input = $(id);
  const label = $(id + "-val");
  input.value = String(get());
  const show = () => { label.textContent = `${Number(input.value).toFixed(digits)} ${unit}`; };
  show();
  input.addEventListener("input", () => {
    set(Number(input.value));
    show();
  });
  input.addEventListener("change", () => {
    saveNum(storeKey, Number(input.value));
    input.blur(); // キーボード操作（WASD）に戻す
  });
}
setupSlider("max-v", "m/s", 2, () => maxV, (v) => { maxV = v; }, STORE_V);
setupSlider("max-w", "rad/s", 1, () => maxW, (v) => { maxW = v; }, STORE_W);

// ==== 送信 ====
function wsOpen() {
  return ws && ws.readyState === WebSocket.OPEN;
}
function send(obj) {
  if (wsOpen()) ws.send(JSON.stringify(obj));
}

// ==== 操縦入力 ====
// 入力中なら {fwd, turn}（-1〜1, turn は右が +）、入力なしなら null
function currentInput() {
  if (joy.active) return { fwd: joy.fwd, turn: joy.turn };
  if (keys.size === 0) return null;
  const fwd = (keys.has("w") ? 1 : 0) - (keys.has("s") ? 1 : 0);
  const turn = (keys.has("d") ? 1 : 0) - (keys.has("a") ? 1 : 0);
  return { fwd, turn };
}

// 100ms ごとに呼ぶ。離した直後にも即時に呼ぶ
function driveTick() {
  const inp = sending ? currentInput() : null;
  if (inp) {
    const v = inp.fwd * maxV;
    const w = -inp.turn * maxW; // 右へ倒す = 右旋回 = 角速度マイナス
    send({ t: "drive", v: round3(v), w: round3(w) });
    zeroLeft = ZERO_REPEAT;
  } else if (zeroLeft > 0) {
    zeroLeft--;
    if (sending) send({ t: "drive", v: 0, w: 0 });
  }
}
setInterval(driveTick, SEND_INTERVAL_MS);

function round3(x) {
  return Math.round(x * 1000) / 1000;
}

function resetInput() {
  keys.clear();
  joy.active = false;
  joy.pointerId = null;
  joy.fwd = joy.turn = 0;
  $("knob").style.transform = "";
}

function emergencyStop() {
  resetInput();
  zeroLeft = 0;
  send({ t: "stop" });
}

// ==== 表示（バッジ・ヒント・トースト）====
function setBadge(id, table, key) {
  const el = $(id);
  const m = table[key];
  el.textContent = m ? m[0] : "";
  el.className = "badge " + (m ? m[1] : "");
}

function setConn(text, cls) {
  const el = $("conn");
  el.textContent = text;
  el.className = "badge " + cls;
}

function setHint(mode) {
  const el = $("hint");
  let text = "";
  let err = false;
  if (mode === "manual") {
    text = "ESP32 のスマホ画面が操縦中のため、この画面からの指令は無視されます";
  } else if (mode === "disconnected") {
    text = "ESP32（モーター制御）と接続されていません";
    err = true;
  } else if (mode === "idle") {
    text = "ESP32 に Jetson の指令が届いていません（cugo_base を確認）";
  }
  el.textContent = text;
  el.hidden = !text;
  el.classList.toggle("err", err);
  if (text !== lastHint) {
    lastHint = text;
    requestAnimationFrame(resize); // ヒントの有無で地図の高さが変わる
  }
}
let lastHint = "";

function toast(text, level) {
  const box = $("toasts");
  const el = document.createElement("div");
  el.className = "toast " + (level || "info");
  el.textContent = text;
  box.appendChild(el);
  while (box.children.length > 4) box.removeChild(box.firstChild);
  setTimeout(() => el.classList.add("fade"), TOAST_MS - 300);
  setTimeout(() => el.remove(), TOAST_MS);
}

// ==== WebSocket ====
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  setConn("接続中…", "warn");
  ws.onopen = () => setConn("接続", "ok");
  ws.onmessage = (ev) => {
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch (_) {
      return;
    }
    if (msg && typeof msg.t === "string") handleMessage(msg);
  };
  ws.onclose = () => {
    setConn("切断", "off");
    setBadge("mode", MODES, null);
    setBadge("nav", NAVS, null);
    setHint(null);
    resetInput();
    zeroLeft = 0;
    world.scan = [];
    requestDraw();
    setTimeout(connect, RECONNECT_MS);
  };
  ws.onerror = () => { /* onclose で再接続する */ };
}

function handleMessage(msg) {
  switch (msg.t) {
    case "map": onMap(msg); break;
    case "state":
      setBadge("mode", MODES, msg.mode);
      setBadge("nav", NAVS, msg.nav);
      setHint(msg.mode);
      world.pose = validPose(msg.pose);
      world.goal = validPose(msg.goal);
      requestDraw();
      break;
    case "scan":
      world.scan = Array.isArray(msg.pts) ? msg.pts : [];
      requestDraw();
      break;
    case "plan":
      world.plan = Array.isArray(msg.pts) ? msg.pts : [];
      requestDraw();
      break;
    case "msg":
      if (typeof msg.text === "string") toast(msg.text, msg.level);
      break;
  }
}

function validPose(p) {
  if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y)) return null;
  return { x: p.x, y: p.y, yaw: Number.isFinite(p.yaw) ? p.yaw : 0 };
}

function onMap(msg) {
  if (!(msg.w > 0 && msg.h > 0 && msg.res > 0) || typeof msg.png !== "string") return;
  const img = new Image();
  img.onload = () => {
    const first = !world.map;
    world.map = { img, w: msg.w, h: msg.h, res: msg.res, ox: msg.ox, oy: msg.oy };
    $("nomap").hidden = true;
    if (first && !(follow && world.pose)) fitMap(false);
    requestDraw();
  };
  img.src = "data:image/png;base64," + msg.png;
}

// ==== 座標変換 ====
function toScreen(x, y) {
  return [view.w / 2 + (x - view.cx) * view.scale, view.h / 2 - (y - view.cy) * view.scale];
}
function toWorld(sx, sy) {
  return [view.cx + (sx - view.w / 2) / view.scale, view.cy - (sy - view.h / 2) / view.scale];
}

function clampScale(s) {
  return Math.min(MAX_SCALE, Math.max(MIN_SCALE, s));
}

// 画面上の点 (sx,sy) を固定してズーム。追従中は画面中央（ロボット）を中心にする
function zoomAt(sx, sy, factor) {
  if (follow && world.pose) {
    sx = view.w / 2;
    sy = view.h / 2;
  }
  const [wx, wy] = toWorld(sx, sy);
  view.scale = clampScale(view.scale * factor);
  view.cx = wx - (sx - view.w / 2) / view.scale;
  view.cy = wy + (sy - view.h / 2) / view.scale;
  requestDraw();
}

function panBy(dx, dy) {
  view.cx -= dx / view.scale;
  view.cy += dy / view.scale;
  requestDraw();
}

function setFollow(on) {
  follow = on;
  $("btn-follow").classList.toggle("on", on);
  requestDraw();
}

function fitMap(turnOffFollow) {
  const m = world.map;
  if (!m || view.w === 0 || view.h === 0) return;
  const mw = m.w * m.res;
  const mh = m.h * m.res;
  view.scale = clampScale(Math.min(view.w / mw, view.h / mh) * 0.95);
  view.cx = m.ox + mw / 2;
  view.cy = m.oy + mh / 2;
  if (turnOffFollow) setFollow(false);
  requestDraw();
}

// ==== 描画 ====
const canvas = $("map");
const ctx = canvas.getContext("2d");
let drawPending = false;

function requestDraw() {
  if (drawPending) return;
  drawPending = true;
  requestAnimationFrame(draw);
}

function resize() {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  view.w = rect.width;
  view.h = rect.height;
  view.dpr = dpr;
  const pw = Math.max(1, Math.round(rect.width * dpr));
  const ph = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== pw || canvas.height !== ph) {
    canvas.width = pw;
    canvas.height = ph;
  }
  requestDraw();
}

function draw() {
  drawPending = false;
  if (follow && world.pose) {
    view.cx = world.pose.x;
    view.cy = world.pose.y;
  }
  ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
  ctx.clearRect(0, 0, view.w, view.h);

  drawMapImage();
  drawScan();
  drawPlan();
  if (world.goal) drawArrow(world.goal.x, world.goal.y, world.goal.yaw, "#ff7b72", "#ffffff");
  if (goalDrag) drawGoalPreview();
  drawRobot();
  drawScaleBar();
}

function drawMapImage() {
  const m = world.map;
  if (!m) return;
  const [sx, sy] = toScreen(m.ox, m.oy + m.h * m.res);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(m.img, sx, sy, m.w * m.res * view.scale, m.h * m.res * view.scale);
}

function drawScan() {
  const pts = world.scan;
  if (!pts.length) return;
  ctx.fillStyle = "#ff3b30";
  ctx.beginPath();
  for (let i = 0; i + 1 < pts.length; i += 2) {
    const [sx, sy] = toScreen(pts[i], pts[i + 1]);
    ctx.rect(sx - 1.5, sy - 1.5, 3, 3);
  }
  ctx.fill();
}

function drawPlan() {
  const pts = world.plan;
  if (pts.length < 4) return;
  ctx.strokeStyle = "#3fb950";
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  for (let i = 0; i + 1 < pts.length; i += 2) {
    const [sx, sy] = toScreen(pts[i], pts[i + 1]);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  }
  ctx.stroke();
}

// 目的地マーカー: 丸 + 向きの矢印
function drawArrow(x, y, yaw, color, outline) {
  const [sx, sy] = toScreen(x, y);
  const len = Math.max(26, 0.5 * view.scale);
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-yaw); // 画面は y 下向きなので符号反転
  ctx.lineCap = "round";
  ctx.strokeStyle = outline;
  ctx.lineWidth = 6;
  arrowPath(len);
  ctx.stroke();
  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  arrowPath(len);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(0, 0, 6, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = outline;
  ctx.stroke();
  ctx.restore();
}
function arrowPath(len) {
  const head = Math.min(12, len * 0.4);
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(len, 0);
  ctx.moveTo(len - head, -head * 0.6);
  ctx.lineTo(len, 0);
  ctx.lineTo(len - head, head * 0.6);
}

function drawGoalPreview() {
  const g = goalDrag;
  drawArrow(g.x, g.y, goalYaw(g), "#d2a8ff", "#000000");
}

function drawRobot() {
  const p = world.pose;
  if (!p) return;
  const [sx, sy] = toScreen(p.x, p.y);
  const L = Math.max(12, 0.3 * view.scale);
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-p.yaw);
  ctx.beginPath();
  ctx.moveTo(L, 0);
  ctx.lineTo(-L * 0.7, L * 0.65);
  ctx.lineTo(-L * 0.35, 0);
  ctx.lineTo(-L * 0.7, -L * 0.65);
  ctx.closePath();
  ctx.fillStyle = "#58a6ff";
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#ffffff";
  ctx.stroke();
  ctx.restore();
}

// 縮尺バー: 通常 1 m。極端なズームでは見やすい長さに切り替える
const SCALE_STEPS = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100];
function drawScaleBar() {
  let len = 1;
  if (view.scale < 30 || view.scale > 200) {
    len = SCALE_STEPS.find((s) => s * view.scale >= 50) || SCALE_STEPS[SCALE_STEPS.length - 1];
  }
  const px = len * view.scale;
  const x0 = 12;
  const y0 = view.h - 14;
  ctx.save();
  ctx.lineCap = "butt";
  for (const [color, width] of [["#000000aa", 5], ["#ffffff", 2]]) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(x0, y0 - 6);
    ctx.lineTo(x0, y0);
    ctx.lineTo(x0 + px, y0);
    ctx.lineTo(x0 + px, y0 - 6);
    ctx.stroke();
  }
  ctx.font = "12px system-ui, sans-serif";
  ctx.fillStyle = "#ffffff";
  ctx.strokeStyle = "#000000aa";
  ctx.lineWidth = 3;
  const label = len < 1 ? `${Math.round(len * 100)} cm` : `${len} m`;
  ctx.strokeText(label, x0 + 4, y0 - 8);
  ctx.fillText(label, x0 + 4, y0 - 8);
  ctx.restore();
}

new ResizeObserver(resize).observe(canvas);
window.addEventListener("resize", resize);

// ==== 地図の操作（パン・ピンチ・ホイール・目的地指定）====
const pointers = new Map(); // pointerId -> {x, y}
let panMoved = 0;

function localPos(e) {
  const r = canvas.getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
}

function goalYaw(g) {
  const dx = g.curSx - g.sx;
  const dy = g.curSy - g.sy;
  if (Math.hypot(dx, dy) >= GOAL_TAP_PX) return Math.atan2(-dy, dx);
  const p = world.pose;
  if (p && Math.hypot(g.x - p.x, g.y - p.y) > 1e-3) return Math.atan2(g.y - p.y, g.x - p.x);
  return p ? p.yaw : 0;
}

function setGoalMode(on) {
  goalMode = on;
  goalDrag = null;
  $("btn-goal").classList.toggle("on", on);
  $("view").classList.toggle("goal-mode", on);
  $("goal-hint").hidden = !on;
  requestDraw();
}

canvas.addEventListener("pointerdown", (e) => {
  e.preventDefault();
  const [sx, sy] = localPos(e);
  if (goalMode) {
    if (goalDrag) return; // 2 本目の指は無視
    const [x, y] = toWorld(sx, sy);
    goalDrag = { id: e.pointerId, sx, sy, x, y, curSx: sx, curSy: sy };
    canvas.setPointerCapture(e.pointerId);
    requestDraw();
    return;
  }
  canvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, { x: sx, y: sy });
  panMoved = 0;
});

canvas.addEventListener("pointermove", (e) => {
  const [sx, sy] = localPos(e);
  if (goalDrag) {
    if (e.pointerId !== goalDrag.id) return;
    goalDrag.curSx = sx;
    goalDrag.curSy = sy;
    requestDraw();
    return;
  }
  const prev = pointers.get(e.pointerId);
  if (!prev) return;

  if (pointers.size === 1) {
    const dx = sx - prev.x;
    const dy = sy - prev.y;
    panMoved += Math.hypot(dx, dy);
    if (panMoved > PAN_THRESHOLD_PX) {
      if (follow) setFollow(false);
      panBy(dx, dy);
    }
    prev.x = sx;
    prev.y = sy;
  } else if (pointers.size >= 2) {
    // 2 本指: 距離の変化でズーム、中点の移動でパン
    const ids = [...pointers.keys()].slice(0, 2);
    if (!ids.includes(e.pointerId)) return;
    const other = pointers.get(ids[0] === e.pointerId ? ids[1] : ids[0]);
    const d0 = Math.hypot(prev.x - other.x, prev.y - other.y);
    const d1 = Math.hypot(sx - other.x, sy - other.y);
    const m0x = (prev.x + other.x) / 2;
    const m0y = (prev.y + other.y) / 2;
    const m1x = (sx + other.x) / 2;
    const m1y = (sy + other.y) / 2;
    if (d0 > 1 && d1 > 1) zoomAt(m0x, m0y, d1 / d0);
    if (!follow) panBy(m1x - m0x, m1y - m0y);
    prev.x = sx;
    prev.y = sy;
  }
});

function endPointer(e, canceled) {
  if (goalDrag && e.pointerId === goalDrag.id) {
    const g = goalDrag;
    if (!canceled) {
      g.curSx = localPos(e)[0];
      g.curSy = localPos(e)[1];
      const yaw = goalYaw(g);
      if (wsOpen()) {
        send({ t: "goal", x: round3(g.x), y: round3(g.y), yaw: round3(yaw) });
      } else {
        toast("未接続のため目的地を送れません", "warn");
      }
    }
    setGoalMode(false);
    return;
  }
  pointers.delete(e.pointerId);
}
canvas.addEventListener("pointerup", (e) => endPointer(e, false));
canvas.addEventListener("pointercancel", (e) => endPointer(e, true));

canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const [sx, sy] = localPos(e);
  zoomAt(sx, sy, Math.exp(-e.deltaY * 0.0015));
}, { passive: false });

// iOS Safari のピンチによるページ拡大を防ぐ
document.addEventListener("gesturestart", (e) => e.preventDefault());
canvas.addEventListener("contextmenu", (e) => e.preventDefault());

// ==== ボタン ====
$("btn-goal").addEventListener("click", () => setGoalMode(!goalMode));
$("btn-cancel").addEventListener("click", () => {
  setGoalMode(false);
  send({ t: "cancel" });
});
$("btn-follow").addEventListener("click", () => setFollow(!follow));
$("btn-fit").addEventListener("click", () => {
  if (!world.map) {
    toast("まだ地図がありません", "warn");
    return;
  }
  fitMap(true);
});
$("btn-save").addEventListener("click", () => {
  if (!wsOpen()) {
    toast("未接続のため保存できません", "warn");
    return;
  }
  const d = new Date();
  const p2 = (n) => String(n).padStart(2, "0");
  const def = `map_${d.getFullYear()}${p2(d.getMonth() + 1)}${p2(d.getDate())}_${p2(d.getHours())}${p2(d.getMinutes())}`;
  resetInput(); // ダイアログ中は keyup が届かないため
  const name = window.prompt("地図の名前（英数字・_・- のみ）", def);
  if (name === null) return;
  const trimmed = name.trim();
  if (!/^[A-Za-z0-9_-]+$/.test(trimmed)) {
    toast("名前は英数字・_・- のみ使えます", "error");
    return;
  }
  send({ t: "save_map", name: trimmed });
});
// ボタンにフォーカスが残ると Space で押されてしまうので外す
for (const b of document.querySelectorAll(".toolbar button")) {
  b.addEventListener("click", () => b.blur());
}

// ==== 停止ボタン ====
$("stop").addEventListener("pointerdown", (e) => {
  e.preventDefault();
  emergencyStop();
});

// ==== キーボード ====
const DRIVE_KEYS = new Set(["w", "a", "s", "d"]);

window.addEventListener("keydown", (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (k === " " || e.code === "Space") {
    e.preventDefault();
    emergencyStop();
    return;
  }
  if (k === "escape" && goalMode) {
    setGoalMode(false);
    return;
  }
  if (DRIVE_KEYS.has(k) && sending && wsOpen()) keys.add(k);
});
window.addEventListener("keyup", (e) => {
  const k = e.key.toLowerCase();
  if (!keys.has(k)) return;
  keys.delete(k);
  if (keys.size === 0 && !joy.active) driveTick(); // 即座に 0 を送る
});
// フォーカスを失うと keyup が届かず押しっぱなし扱いになるため全解除
window.addEventListener("blur", () => {
  const had = keys.size > 0;
  keys.clear();
  if (had && !joy.active) driveTick();
});

// ==== タブ非表示で停止 ====
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    emergencyStop();
    sending = false;
  } else {
    sending = true;
  }
});
window.addEventListener("pagehide", () => {
  emergencyStop();
  sending = false;
});

// ==== 仮想ジョイスティック ====
(() => {
  const pad = $("joy");
  const knob = $("knob");

  function update(e) {
    const rect = pad.getBoundingClientRect();
    const radius = rect.width / 2;
    let dx = e.clientX - (rect.left + radius);
    let dy = e.clientY - (rect.top + radius);
    const dist = Math.hypot(dx, dy);
    if (dist > radius) {
      dx = (dx / dist) * radius;
      dy = (dy / dist) * radius;
    }
    knob.style.transform = `translate(${dx}px, ${dy}px)`;
    joy.turn = dx / radius;
    joy.fwd = -dy / radius;
  }

  function release(e) {
    if (e.pointerId !== joy.pointerId) return;
    joy.pointerId = null;
    joy.active = false;
    joy.fwd = joy.turn = 0;
    knob.style.transform = "";
    driveTick(); // 即座に 0 を送る（残りは送信ループが繰り返す）
  }

  pad.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    if (joy.pointerId !== null) return;
    if (!sending || !wsOpen()) return; // 未接続中は操作を受け付けない
    joy.pointerId = e.pointerId;
    pad.setPointerCapture(e.pointerId);
    joy.active = true;
    update(e);
  });
  pad.addEventListener("pointermove", (e) => {
    if (e.pointerId === joy.pointerId) update(e);
  });
  pad.addEventListener("pointerup", release);
  pad.addEventListener("pointercancel", release);
  pad.addEventListener("lostpointercapture", release);
})();

// ==== 起動 ====
resize();
connect();
