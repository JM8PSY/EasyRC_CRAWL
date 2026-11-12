"use strict";

// ==== 定数 ====
const SEND_INTERVAL_MS = 100;
const RECONNECT_MS = 1000;
const ZERO_REPEAT = 3; // 離したときに v=0,w=0 を送る回数（100ms 間隔）
const TOAST_MS = 3000;
const GOAL_TAP_PX = 15; // これより短いドラッグは「ロボット→指定点」の向きを使う
const PAN_THRESHOLD_PX = 4;
const TAP_PX = 10; // これより動きの小さい押して離すは「タップ」
const ZONE_CLOSE_PX = 15; // 最初の角のこの距離内をタップすると多角形を閉じる
const MIN_SCALE = 2; // px/m
const MAX_SCALE = 600;
const STORE_V = "cugo_web.maxV";
const STORE_W = "cugo_web.maxW";
const STORE_PAD_HINT = "cugo_web.padHintClosed";
const HTTPS_PORT = 8443;
const PAD_DEADZONE = 0.12; // スティックの不感帯（半径）
const PAD_SLOW = 0.5; // R1 を押している間の速度倍率
// 標準マッピング（DualShock 4 / 5）のボタン番号
const PAD_BTN = { cross: 0, circle: 1, square: 2, triangle: 3, l1: 4, r1: 5, options: 9 };

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
// ゲームパッド: active = L1 を押してスティックを倒している（fwd/turn は低速倍率込み）
// blocked = 停止後、L1 を一度離すまで操縦しない
const gp = { connected: false, active: false, fwd: 0, turn: 0, prev: [], blocked: false };
// 自動探索: state = "off" | "running" | "done" | "failed"、frontiers = 残りの未探索境界数（不明なら null）
const explore = { state: "off", frontiers: null };

const world = {
  map: null, // {img, w, h, res, ox, oy}
  pose: null, // {x, y, yaw}
  goal: null, // {x, y, yaw}
  scan: [],
  cliff: [],   // 下り段差のセル中心（map 座標 x,y の並び）
  plan: [],
  keepout: [], // [{id, pts: [[x, y], ...]}] 進入禁止エリア（map 座標）
};
const view = { cx: 0, cy: 0, scale: 40, w: 0, h: 0, dpr: 1 };
let follow = true;
let goalMode = false;
let goalDrag = null; // {sx, sy, x, y, curSx, curSy}
// 進入禁止エリアの作図: on = 作図モード中、pts = 追加済みの角 [[x, y], ...]（map 座標）
const zoneDraw = { on: false, pts: [] };

// LiDAR 取り付け向きの補正
const calib = {
  saved: null, // サーバーに保存されている lidar_yaw [rad]（未受信なら null）
  cam: null, // {pitch, roll, z} カメラの床補正値 [rad, rad, m]（未受信なら null）
  open: false, // 補正パネル表示中
  candDeg: 0, // 候補値 [deg]（整数、-180〜180）
};

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
  if (gp.active) return { fwd: gp.fwd, turn: gp.turn };
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
setInterval(() => {
  pollGamepad(); // rAF が止まっている場合の保険
  driveTick();
}, SEND_INTERVAL_MS);

function round3(x) {
  return Math.round(x * 1000) / 1000;
}

function resetInput() {
  keys.clear();
  joy.active = false;
  joy.pointerId = null;
  joy.fwd = joy.turn = 0;
  $("knob").style.transform = "";
  gp.active = false;
  gp.fwd = gp.turn = 0;
}

function emergencyStop() {
  resetInput();
  gp.blocked = true; // L1 を離すまでゲームパッドで動かさない
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
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${scheme}://${location.host}/ws`);
  setConn("接続中…", "warn");
  ws.onopen = () => {
    setConn("接続", "ok");
    updateCalibUI();
    updateZoneBar();
  };
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
    setExplore("off", null);
    resetInput();
    zeroLeft = 0;
    world.scan = [];
    world.cliff = [];
    updateCalibUI();
    updateZoneBar();
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
      setExplore(
        typeof msg.explore === "string" ? msg.explore : "off",
        Number.isFinite(msg.frontiers) ? Math.round(msg.frontiers) : null,
      );
      world.pose = validPose(msg.pose);
      world.goal = validPose(msg.goal);
      requestDraw();
      break;
    case "cliff":
      world.cliff = Array.isArray(msg.pts) ? msg.pts : [];
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
    case "keepout":
      world.keepout = validZones(msg.zones);
      updateZoneBar();
      requestDraw();
      break;
    case "calib":
      // 保存値だけ更新する。候補値（スライダー）はパネルを開いたときにだけ合わせる
      if (Number.isFinite(msg.lidar_yaw)) {
        calib.saved = msg.lidar_yaw;
        updateCalibUI();
        requestDraw();
      }
      if (Number.isFinite(msg.camera_pitch) || Number.isFinite(msg.camera_roll) || Number.isFinite(msg.camera_z)) {
        const num = (v) => (Number.isFinite(v) ? v : null);
        calib.cam = { pitch: num(msg.camera_pitch), roll: num(msg.camera_roll), z: num(msg.camera_z) };
        updateCalibUI();
      }
      break;
  }
}

function validPose(p) {
  if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y)) return null;
  return { x: p.x, y: p.y, yaw: Number.isFinite(p.yaw) ? p.yaw : 0 };
}

// サーバーからの進入禁止エリア一覧を検査する（不正なものは捨てる）
function validZones(zones) {
  if (!Array.isArray(zones)) return [];
  const out = [];
  for (const z of zones) {
    if (!z || (typeof z.id !== "string" && typeof z.id !== "number") || !Array.isArray(z.pts)) continue;
    const pts = z.pts.filter((p) => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1]))
      .map((p) => [p[0], p[1]]);
    if (pts.length >= 3) out.push({ id: String(z.id), pts });
  }
  return out;
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
  drawKeepout();
  drawCliff();
  if (calib.open) {
    drawHeadingLine();
    drawScanPreview();
  } else {
    drawScan();
  }
  drawPlan();
  if (world.goal) drawArrow(world.goal.x, world.goal.y, world.goal.yaw, "#ff7b72", "#ffffff");
  if (goalDrag) drawGoalPreview();
  if (zoneDraw.on) drawZoneDraft();
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

// 多角形の重心（面積がほぼ 0 なら頂点の平均）
function polyCentroid(pts) {
  let a = 0;
  let cx = 0;
  let cy = 0;
  for (let i = 0; i < pts.length; i++) {
    const [x0, y0] = pts[i];
    const [x1, y1] = pts[(i + 1) % pts.length];
    const c = x0 * y1 - x1 * y0;
    a += c;
    cx += (x0 + x1) * c;
    cy += (y0 + y1) * c;
  }
  if (Math.abs(a) > 1e-9) return [cx / (3 * a), cy / (3 * a)];
  let sx = 0;
  let sy = 0;
  for (const [x, y] of pts) {
    sx += x;
    sy += y;
  }
  return [sx / pts.length, sy / pts.length];
}

// 点が多角形の内側か（レイキャスト法、map 座標）
function pointInPoly(x, y, pts) {
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const [xi, yi] = pts[i];
    const [xj, yj] = pts[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

// 進入禁止エリア: 赤い半透明の塗り + 赤い枠 + 重心に「禁止」
function drawKeepout() {
  const zones = world.keepout;
  if (!zones.length) return;
  ctx.save();
  ctx.lineJoin = "round";
  ctx.font = "bold 12px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (const z of zones) {
    ctx.beginPath();
    z.pts.forEach(([x, y], i) => {
      const [sx, sy] = toScreen(x, y);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
    });
    ctx.closePath();
    ctx.fillStyle = "rgba(255,60,60,0.25)";
    ctx.fill();
    ctx.strokeStyle = "#ff3c3c";
    ctx.lineWidth = 2;
    ctx.stroke();
    const [cx, cy] = toScreen(...polyCentroid(z.pts));
    ctx.strokeStyle = "#000000aa";
    ctx.lineWidth = 3;
    ctx.strokeText("禁止", cx, cy);
    ctx.fillStyle = "#ffb3ad";
    ctx.fillText("禁止", cx, cy);
  }
  ctx.restore();
}

// 作図中の多角形: 角は点、辺は破線、最後の角 → 最初の角は薄い破線
function drawZoneDraft() {
  const pts = zoneDraw.pts.map(([x, y]) => toScreen(x, y));
  if (!pts.length) return;
  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  if (pts.length >= 3) {
    ctx.beginPath();
    pts.forEach(([sx, sy], i) => (i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy)));
    ctx.closePath();
    ctx.fillStyle = "rgba(255,60,60,0.12)";
    ctx.fill();
  }
  if (pts.length >= 2) {
    ctx.setLineDash([8, 6]);
    ctx.strokeStyle = "#ff6b6b";
    ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach(([sx, sy], i) => (i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy)));
    ctx.stroke();
    // 閉じる辺のプレビュー
    ctx.setLineDash([3, 6]);
    ctx.strokeStyle = "#ff6b6b99";
    ctx.beginPath();
    ctx.moveTo(pts[pts.length - 1][0], pts[pts.length - 1][1]);
    ctx.lineTo(pts[0][0], pts[0][1]);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  pts.forEach(([sx, sy], i) => {
    ctx.beginPath();
    // 3 点以上なら最初の角を大きく（ここをタップすると閉じる）
    ctx.arc(sx, sy, i === 0 && pts.length >= 3 ? 7 : 4.5, 0, Math.PI * 2);
    ctx.fillStyle = i === 0 ? "#ffffff" : "#ff3c3c";
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = i === 0 ? "#ff3c3c" : "#ffffff";
    ctx.stroke();
  });
  ctx.restore();
}

// 下り段差（RealSense で検出、5cm セル）を橙の四角で描く
function drawCliff() {
  const pts = world.cliff;
  if (!pts.length) return;
  const half = Math.max(2, 0.025 * view.scale);
  ctx.fillStyle = "rgba(255, 149, 0, 0.85)";
  ctx.beginPath();
  for (let i = 0; i + 1 < pts.length; i += 2) {
    const [sx, sy] = toScreen(pts[i], pts[i + 1]);
    ctx.rect(sx - half, sy - half, half * 2, half * 2);
  }
  ctx.fill();
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

// 補正プレビュー: 候補値と保存値の差だけ、ロボット位置を中心にスキャン点を回して描く
function drawScanPreview() {
  const pts = world.scan;
  if (!pts.length) return;
  const p = world.pose;
  const delta = calibDelta();
  const c = Math.cos(delta);
  const s = Math.sin(delta);
  ctx.fillStyle = "#ffd60a";
  ctx.beginPath();
  for (let i = 0; i + 1 < pts.length; i += 2) {
    let x = pts[i];
    let y = pts[i + 1];
    if (p) {
      const dx = x - p.x;
      const dy = y - p.y;
      x = p.x + c * dx - s * dy;
      y = p.y + s * dx + c * dy;
    }
    const [sx, sy] = toScreen(x, y);
    ctx.rect(sx - 2, sy - 2, 4, 4);
  }
  ctx.fill();
}

// ロボットの正面方向に 2 m の線と「前」ラベル
const HEADING_LINE_M = 2;
function drawHeadingLine() {
  const p = world.pose;
  if (!p) return;
  const [sx, sy] = toScreen(p.x, p.y);
  const [ex, ey] = toScreen(p.x + HEADING_LINE_M * Math.cos(p.yaw), p.y + HEADING_LINE_M * Math.sin(p.yaw));
  ctx.save();
  ctx.lineCap = "round";
  ctx.setLineDash([8, 6]);
  ctx.strokeStyle = "#00e5ff";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.font = "bold 14px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  const lx = ex + 12 * Math.cos(p.yaw);
  const ly = ey - 12 * Math.sin(p.yaw);
  ctx.strokeStyle = "#000000cc";
  ctx.lineWidth = 3;
  ctx.strokeText("前", lx, ly);
  ctx.fillStyle = "#00e5ff";
  ctx.fillText("前", lx, ly);
  ctx.restore();
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
let gestureMulti = false; // 今回の操作で 2 本指になった（タップ扱いしない）

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
  if (on) setZoneDraw(false);
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
  gestureMulti = pointers.size >= 2; // 1 本目で解除、2 本目以降で設定（全部離すまで維持）
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
  const had = pointers.delete(e.pointerId);
  if (!had || pointers.size > 0) return;
  // 1 本指で、ほとんど動かさずに離した = タップ
  const tap = !canceled && !gestureMulti && panMoved <= TAP_PX;
  gestureMulti = false;
  if (tap) {
    const [sx, sy] = localPos(e);
    onMapTap(sx, sy);
  }
}
canvas.addEventListener("pointerup", (e) => endPointer(e, false));
canvas.addEventListener("pointercancel", (e) => endPointer(e, true));

// 地図のタップ: 作図中は角を追加、通常時は進入禁止エリアの削除
function onMapTap(sx, sy) {
  if (zoneDraw.on) {
    addZonePoint(sx, sy);
    return;
  }
  if (goalMode) return;
  const [x, y] = toWorld(sx, sy);
  let zone = null;
  for (let i = world.keepout.length - 1; i >= 0; i--) {
    if (pointInPoly(x, y, world.keepout[i].pts)) {
      zone = world.keepout[i];
      break;
    }
  }
  if (!zone) return;
  if (!wsOpen()) {
    toast("未接続のため削除できません", "warn");
    return;
  }
  resetInput(); // ダイアログ中は keyup が届かないため
  if (!window.confirm("この進入禁止エリアを削除しますか？")) return;
  send({ t: "keepout_remove", id: zone.id });
}

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
$("btn-reset").addEventListener("click", () => {
  if (!wsOpen()) {
    toast("未接続のためリセットできません", "warn");
    return;
  }
  resetInput(); // ダイアログ中は keyup が届かないため
  if (!window.confirm("地図を消して作り直しますか？")) return;
  send({ t: "reset_map" });
});
$("btn-explore").addEventListener("click", toggleExplore);
$("btn-skip").addEventListener("click", () => {
  if (!wsOpen()) {
    toast("未接続のためスキップできません", "warn");
    return;
  }
  send({ t: "explore_skip" });
});
$("btn-keepout").addEventListener("click", () => setZoneDraw(!zoneDraw.on));
$("btn-calib").addEventListener("click", () => {
  if (calib.open) closeCalib();
  else openCalib();
});

// ==== LiDAR 補正パネル ====
const RAD = Math.PI / 180;

// 角度 [deg] を -180〜180 に収める
function wrapDeg(d) {
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}
function fmtDeg(d) {
  const r = Math.round(d);
  return `${r > 0 ? "+" : ""}${r}°`;
}
function savedDeg() {
  return calib.saved === null ? 0 : wrapDeg(Math.round(calib.saved / RAD));
}
// プレビューで回す量 [rad] = 候補 - 保存値
function calibDelta() {
  return calib.candDeg * RAD - (calib.saved === null ? 0 : calib.saved);
}

function updateCalibUI() {
  $("calib-saved").textContent = calib.saved === null ? "—" : fmtDeg(savedDeg());
  $("calib-cand").textContent = fmtDeg(calib.candDeg);
  $("calib-apply").disabled = !wsOpen();
  $("calib-floor").disabled = !wsOpen();
  const cam = $("calib-cam");
  const c = calib.cam;
  if (c) {
    const deg = (r) => (r === null ? "—" : `${(r / RAD).toFixed(1)}°`);
    const z = c.z === null ? "—" : `${c.z.toFixed(2)} m`;
    cam.textContent = `カメラ: 下向き ${deg(c.pitch)} / 横 ${deg(c.roll)} / 高さ ${z}`;
  }
  cam.hidden = !c;
}

function setCandidate(d) {
  calib.candDeg = wrapDeg(Math.round(d));
  $("calib-slider").value = String(calib.candDeg);
  updateCalibUI();
  requestDraw();
}

function openCalib() {
  setGoalMode(false);
  setZoneDraw(false);
  calib.open = true;
  setCandidate(savedDeg()); // 開くたびに保存値に合わせる
  $("calib").hidden = false;
  $("view").classList.add("calib-open");
  $("btn-calib").classList.add("on");
  requestDraw();
}

function closeCalib() {
  calib.open = false;
  $("calib").hidden = true;
  $("view").classList.remove("calib-open");
  $("btn-calib").classList.remove("on");
  requestDraw();
}

$("calib-slider").addEventListener("input", (e) => setCandidate(Number(e.target.value)));
$("calib-slider").addEventListener("change", (e) => e.target.blur()); // キーボード操作（WASD）に戻す
for (const b of document.querySelectorAll("#calib .calib-steps button")) {
  b.addEventListener("click", () => setCandidate(calib.candDeg + Number(b.dataset.step)));
}
$("calib-cancel").addEventListener("click", closeCalib);
$("calib-apply").addEventListener("click", () => {
  if (!wsOpen()) {
    toast("未接続のため適用できません", "warn");
    return;
  }
  const yaw = Math.round(calib.candDeg * RAD * 10000) / 10000;
  send({ t: "set_lidar_yaw", yaw });
  resetInput(); // ダイアログ中は keyup が届かないため
  if (window.confirm("地図をリセットしますか？（向きを変えたあとは作り直しを推奨）")) {
    send({ t: "reset_map" });
  }
  closeCalib();
});
$("calib-floor").addEventListener("click", () => {
  if (!wsOpen()) {
    toast("未接続のため補正できません", "warn");
    return;
  }
  send({ t: "calib_floor" });
  toast("カメラの床補正を開始しました");
});

// ==== 進入禁止エリアの作図 ====
function setZoneDraw(on) {
  if (on) {
    setGoalMode(false);
    if (calib.open) closeCalib();
  }
  zoneDraw.on = on;
  zoneDraw.pts = [];
  $("btn-keepout").classList.toggle("on", on);
  $("view").classList.toggle("zone-mode", on);
  $("keepout-bar").hidden = !on;
  updateZoneBar();
  requestDraw();
}

function updateZoneBar() {
  const n = zoneDraw.pts.length;
  $("keepout-done").disabled = n < 3;
  $("keepout-undo").disabled = n === 0;
  $("keepout-clear").disabled = !wsOpen() || world.keepout.length === 0;
}

function addZonePoint(sx, sy) {
  const pts = zoneDraw.pts;
  if (pts.length >= 3) {
    const [fx, fy] = toScreen(pts[0][0], pts[0][1]);
    if (Math.hypot(sx - fx, sy - fy) < ZONE_CLOSE_PX) {
      finishZone();
      return;
    }
  }
  pts.push(toWorld(sx, sy));
  updateZoneBar();
  requestDraw();
}

function finishZone() {
  if (zoneDraw.pts.length < 3) return;
  if (!wsOpen()) {
    toast("未接続のため進入禁止エリアを送れません", "warn");
    return;
  }
  send({ t: "keepout_add", pts: zoneDraw.pts.map(([x, y]) => [round3(x), round3(y)]) });
  setZoneDraw(false);
}

$("keepout-done").addEventListener("click", finishZone);
$("keepout-undo").addEventListener("click", () => {
  zoneDraw.pts.pop();
  updateZoneBar();
  requestDraw();
});
$("keepout-quit").addEventListener("click", () => setZoneDraw(false));
$("keepout-clear").addEventListener("click", () => {
  if (!wsOpen()) {
    toast("未接続のため消せません", "warn");
    return;
  }
  resetInput(); // ダイアログ中は keyup が届かないため
  if (!window.confirm("進入禁止エリアをすべて消しますか？")) return;
  send({ t: "keepout_clear" });
});

// ボタンにフォーカスが残ると Space で押されてしまうので外す
for (const b of document.querySelectorAll(".toolbar button, #calib button, #keepout-bar button, #btn-skip")) {
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
  if (k === "escape" && zoneDraw.on) {
    setZoneDraw(false);
    return;
  }
  if (k === "escape" && calib.open) {
    closeCalib();
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

// ==== 自動探索 ====
const EXPLORE_CONFIRM = "ロボットが自動で走り回って地図を作ります。周りに人や障害物がないことを確認してください。開始しますか？";

function setExplore(state, frontiers) {
  explore.state = state;
  explore.frontiers = frontiers;
  const running = state === "running";
  const btn = $("btn-explore");
  btn.textContent = running ? "探索停止" : "自動探索";
  btn.classList.toggle("on", running);
  const badge = $("explore");
  badge.textContent = running ? (frontiers === null ? "探索中" : `探索中（残り ${frontiers}）`) : "";
  badge.className = "badge" + (running ? " ok" : "");
  $("btn-skip").hidden = !running;
}

function toggleExplore() {
  if (!wsOpen()) {
    toast("未接続のため自動探索を操作できません", "warn");
    return;
  }
  if (explore.state === "running") {
    send({ t: "explore", on: false });
    return;
  }
  setGoalMode(false);
  resetInput(); // ダイアログ中は keyup が届かないため
  if (!window.confirm(EXPLORE_CONFIRM)) return;
  send({ t: "explore", on: true });
}

// ==== ゲームパッド（DualShock / Gamepad API）====
// Gamepad API は https（セキュアコンテキスト）でしか使えない
function padSupported() {
  return typeof navigator.getGamepads === "function" && window.isSecureContext === true;
}

// mapping === "standard" の最初のパッド、なければ最初のパッド
function findPad() {
  if (!padSupported()) return null;
  let list;
  try {
    list = navigator.getGamepads() || [];
  } catch (_) {
    return null;
  }
  let first = null;
  for (const p of list) {
    if (!p || p.connected === false) continue;
    if (p.mapping === "standard") return p;
    if (!first) first = p;
  }
  return first;
}

// 円形の不感帯。外側は 0〜1 に引き伸ばす
function padDeadzone(x, y) {
  x = Number.isFinite(x) ? x : 0;
  y = Number.isFinite(y) ? y : 0;
  const m = Math.hypot(x, y);
  if (m < PAD_DEADZONE) return [0, 0];
  const s = Math.min(1, (m - PAD_DEADZONE) / (1 - PAD_DEADZONE)) / m;
  return [x * s, y * s];
}

function padPressed(p, i) {
  const b = p.buttons[i];
  return !!b && (b.pressed || b.value > 0.5);
}

function updatePadUI() {
  $("pad").textContent = gp.connected ? "コントローラー" : "";
}

function pollGamepad() {
  const p = findPad();
  const wasActive = gp.active;
  if (!p) {
    if (gp.connected) {
      gp.connected = false;
      gp.prev = [];
      updatePadUI();
    }
    gp.active = false;
    gp.fwd = gp.turn = 0;
    if (wasActive) driveTick(); // 切断 = 離したのと同じ扱い（0 を送る）
    return;
  }
  if (!gp.connected) {
    gp.connected = true;
    updatePadUI();
  }

  const pressed = Array.from(p.buttons, (_, i) => padPressed(p, i));
  const firstPoll = gp.prev.length === 0;
  const edge = (i) => pressed[i] && !gp.prev[i];
  gp.prev = pressed;

  // 操縦（L1 = デッドマン）
  const l1 = pressed[PAD_BTN.l1];
  if (!l1) gp.blocked = false;
  let fwd = 0;
  let turn = 0;
  if (l1 && !gp.blocked && sending) {
    const [, ly] = padDeadzone(p.axes[0], p.axes[1]);
    const [rx] = padDeadzone(p.axes[2], p.axes[3]);
    const k = pressed[PAD_BTN.r1] ? PAD_SLOW : 1;
    fwd = -ly * k; // 上 = マイナス
    turn = rx * k; // 右 = プラス（driveTick で右旋回 = w マイナスにする）
  }
  gp.active = fwd !== 0 || turn !== 0;
  gp.fwd = fwd;
  gp.turn = turn;
  if (wasActive && !gp.active && !joy.active) driveTick(); // 即座に 0 を送る

  // ボタン（押した瞬間だけ）。✕ は接続直後の押下でも効かせる
  if (edge(PAD_BTN.cross)) {
    emergencyStop();
    return;
  }
  if (firstPoll) return;
  if (edge(PAD_BTN.triangle)) {
    setGoalMode(false);
    send({ t: "cancel" });
  }
  if (edge(PAD_BTN.options)) {
    toggleExplore();
  }
}

function padLoop() {
  pollGamepad();
  requestAnimationFrame(padLoop);
}

function setupPadHint() {
  if (padSupported()) return;
  try {
    if (localStorage.getItem(STORE_PAD_HINT)) return;
  } catch (_) { /* 無視 */ }
  const text = $("pad-hint-text");
  if (location.protocol === "https:" && window.isSecureContext) {
    text.textContent = "このブラウザはコントローラー（Gamepad API）に対応していません";
  } else {
    const url = `https://${location.hostname}:${HTTPS_PORT}/`;
    const a = document.createElement("a");
    a.href = url;
    a.textContent = url;
    text.append("DualShock を使うには ", a, " で開いてください（証明書の警告は「詳細」→「続行」）");
  }
  $("pad-hint").hidden = false;
  $("pad-hint-close").addEventListener("click", () => {
    $("pad-hint").hidden = true;
    try { localStorage.setItem(STORE_PAD_HINT, "1"); } catch (_) { /* 無視 */ }
    requestAnimationFrame(resize);
  });
}

if (padSupported()) {
  window.addEventListener("gamepadconnected", pollGamepad);
  window.addEventListener("gamepaddisconnected", pollGamepad);
  requestAnimationFrame(padLoop);
}
setupPadHint();

// ==== 起動 ====
setExplore("off", null);
resize();
connect();
