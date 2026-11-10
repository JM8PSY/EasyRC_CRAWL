"use strict";

const WS_PORT = 81;
const SEND_INTERVAL_MS = 50;
const RECONNECT_MS = 1000;
const TURN_KEY = 0.6;
const SLOW_FACTOR = 0.4;

const $ = (id) => document.getElementById(id);

let ws = null;
let role = null;
let sending = !document.hidden;
let maxSpeed = 1; // ESP32 から受け取った出力上限 (0〜1)
let draggingMax = false;
const keys = new Set();
const joy = { active: false, fwd: 0, turn: 0 };

// 前後・旋回 → 左右速度。どちらかが 1 を超えたら比率を保って正規化する
function mix(fwd, turn) {
  const l = fwd + turn;
  const r = fwd - turn;
  const m = Math.max(1, Math.abs(l), Math.abs(r));
  return [l / m, r / m];
}

function currentInput() {
  if (joy.active) return mix(joy.fwd, joy.turn);
  let fwd = (keys.has("w") ? 1 : 0) - (keys.has("s") ? 1 : 0);
  let turn = ((keys.has("d") ? 1 : 0) - (keys.has("a") ? 1 : 0)) * TURN_KEY;
  if (keys.has("shift")) {
    fwd *= SLOW_FACTOR;
    turn *= SLOW_FACTOR;
  }
  return mix(fwd, turn);
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN && role === "driver") ws.send(JSON.stringify(obj));
}

function resetInput() {
  keys.clear();
  joy.active = false;
  joy.fwd = joy.turn = 0;
  $("knob").style.transform = "";
}

function emergencyStop() {
  resetInput();
  send({ stop: true });
}

// ---- 表示 ----
function setMeter(side, v) {
  const bar = $("bar-" + side);
  bar.style.height = `${Math.abs(v) * 50}%`;
  bar.classList.toggle("rev", v < 0);
  $("val-" + side).textContent = v.toFixed(2);
}

function showMax(percent) {
  $("max-val").textContent = `${percent}%`;
}

function applyMax(v) {
  maxSpeed = v;
  if (draggingMax) return; // 操作中のスライダーは動かさない
  const percent = Math.round(v * 100);
  $("max").value = percent;
  showMax(percent);
}

function setConn(text, cls) {
  const el = $("conn");
  el.textContent = text;
  el.className = "badge " + cls;
}

// ---- WebSocket ----
function connect() {
  ws = new WebSocket(`ws://${location.hostname}:${WS_PORT}/`);
  setConn("接続中…", "warn");
  ws.onmessage = (ev) => {
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch {
      return;
    }
    if (msg.role) {
      role = msg.role;
      setConn("接続", "ok");
      $("role").textContent = role === "driver" ? "操縦者" : "閲覧のみ（他の端末が操縦中）";
      $("role").className = "badge " + (role === "driver" ? "ok" : "warn");
      document.body.classList.toggle("viewer", role !== "driver");
    }
    if (typeof msg.max === "number") applyMax(msg.max);
  };
  ws.onclose = () => {
    role = null;
    setConn("切断", "off");
    $("role").textContent = "";
    resetInput();
    setTimeout(connect, RECONNECT_MS);
  };
}

setInterval(() => {
  const [l, r] = currentInput();
  // バーは実際の duty（入力 × 出力上限）を表示する
  setMeter("l", l * maxSpeed);
  setMeter("r", r * maxSpeed);
  if (sending) send({ l, r });
}, SEND_INTERVAL_MS);

// ---- 出力上限スライダー ----
$("max").addEventListener("input", (e) => {
  draggingMax = true;
  showMax(e.target.value);
});
// 指を離したときだけ送る（本体への保存回数を抑える）
$("max").addEventListener("change", (e) => {
  draggingMax = false;
  send({ max: Number(e.target.value) / 100 });
  e.target.blur(); // キーボード操作（WASD）に戻す
});

// ---- キーボード ----
const KEY_MAP = { w: "w", a: "a", s: "s", d: "d", shift: "shift" };

window.addEventListener("keydown", (e) => {
  const k = e.key.toLowerCase();
  if (k === " ") {
    e.preventDefault();
    emergencyStop();
    return;
  }
  if (KEY_MAP[k]) keys.add(KEY_MAP[k]);
});
window.addEventListener("keyup", (e) => keys.delete(e.key.toLowerCase()));
// フォーカスを失うと keyup が届かず押しっぱなし扱いになるため全解除
window.addEventListener("blur", () => keys.clear());

// ---- タブ非表示で停止 ----
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    emergencyStop();
    sending = false;
  } else {
    sending = true;
  }
});

// ---- 停止ボタン ----
$("stop").addEventListener("pointerdown", (e) => {
  e.preventDefault();
  emergencyStop();
});

// ---- 仮想ジョイスティック ----
(() => {
  const pad = $("joy");
  const knob = $("knob");
  let pointerId = null;

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
    if (e.pointerId !== pointerId) return;
    pointerId = null;
    joy.active = false;
    joy.fwd = joy.turn = 0;
    knob.style.transform = "";
  }

  pad.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    pointerId = e.pointerId;
    pad.setPointerCapture(pointerId);
    joy.active = true;
    update(e);
  });
  pad.addEventListener("pointermove", (e) => {
    if (e.pointerId === pointerId) update(e);
  });
  pad.addEventListener("pointerup", release);
  pad.addEventListener("pointercancel", release);
})();

connect();
