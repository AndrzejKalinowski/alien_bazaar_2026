// Supervisor panel. Server data is inserted with textContent only.
// Commands: POST -> 202 with a command ID -> poll /api/commands/{id} for the
// supervisor's reply. HTTP acceptance alone is never shown as success.
"use strict";

const $ = (id) => document.getElementById(id);
const STALE_AFTER = 2000;      // ms without a status message -> link marked lost
const POLL_PERIOD = 200;       // ms, command result polling
const POLL_LIMIT = 50;         // polls before giving up (10 s)
const LOG_LIMIT = 200;         // events kept in the list
const CAMERA_STALE = 1.0;      // s, camera frame age shown as stale

let status = null;
let lastStatusAt = 0;
let cameraStarted = false;

// --- token --------------------------------------------------------------------
function storedToken() {
  try { return sessionStorage.getItem("supervisorToken") || ""; } catch { return ""; }
}
function storeToken(value) {
  try { sessionStorage.setItem("supervisorToken", value); } catch { /* private mode */ }
}
(function readTokenFromHash() {
  const match = location.hash.match(/token=([^&]+)/);
  if (match) {
    storeToken(decodeURIComponent(match[1]));
    history.replaceState(null, "", location.pathname);  // keep it out of the address bar
  }
  $("token").value = storedToken();
  $("token").addEventListener("change", (e) => storeToken(e.target.value.trim()));
  $("token-box").open = !storedToken();
})();

function requestId() {
  // crypto.randomUUID needs a secure context; plain-HTTP LAN pages do not have one.
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");
}

// --- commands -----------------------------------------------------------------
function say(text, bad = false) {
  $("reply").textContent = text;
  $("reply").className = bad ? "fault" : "muted";
}

async function post(path, body, retries = 0) {
  for (let attempt = 0; ; attempt++) {
    try {
      const response = await fetch(path, {
        method: "POST",
        headers: { "Content-Type": "application/json", "X-Supervisor-Token": storedToken() },
        body: JSON.stringify(body),
      });
      const data = await response.json();
      if (response.status !== 202) throw new Error(data.error || data.message || response.statusText);
      return data;
    } catch (error) {
      // Only idempotent commands (START with the same request_id, STOP) are retried.
      if (attempt >= retries || !(error instanceof TypeError)) throw error;
    }
  }
}

async function command(label, path, body = {}, retries = 0) {
  say(`${label}: wysłano…`);
  let record;
  try {
    record = await post(path, body, retries);
  } catch (error) {
    say(`${label}: odrzucono (${error.message}). Sprawdź stan przed ponowieniem.`, true);
    return;
  }
  for (let i = 0; i < POLL_LIMIT && record.status === "queued"; i++) {
    await new Promise((resolve) => setTimeout(resolve, POLL_PERIOD));
    try {
      const response = await fetch(`/api/commands/${encodeURIComponent(record.id)}`);
      if (response.ok) record = await response.json();
    } catch { /* keep polling; the status stream shows the actual state */ }
  }
  if (record.status === "queued") say(`${label}: brak odpowiedzi nadzorcy, sprawdź stan`, true);
  else if (record.status === "cancelled") say(`${label}: anulowano (${record.message})`, true);
  else say(`${label}: ${record.message}`, !record.accepted);
}

$("start").addEventListener("click", () =>
  command("START", "/api/batch/start", { request_id: requestId() }, 2));
$("stop").addEventListener("click", () => command("STOP", "/api/stop", {}, 2));
$("reset").addEventListener("click", () => command("RESET", "/api/fault/reset"));
$("add").addEventListener("click", () =>
  command("Dodaj szklanki", "/api/sim/add-glasses", { count: Number($("add-count").value) }));
$("clear").addEventListener("click", () => {
  const ids = [...document.querySelectorAll("#outputs input:checked")].map((box) => box.value);
  if (!ids.length) { say("Zaznacz opróżnione miejsca", true); return; }
  command("Opróżnienie odbioru", "/api/output/confirm-cleared", { slot_ids: ids });
});

// --- rendering ----------------------------------------------------------------
const STATE_PL = {
  READY: "GOTOWY", RUNNING: "PRACA", WAITING_OUTPUT: "CZEKA NA ODBIÓR", STOPPING: "ZATRZYMYWANIE",
  STOPPED: "ZATRZYMANY", FAULT: "BŁĄD", COMPLETED: "ZAKOŃCZONO",
};

function row(cells) {
  const tr = document.createElement("tr");
  for (const cell of cells) {
    const td = document.createElement("td");
    if (cell instanceof Node) td.append(cell); else td.textContent = cell ?? "";
    tr.append(td);
  }
  return tr;
}

function render(s) {
  status = s;
  const state = s.state;
  $("state").textContent = STATE_PL[state] || state;
  $("state").className = `state ${state}`;
  $("operation").textContent = s.operation ? `${s.target_id}: ${s.operation}` : (s.target_id || "");
  $("fault").hidden = !s.fault;
  $("fault").textContent = s.fault;
  $("loop-warning").hidden = s.loop.alive;
  $("loop-warning").textContent = s.loop.error || "Pętla sterowania nie odpowiada";

  $("start").disabled = !["READY", "COMPLETED"].includes(state);
  $("reset").disabled = !["STOPPED", "FAULT"].includes(state);
  $("clear").disabled = ["RUNNING", "STOPPING"].includes(state);

  const simulated = s.world && s.world.simulated;
  $("mode").textContent = simulated
    ? (s.world.camera_targets ? "SYMULACJA ROBOTA + KAMERA" : "SYMULACJA") : "SPRZĘT";
  $("sim-tools").hidden = !simulated || s.world.camera_targets;

  const c = s.counts || {};
  $("counts").textContent = s.batch_id
    ? `Ukończone ${c.COMPLETED || 0}, oczekuje ${c.PENDING || 0}, w toku ${c.ACTIVE || 0}, ` +
      `pominięte ${c.SKIPPED || 0}, przerwane ${c.INTERRUPTED || 0}`
    : "brak partii";

  const recipe = $("recipe");
  if (recipe.children.length !== s.recipe.length) {
    recipe.replaceChildren(...s.recipe.map((step) => {
      const li = document.createElement("li"); li.textContent = step; return li;
    }));
  }
  [...recipe.children].forEach((li, i) => {
    li.className = s.step_index === null ? "" : i < s.step_index ? "done" : i === s.step_index ? "active" : "";
  });

  const checked = new Set([...document.querySelectorAll("#outputs input:checked")].map((b) => b.value));
  $("outputs").replaceChildren(...Object.entries(s.outputs).map(([id, slot]) => {
    const box = document.createElement("input");
    box.type = "checkbox"; box.value = id; box.checked = checked.has(id);
    box.disabled = slot.state === "FREE" || slot.state === "RESERVED";
    box.setAttribute("aria-label", `opróżniono ${id}`);
    return row([box, id, slot.state, slot.target_id]);
  }));
  $("targets").replaceChildren(...s.targets.map((t) => row([t.id, t.state, t.detail])));

  renderView(s);
}

function renderView(s) {
  const camera = s.camera || { available: false };
  $("camera").hidden = !camera.available;
  $("camera-stale").hidden = !camera.available || (camera.age !== null && camera.age < CAMERA_STALE);
  if (camera.available && !cameraStarted) { $("camera").src = "/camera.mjpg"; cameraStarted = true; }
  const showMap = !!(s.world && s.world.simulated);
  $("map").hidden = !showMap;
  $("map-note").hidden = !showMap;
  $("view-title").textContent = camera.available ? "Kamera nad stołem" : "Podgląd";
  if (showMap) drawMap(s);
}

function drawMap(s) {
  const world = s.world, canvas = $("map"), ctx = canvas.getContext("2d");
  const css = getComputedStyle(document.documentElement);
  const color = (name) => css.getPropertyValue(name).trim();
  const xs = [], ys = [];
  for (const st of world.stations) { xs.push(st.low[0], st.high[0]); ys.push(st.low[1], st.high[1]); }
  for (const t of world.targets) { xs.push(t.x); ys.push(t.y); }
  for (const o of Object.values(world.outputs)) { xs.push(o.x); ys.push(o.y); }
  xs.push(world.tcp[0]); ys.push(world.tcp[1]);
  const pad = 0.12, minX = Math.min(...xs) - pad, maxX = Math.max(...xs) + pad;
  const minY = Math.min(...ys) - pad, maxY = Math.max(...ys) + pad;
  const scale = Math.min(canvas.width / (maxX - minX), canvas.height / (maxY - minY));
  const px = (x, y) => [(x - minX) * scale, canvas.height - (y - minY) * scale];  // base +y is up

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.font = "12px system-ui, sans-serif";
  ctx.lineWidth = 2;
  for (const st of world.stations) {
    const [x0, y0] = px(st.low[0], st.high[1]), [x1, y1] = px(st.high[0], st.low[1]);
    ctx.strokeStyle = color("--muted"); ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    ctx.fillStyle = color("--muted");
    ctx.fillText(`${st.id}  z≤${st.high[2].toFixed(2)} m`, x0 + 4, y0 + 14);
  }
  for (const [id, o] of Object.entries(world.outputs)) {
    const [x, y] = px(o.x, o.y), slot = s.outputs[id];
    const shown = slot && slot.state !== "FREE";
    ctx.strokeStyle = color("--info"); ctx.strokeRect(x - 12, y - 12, 24, 24);
    ctx.fillStyle = color("--info"); ctx.fillText(id, x - 12, y + 26);
    if (shown && o.target_id) { ctx.beginPath(); ctx.arc(x, y, 8, 0, 2 * Math.PI); ctx.fill(); }
  }
  const states = Object.fromEntries(s.targets.map((t) => [t.id, t.state]));
  for (const t of world.targets) {
    const [x, y] = px(t.x, t.y), state = states[t.id];
    ctx.strokeStyle = state === "ACTIVE" ? color("--go") : state ? color("--text") : color("--idle");
    ctx.beginPath(); ctx.arc(x, y, 10, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = ctx.strokeStyle; ctx.fillText(t.id + (state ? "" : " (poza partią)"), x + 13, y + 4);
  }
  const [tx, ty] = px(world.tcp[0], world.tcp[1]);
  ctx.strokeStyle = world.grip === "OK" ? color("--go") : color("--warn");
  ctx.beginPath(); ctx.moveTo(tx - 9, ty); ctx.lineTo(tx + 9, ty); ctx.moveTo(tx, ty - 9); ctx.lineTo(tx, ty + 9); ctx.stroke();
  ctx.fillStyle = ctx.strokeStyle;
  ctx.fillText(`TCP z=${world.tcp[2].toFixed(2)} m, chwyt ${world.grip}, ${world.orientation}`, tx + 12, ty - 8);
}

function logEvent(event) {
  const li = document.createElement("li");
  const bad = ["fault", "stop_requested", "operation_cancelled", "target_skipped"].includes(event.event);
  li.className = bad ? "bad" : "";
  li.textContent = `${event.event}  ${event.target_id || ""}  ${event.message}`;
  $("log").prepend(li);
  while ($("log").children.length > LOG_LIMIT) $("log").lastChild.remove();
}

// --- live connection ----------------------------------------------------------
function setLink(ok, text) {
  $("link").textContent = text;
  $("link").className = `tag ${ok ? "ok" : "bad"}`;
}

function connect() {
  const source = new EventSource("/api/events");
  source.addEventListener("status", (e) => { lastStatusAt = Date.now(); render(JSON.parse(e.data)); });
  source.addEventListener("supervisor", (e) => logEvent(JSON.parse(e.data)));
  source.addEventListener("gap", () => logEvent({ event: "gap", message: "pominięto starsze zdarzenia" }));
  source.onerror = () => setLink(false, "łączenie…");  // EventSource reconnects by itself
}

setInterval(() => {
  const fresh = Date.now() - lastStatusAt < STALE_AFTER;
  if (fresh) setLink(status && status.loop.alive, status && status.loop.alive ? "połączono" : "pętla nie odpowiada");
  else setLink(false, "brak aktualnych danych");
  // Controls act on what the operator sees; do not offer them against stale data (STOP stays).
  for (const id of ["start", "reset", "clear", "add"]) if (!fresh) $(id).disabled = true;
}, 500);

fetch("/api/status").then((r) => r.json()).then((s) => { lastStatusAt = Date.now(); render(s); }).catch(() => {});
connect();
