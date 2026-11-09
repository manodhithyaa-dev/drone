/* SKYCTRL - web ground control station (SIMULATION ONLY)
 *
 * CONTROL OBJECT (shared by joysticks, gestures, 3D view, telemetry, output adapter):
 *   controlState = { throttle, yaw, pitch, roll }, every axis normalized to [-1, +1]
 * SIGN CONVENTION:
 *   throttle +1 = climb (stick centered = hover; it is a simulated climb-rate command)
 *   yaw      +1 = rotate clockwise seen from above (nose right)
 *   pitch    +1 = nose down / move forward
 *   roll     +1 = bank right (right side down)
 * Pipeline: source -> normalize/validate -> dead zone -> limits -> smoothing -> controlState
 *           -> 3D view -> telemetry -> output adapter. One requestAnimationFrame loop drives all.
 * The 3D model shows REQUESTED state, not measured attitude. All telemetry here is simulated.
 */
const $ = (id) => document.getElementById(id);
const clamp = (v, a = -1, b = 1) => Math.min(b, Math.max(a, v));
const AXES = ['throttle', 'yaw', 'pitch', 'roll'];
const CONFIG = { tiltDeg: 25, maxAltitude: 50, maxClimb: 3, yawRate: 90, publishHz: 20, uiHz: 12, groundY: 0.53, altScale: 0.08, manualDeadZone: 0.05, manualSmoothing: 18, gestureRange: 0.22, minConfidence: 0.6 };
const controlState = { throttle: 0, yaw: 0, pitch: 0, roll: 0 };

const sgn = (v) => { const r = Math.round(v * 100) / 100; return (r < 0 ? '-' : '+') + Math.abs(r).toFixed(2); };
const pct = (v) => { const r = Math.round(v * 100); return (r < 0 ? '-' : '+') + Math.abs(r) + '%'; };
let toastTimer = 0;
function toast(msg) { const t = $('toast'); t.textContent = msg; t.classList.add('show'); clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('show'), 3200); }
const loadScript = (src) => new Promise((res, rej) => { const s = document.createElement('script'); s.src = src; s.crossOrigin = 'anonymous'; s.onload = res; s.onerror = () => rej(new Error('Failed to load ' + src)); document.head.appendChild(s); });

/* ---------------- Communication layer ----------------
 * ControlAdapter interface: { name, status, connect(), disconnect(), send(msg) -> bool, onStatus, onMessage }
 * Wire format (JSON): {type:'control', seq, t, armed, throttle, yaw, pitch, roll} | {type:'event', name}
 * A real link needs a local bridge/flight controller speaking a validated protocol (e.g. MAVLink via a
 * backend); browsers cannot open arbitrary serial ports to ESCs, and raw sticks must never drive motors. */
class SimulatedAdapter {
  constructor() { this.name = 'Simulated'; this.status = 'connected'; this.lines = []; this.sent = 0; }
  async connect() { this.status = 'connected'; }
  disconnect() { this.status = 'disconnected'; }
  send(msg) { this.sent++; this.lines.push(JSON.stringify(msg)); if (this.lines.length > 6) this.lines.shift(); return true; } // logs only
}
class WebSocketAdapter { // ready for a future local bridge; not wired to the UI
  constructor(url) { this.name = 'WebSocket'; this.url = url; this.status = 'disconnected'; this.ws = null; this.onStatus = () => {}; this.onMessage = () => {}; }
  connect() {
    return new Promise((res, rej) => {
      try { this.ws = new WebSocket(this.url); } catch (e) { this.status = 'error'; this.onStatus(this.status); return rej(e); }
      this.status = 'connecting'; this.onStatus(this.status);
      this.ws.onopen = () => { this.status = 'connected'; this.onStatus(this.status); res(); };
      this.ws.onerror = () => { this.status = 'error'; this.onStatus(this.status); rej(new Error('WebSocket error')); };
      this.ws.onclose = () => { this.status = 'disconnected'; this.onStatus(this.status); };
      this.ws.onmessage = (e) => { try { this.onMessage(JSON.parse(e.data)); } catch { /* ignore malformed */ } };
    });
  }
  disconnect() { this.ws?.close(); }
  send(msg) { if (this.ws?.readyState !== 1) return false; this.ws.send(JSON.stringify(msg)); return true; }
}
class ConnectionManager {
  constructor() { this.adapter = new SimulatedAdapter(); this.seq = 0; }
  get label() { return this.adapter.name + ' link (' + this.adapter.status + ')'; }
  publish(msg) { return this.adapter.send({ seq: this.seq++, t: Date.now(), ...msg }); }
}

/* ---------------- Joystick ---------------- */
class JoystickController {
  constructor(el) {
    this.el = el; this.knob = el.querySelector('.knob'); this.x = 0; this.y = 0; this.pid = null; this.enabled = false; this.holdY = false;
    this.h = { down: (e) => this.onDown(e), move: (e) => this.onMove(e), up: (e) => this.onUp(e), key: (e) => this.onKey(e) };
    el.addEventListener('pointerdown', this.h.down); el.addEventListener('pointermove', this.h.move);
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((t) => el.addEventListener(t, this.h.up));
    el.addEventListener('keydown', this.h.key); this.render();
  }
  destroy() {
    this.el.removeEventListener('pointerdown', this.h.down); this.el.removeEventListener('pointermove', this.h.move);
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((t) => this.el.removeEventListener(t, this.h.up));
    this.el.removeEventListener('keydown', this.h.key);
  }
  setEnabled(on) { this.enabled = on; this.el.classList.toggle('disabled', !on); this.el.setAttribute('aria-disabled', String(!on)); this.el.tabIndex = on ? 0 : -1; if (!on) this.release(true); }
  fromEvent(e) { // circular clamp, +y = up
    const r = this.el.getBoundingClientRect(), rad = r.width * 0.36;
    let dx = (e.clientX - (r.left + r.width / 2)) / rad, dy = ((r.top + r.height / 2) - e.clientY) / rad;
    const len = Math.hypot(dx, dy); if (len > 1) { dx /= len; dy /= len; }
    this.x = dx; this.y = dy; this.render();
  }
  onDown(e) { if (!this.enabled || this.pid !== null) return; e.preventDefault(); this.pid = e.pointerId; try { this.el.setPointerCapture(e.pointerId); } catch { /* ignore */ } this.el.classList.add('dragging'); this.el.focus({ preventScroll: true }); this.fromEvent(e); }
  onMove(e) { if (e.pointerId === this.pid) { e.preventDefault(); this.fromEvent(e); } }
  onUp(e) { if (e.pointerId !== this.pid) return; try { this.el.releasePointerCapture(e.pointerId); } catch { /* already released */ } this.pid = null; this.el.classList.remove('dragging'); this.release(false); }
  release(force) { this.x = 0; if (force || !this.holdY) this.y = 0; this.render(); }
  setValue(x, y) { this.x = clamp(x); this.y = clamp(y); this.render(); }
  onKey(e) { // arrows/WASD are handled globally (see readKeyboard); here only centering
    if (this.enabled && (e.key === ' ' || e.key === 'Escape' || e.key === 'Home')) { e.preventDefault(); this.x = 0; this.y = 0; this.render(); }
  }
  render() { this.knob.style.transform = `translate(${this.x * 128.57}%, ${-this.y * 128.57}%)`; }
}

/* ---------------- Camera ---------------- */
class CameraManager {
  constructor(ui) { this.ui = ui; this.stream = null; this.startedAt = 0; this.busy = false; this.error = ''; }
  get active() { return !!this.stream; }
  async refreshDevices() {
    if (!navigator.mediaDevices?.enumerateDevices) return;
    const cams = (await navigator.mediaDevices.enumerateDevices()).filter((d) => d.kind === 'videoinput');
    const sel = this.ui.select, cur = sel.value; sel.replaceChildren(new Option('Default camera', ''));
    cams.forEach((d, i) => sel.add(new Option(d.label || 'Camera ' + (i + 1), d.deviceId)));
    if ([...sel.options].some((o) => o.value === cur)) sel.value = cur; sel.disabled = cams.length < 2;
  }
  async start() {
    if (this.busy) return; this.busy = true;
    try {
      if (!navigator.mediaDevices?.getUserMedia) throw Object.assign(new Error('unsupported'), { name: 'Unsupported' });
      this.stop(true);
      const id = this.ui.select.value, size = { width: { ideal: 1280 }, height: { ideal: 720 } };
      this.stream = await navigator.mediaDevices.getUserMedia({ audio: false, video: id ? { deviceId: { exact: id }, ...size } : size });
      this.stream.getVideoTracks()[0].addEventListener('ended', () => { this.fail({ name: 'Ended' }); });
      this.ui.video.srcObject = this.stream; await this.ui.video.play();
      this.startedAt = performance.now(); this.error = '';
      const tid = this.stream.getVideoTracks()[0].getSettings().deviceId; await this.refreshDevices(); if (tid) this.ui.select.value = tid;
      this.ui.onChange();
    } catch (e) { this.fail(e); } finally { this.busy = false; }
  }
  fail(e) {
    const m = { NotAllowedError: 'Camera permission was denied. Allow camera access in the browser and try again.', NotFoundError: 'No camera was found on this device.', NotReadableError: 'The camera is in use by another application.', OverconstrainedError: 'The selected camera is unavailable. Choose another one.', Unsupported: 'Camera access needs a secure context: open via http://localhost or https.', Ended: 'The camera was disconnected.' };
    this.stop(true); this.error = m[e.name] || 'Camera error: ' + (e.message || e.name); this.ui.onChange();
  }
  stop(silent) { this.stream?.getTracks().forEach((t) => t.stop()); this.stream = null; this.ui.video.srcObject = null; if (!silent) { this.error = ''; this.ui.onChange(); } }
}

/* ---------------- Gestures (MediaPipe Hands) ---------------- */
const HAND_LINKS = [[0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[5,9],[9,10],[10,11],[11,12],[9,13],[13,14],[14,15],[15,16],[13,17],[17,18],[18,19],[19,20],[0,17]];
class GestureController {
  constructor(video, cfg) { this.video = video; this.cfg = cfg; this.running = false; this.hands = null; this.busy = false; this.latest = null; this.conf = null; this.ready = false; this.neutral = { x: 0.5, y: 0.5, scale: 0 }; this.pose = '-'; this.signals = { throttle: 0, yaw: 0, pitch: 0, roll: 0 }; this.status = 'Off'; }
  async start() {
    if (this.running) return; this.status = 'Loading model...';
    if (!window.Hands) await loadScript('https://cdn.jsdelivr.net/npm/@mediapipe/hands@0.4.1675469240/hands.js');
    if (!window.Hands) throw new Error('MediaPipe Hands unavailable');
    this.hands = new window.Hands({ locateFile: (f) => 'https://cdn.jsdelivr.net/npm/@mediapipe/hands@0.4.1675469240/' + f });
    this.hands.setOptions({ maxNumHands: 1, modelComplexity: 1, minDetectionConfidence: 0.6, minTrackingConfidence: 0.5 });
    this.hands.onResults((r) => this.onResults(r)); this.running = true; this.ready = false; this.latest = null;
  }
  stop() { this.running = false; try { this.hands?.close(); } catch { /* ignore */ } this.hands = null; this.latest = null; this.conf = null; this.pose = '-'; this.status = 'Off'; this.signals = { throttle: 0, yaw: 0, pitch: 0, roll: 0 }; }
  pump() { // called from the main loop; never overlaps requests
    if (!this.running || this.busy || this.video.readyState < 2) return;
    this.busy = true; this.hands.send({ image: this.video }).catch(() => { this.status = 'Model error'; }).finally(() => { this.busy = false; });
  }
  onResults(res) {
    if (!this.running) return; this.ready = true;
    const lm = res.multiHandLandmarks?.[0]; this.conf = res.multiHandedness?.[0]?.score ?? null;
    if (!lm || (this.conf ?? 1) < CONFIG.minConfidence) { this.latest = null; return; } // ignore low confidence
    const d = (a, b) => Math.hypot(a.x - b.x, a.y - b.y), w = lm[0];
    let ext = 0, curl = 0;
    [[8, 6], [12, 10], [16, 14], [20, 18]].forEach(([t, p]) => { const r = d(lm[t], w) / d(lm[p], w); if (r > 1.1) ext++; else if (r < 0.95) curl++; });
    const pose = ext >= 4 ? 'open' : curl >= 4 ? 'fist' : 'ambiguous';
    const ids = [0, 5, 9, 13, 17];
    this.latest = { lm, pose, x: ids.reduce((s, i) => s + lm[i].x, 0) / 5, y: ids.reduce((s, i) => s + lm[i].y, 0) / 5, scale: d(lm[0], lm[9]), time: performance.now() };
  }
  calibrate() { const L = this.latest; if (!L || performance.now() - L.time > 350) return false; this.neutral = { x: 1 - L.x, y: L.y, scale: L.scale }; return true; }
  /* Mapping: neutral pose -> hover (or landing); the other pose steers: hand left/right -> roll or yaw,
   * up/down -> pitch (up = forward), optional hand distance -> throttle. Ambiguous/lost -> neutral axes. */
  read(now) {
    const c = this.cfg, L = this.latest, s = { throttle: 0, yaw: 0, pitch: 0, roll: 0 };
    const tracking = this.running && L && now - L.time < 350;
    if (!this.running) { this.status = 'Off'; this.pose = '-'; }
    else if (!tracking) { this.status = this.ready ? 'No hand: safe neutral' : 'Loading model...'; this.pose = 'none'; if (c.lost === 'descend') s.throttle = -0.4; }
    else {
      this.status = 'Tracking'; this.pose = L.pose; const steer = c.neutral === 'fist' ? 'open' : 'fist';
      if (L.pose === c.neutral) { if (c.action === 'land') s.throttle = -0.5; }
      else if (L.pose === steer) {
        s[c.hmap] = clamp(((1 - L.x) - this.neutral.x) / CONFIG.gestureRange); s.pitch = clamp((this.neutral.y - L.y) / CONFIG.gestureRange);
        if (c.depth && this.neutral.scale) s.throttle = clamp((L.scale / this.neutral.scale - 1) / 0.35);
      }
    }
    this.signals = s; return s;
  }
  drawOverlay(ctx, r) { // r = displayed video rect; video is mirrored so x flips
    const L = this.latest; if (!this.running) return;
    const X = (x) => r.x + (1 - x) * r.w, Y = (y) => r.y + y * r.h;
    ctx.strokeStyle = 'rgba(255,255,255,.35)'; ctx.setLineDash([4, 4]); ctx.beginPath();
    ctx.arc(r.x + this.neutral.x * r.w, Y(this.neutral.y), Math.max(4, this.cfg.dead * CONFIG.gestureRange * r.w), 0, 7); ctx.stroke(); ctx.setLineDash([]);
    if (!L || performance.now() - L.time > 350) return;
    ctx.strokeStyle = '#2ec4e6'; ctx.lineWidth = 2; ctx.beginPath();
    HAND_LINKS.forEach(([a, b]) => { ctx.moveTo(X(L.lm[a].x), Y(L.lm[a].y)); ctx.lineTo(X(L.lm[b].x), Y(L.lm[b].y)); }); ctx.stroke();
    ctx.fillStyle = '#3ddc84'; L.lm.forEach((p) => { ctx.beginPath(); ctx.arc(X(p.x), Y(p.y), 3, 0, 7); ctx.fill(); });
  }
}

/* ---------------- Flight control & simulation ---------------- */
class FlightControlManager {
  constructor() { this.source = null; this.armed = false; this.estop = false; this.deadZone = CONFIG.manualDeadZone; this.smoothing = CONFIG.manualSmoothing; this.limits = { throttle: 1, yaw: 1, pitch: 1, roll: 1 }; }
  process(raw, dt) { // pipeline steps 2-5 -> shared controlState
    const active = !!this.source && !this.estop;
    for (const ax of AXES) {
      let v = active && Number.isFinite(raw[ax]) ? clamp(raw[ax]) : 0;                       // normalize + validate
      const dz = this.deadZone; v = Math.abs(v) < dz ? 0 : Math.sign(v) * (Math.abs(v) - dz) / (1 - dz); // dead zone
      v *= this.limits[ax];                                                                   // limits
      controlState[ax] = this.estop ? 0 : controlState[ax] + (v - controlState[ax]) * (1 - Math.exp(-dt * this.smoothing)); // smoothing
    }
  }
}
class TelemetryManager { // SIMULATED values only
  constructor() { this.reset(); }
  reset() { this.alt = 0; this.vz = 0; this.heading = 0; }
  update(dt, cs, armed, frozen) {
    if (frozen) return;
    this.vz += ((armed ? cs.throttle * CONFIG.maxClimb : 0) - this.vz) * (1 - Math.exp(-dt * 4));
    this.alt = clamp(this.alt + this.vz * dt, 0, CONFIG.maxAltitude);
    if ((this.alt <= 0 && this.vz < 0) || (this.alt >= CONFIG.maxAltitude && this.vz > 0)) this.vz = 0;
    if (armed) this.heading = (this.heading + cs.yaw * CONFIG.yawRate * dt + 360) % 360;
  }
}

/* ---------------- 3D visualization ---------------- */
class DroneVisualizer {
  constructor(host, THREE, OrbitControls) {
    this.T = THREE; this.host = host; this.att = { roll: 0, pitch: 0 }; this.spin = 0; const T = THREE;
    this.renderer = new T.WebGLRenderer({ antialias: true }); // throws if WebGL is unavailable
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); this.renderer.shadowMap.enabled = true; this.renderer.setClearColor(0x14161a);
    host.appendChild(this.renderer.domElement);
    this.scene = new T.Scene(); this.camera = new T.PerspectiveCamera(45, 1, 0.1, 200);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    Object.assign(this.controls, { enableDamping: true, maxPolarAngle: Math.PI * 0.495, minDistance: 3, maxDistance: 20 });
    this.resetCamera();
    this.scene.add(new T.HemisphereLight(0xbfd4e6, 0x20242a, 0.8));
    const sun = new T.DirectionalLight(0xffffff, 1.1); sun.position.set(4, 8, 3); sun.castShadow = true; sun.shadow.mapSize.set(1024, 1024);
    Object.assign(sun.shadow.camera, { left: -8, right: 8, top: 8, bottom: -8 }); this.scene.add(sun);
    const grid = new T.GridHelper(20, 20, 0x46515a, 0x2a2f35); this.scene.add(grid);
    const floor = new T.Mesh(new T.PlaneGeometry(20, 20), new T.ShadowMaterial({ opacity: 0.35 })); floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; this.scene.add(floor);
    const axes = new T.AxesHelper(1.5); axes.position.y = 0.01; axes.material.transparent = true; axes.material.opacity = 0.45; this.scene.add(axes);
    this.drone = this.build(); this.scene.add(this.drone);
    this.ro = new ResizeObserver(() => this.resize()); this.ro.observe(host); this.resize();
  }
  resetCamera() { this.camera.position.set(4.5, 3.6, 5.5); this.controls.target.set(0, 1.4, 0); this.controls.update(); }
  build() { // quadcopter, front = -Z
    const T = this.T, g = new T.Group(); g.rotation.order = 'YXZ'; this.props = [];
    const mat = (c) => new T.MeshStandardMaterial({ color: c, roughness: 0.6, metalness: 0.2 });
    const dark = mat(0x2b3036), mid = mat(0x4a525a), red = mat(0xff4d4f), cyan = mat(0x2ec4e6), white = mat(0xe6e9ec);
    const add = (geo, m, x, y, z, rx = 0, ry = 0, rz = 0, parent = g) => { const o = new T.Mesh(geo, m); o.position.set(x, y, z); o.rotation.set(rx, ry, rz); o.castShadow = true; parent.add(o); return o; };
    add(new T.BoxGeometry(0.9, 0.22, 1.2), dark, 0, 0, 0); add(new T.BoxGeometry(0.6, 0.06, 0.8), mid, 0, 0.14, 0);
    add(new T.BoxGeometry(2.5, 0.07, 0.12), mid, 0, 0, 0, 0, Math.PI / 4, 0); add(new T.BoxGeometry(2.5, 0.07, 0.12), mid, 0, 0, 0, 0, -Math.PI / 4, 0);
    [[-1, -1, red, 1], [1, -1, red, -1], [-1, 1, cyan, -1], [1, 1, cyan, 1]].forEach(([sx, sz, col, dir]) => { // front motors red, rear cyan
      const x = sx * 0.88, z = sz * 0.88; add(new T.CylinderGeometry(0.14, 0.14, 0.18, 20), col, x, 0.1, z);
      const p = new T.Group(); p.position.set(x, 0.25, z); p.userData.dir = dir; g.add(p);
      add(new T.BoxGeometry(1.1, 0.015, 0.09), white, 0, 0, 0, 0, 0, 0, p); add(new T.BoxGeometry(1.1, 0.015, 0.09), white, 0, 0, 0, 0, Math.PI / 2, 0, p); this.props.push(p);
    });
    add(new T.ConeGeometry(0.12, 0.4, 16), red, 0, 0.05, -0.8, -Math.PI / 2); // front indicator
    add(new T.BoxGeometry(0.5, 0.05, 0.04), red, 0, 0, -0.61); add(new T.BoxGeometry(0.5, 0.05, 0.04), cyan, 0, 0, 0.61);
    [-0.5, 0.5].forEach((x) => { add(new T.CylinderGeometry(0.03, 0.03, 1.3, 10), dark, x, -0.5, 0, Math.PI / 2); [-0.4, 0.4].forEach((z) => add(new T.BoxGeometry(0.04, 0.4, 0.04), dark, x, -0.3, z)); });
    return g;
  }
  resize() { const w = this.host.clientWidth, h = this.host.clientHeight; if (!w || !h) return; this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix(); }
  resetOrientation() { this.att.roll = 0; this.att.pitch = 0; this.resetCamera(); }
  update(dt, cs, tm, armed) {
    const lim = CONFIG.tiltDeg * Math.PI / 180, k = 1 - Math.exp(-dt * 8);
    this.att.pitch += (-cs.pitch * lim - this.att.pitch) * k; this.att.roll += (-cs.roll * lim - this.att.roll) * k; // nose down / right side down
    this.drone.rotation.set(this.att.pitch, -tm.heading * Math.PI / 180, this.att.roll);
    this.drone.position.y = CONFIG.groundY + tm.alt * CONFIG.altScale;
    this.spin += ((armed ? 18 + 22 * (cs.throttle * 0.5 + 0.5) : 0) - this.spin) * (1 - Math.exp(-dt * 3));
    this.props.forEach((p) => { p.rotation.y += p.userData.dir * this.spin * dt; });
    this.controls.update(); this.renderer.render(this.scene, this.camera);
  }
  dispose() {
    this.ro.disconnect(); this.controls.dispose();
    this.scene.traverse((o) => { o.geometry?.dispose(); [].concat(o.material || []).forEach((m) => m.dispose()); });
    this.renderer.dispose(); this.renderer.domElement.remove();
  }
}

/* ---------------- Application wiring ---------------- */
const conn = new ConnectionManager(), flight = new FlightControlManager(), telemetry = new TelemetryManager();
const gestCfg = { neutral: 'fist', action: 'hover', hmap: 'roll', lost: 'descend', depth: false, dead: 0.15, smooth: 8 };
const video = $('cam-video'), overlay = $('overlay'), octx = overlay.getContext('2d');
const joyL = new JoystickController($('joy-left')), joyR = new JoystickController($('joy-right'));
const camera = new CameraManager({ video, select: $('cam-select'), onChange: syncCamera });
const gesture = new GestureController(video, gestCfg);
let viz = null, lastT = performance.now(), fpsN = 0, fpsT = 0, fps = 0, uiT = 0, pubT = 0, logT = 0, lastLog = '';

function pill(id, text, cls) { const e = $(id); e.textContent = text; e.className = 'pill ' + (cls || ''); }
function setSource(src) {
  if (flight.estop && src) return toast('Emergency stop active. Reset the simulation first.');
  if (src === 'gesture' && !gesture.running) return toast('Start gesture tracking first.');
  flight.source = src; joyL.setEnabled(src === 'manual'); joyR.setEnabled(src === 'manual');
  if (src !== 'manual') { joyL.release(true); joyR.release(true); }
  if (src === 'manual') { flight.deadZone = CONFIG.manualDeadZone; flight.smoothing = CONFIG.manualSmoothing; }
  syncSource();
}
function syncSource() {
  const s = flight.source, name = s === 'manual' ? 'Manual joysticks' : s === 'gesture' ? 'Webcam gestures' : 'None';
  $('src-label').textContent = name; $('tm-src').textContent = name; $('chk-gest-control').checked = s === 'gesture';
  const mb = $('manual-badge'); mb.textContent = s === 'manual' ? 'ACTIVE' : 'DISABLED'; mb.className = 'badge' + (s === 'manual' ? ' on' : '');
  $('btn-manual-on').disabled = s === 'manual' || flight.estop; $('btn-manual-off').disabled = s !== 'manual'; $('chk-gest-control').disabled = flight.estop;
  $('btn-arm').disabled = flight.estop; $('btn-arm').textContent = flight.armed ? 'Disarm' : 'Arm';
}
function syncCamera() {
  const on = camera.active, ph = $('cam-placeholder');
  $('btn-cam').textContent = on ? 'Stop camera' : 'Start camera'; const b = $('cam-live'); b.textContent = on ? 'LIVE' : camera.error ? 'ERROR' : 'OFF'; b.className = 'badge' + (on ? ' crit' : '');
  ph.hidden = on; ph.classList.toggle('err', !!camera.error); $('cam-msg').textContent = camera.error || 'Camera is off. Start the camera to use the local webcam.';
  $('tm-cam').textContent = on ? 'Streaming' : camera.error ? 'Error' : 'Off'; if (!on) $('cam-meta').textContent = '';
  if (!on && gesture.running) stopGestures(); 
}
function syncGesture() {
  const on = gesture.running; $('btn-gest-start').textContent = on ? 'Stop tracking' : 'Start tracking';
  const b = $('gest-badge'); b.textContent = on ? 'TRACKING' : 'OFF'; b.className = 'badge' + (on ? ' on' : '');
}
async function startGestures() {
  $('btn-gest-start').disabled = true;
  try {
    if (!camera.active) await camera.start(); if (!camera.active) throw new Error('Camera unavailable');
    await gesture.start(); syncGesture();
  } catch (e) { gesture.stop(); toast('Gesture tracking failed: ' + e.message); syncGesture(); }
  $('btn-gest-start').disabled = false;
}
function stopGestures() { gesture.stop(); if (flight.source === 'gesture') setSource(null); syncGesture(); }
function emergencyStop() {
  if (flight.estop) return; flight.estop = true; flight.armed = false; flight.source = null;
  joyL.setEnabled(false); joyR.setEnabled(false); AXES.forEach((a) => { controlState[a] = 0; });
  conn.publish({ type: 'event', name: 'emergency_stop' }); conn.adapter.status = 'connected';
  $('estop-banner').hidden = false; syncSource();
}
function resetSimulation() {
  flight.estop = false; flight.armed = false; telemetry.reset(); AXES.forEach((a) => { controlState[a] = 0; });
  setSource(null); joyL.release(true); joyR.release(true); viz?.resetOrientation(); $('estop-banner').hidden = true; syncSource(); toast('Simulation reset. Disarmed.');
}
function toggleArm() {
  if (flight.estop) return;
  if (!flight.armed) { if (controlState.throttle > 0.05 || (flight.source === 'manual' && joyL.y > 0.05)) return toast('Lower throttle before arming.'); flight.armed = true; }
  else { if (telemetry.alt > 0.3) return toast('Land before disarming (or use Emergency stop).'); flight.armed = false; }
  conn.publish({ type: 'event', name: flight.armed ? 'arm' : 'disarm' }); syncSource();
}

/* Overlay: crosshair, thirds and hand landmarks, drawn inside the displayed video rect */
function videoRect() {
  const cw = overlay.clientWidth, ch = overlay.clientHeight, vw = video.videoWidth || 16, vh = video.videoHeight || 9, s = Math.min(cw / vw, ch / vh);
  return { x: (cw - vw * s) / 2, y: (ch - vh * s) / 2, w: vw * s, h: vh * s };
}
function drawOverlay() {
  const w = overlay.clientWidth, h = overlay.clientHeight; if (overlay.width !== w || overlay.height !== h) { overlay.width = w; overlay.height = h; }
  octx.clearRect(0, 0, w, h); if (!camera.active) return;
  const r = videoRect(); octx.lineWidth = 1; octx.strokeStyle = 'rgba(255,255,255,.4)';
  if ($('chk-thirds').checked) { octx.beginPath(); for (let i = 1; i < 3; i++) { octx.moveTo(r.x + r.w * i / 3, r.y); octx.lineTo(r.x + r.w * i / 3, r.y + r.h); octx.moveTo(r.x, r.y + r.h * i / 3); octx.lineTo(r.x + r.w, r.y + r.h * i / 3); } octx.stroke(); }
  if ($('chk-cross').checked) { const cx = r.x + r.w / 2, cy = r.y + r.h / 2; octx.strokeStyle = '#3ddc84'; octx.beginPath(); octx.moveTo(cx - 20, cy); octx.lineTo(cx + 20, cy); octx.moveTo(cx, cy - 20); octx.lineTo(cx, cy + 20); octx.stroke(); octx.beginPath(); octx.arc(cx, cy, 8, 0, 7); octx.stroke(); }
  gesture.drawOverlay(octx, r);
}

/* Keyboard: arrows = pitch/roll (right stick), W/S = throttle, A/D = yaw (left stick). Hold = full deflection, Shift = half. */
const keysDown = new Set(); let keyShift = false, keyDrivenL = false, keyDrivenR = false;
const KEY_LIST = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd'];
function keyAxis(pos, neg) { return (keysDown.has(pos) ? 1 : 0) - (keysDown.has(neg) ? 1 : 0); }
function readKeyboard() {
  const f = keyShift ? 0.5 : 1, anyL = ['w', 's', 'a', 'd'].some((k) => keysDown.has(k)), anyR = KEY_LIST.slice(0, 4).some((k) => keysDown.has(k));
  if (anyR && joyR.pid === null) { joyR.setValue(keyAxis('ArrowRight', 'ArrowLeft') * f, keyAxis('ArrowUp', 'ArrowDown') * f); keyDrivenR = true; }
  else if (keyDrivenR && !anyR) { joyR.release(true); keyDrivenR = false; }
  if (anyL && joyL.pid === null) { joyL.setValue(keyAxis('d', 'a') * f, keyAxis('w', 's') * f); keyDrivenL = true; }
  else if (keyDrivenL && !anyL) { joyL.release(false); keyDrivenL = false; } // keeps throttle if "Hold throttle" is on
}
function readManual() {
  readKeyboard();
  const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [], p = pads[0];
  if (p && flight.source === 'manual' && p.axes.length >= 4 && p.axes.slice(0, 4).some((a) => Math.abs(a) > 0.08)) { joyL.setValue(p.axes[0], -p.axes[1]); joyR.setValue(p.axes[2], -p.axes[3]); }
  return { yaw: joyL.x, throttle: joyL.y, roll: joyR.x, pitch: joyR.y };
}
function updateUI() {
  [['throttle', joyL.y], ['yaw', joyL.x], ['pitch', joyR.y], ['roll', joyR.x]].forEach(([a, v]) => {
    $('ax-' + a).textContent = sgn(v) + ' (' + pct(v) + ')'; const b = $('bar-' + a).firstElementChild; b.style.setProperty('--w', Math.abs(v).toFixed(3)); b.style.setProperty('--sh', v < 0 ? '-100%' : '0%');
  });
  const tilt = CONFIG.tiltDeg, cs = controlState;
  $('tm-roll').textContent = (cs.roll * tilt).toFixed(1) + '\u00B0'; $('tm-pitch').textContent = (cs.pitch * tilt).toFixed(1) + '\u00B0'; $('tm-yaw').textContent = Math.round(telemetry.heading) + '\u00B0';
  $('tm-throttle').textContent = pct(cs.throttle); $('tm-alt').textContent = telemetry.alt.toFixed(1) + ' m'; $('tm-vs').textContent = telemetry.vz.toFixed(1) + ' m/s';
  $('tm-conn').textContent = conn.label; $('tm-fps').textContent = fps ? fps + ' fps' : '-';
  const mode = flight.estop ? 'STOPPED' : !flight.armed ? 'IDLE' : telemetry.alt < 0.05 && cs.throttle <= 0.05 ? 'READY' : telemetry.vz > 0.2 ? 'CLIMB' : telemetry.vz < -0.2 ? 'DESCEND' : 'HOVER';
  pill('pill-conn', 'Link: ' + conn.adapter.name + (conn.adapter.status === 'connected' ? ' OK' : ''), conn.adapter.status === 'connected' ? 'ok' : 'warn');
  pill('pill-mode', 'Mode: ' + mode, flight.estop ? 'crit' : ''); pill('pill-arm', flight.armed ? 'ARMED' : 'DISARMED', flight.estop ? 'crit' : flight.armed ? 'warn' : '');
  if (camera.active) { const s = Math.floor((performance.now() - camera.startedAt) / 1000), p = (n) => String(n).padStart(2, '0'); $('cam-meta').textContent = video.videoWidth + 'x' + video.videoHeight + '  ' + p(Math.floor(s / 3600)) + ':' + p(Math.floor(s / 60) % 60) + ':' + p(s % 60); }
  $('gest-status').textContent = gesture.status; $('gest-conf').textContent = gesture.conf == null ? '-' : Math.round(gesture.conf * 100) + '%'; $('gest-pose').textContent = gesture.pose;
  const g = gesture.signals; $('gest-signals').textContent = `T${sgn(g.throttle)} Y${sgn(g.yaw)} P${sgn(g.pitch)} R${sgn(g.roll)}`;
}

function frame(now) {
  const dt = Math.min((now - lastT) / 1000, 0.1); lastT = now; fpsN++; if (now - fpsT > 500) { fps = Math.round(fpsN * 1000 / (now - fpsT)); fpsN = 0; fpsT = now; }
  let raw = { throttle: 0, yaw: 0, pitch: 0, roll: 0 };
  if (flight.source === 'manual') raw = readManual();
  if (gesture.running) { gesture.pump(); const g = gesture.read(now); if (flight.source === 'gesture') { raw = g; flight.deadZone = gestCfg.dead; flight.smoothing = gestCfg.smooth; } } else gesture.read(now);
  flight.process(raw, dt);
  telemetry.update(dt, controlState, flight.armed, flight.estop);
  try { viz?.update(dt, controlState, telemetry, flight.armed && !flight.estop); } catch (e) { viz = null; showVizError('3D rendering stopped: ' + e.message); }
  if (now - pubT > 1000 / CONFIG.publishHz) { pubT = now; if (flight.armed && !flight.estop) conn.publish({ type: 'control', armed: true, source: flight.source, throttle: +controlState.throttle.toFixed(3), yaw: +controlState.yaw.toFixed(3), pitch: +controlState.pitch.toFixed(3), roll: +controlState.roll.toFixed(3) }); }
  if (now - uiT > 1000 / CONFIG.uiHz) { uiT = now; updateUI(); }
  if (now - logT > 250) { logT = now; const t = conn.adapter.lines.join('\n') || 'No commands sent.'; if (t !== lastLog) { lastLog = t; $('cmd-log').textContent = t; } }
  drawOverlay(); requestAnimationFrame(frame);
}
function showVizError(msg) { const m = $('viz-msg'); m.textContent = msg; m.hidden = false; }

function bindUI() {
  $('btn-cam').addEventListener('click', () => (camera.active ? camera.stop() : camera.start()));
  $('cam-select').addEventListener('change', () => { if (camera.active) camera.start(); });
  navigator.mediaDevices?.addEventListener?.('devicechange', () => camera.refreshDevices());
  $('btn-fs').addEventListener('click', () => { const st = $('video-stage'); if (document.fullscreenElement) document.exitFullscreen(); else if (st.requestFullscreen) st.requestFullscreen().catch(() => toast('Fullscreen was blocked.')); else toast('Fullscreen is not supported here.'); });
  document.addEventListener('fullscreenchange', () => { $('btn-fs').textContent = document.fullscreenElement ? 'Exit fullscreen' : 'Fullscreen'; });
  $('btn-manual-on').addEventListener('click', () => { if (flight.source === 'gesture') toast('Gesture control switched off.'); setSource('manual'); });
  $('btn-manual-off').addEventListener('click', () => setSource(null));
  [joyL, joyR].forEach((j) => j.el.addEventListener('pointerdown', () => { if (!j.enabled && !flight.estop) { setSource('manual'); toast('Manual control enabled. Drag again.'); } }));
  $('btn-reset-controls').addEventListener('click', () => { joyL.release(true); joyR.release(true); });
  $('btn-center-att').addEventListener('click', () => { joyL.x = 0; joyR.x = 0; joyR.y = 0; joyL.render(); joyR.render(); });
  $('chk-hold-throttle').addEventListener('change', (e) => { joyL.holdY = e.target.checked; if (!e.target.checked && joyL.pid === null) joyL.release(true); });
  $('btn-gest-start').addEventListener('click', () => (gesture.running ? stopGestures() : startGestures()));
  $('btn-calibrate').addEventListener('click', () => toast(gesture.calibrate() ? 'Neutral position calibrated.' : 'Show your hand to the camera, then calibrate.'));
  $('chk-gest-control').addEventListener('change', (e) => { if (e.target.checked) { if (flight.source === 'manual') toast('Manual control switched off.'); setSource('gesture'); if (flight.source !== 'gesture') e.target.checked = false; } else setSource(null); });
  [['gest-neutral', 'neutral'], ['gest-action', 'action'], ['gest-hmap', 'hmap'], ['gest-lost', 'lost']].forEach(([id, k]) => $(id).addEventListener('change', (e) => { gestCfg[k] = e.target.value; }));
  $('chk-depth').addEventListener('change', (e) => { gestCfg.depth = e.target.checked; if (e.target.checked && !gesture.neutral.scale) toast('Calibrate neutral to enable depth throttle.'); });
  $('gest-dead').addEventListener('input', (e) => { gestCfg.dead = +e.target.value; $('out-gdead').textContent = e.target.value; });
  $('gest-smooth').addEventListener('input', (e) => { gestCfg.smooth = +e.target.value; $('out-gsmooth').textContent = e.target.value; });
  $('tilt-limit').addEventListener('input', (e) => { CONFIG.tiltDeg = +e.target.value; $('out-tilt').textContent = e.target.value + '\u00B0'; });
  $('btn-arm').addEventListener('click', toggleArm); $('btn-reset-sim').addEventListener('click', resetSimulation); $('btn-estop').addEventListener('click', emergencyStop);
  $('btn-reset-orient').addEventListener('click', () => { telemetry.heading = 0; viz?.resetOrientation(); });
  $('chk-sim').addEventListener('change', (e) => { if (!e.target.checked) { e.target.checked = true; toast('Live mode needs a hardware bridge. Staying in simulation.'); } });
  addEventListener('keydown', (e) => {
    if (flight.source !== 'manual' || e.ctrlKey || e.metaKey || e.altKey || ['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;
    const k = e.key.length === 1 ? e.key.toLowerCase() : e.key; if (!KEY_LIST.includes(k)) return;
    e.preventDefault(); keysDown.add(k); keyShift = e.shiftKey;
  });
  addEventListener('keyup', (e) => { keysDown.delete(e.key.length === 1 ? e.key.toLowerCase() : e.key); keyShift = e.shiftKey; });
  addEventListener('blur', () => keysDown.clear());
  addEventListener('gamepadconnected', (e) => toast('Gamepad connected: ' + e.gamepad.id.slice(0, 40)));
  addEventListener('pagehide', () => { camera.stop(true); gesture.stop(); viz?.dispose(); joyL.destroy(); joyR.destroy(); });
}

async function boot() {
  bindUI(); setSource('manual'); syncCamera(); syncGesture(); // manual joysticks are the default source camera.refreshDevices().catch(() => {});
  try { // dynamic import so a failed CDN only disables the 3D view
    const [THREE, { OrbitControls }] = await Promise.all([import('three'), import('three/addons/controls/OrbitControls.js')]);
    viz = new DroneVisualizer($('viz'), THREE, OrbitControls);
  } catch (e) { viz = null; showVizError('3D view unavailable (' + (e.message || 'WebGL or CDN error') + '). Controls and telemetry still work.'); }
  requestAnimationFrame(frame);
}
boot();