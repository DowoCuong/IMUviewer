import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/GLTFLoader.js';
import { OrbitControls } from 'three/addons/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const ui = {
  port: $('portSelect'),
  portDescription: $('portDescription'),
  baud: $('baudSelect'),
  protocol: $('protocolSelect'),
  neutral: $('neutralButton'),
  functional: $('functionalButton'),
  calibrationJoint: $('calibrationJoint'),
  calibrationState: $('calibrationState'),
  sensorHealth: $('sensorHealth'),
  loadProfile: $('loadProfileButton'),
  saveProfile: $('saveProfileButton'),
  gaitEvents: $('gaitEventsButton'),
  gaitMode: $('gaitModeButton'),
  gaitSummary: $('gaitSummary'),
  refreshPorts: $('refreshPorts'),
  start: $('startButton'),
  stop: $('stopButton'),
  resetMax: $('resetMaxButton'),
  export: $('exportButton'),
  resetPose: $('resetPoseButton'),
  clearChart: $('clearChartButton'),
  connectionBadge: $('connectionBadge'),
  connectionText: $('connectionText'),
  sessionState: $('sessionState'),
  sampleRate: $('sampleRate'),
  sampleCount: $('sampleCount'),
  sessionDuration: $('sessionDuration'),
  modelStatus: $('modelStatus'),
  modelTitle: $('modelTitle'),
  chart: $('angleChart'),
  chartEmpty: $('chartEmpty'),
  toast: $('toast'),
  live: { hip: $('liveHip'), knee: $('liveKnee'), ankle: $('liveAnkle') },
  max: { hip: $('maxHip'), knee: $('maxKnee'), ankle: $('maxAnkle') },
  rom: { hip: $('romHip'), knee: $('romKnee'), ankle: $('romAnkle') },
};

let desktop = null;
let isConnected = false;
let isSessionActive = false;
let sampleCount = 0;
let chartSamples = [];
let recentTimes = [];
let firstSampleTime = null;
let lastSampleTime = null;
let chartFramePending = false;
let toastTimer = null;
let collectingJoint = null;
let gaitReport = null;
let showGait = false;
let gaitSamples = [];

const MODEL_LABELS = {
  lower: 'Mô hình thân dưới',
  full: 'Mô hình toàn thân',
};

function showToast(message, isError = false) {
  if (!message) return;
  window.clearTimeout(toastTimer);
  ui.toast.textContent = message;
  ui.toast.classList.toggle('error', isError);
  ui.toast.classList.add('visible');
  toastTimer = window.setTimeout(() => ui.toast.classList.remove('visible'), 3200);
}

async function callDesktop(method, ...args) {
  if (!desktop) throw new Error('Ứng dụng đang khởi động. Vui lòng đợi một chút.');
  return new Promise((resolve, reject) => {
    try { desktop[method](...args, resolve); } catch (error) { reject(error); }
  });
}

async function performAction(method, ...args) {
  try { return await callDesktop(method, ...args); }
  catch (error) { return { ok: false, message: String(error) }; }
}

function initializeBridge() {
  if (typeof QWebChannel === 'undefined') {
    showToast('Không kết nối được giao diện. Hãy mở lại ứng dụng.', true);
    return;
  }
  new QWebChannel(qt.webChannelTransport, (channel) => {
    desktop = channel.objects.desktop;
    initializeDesktop().catch((error) => showToast(String(error), true));
  });
}

async function initializeDesktop() {
  const settings = await callDesktop('initial_settings');
  if (settings.baud) ui.baud.value = String(settings.baud);
  if (settings.protocol) ui.protocol.value = settings.protocol;
  setConnection(Boolean(settings.running));
  await refreshPorts(settings.port || '');
}

async function refreshPorts(preferredPort = '') {
  ui.refreshPorts.disabled = true;
  try {
    const ports = await callDesktop('list_ports');
    const previous = preferredPort || ui.port.value;
    ui.port.replaceChildren();
    const automatic = document.createElement('option');
    automatic.value = '';
    automatic.textContent = 'Tự tìm cổng khi bắt đầu';
    ui.port.appendChild(automatic);

    for (const item of ports) {
      const option = document.createElement('option');
      option.value = item.device;
      option.textContent = `${item.device} — ${item.description}`;
      option.dataset.description = item.description;
      ui.port.appendChild(option);
    }

    if (previous && ports.some((item) => item.device === previous)) {
      ui.port.value = previous;
    }
    updatePortDescription();
    if (!ports.length) {
      ui.portDescription.textContent = 'Chưa có cổng kết nối. Cắm bộ thu IMU rồi làm mới danh sách.';
    }
  } catch (error) {
    ui.portDescription.textContent = 'Không đọc được danh sách cổng kết nối.';
    showToast(String(error), true);
  } finally {
    ui.refreshPorts.disabled = false;
  }
}

function updatePortDescription() {
  const selected = ui.port.selectedOptions[0];
  ui.portDescription.textContent = selected?.dataset.description || 'Ứng dụng sẽ tự tìm cổng của bộ thu.';
}

function setConnection(connected) {
  isConnected = Boolean(connected);
  ui.connectionBadge.classList.toggle('online', isConnected);
  ui.connectionBadge.classList.toggle('offline', !isConnected);
  ui.connectionBadge.classList.remove('error');
  ui.connectionText.textContent = isConnected ? 'Đã mở cổng kết nối' : 'Chưa kết nối';
  ui.start.disabled = isConnected;
  ui.stop.disabled = !isConnected;
  ui.neutral.disabled = !isConnected;
  ui.functional.disabled = !isConnected;
  ui.protocol.disabled = isConnected;
  ui.gaitEvents.disabled = isConnected;
  ui.sessionState.textContent = isConnected ? 'Đợi hiệu chuẩn' : (sampleCount ? 'Đã dừng' : 'Sẵn sàng');
  if (!isConnected) ui.sampleRate.textContent = '0 Hz';
}

function setConnectionError(message) {
  setConnection(false);
  ui.connectionBadge.classList.add('error');
  ui.connectionBadge.classList.remove('offline');
  ui.connectionText.textContent = 'Lỗi kết nối';
  showToast(message, true);
}

function formatDegrees(value) {
  if (value == null) return '—';
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric.toFixed(2) : '—';
}

function updateMetrics(sample) {
  if (sample.metricsOnly) {
    for (const joint of ['hip', 'knee', 'ankle']) ui.max[joint].textContent = '—';
    return;
  }
  ui.live.hip.textContent = formatDegrees(sample.hip);
  ui.live.knee.textContent = formatDegrees(sample.knee);
  ui.live.ankle.textContent = formatDegrees(sample.ankle);
  ui.max.hip.textContent = `${formatDegrees(sample.maxHip)}°`;
  ui.max.knee.textContent = `${formatDegrees(sample.maxKnee)}°`;
  ui.max.ankle.textContent = `${formatDegrees(sample.maxAnkle)}°`;
  ui.rom.hip.textContent = `${formatDegrees(sample.romHip)}°`;
  ui.rom.knee.textContent = `${formatDegrees(sample.romKnee)}°`;
  ui.rom.ankle.textContent = `${formatDegrees(sample.romAnkle)}°`;
  for (const joint of ['hip', 'knee', 'ankle']) {
    const title = joint[0].toUpperCase() + joint.slice(1);
    const components = sample.components || {};
    $(`components${title}`).textContent = `${joint === 'ankle' ? 'Nghiêng trong/ngoài' : 'Dạng/khép'} ${formatDegrees(components[`${joint}_abduction`])}° · Xoay ${formatDegrees(components[`${joint}_rotation`])}°`;
  }
  const aligned = sample.functional || [];
  ui.calibrationState.textContent = sample.preview
    ? `${sample.calibration === 'uncalibrated' ? 'Cần lấy mốc đứng thẳng' : 'Đã lấy mốc đứng thẳng'} · trục ${aligned.map(j => ({hip:'hông',knee:'gối',ankle:'cổ chân'}[j]||j)).join(', ') || 'chưa hiệu chuẩn'} · xem trước`
    : `Đang đo · sai lệch giờ ước tính ±${Number(sample.clockUncertaintyUs).toFixed(0)} µs · chưa xác nhận trễ bên trong BNO055`;
  ui.calibrationState.classList.toggle('ready', !sample.preview);
  ui.sessionState.textContent = sample.preview ? 'Hiệu chuẩn / xem trước' : 'Đang ghi';
}

function updateSessionTiming() {
  ui.sampleCount.textContent = sampleCount.toLocaleString();
  if (firstSampleTime != null && lastSampleTime != null) {
    ui.sessionDuration.textContent = `${Math.max(0, (lastSampleTime - firstSampleTime) / 1000).toFixed(1)} s`;
  } else {
    ui.sessionDuration.textContent = '0.0 s';
  }
}

function updateSampleRate(timestamp) {
  recentTimes.push(Number(timestamp));
  if (recentTimes.length > 80) recentTimes.shift();
  if (recentTimes.length < 2) return;
  const elapsed = recentTimes.at(-1) - recentTimes[0];
  const rate = elapsed > 0 ? ((recentTimes.length - 1) * 1000) / elapsed : 0;
  ui.sampleRate.textContent = `${Math.round(rate)} Hz`;
}

function resetSessionView() {
  sampleCount = 0;
  chartSamples = [];
  recentTimes = [];
  firstSampleTime = null;
  lastSampleTime = null;
  collectingJoint = null;
  gaitReport = null;
  showGait = false;
  gaitSamples = [];
  ui.gaitMode.disabled = true;
  ui.gaitSummary.textContent = 'Chưa có chu kỳ bước đi đã xác nhận';
  ui.functional.textContent = 'Ghi chuyển động gấp/duỗi';
  ui.calibrationJoint.disabled = false;
  ui.calibrationState.textContent = 'Xem trước · chưa hiệu chuẩn';
  ui.calibrationState.classList.remove('ready');
  ui.sampleCount.textContent = '0';
  ui.sessionDuration.textContent = '0.0 s';
  ui.sampleRate.textContent = '0 Hz';
  ui.chartEmpty.classList.remove('hidden');
  drawChart();
}

function pushChartSample(sample) {
  chartSamples.push({ t: Number(sample.t), hip: Number(sample.hip), knee: Number(sample.knee), ankle: Number(sample.ankle),
    gapBefore: chartSamples.length > 0 && Number(sample.t) - chartSamples.at(-1).t > 30 });
  if (chartSamples.length > 20000) {
    chartSamples = envelopeSamples(chartSamples.slice(0, -4000), 1000).concat(chartSamples.slice(-4000));
    if (chartSamples.length > 20000) chartSamples = chartSamples.slice(-20000);
  }
  if (!chartFramePending) {
    chartFramePending = true;
    requestAnimationFrame(() => {
      chartFramePending = false;
      drawChart();
    });
  }
}

function envelopeSamples(samples, buckets) {
  if (samples.length <= buckets * 8) return samples;
  const result = [];
  for (let bucket = 0; bucket < buckets; bucket += 1) {
    const start = Math.floor(bucket * samples.length / buckets);
    const end = Math.floor((bucket + 1) * samples.length / buckets);
    const indices = new Set([start, end - 1]);
    for (let i = start; i < end; i += 1) {
      if (samples[i].gapBefore) { indices.add(i); if (i > 0) indices.add(i - 1); }
    }
    for (const key of ['hip', 'knee', 'ankle']) {
      let low = start; let high = start;
      for (let i = start + 1; i < end; i += 1) {
        if (samples[i][key] < samples[low][key]) low = i;
        if (samples[i][key] > samples[high][key]) high = i;
      }
      indices.add(low); indices.add(high);
    }
    result.push(...[...indices].sort((a, b) => a - b).map((i) => samples[i]));
  }
  return result;
}

function setBusy(button, busy) {
  button.disabled = busy;
  if (busy) button.dataset.previousDisabled = button.disabled ? '1' : '0';
}

async function startSession() {
  setBusy(ui.start, true);
  resetSessionView();
  try {
    const result = await callDesktop('start_session', ui.port.value, Number(ui.baud.value), ui.protocol.value);
    if (!result.ok) {
      setConnectionError(result.message);
      showToast(result.message, true);
      return;
    }
    isSessionActive = true;
    setConnection(true);
    showToast(result.message);
  } catch (error) {
    setConnectionError(String(error));
  } finally {
    setBusy(ui.start, false);
    ui.start.disabled = isConnected;
  }
}

async function stopSession() {
  setBusy(ui.stop, true);
  try {
    isSessionActive = false;
    const result = await callDesktop('stop_session');
    setConnection(false);
    showToast(result.message);
  } catch (error) {
    showToast(String(error), true);
  } finally {
    setBusy(ui.stop, false);
    ui.stop.disabled = true;
  }
}

async function resetMaxima() {
  try {
    const result = await callDesktop('reset_maxima');
    showToast(result.message);
  } catch (error) {
    showToast(String(error), true);
  }
}

async function exportCsv() {
  try {
    const result = await callDesktop('export_csv');
    if (result.ok || result.cancelled) showToast(result.message, !result.ok && !result.cancelled);
  } catch (error) {
    showToast(String(error), true);
  }
}

window.handleSample = (sample) => {
  updateMetrics(sample);
  if (sample.metricsOnly) return;
  if (!sample.preview && Number.isFinite(Number(sample.t))) {
    const timestamp = Number(sample.t);
    if (firstSampleTime == null) firstSampleTime = timestamp;
    lastSampleTime = timestamp;
    sampleCount = Number(sample.sampleCount);
    updateSampleRate(timestamp);
    pushChartSample(sample);
    ui.chartEmpty.classList.add('hidden');
    updateSessionTiming();
  }
  pushAngles(Number(sample.hip) || 0, Number(sample.knee) || 0, Number(sample.ankle) || 0);
};

window.handleBatch = (samples) => samples.forEach(window.handleSample);

window.handleHealth = (health) => {
  const labels = Object.entries(health.nodes || {}).map(([node, fresh]) => {const h=health.quality_nodes?.[node];const q=h?(['chất lượng chưa biết','fusion tốt','fusion lỗi'][h.health]+' · epoch '+h.sensor_epoch):'chưa có health';return `${node}: ${fresh ? 'đang nhận' : 'chưa có'} · ${q}`;});
  const unmapped = Number(health.alignment?.unmapped_clock || 0);
  ui.sensorHealth.textContent = labels.join(' · ') + (unmapped ? ' · thiếu thông tin đồng hồ; cần cập nhật firmware' : '');
  if (Object.values(health.nodes || {}).some((fresh) => !fresh)) {
    ui.sampleRate.textContent = 'Đợi cảm biến';
    ui.sessionState.textContent = 'Chưa đủ dữ liệu cảm biến';
  }
};

window.handleGait = (report) => {
  gaitReport = report;
  ui.gaitMode.disabled = !report.count;
  ui.gaitSummary.textContent = `${report.count} chu kỳ · loại ${report.rejected.length}` +
    (report.count ? ` · thời gian chu kỳ ${report.mean_stride_s.toFixed(3)} s` : '');
  gaitSamples = (report.phase_percent || []).map((phase, i) => ({ t: phase,
    hip: report.curves.hip_flexion?.mean[i] ?? report.curves.hip?.mean[i] ?? 0,
    knee: report.curves.knee_flexion?.mean[i] ?? report.curves.knee?.mean[i] ?? 0,
    ankle: report.curves.ankle_flexion?.mean[i] ?? report.curves.ankle?.mean[i] ?? 0 }));
  drawChart();
};

window.handleStatus = (message) => {
  if (!message) return;
  window.__lastStatus = message;
  const lower = message.toLowerCase();
  if (['mất kết nối', 'không mở được', 'đã dừng thu dữ liệu', 'lost', 'could not open', 'acquisition stopped'].some(text => lower.includes(text))) {
    setConnectionError(message);
  }
};

window.handleConnection = (connected) => {
  if (!connected) isSessionActive = false;
  setConnection(connected);
};

window.handleSummary = (summary) => {
  isSessionActive = false;
  ui.rom.hip.textContent = `${formatDegrees(summary.romHip)}°`;
  ui.rom.knee.textContent = `${formatDegrees(summary.romKnee)}°`;
  ui.rom.ankle.textContent = `${formatDegrees(summary.romAnkle)}°`;
  ui.sessionState.textContent = summary.count ? 'Hoàn tất' : 'Chưa có mẫu đo';
  updateSessionTiming();
};

function drawChart() {
  const cycleView = showGait && gaitReport?.count;
  const samples = cycleView ? gaitSamples : envelopeSamples(chartSamples, 300);
  const canvas = ui.chart;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 20 || rect.height < 20) return;
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.round(rect.width * ratio);
  const height = Math.round(rect.height * ratio);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }

  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, rect.width, rect.height);
  if (!samples.length) return;

  const margin = { left: 38, right: 12, top: 8, bottom: 24 };
  const plotWidth = Math.max(1, rect.width - margin.left - margin.right);
  const plotHeight = Math.max(1, rect.height - margin.top - margin.bottom);
  const start = samples[0].t;
  const end = samples.at(-1).t;
  const span = Math.max(1, end - start);
  let yMin = 0;
  let yMax = 1;
  for (const row of samples) {
    yMin = Math.min(yMin, row.hip, row.knee, row.ankle);
    yMax = Math.max(yMax, row.hip, row.knee, row.ankle);
  }
  const padding = Math.max(8, (yMax - yMin) * 0.1);
  yMin -= padding;
  yMax += padding;

  const xFor = (t) => margin.left + ((t - start) / span) * plotWidth;
  const yFor = (value) => margin.top + (1 - (value - yMin) / (yMax - yMin)) * plotHeight;

  context.lineWidth = 1;
  context.font = '9px Segoe UI';
  context.textBaseline = 'middle';
  for (let i = 0; i <= 4; i += 1) {
    const ratioY = i / 4;
    const y = margin.top + ratioY * plotHeight;
    const value = yMax - ratioY * (yMax - yMin);
    context.strokeStyle = '#e7ecf1';
    context.beginPath();
    context.moveTo(margin.left, y);
    context.lineTo(margin.left + plotWidth, y);
    context.stroke();
    context.fillStyle = '#8b98a5';
    context.textAlign = 'right';
    context.fillText(`${Math.round(value)}°`, margin.left - 7, y);
  }

  for (let i = 0; i <= 5; i += 1) {
    const ratioX = i / 5;
    const x = margin.left + ratioX * plotWidth;
    const seconds = (span * ratioX) / 1000;
    context.strokeStyle = '#f0f3f6';
    context.beginPath();
    context.moveTo(x, margin.top);
    context.lineTo(x, margin.top + plotHeight);
    context.stroke();
    context.fillStyle = '#8b98a5';
    context.textAlign = i === 0 ? 'left' : (i === 5 ? 'right' : 'center');
    context.textBaseline = 'top';
    context.fillText(cycleView ? `${Math.round(ratioX * 100)}%` : `${seconds.toFixed(1)}s`, x, margin.top + plotHeight + 7);
  }

  const series = [
    { key: 'hip', color: '#e45858' },
    { key: 'knee', color: '#2f73df' },
    { key: 'ankle', color: '#22a679' },
  ];
  for (const item of series) {
    context.strokeStyle = item.color;
    context.lineWidth = 1.8;
    context.lineJoin = 'round';
    context.lineCap = 'round';
    context.beginPath();
    for (let index = 0; index < samples.length; index += 1) {
      const row = samples[index];
      const x = xFor(row.t);
      const y = yFor(row[item.key]);
      if (index === 0 || (!cycleView && row.gapBefore)) context.moveTo(x, y);
      else context.lineTo(x, y);
    }
    const last = samples.at(-1);
    context.lineTo(xFor(last.t), yFor(last[item.key]));
    context.stroke();
  }
}

const mount = $('threeMount');
let renderer = null;
let scene = null;
let camera = null;
let controls = null;
let modelRoot = null;
let grid = null;
let loader = null;
let currentModel = null;
let currentBoneRegistry = new Map();
let loadToken = 0;
let activeModelId = 'lower';
let latestAngles = null;
let poseReady = false;

const GRID_SIZE = 240;
const JOINT_NAME_MAP = {
  hip: ['thighL', 'thigh.L', 'thigh_l', 'leftThigh', 'upperLeg.L', 'upperleg.L'],
  knee: ['shinL', 'shin.L', 'shin_l', 'leftShin', 'lowerLeg.L', 'lowerleg.L', 'calf.L'],
  ankle: ['footL', 'foot.L', 'foot_l', 'leftFoot', 'ankle.L'],
};
const AXIS_VECTOR = { x: new THREE.Vector3(1, 0, 0), y: new THREE.Vector3(0, 1, 0), z: new THREE.Vector3(0, 0, 1) };
const AXIS = { hip: 'x', knee: 'x', ankle: 'x' };
const SIGN = { hip: -1, knee: 1, ankle: 1 };
const OFFSET = { hip: 0, knee: 0, ankle: 0 };
const MODEL_URLS = {
  full: new URL('./models/fullbody3.glb', import.meta.url).href,
  lower: new URL('./models/lower_body1.glb', import.meta.url).href,
};
const toRadians = (degrees) => (Number(degrees) || 0) * Math.PI / 180;

function initializeScene() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xdce8f7);
  camera = new THREE.PerspectiveCamera(55, 1, 0.1, 5000);
  camera.position.set(0, 120, 260);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(mount.clientWidth, mount.clientHeight);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;
  mount.appendChild(renderer.domElement);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x5d6b78, 1.3));
  const directional = new THREE.DirectionalLight(0xffffff, 1.1);
  directional.position.set(2, 4, 2);
  scene.add(directional);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.enablePan = false;
  controls.enableRotate = false;

  grid = new THREE.GridHelper(GRID_SIZE, 24, 0x91a0af, 0xbdc9d5);
  scene.add(grid);
  loader = new GLTFLoader();

  const resize = () => {
    if (!renderer || !camera) return;
    const width = mount.clientWidth || 1;
    const height = mount.clientHeight || 1;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  };
  new ResizeObserver(resize).observe(mount);
  resize();
  requestAnimationFrame(animate);
  loadModel('lower');
}

function animate() {
  requestAnimationFrame(animate);
  if (!renderer) return;
  controls.update();
  renderer.render(scene, camera);
}

function normalizeBoneKey(name) {
  return (name || '').toLowerCase().replace(/[^a-z0-9]/g, '');
}

function buildBoneRegistry() {
  currentBoneRegistry = new Map();
  for (const skinMesh of currentModel.userData.skinMeshes || []) {
    for (const bone of skinMesh.skeleton.bones) {
      const key = normalizeBoneKey(bone.name);
      if (!key) continue;
      if (!currentBoneRegistry.has(key)) currentBoneRegistry.set(key, []);
      currentBoneRegistry.get(key).push(bone);
      if (!bone.userData.bindQ) bone.userData.bindQ = bone.quaternion.clone();
    }
  }
}

function getBones(joint) {
  for (const alias of JOINT_NAME_MAP[joint] || []) {
    const bones = currentBoneRegistry.get(normalizeBoneKey(alias));
    if (bones?.length) return bones;
  }
  return [];
}

function setJointDegrees(joint, degrees) {
  const bones = getBones(joint);
  if (!bones.length) return;
  const axis = AXIS_VECTOR[AXIS[joint]] || AXIS_VECTOR.x;
  const delta = new THREE.Quaternion().setFromAxisAngle(
    axis,
    SIGN[joint] * toRadians((OFFSET[joint] || 0) + (Number(degrees) || 0)),
  );
  for (const bone of bones) {
    const bind = bone.userData.bindQ || bone.quaternion;
    bone.quaternion.copy(bind).multiply(delta);
  }
}

function applyLegAngles(hip, knee, ankle) {
  setJointDegrees('hip', hip);
  setJointDegrees('knee', knee);
  setJointDegrees('ankle', ankle);
}

function resetPose() {
  for (const bones of currentBoneRegistry.values()) {
    for (const bone of bones) {
      if (bone.userData.bindQ) bone.quaternion.copy(bone.userData.bindQ);
    }
  }
  if (latestAngles) applyLegAngles(latestAngles.hip, latestAngles.knee, latestAngles.ankle);
}

function pushAngles(hip, knee, ankle) {
  latestAngles = { hip, knee, ankle };
  if (poseReady) applyLegAngles(hip, knee, ankle);
}

function disposeModel(model) {
  model.traverse((object) => {
    if (object.geometry) object.geometry.dispose();
    if (!object.material) return;
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    for (const material of materials) material.dispose();
  });
}

function clearCurrentModel() {
  if (modelRoot) {
    scene.remove(modelRoot);
    modelRoot.clear();
    modelRoot = null;
  }
  if (currentModel) {
    disposeModel(currentModel);
    currentModel = null;
  }
  currentBoneRegistry = new Map();
  poseReady = false;
}

function prepareModel(model) {
  const skinMeshes = [];
  model.traverse((object) => {
    if (object.isSkinnedMesh) {
      object.frustumCulled = false;
      skinMeshes.push(object);
    } else if (object.isMesh) {
      object.visible = false;
    }
  });

  model.rotation.set(0, 0, 0);
  model.scale.set(1, 1, 1);
  model.position.set(0, 0, 0);
  model.updateMatrixWorld(true);
  for (const skinMesh of skinMeshes) {
    skinMesh.normalizeSkinWeights();
    skinMesh.skeleton.pose();
    skinMesh.skeleton.calculateInverses();
    skinMesh.bind(skinMesh.skeleton);
  }
  model.userData.skinMeshes = skinMeshes;

  const initialBox = new THREE.Box3().setFromObject(model);
  const initialSize = initialBox.getSize(new THREE.Vector3());
  const initialCenter = initialBox.getCenter(new THREE.Vector3());
  model.position.sub(initialCenter);
  model.updateMatrixWorld(true);

  const maximumDimension = Math.max(initialSize.x, initialSize.y, initialSize.z) || 1;
  model.scale.setScalar((GRID_SIZE * 0.55) / maximumDimension);
  model.updateMatrixWorld(true);

  const groundedBox = new THREE.Box3().setFromObject(model);
  model.position.y += -groundedBox.min.y;
  model.updateMatrixWorld(true);

  const centeredBox = new THREE.Box3().setFromObject(model);
  const centered = centeredBox.getCenter(new THREE.Vector3());
  model.position.x -= centered.x;
  model.position.z -= centered.z;
  model.updateMatrixWorld(true);

  const root = new THREE.Group();
  root.rotation.y = Math.PI;
  root.add(model);
  scene.add(root);
  root.updateMatrixWorld(true);
  modelRoot = root;

  const sphere = new THREE.Sphere();
  new THREE.Box3().setFromObject(root).getBoundingSphere(sphere);
  camera.position.set(sphere.radius * 2.2, sphere.radius * 0.35, 0);
  camera.lookAt(0, sphere.center.y, 0);
  controls.target.set(0, sphere.center.y, 0);
  controls.update();
  controls.minDistance = sphere.radius * 0.8;
  controls.maxDistance = sphere.radius * 3.0;

  const bounds = new THREE.Box3().setFromObject(root).getSize(new THREE.Vector3());
  const radius = bounds.length() * 0.5 || 1;
  camera.near = Math.max(0.1, radius * 0.01);
  camera.far = radius * 20;
  camera.updateProjectionMatrix();
  buildBoneRegistry();
}

function missingModelBones() {
  return ['hip', 'knee', 'ankle'].filter((joint) => getBones(joint).length === 0);
}

function setActiveModel(modelId) {
  activeModelId = modelId;
  document.querySelectorAll('.model-option').forEach((button) => {
    const active = button.dataset.modelId === modelId;
    button.classList.toggle('active', active);
    button.setAttribute('aria-checked', String(active));
  });
  ui.modelTitle.textContent = MODEL_LABELS[modelId];
}

function loadModel(modelId) {
  if (!loader || !MODEL_URLS[modelId]) return;
  const token = ++loadToken;
  poseReady = false;
  setActiveModel(modelId);
  ui.modelStatus.textContent = `Đang tải ${MODEL_LABELS[modelId].toLowerCase()}…`;

  loader.load(
    MODEL_URLS[modelId],
    (gltf) => {
      if (token !== loadToken) return;
      const model = gltf.scene || gltf.scenes?.[0];
      if (!model) throw new Error('Tệp mô hình 3D không có nội dung hiển thị.');
      clearCurrentModel();
      currentModel = model;
      prepareModel(model);
      const missing = missingModelBones();
      poseReady = true;
      if (latestAngles) applyLegAngles(latestAngles.hip, latestAngles.knee, latestAngles.ankle);
      ui.modelStatus.textContent = missing.length
        ? `Đã tải; thiếu các khớp: ${missing.join(', ')}`
        : MODEL_LABELS[modelId];
      if (missing.length) ui.modelStatus.classList.add('warning');
    },
    (progress) => {
      if (token !== loadToken) return;
      const percent = (progress.loaded / (progress.total || 1)) * 100;
      ui.modelStatus.textContent = `Đang tải mô hình ${Math.round(percent)}%`;
    },
    (error) => {
      if (token !== loadToken) return;
      poseReady = Boolean(currentModel);
      ui.modelStatus.textContent = `Không tải được ${MODEL_LABELS[modelId].toLowerCase()}`;
      console.error(error);
    },
  );
}

ui.start.addEventListener('click', startSession);
for (const id of ['benchLink', 'motionLink']) $(id).addEventListener('click', (event) => {
  if (isConnected) { event.preventDefault(); showToast('Dừng phiên đo khớp trước khi chuyển màn hình.', true); }
});
ui.stop.addEventListener('click', stopSession);
ui.resetMax.addEventListener('click', resetMaxima);
ui.export.addEventListener('click', exportCsv);
ui.refreshPorts.addEventListener('click', () => refreshPorts(ui.port.value));
ui.neutral.addEventListener('click', async () => {
  const result = await performAction('capture_neutral');
  showToast(result.message, !result.ok);
});
ui.functional.addEventListener('click', async () => {
  const joint = collectingJoint || ui.calibrationJoint.value;
  const result = await performAction('functional_calibration', joint, Boolean(collectingJoint));
  if (result.ok) collectingJoint = collectingJoint ? null : joint;
  else collectingJoint = null;
  ui.functional.textContent = collectingJoint ? 'Xác nhận trục vừa ghi' : 'Ghi chuyển động gấp/duỗi';
  ui.calibrationJoint.disabled = Boolean(collectingJoint);
  showToast(result.message, !result.ok);
});
for (const [button, save] of [[ui.loadProfile, false], [ui.saveProfile, true]]) {
  button.addEventListener('click', async () => {
    const result = await performAction('calibration_profile', save);
    showToast(result.message, !result.ok && !result.cancelled);
  });
}
ui.gaitEvents.addEventListener('click', async () => {
  const result = await performAction('import_gait_events');
  showToast(result.message, !result.ok && !result.cancelled);
});
ui.gaitMode.addEventListener('click', () => {
  showGait = !showGait;
  ui.gaitMode.textContent = showGait ? 'Theo thời gian' : 'Trung bình chu kỳ';
  drawChart();
});
ui.port.addEventListener('change', updatePortDescription);
ui.resetPose.addEventListener('click', resetPose);
ui.clearChart.addEventListener('click', () => {
  chartSamples = [];
  showGait = false;
  ui.gaitMode.textContent = 'Trung bình chu kỳ';
  ui.chartEmpty.classList.remove('hidden');
  drawChart();
});
document.querySelectorAll('.model-option').forEach((button) => {
  button.addEventListener('click', () => loadModel(button.dataset.modelId));
});
new ResizeObserver(drawChart).observe(ui.chart.parentElement);

initializeBridge();
try {
  initializeScene();
} catch (error) {
  window.__sceneError = String(error?.stack || error);
  ui.modelStatus.textContent = 'Máy này chưa hiển thị được mô hình 3D. Vẫn có thể đo và xuất dữ liệu.';
  console.error(error);
}
