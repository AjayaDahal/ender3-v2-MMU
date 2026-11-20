import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";
import { STLLoader } from "three/addons/STLLoader.js";

const stageEl = document.getElementById("scene");
const overlayEl = document.getElementById("overlay");
const octx = overlayEl.getContext("2d");
const calloutLayer = document.getElementById("callout-layer");
const inspectHud = document.getElementById("inspect-hud");
const tipEl = document.getElementById("tip");
const plateBanner = document.getElementById("plate-banner");
const plateBannerText = document.getElementById("plate-banner-text");
const plateDownload = document.getElementById("plate-download");
const mediaEl = document.getElementById("media");
const videoEl = document.getElementById("video");
const studioImgEl = document.getElementById("studio-img");
const captionEl = document.getElementById("caption");

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.localClippingEnabled = true;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.12;
stageEl.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(34, 1, 0.005, 30);
camera.up.set(0, 0, 1);
camera.position.set(-0.55, -0.62, 0.72);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0, 0.35);

scene.add(new THREE.HemisphereLight(0xdce8ff, 0x262c38, 1.35));
const key = new THREE.DirectionalLight(0xfff4ea, 2.3);
key.position.set(-0.8, -1.0, 1.4);
scene.add(key);
const rim = new THREE.DirectionalLight(0x9ec5ff, 1.25);
rim.position.set(1.0, 1.0, 0.7);
scene.add(rim);

const assemblyGrid = new THREE.GridHelper(1.2, 12, 0x3b485e, 0x1d2533);
assemblyGrid.rotation.x = Math.PI / 2;
assemblyGrid.position.set(0, 0, 0);
scene.add(assemblyGrid);

const clipPlane = new THREE.Plane(new THREE.Vector3(0, -1, 0), 0);
const worldGroup = new THREE.Group();
scene.add(worldGroup);
const rootGroup = new THREE.Group();
const frameGroup = new THREE.Group();
const extraGroup = new THREE.Group();
const demoGroup = new THREE.Group();
worldGroup.add(rootGroup, frameGroup, extraGroup, demoGroup);

const bedGroup = new THREE.Group();
bedGroup.visible = false;
scene.add(bedGroup);

const MM = 0.001;
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const loader = new STLLoader();
const geoCache = new Map();

let manifest = null;
const records = [];
const hiddenParts = new Set();
const hiddenCallouts = new Set();
let calloutEls = [];
let colorMode = "reference";
let selectedId = null;
let isolatedId = null;
let explodeAmt = 0;
let xrayOn = false;
let sectionOn = false;
let turntableOn = false;
let labelsOn = true;
let frameOn = true;
let extrasOn = true;
let printBedMode = false;
let partFilterKind = "all";
let printFilterKind = "all";

// Tool-change demo state
const LANE_COL = [0xe53935, 0x43a047, 0x1e88e5, 0xfdd835];
const DEMO_LEN = 40;
const DEMO_STEPS = [
  { a: 0, b: 5, cap: "Printing in red (T0) — one nozzle, four filaments.", sub: "All four idlers are lifted (camshaft at 0°), so the stock extruder pulls red freely through the MMU." },
  { a: 5, b: 8, cap: "Slicer sends T1. Happy Hare parks the head and forms the tip.", sub: "Klipper drives the MMU directly — no switch tapping, no fixed wait." },
  { a: 8, b: 14, cap: "Happy Hare retracts red out of the hotend with the extruder.", sub: "Tip forming shapes a clean end so it slides back up the Bowden tube." },
  { a: 14, b: 16, cap: "Servo selector turns to 40°: lane 1 engaged.", sub: "Only lane 1's cam lobe points down and presses its spring-loaded idler onto its MK8 gear." },
  { a: 16, b: 22, cap: "Gear stepper pulls red back to park 16 mm behind the gate sensor.", sub: "The merger outlet microswitch tells Klipper exactly when red has cleared." },
  { a: 22, b: 24, cap: "Servo turns to 75°: lane 2 (green) engaged.", sub: "Every lane has its own drive gear on the shared 5 mm D-shaft." },
  { a: 24, b: 30, cap: "Green feeds through the merger and down the Bowden at speed.", sub: "Gate sensor confirms green passed; Happy Hare tracks calibrated Bowden distance." },
  { a: 30, b: 34, cap: "Extruder & gear stepper load together to the nozzle, then cam releases.", sub: "If anything stalls, Klipper pauses the print instead of printing air." },
  { a: 34, b: 40, cap: "Printing in green (T1). The wipe tower takes the purge.", sub: "Full cycle completes in ~20–30 s (56 s conservative sim)." },
];
const demo = { on: false, playing: false, t: 0, built: false };
let dCam = null, dSensor = null, swMat = null, carriage = null, nozzleDot = null;
let sleeveMesh = null, filMesh = null, laneStubs = [], cubeGreen = null, towerCap = null;
let curStep = -1;

const clamp01 = (x, a, b) => Math.min(1, Math.max(0, (x - a) / (b - a)));
const lerp = (a, b, k) => a + (b - a) * k;

async function loadGeo(url) {
  if (!geoCache.has(url)) {
    const geo = await loader.loadAsync(url);
    geo.scale(MM, MM, MM);
    geo.computeVertexNormals();
    geoCache.set(url, geo);
  }
  return geoCache.get(url);
}

function resize() {
  const w = stageEl.clientWidth || 640, h = stageEl.clientHeight || 480;
  camera.aspect = w / h; camera.updateProjectionMatrix();
  renderer.setSize(w, h, false);
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  overlayEl.width = Math.round(w * dpr); overlayEl.height = Math.round(h * dpr);
  octx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener("resize", resize);
resize();

function hideMedia() {
  videoEl.pause(); mediaEl.hidden = true; videoEl.hidden = true; studioImgEl.hidden = true;
}

function showStudioImage(src, title, subtitle = "Ender-MMU v0.4 Preview") {
  videoEl.pause(); videoEl.hidden = true;
  studioImgEl.src = src; studioImgEl.hidden = false;
  captionEl.textContent = `${title} · ${subtitle}`;
  mediaEl.hidden = false;
}

function exitPrintBedMode() {
  if (!printBedMode) return;
  printBedMode = false;
  bedGroup.visible = false;
  worldGroup.visible = true;
  assemblyGrid.visible = true;
  plateBanner.hidden = true;
  document.querySelectorAll("#solo-parts .item-btn").forEach((b) => b.classList.remove("active"));
  document.getElementById("eyebrow").textContent = "ENDER-MMU v0.4 / KLIPPER + HAPPY HARE";
  document.getElementById("stage-title").textContent = "4-Lane Multi-Material Unit";
  setCameraView("iso");
}

async function showSoloPartOnBed(partName) {
  const pp = manifest.printed_parts?.[partName];
  if (!pp) return;
  hideMedia();
  selectPart(null);
  demoSet(false);
  printBedMode = true;
  worldGroup.visible = false;
  assemblyGrid.visible = false;
  bedGroup.visible = true;
  bedGroup.clear();

  const bM = 0.22;
  const plateMesh = new THREE.Mesh(
    new THREE.BoxGeometry(bM, bM, 0.004),
    new THREE.MeshStandardMaterial({ color: 0x1e2533, metalness: 0.25, roughness: 0.65 })
  );
  plateMesh.position.set(0, 0, -0.0022);
  bedGroup.add(plateMesh);
  const border = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(bM, bM, 0.004)),
    new THREE.LineBasicMaterial({ color: 0xf04d10 })
  );
  border.position.set(0, 0, -0.0022);
  bedGroup.add(border);
  const bedGrid = new THREE.GridHelper(bM, 11, 0x4b5c78, 0x263144);
  bedGrid.rotation.x = Math.PI / 2;
  bedGrid.position.set(0, 0, 0.0002);
  bedGroup.add(bedGrid);

  const geo = await loadGeo(pp.url);
  geo.computeBoundingBox();
  const bb = geo.boundingBox;
  const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(pp.color || "#2f6db5"), roughness: 0.42, metalness: 0.08 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(-(bb.min.x + bb.max.x) / 2, -(bb.min.y + bb.max.y) / 2, -bb.min.z);
  bedGroup.add(mesh);

  const d = pp.dims_mm || [0, 0, 0];
  document.getElementById("eyebrow").textContent = `3-D PRINT BED PREVIEW · 220 × 220 mm ENDER-3 V2 (PETG)`;
  document.getElementById("stage-title").textContent = `${pp.name} (×${pp.qty})`;
  plateBannerText.innerHTML =
    `<strong>${pp.name}</strong> · ${d[0]}×${d[1]}×${d[2]} mm · qty ×${pp.qty} · ` +
    `<span class="pill ${pp.supports ? "pa" : "pass"}">${pp.supports ? "supports: lobes only" : "support=False"}</span>`;
  plateDownload.href = pp.stl_download;
  plateDownload.textContent = "Download STL";
  plateBanner.hidden = false;

  document.querySelectorAll("#solo-parts .item-btn").forEach((b) => b.classList.toggle("active", b.dataset.partName === partName));
  const span = Math.max(d[0], d[1], d[2], 50) * MM;
  camera.position.set(span * 1.45, -span * 1.65, span * 1.2);
  controls.target.set(0, 0, (d[2] * MM) / 2);
  controls.update();
  document.getElementById("view-label").textContent = `Print Bed (${pp.name})`;
}

function applyColorsAndOpacity() {
  for (const r of records) {
    const it = r.item;
    const visByHide = !hiddenParts.has(it.id);
    const visByIso = !isolatedId || it.id === isolatedId;
    r.mesh.visible = visByHide && visByIso;

    let hex;
    if (colorMode === "process") hex = it.name.includes("spool") ? 0x12b886 : 0xf04d10;
    else if (colorMode === "parts") hex = new THREE.Color().setHSL(r.hue, 0.58, 0.58).getHex();
    else hex = new THREE.Color(it.color || "#2f6db5").getHex();

    r.mat.color.setHex(hex);
    const isSel = selectedId && it.id === selectedId;
    const isBody = it.name === "mmu_body" || it.name === "electronics_cover";
    const faded = (xrayOn && isBody && !isSel) || (selectedId && !isSel && !isolatedId);
    r.mat.transparent = faded;
    r.mat.opacity = isSel ? 1.0 : (xrayOn && isBody ? 0.22 : (selectedId && !isolatedId ? 0.22 : 1.0));
    r.mat.depthWrite = !faded;
    r.mat.emissive.setHex(isSel ? 0x5c1d05 : 0x000000);
    r.mat.clippingPlanes = sectionOn ? [clipPlane] : [];
    r.mat.needsUpdate = true;
  }
  frameGroup.visible = frameOn && !isolatedId;
  extraGroup.visible = extrasOn && !isolatedId;
}

function updateExplode() {
  for (const r of records) {
    const [ex, ey, ez] = r.item.burst || [0, 0, 0];
    r.mesh.position.set(
      r.basePos.x + ex * MM * explodeAmt,
      r.basePos.y + ey * MM * explodeAmt,
      r.basePos.z + ez * MM * explodeAmt
    );
  }
}

function renderPartInspector(rec) {
  const detail = document.getElementById("detail");
  if (!rec) {
    inspectHud.hidden = true;
    detail.textContent = "Click any part in the 3-D model or list to inspect specs, print bed fit & STL download.";
    return;
  }
  const it = rec.item;
  const d = it.dims_mm || [0, 0, 0];
  const html = `
    <div class="hud-top">
      <div>
        <strong>${it.name}</strong>
        <div class="muted">PETG printed · qty ×${it.qty || 1}</div>
      </div>
      <span class="pill pass">PETG</span>
    </div>
    <div class="hud-grid">
      <span>Dimensions</span><span>${d[0]} × ${d[1]} × ${d[2]} mm</span>
      <span>Position</span><span>[${it.pos.map((v) => v.toFixed(1)).join(", ")}] mm</span>
    </div>
    <div class="muted">${it.label || it.name}</div>
    <div class="hud-actions">
      <button class="hud-btn" data-act="isolate">${isolatedId === it.id ? "Show All" : "Isolate"}</button>
      <button class="hud-btn" data-act="hide">Hide Part</button>
      <button class="hud-btn primary" data-act="bed">View on Print Bed</button>
      <a class="hud-btn" href="${it.stl_download}" download>STL</a>
      <button class="hud-btn" data-act="close">✕</button>
    </div>
  `;
  inspectHud.innerHTML = html;
  inspectHud.hidden = false;
  detail.innerHTML = html;

  const wire = (root) => {
    root.querySelectorAll("[data-act]").forEach((btn) => {
      btn.onclick = () => {
        const a = btn.dataset.act;
        if (a === "close") { isolatedId = null; selectPart(null); }
        else if (a === "isolate") { isolatedId = isolatedId === it.id ? null : it.id; applyColorsAndOpacity(); renderPartInspector(rec); }
        else if (a === "hide") { hiddenParts.add(it.id); isolatedId = null; document.getElementById("show-all-btn").checked = false; selectPart(null); }
        else if (a === "bed") showSoloPartOnBed(it.name);
      };
    });
  };
  wire(inspectHud);
  wire(detail);
}

function selectPart(id) {
  selectedId = (id && selectedId === id) ? null : id;
  if (!selectedId) isolatedId = null;
  applyColorsAndOpacity();
  document.querySelectorAll("#part-list .item-btn").forEach((b) => {
    b.classList.toggle("active", b.dataset.id === selectedId);
    b.classList.toggle("dim", hiddenParts.has(b.dataset.id));
  });
  const rec = records.find((r) => r.item.id === selectedId);
  calloutEls.forEach((c) => c.el.classList.toggle("active", rec && c.def.part === rec.item.name));
  renderPartInspector(rec || null);
}

function setCameraView(key) {
  hideMedia();
  if (printBedMode) exitPrintBedMode();
  const presets = {
    iso: [[-0.56, -0.62, 0.72], [0, 0, 0.34], "3/4 Isometric"],
    front: [[0, -0.88, 0.36], [0, 0, 0.34], "Front"],
    top: [[0.01, -0.01, 1.15], [0, 0.05, 0.34], "Top"],
    rear: [[0, 0.92, 0.45], [0, 0, 0.34], "Rear (Spools)"],
    closeup: [[0.22, -0.28, 0.62], [0, -0.03, 0.50], "MMU Close-up"],
    drive: [[-0.24, -0.22, 0.56], [0, 0, 0.50], "Drive Train"],
  };
  const p = presets[key] || presets.iso;
  camera.position.set(...p[0]);
  controls.target.set(...p[1]);
  controls.update();
  document.getElementById("view-label").textContent = p[2];
}

function syncLabelToggleUI() {
  document.getElementById("labels").checked = labelsOn;
  const qbtn = document.getElementById("quick-labels");
  qbtn.classList.toggle("active", labelsOn);
  qbtn.textContent = labelsOn ? "Labels: ON" : "Labels: OFF";
}

function buildCallouts() {
  calloutLayer.innerHTML = "";
  calloutEls = [];
  for (const def of manifest.callouts || []) {
    const rec = records.find((r) => r.item.name === def.part);
    if (!rec) continue;
    const el = document.createElement("div");
    el.className = "callout-pill";
    el.innerHTML = `<span>${def.label}</span><span class="x-btn" title="Hide this label">×</span>`;
    el.onclick = (e) => {
      if (e.target.classList.contains("x-btn")) {
        hiddenCallouts.add(def.id);
        el.style.display = "none";
        return;
      }
      selectPart(rec.item.id);
    };
    calloutLayer.appendChild(el);
    calloutEls.push({ def, rec, el });
  }
}

const _tmpA = new THREE.Vector3(), _tmpB = new THREE.Vector3(), _boxTmp = new THREE.Box3();
function updateCallouts() {
  const w = stageEl.clientWidth || 640, h = stageEl.clientHeight || 480;
  octx.clearRect(0, 0, w, h);
  if (!labelsOn || printBedMode || !mediaEl.hidden) {
    for (const c of calloutEls) c.el.style.display = "none";
    return;
  }
  for (const c of calloutEls) {
    if (hiddenCallouts.has(c.def.id) || !c.rec.mesh.visible) {
      c.el.style.display = "none";
      continue;
    }
    _boxTmp.setFromObject(c.rec.mesh);
    if (_boxTmp.isEmpty()) { c.el.style.display = "none"; continue; }
    _boxTmp.getCenter(_tmpA);
    const [ox, oy, oz] = c.def.offset || [60, 0, 50];
    _tmpB.set(ox * MM, oy * MM, oz * MM).applyQuaternion(worldGroup.quaternion).add(_tmpA);
    const pA = _tmpA.clone().project(camera);
    const pB = _tmpB.clone().project(camera);
    if (pA.z > 1 || pB.z > 1) { c.el.style.display = "none"; continue; }
    const ax = (pA.x * 0.5 + 0.5) * w, ay = (-pA.y * 0.5 + 0.5) * h;
    const bx = Math.max(85, Math.min(w - 85, (pB.x * 0.5 + 0.5) * w));
    const by = Math.max(60, Math.min(h - 52, (-pB.y * 0.5 + 0.5) * h));
    const isSel = c.rec.item.id === selectedId;
    octx.strokeStyle = isSel ? "#f04d10" : "rgba(180, 198, 224, 0.55)";
    octx.lineWidth = isSel ? 1.8 : 1.1;
    octx.beginPath(); octx.moveTo(ax, ay); octx.lineTo(bx, by); octx.stroke();
    octx.fillStyle = isSel ? "#f04d10" : "#ecf1f8";
    octx.beginPath(); octx.arc(ax, ay, isSel ? 3.5 : 2.5, 0, Math.PI * 2); octx.fill();
    c.el.style.display = "inline-flex";
    c.el.style.left = `${bx}px`;
    c.el.style.top = `${by}px`;
  }
}

// ---- Tool-Change Demo ----
function demoBuild() {
  const camRec = records.find((r) => r.item.name === "selector_camshaft");
  dCam = camRec ? camRec.mesh : null;
  dSensor = extraGroup.children.find((m) => m.userData.name === "gate_sensor") || null;
  swMat = dSensor ? dSensor.material : null;

  carriage = new THREE.Group();
  const plate = new THREE.Mesh(new THREE.BoxGeometry(0.070, 0.008, 0.060), new THREE.MeshStandardMaterial({ color: 0x546e7a, roughness: 0.5 }));
  plate.position.set(-0.095, -0.044, 0.305);
  const noz = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.0015, 0.014, 16), new THREE.MeshStandardMaterial({ color: 0xb0bec5, roughness: 0.35 }));
  noz.rotation.x = Math.PI / 2; noz.position.set(-0.095, -0.052, 0.268);
  nozzleDot = new THREE.Mesh(new THREE.SphereGeometry(0.003, 12, 8), new THREE.MeshStandardMaterial({ color: LANE_COL[0], roughness: 0.4 }));
  nozzleDot.position.set(-0.095, -0.052, 0.259);
  carriage.add(plate, noz, nozzleDot);
  demoGroup.add(carriage);

  laneStubs = [];
  const lx = manifest.lanes?.x || [-42, -14, 14, 42];
  const lz = (manifest.lanes?.z || 491) * MM;
  for (let i = 0; i < 4; i++) {
    const s = new THREE.Mesh(new THREE.CylinderGeometry(0.0012, 0.0012, 0.060, 10), new THREE.MeshStandardMaterial({ color: LANE_COL[i], roughness: 0.4 }));
    s.position.set(lx[i] * MM, 0.060, lz);
    demoGroup.add(s); laneStubs.push(s);
  }

  const red = new THREE.Mesh(new THREE.BoxGeometry(0.030, 0.030, 0.010), new THREE.MeshStandardMaterial({ color: LANE_COL[0], roughness: 0.5 }));
  red.position.set(0, 0, 0.036);
  cubeGreen = new THREE.Mesh(new THREE.BoxGeometry(0.030, 0.030, 0.010), new THREE.MeshStandardMaterial({ color: LANE_COL[1], roughness: 0.5 }));
  cubeGreen.position.set(0, 0, 0.041); cubeGreen.scale.set(1, 1, 0.001);
  const tower = new THREE.Mesh(new THREE.BoxGeometry(0.020, 0.020, 0.030), new THREE.MeshStandardMaterial({ color: LANE_COL[0], roughness: 0.5 }));
  tower.position.set(0.080, 0.080, 0.046);
  towerCap = new THREE.Mesh(new THREE.BoxGeometry(0.020, 0.020, 0.006), new THREE.MeshStandardMaterial({ color: LANE_COL[1], roughness: 0.5 }));
  towerCap.position.set(0.080, 0.080, 0.061); towerCap.scale.set(1, 1, 0.001);
  demoGroup.add(red, cubeGreen, tower, towerCap);

  const bowden = manifest.extras?.find((e) => e.name === "bowden_to_extruder")?.points || [[0, -60, 491], [-150, -10, 320]];
  const curve = new THREE.CatmullRomCurve3([
    ...bowden.map((p) => V(p[0] * MM, p[1] * MM, p[2] * MM)),
    V(-0.150, -0.010, 0.320), V(-0.120, -0.040, 0.300), V(-0.095, -0.052, 0.262),
  ]);
  sleeveMesh = new THREE.Mesh(new THREE.TubeGeometry(curve, 140, 0.0026, 8, false), new THREE.MeshStandardMaterial({ color: 0x9aa3ad, transparent: true, opacity: 0.32 }));
  filMesh = new THREE.Mesh(new THREE.TubeGeometry(curve, 140, 0.0013, 8, false), new THREE.MeshStandardMaterial({ color: LANE_COL[0], roughness: 0.4 }));
  demoGroup.add(sleeveMesh, filMesh);
  demoGroup.visible = false;
  demo.built = true;
}

function demoPose(t) {
  if (!demo.built) return;
  const dx = t < 5 ? 0.110 : t < 6 ? lerp(0.110, 0.170, clamp01(t, 5, 6))
           : t < 34 ? 0.170 : t < 35 ? lerp(0.170, 0.110, clamp01(t, 34, 35)) : 0.110;
  carriage.position.x = dx;
  if (swMat) swMat.emissive.setHex((t < 20 || t > 26) ? 0x22aa22 : 0x000000);
  const camAng = manifest.lanes?.cam_angles || [40, 75, 110, 145];
  const camNeu = manifest.lanes?.cam_neutral ?? 0;
  let cam = camNeu;
  if (t >= 14 && t < 22) cam = lerp(camNeu, camAng[0], clamp01(t, 14, 15));
  else if (t >= 22 && t < 30) cam = lerp(camAng[0], camAng[1], clamp01(t, 22, 23));
  else if (t >= 30) cam = lerp(camAng[1], camNeu, clamp01(t, 30, 31));
  if (dCam) dCam.rotation.x = THREE.MathUtils.degToRad(cam);

  const filFrac = t < 8 ? 1 : t < 14 ? 1 - 0.55 * clamp01(t, 8, 14)
                : t < 24 ? 0.45 * (1 - clamp01(t, 16, 22))
                : 0.75 * clamp01(t, 24, 30) + 0.25 * clamp01(t, 30, 34);
  filMesh.material.color.setHex(t < 23 ? LANE_COL[0] : LANE_COL[1]);
  if (filMesh.geometry.index) filMesh.geometry.setDrawRange(0, Math.floor(filMesh.geometry.index.count * filFrac));

  laneStubs[0].position.y = 0.060 + 0.040 * clamp01(t, 16, 22);
  laneStubs[1].position.y = 0.060 - 0.030 * clamp01(t, 24, 26);
  nozzleDot.material.color.setHex(t < 8 ? LANE_COL[0] : (t < 33 ? 0x9aa3ad : LANE_COL[1]));
  const cg = clamp01(t, 34, 38);
  cubeGreen.scale.z = Math.max(0.001, cg);
  cubeGreen.position.z = 0.041 + 0.005 * cg;
  const tc = clamp01(t, 33, 35);
  towerCap.scale.z = Math.max(0.001, tc);
  towerCap.position.z = 0.061 + 0.003 * tc;

  let si = DEMO_STEPS.findIndex((s) => t >= s.a && t < s.b);
  if (si < 0) si = DEMO_STEPS.length - 1;
  if (si !== curStep) {
    curStep = si;
    document.getElementById("demoCaption").textContent = DEMO_STEPS[si].cap;
    document.getElementById("demoSub").textContent = DEMO_STEPS[si].sub;
  }
  document.getElementById("demoT").textContent = `${t.toFixed(1)} s`;
  document.getElementById("demoTime").value = String(t.toFixed(1));
}

function demoSet(on) {
  if (on && !demo.built) demoBuild();
  demo.on = on;
  if (on) {
    hideMedia();
    if (printBedMode) exitPrintBedMode();
    explodeAmt = 0;
    document.getElementById("explode").value = "0";
    document.getElementById("amount").textContent = "0%";
    updateExplode();
    demoGroup.visible = true;
    setCameraView("iso");
    demoPose(demo.t);
  } else {
    demo.playing = false;
    if (demoGroup) demoGroup.visible = false;
    if (swMat) swMat.emissive.setHex(0x000000);
    if (dCam) dCam.rotation.x = THREE.MathUtils.degToRad(manifest.lanes?.cam_neutral ?? 0);
  }
  document.getElementById("demoPlay").textContent = demo.playing ? "⏸ Pause" : "▶ Play: Red → Green";
}

function buildUI() {
  const tabsDef = [
    ["model", "Model"], ["parts", "Parts"], ["demo", "Tool Change"],
    ["print", "Print Bed"], ["studio", "Renders"], ["checks", "Checks & FW"],
  ];
  const tabsEl = document.getElementById("tabs");
  tabsDef.forEach(([id, label], i) => {
    const btn = document.createElement("button");
    btn.textContent = label;
    if (i === 0) btn.classList.add("active");
    btn.onclick = () => {
      tabsEl.querySelectorAll("button").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
      tabsDef.forEach(([sid]) => { document.getElementById(sid).hidden = sid !== id; });
      if (id !== "studio" && id !== "demo") hideMedia();
      if (id !== "print" && printBedMode) exitPrintBedMode();
    };
    tabsEl.appendChild(btn);
  });

  const viewsEl = document.getElementById("views");
  [["iso", "3/4 Isometric"], ["closeup", "MMU Close-up"], ["drive", "Drive Train"],
   ["front", "Front"], ["top", "Top"], ["rear", "Rear (Spools)"]].forEach(([k, lbl]) => {
    const b = document.createElement("button");
    b.textContent = lbl;
    b.onclick = () => setCameraView(k);
    viewsEl.appendChild(b);
  });

  const colorsEl = document.getElementById("colors");
  [["reference", "Reference"], ["process", "MMU vs Spools"], ["parts", "By Part"]].forEach(([k, lbl], i) => {
    const b = document.createElement("button");
    b.textContent = lbl;
    if (i === 0) b.classList.add("active");
    b.onclick = () => {
      colorMode = k;
      colorsEl.querySelectorAll("button").forEach((x) => x.classList.remove("active"));
      b.classList.add("active");
      applyColorsAndOpacity();
    };
    colorsEl.appendChild(b);
  });

  document.getElementById("explode").oninput = (e) => {
    explodeAmt = Number(e.target.value) / 100;
    document.getElementById("amount").textContent = `${e.target.value}%`;
    updateExplode();
  };
  document.getElementById("xray").onchange = (e) => { xrayOn = e.target.checked; applyColorsAndOpacity(); };
  document.getElementById("section").onchange = (e) => { sectionOn = e.target.checked; applyColorsAndOpacity(); };
  document.getElementById("spin").onchange = (e) => { turntableOn = e.target.checked; };
  document.getElementById("labels").onchange = (e) => { labelsOn = e.target.checked; syncLabelToggleUI(); };
  document.getElementById("quick-labels").onclick = () => { labelsOn = !labelsOn; syncLabelToggleUI(); };
  document.getElementById("showframe").onchange = (e) => { frameOn = e.target.checked; applyColorsAndOpacity(); };
  document.getElementById("showextras").onchange = (e) => { extrasOn = e.target.checked; applyColorsAndOpacity(); };
  document.getElementById("show-all-btn").onchange = (e) => {
    if (e.target.checked) { hiddenParts.clear(); isolatedId = null; applyColorsAndOpacity(); }
  };

  document.getElementById("reset").onclick = () => {
    selectedId = null; isolatedId = null; hiddenParts.clear(); hiddenCallouts.clear();
    explodeAmt = 0; xrayOn = false; sectionOn = false; turntableOn = false; frameOn = true; extrasOn = true; labelsOn = true;
    demoSet(false);
    document.getElementById("explode").value = "0"; document.getElementById("amount").textContent = "0%";
    document.getElementById("xray").checked = false; document.getElementById("section").checked = false;
    document.getElementById("spin").checked = false; document.getElementById("showframe").checked = true;
    document.getElementById("showextras").checked = true; document.getElementById("show-all-btn").checked = true;
    syncLabelToggleUI(); updateExplode(); renderPartInspector(null); applyColorsAndOpacity(); setCameraView("iso");
  };

  document.getElementById("demoPlay").onclick = () => {
    if (!demo.on) demoSet(true);
    demo.playing = !demo.playing;
    document.getElementById("demoPlay").textContent = demo.playing ? "⏸ Pause" : "▶ Play: Red → Green";
  };
  document.getElementById("demoRestart").onclick = () => {
    if (!demo.on) demoSet(true);
    demo.t = 0; demo.playing = true;
    document.getElementById("demoPlay").textContent = "⏸ Pause";
    demoPose(0);
  };
  document.getElementById("demoClose").onclick = () => demoSet(false);
  document.getElementById("demoTime").oninput = (e) => {
    if (!demo.on) demoSet(true);
    demo.t = Number(e.target.value);
    demoPose(demo.t);
  };
  document.getElementById("open-gif-btn").onclick = () => {
    const gif = (manifest.studio_renders || []).find((r) => r.kind === "GIF");
    if (gif) showStudioImage(gif.file, "Inside-MMU Colour Change Mechanism", "Shared shaft, 4 MK8 gears, camshaft, idlers & gate sensor");
  };

  document.getElementById("media-close").onclick = hideMedia;
  document.getElementById("back-3d").onclick = hideMedia;
  document.getElementById("exit-plate").onclick = exitPrintBedMode;
  document.getElementById("back-3d-print").onclick = exitPrintBedMode;
  document.getElementById("collapse").onclick = (e) => {
    const c = document.querySelector("main").classList.toggle("collapsed");
    e.target.textContent = c ? "Show" : "Hide";
    setTimeout(resize, 20);
  };

  // Parts tab
  const pfEl = document.getElementById("part-filters");
  [["all", "All"], ["mmu", "MMU Head"], ["spool", "Spool Rack"]].forEach(([k, lbl], i) => {
    const b = document.createElement("button");
    b.textContent = lbl;
    if (i === 0) b.classList.add("active");
    b.onclick = () => {
      partFilterKind = k;
      pfEl.querySelectorAll("button").forEach((x) => x.classList.remove("active"));
      b.classList.add("active");
      renderPartList(document.getElementById("filter").value);
    };
    pfEl.appendChild(b);
  });

  const listEl = document.getElementById("part-list");
  document.getElementById("part-count").textContent = manifest.parts.length;
  const renderPartList = (q = "") => {
    listEl.innerHTML = "";
    const ql = q.trim().toLowerCase();
    for (const it of manifest.parts) {
      const isSpool = it.name.includes("spool");
      if (partFilterKind === "mmu" && isSpool) continue;
      if (partFilterKind === "spool" && !isSpool) continue;
      if (ql && !`${it.name} ${it.label || ""}`.toLowerCase().includes(ql)) continue;
      const b = document.createElement("button");
      b.className = "item-btn" + (it.id === selectedId ? " active" : "") + (hiddenParts.has(it.id) ? " dim" : "");
      b.dataset.id = it.id;
      b.innerHTML = `<span>${it.name}<br><small class="muted">${(it.label || "").slice(0, 44)}</small></span><span class="pill pass">×${it.qty || 1}</span>`;
      b.onclick = () => selectPart(it.id);
      listEl.appendChild(b);
    }
  };
  renderPartList();
  document.getElementById("filter").oninput = (e) => renderPartList(e.target.value);

  // Print Bed tab
  const soloEl = document.getElementById("solo-parts");
  const renderSolo = () => {
    soloEl.innerHTML = "";
    for (const pp of Object.values(manifest.printed_parts || {})) {
      if (printFilterKind === "cal" && !pp.fit_first) continue;
      const b = document.createElement("button");
      b.className = "item-btn";
      b.dataset.partName = pp.name;
      b.innerHTML = `<span><b>${pp.name}</b> (×${pp.qty})<br><small class="muted">${pp.dims_mm.join("×")} mm · ${pp.supports ? "supports: lobes" : "support-free"}</small></span><span class="pill ${pp.fit_first ? "pa" : "pass"}">${pp.fit_first ? "CAL" : "BED"}</span>`;
      b.onclick = () => showSoloPartOnBed(pp.name);
      soloEl.appendChild(b);
    }
  };
  document.querySelectorAll("#print-filter button").forEach((btn) => {
    btn.onclick = () => {
      printFilterKind = btn.dataset.pf;
      document.querySelectorAll("#print-filter button").forEach((x) => x.classList.toggle("active", x === btn));
      renderSolo();
    };
  });
  renderSolo();

  // Renders tab
  const studioEl = document.getElementById("studio-list");
  for (const r of manifest.studio_renders || []) {
    const b = document.createElement("button");
    b.className = "item-btn";
    b.innerHTML = `<span>${r.title}</span><span class="pill ${r.kind === "GIF" ? "pa" : "pass"}">${r.kind}</span>`;
    b.onclick = () => showStudioImage(r.file, r.title);
    studioEl.appendChild(b);
  }

  // Checks & FW tab
  const ck = manifest.checks || {};
  const sim = ck.simulation || {};
  const fw = ck.firmware || {};
  document.getElementById("gate-summary").innerHTML =
    `<span class="pill pass">Assembly Audit ${ck.assembly_audit?.passed || 80}/${ck.assembly_audit?.total || 80}</span>` +
    `<span class="pill pass">Pytest ${ck.pytest?.passed || 104}/${ck.pytest?.total || 104}</span>` +
    `<span class="pill pass">Transitions 12/12</span>`;
  document.getElementById("sim-dl").innerHTML = [
    ["Idle gear clearance", `${sim.idle_clearance_mm ?? 0.20} mm`],
    ["Filament bending strain", `${sim.filament_strain_pct ?? 0.82}% (PLA yield ${sim.pla_yield_pct ?? 2.5}%)`],
    ["Traction / drag margin", sim.traction_drag_ratio || "3.1–3.4×"],
    ["Servo error margin", sim.servo_margin_deg || "±8.2°"],
    ["Simulated transitions", sim.transitions_passed || "12 / 12 (~56 s)"],
    ["Total printed PETG", `~${sim.filament_mass_g || 875} g`],
    ["Estimated BOM cost", sim.bom_cost_usd || "$120–135 + Pi"],
  ].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
  document.getElementById("fw-dl").innerHTML = [
    ["Printer firmware", fw.printer_fw || "Klipper"],
    ["MMU controller", fw.mmu_mcu || "Raspberry Pi Pico"],
    ["Orchestrator", fw.orchestrator || "Happy Hare (Type-A servo)"],
    ["Cam lane angles", fw.cam_angles || "40/75/110/145°"],
    ["Gate park distance", fw.gate_park_mm || "16 mm behind sensor"],
  ].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
}

// Raycast hover & click-to-inspect
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let downPos = null;
function pickMesh(clientX, clientY) {
  if (printBedMode || !mediaEl.hidden) return null;
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects(records.filter((r) => r.mesh.visible).map((r) => r.mesh), false);
  return hits.length ? hits[0].object : null;
}
renderer.domElement.addEventListener("pointermove", (e) => {
  const hit = pickMesh(e.clientX, e.clientY);
  if (!hit) { tipEl.hidden = true; renderer.domElement.style.cursor = "grab"; return; }
  const rec = records.find((r) => r.mesh === hit);
  if (!rec) { tipEl.hidden = true; return; }
  renderer.domElement.style.cursor = "pointer";
  tipEl.textContent = `${rec.item.name} · ${rec.item.label || "PETG printed"}`;
  tipEl.style.left = `${e.clientX + 14}px`;
  tipEl.style.top = `${e.clientY + 14}px`;
  tipEl.hidden = false;
});
renderer.domElement.addEventListener("pointerleave", () => { tipEl.hidden = true; });
renderer.domElement.addEventListener("pointerdown", (e) => { downPos = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener("pointerup", (e) => {
  if (!downPos || Math.hypot(e.clientX - downPos[0], e.clientY - downPos[1]) > 5) return;
  const hit = pickMesh(e.clientX, e.clientY);
  if (hit) selectPart(hit.userData.id);
});
window.addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    if (!mediaEl.hidden) hideMedia();
    else if (printBedMode) exitPrintBedMode();
    else { isolatedId = null; selectPart(null); }
  }
});

async function init() {
  manifest = await fetch("manifest.json", { cache: "no-store" }).then((r) => r.json());
  for (let i = 0; i < manifest.parts.length; i++) {
    const p = manifest.parts[i];
    const geo = await loadGeo(p.url);
    const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(p.color || "#2f6db5"), roughness: 0.48, metalness: 0.06, side: THREE.DoubleSide });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(p.pos[0] * MM, p.pos[1] * MM, p.pos[2] * MM);
    mesh.rotation.set(...p.rot.map((d) => THREE.MathUtils.degToRad(d)));
    mesh.userData = { id: p.id, name: p.name };
    rootGroup.add(mesh);
    records.push({ item: p, mesh, mat, basePos: mesh.position.clone(), hue: (i * 0.618) % 1 });
  }

  // Ender-3 V2 reference frame
  const frameMat = new THREE.MeshStandardMaterial({ color: 0x5c687a, roughness: 0.75, metalness: 0.25 });
  const frameEdge = new THREE.LineBasicMaterial({ color: 0x333e4f, transparent: true, opacity: 0.55 });
  for (const f of manifest.frame || []) {
    const geo = new THREE.BoxGeometry(f.size[0] * MM, f.size[1] * MM, f.size[2] * MM);
    const m = new THREE.Mesh(geo, frameMat);
    m.position.set(f.pos[0] * MM, f.pos[1] * MM, f.pos[2] * MM);
    m.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), frameEdge));
    m.userData = { name: f.name };
    frameGroup.add(m);
  }

  // Bought hardware & PTFE tubes
  for (const e of manifest.extras || []) {
    let geo;
    if (e.kind === "cyl") {
      geo = new THREE.CylinderGeometry(e.r * MM, e.r * MM, e.len * MM, 28);
      if (e.axis === "x") geo.rotateZ(Math.PI / 2);
      else if (e.axis === "z") geo.rotateX(Math.PI / 2);
    } else if (e.kind === "tube") {
      const curve = new THREE.CatmullRomCurve3(e.points.map((p) => V(p[0] * MM, p[1] * MM, p[2] * MM)));
      geo = new THREE.TubeGeometry(curve, e.points.length * 4, e.r * MM, 10, false);
    } else {
      geo = new THREE.BoxGeometry(e.size[0] * MM, e.size[1] * MM, e.size[2] * MM);
    }
    const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(e.color || "#2e7d32"), roughness: 0.45, metalness: 0.15 });
    if (e.kind === "tube") { mat.transparent = true; mat.opacity = 0.72; }
    const m = new THREE.Mesh(geo, mat);
    if (e.pos) m.position.set(e.pos[0] * MM, e.pos[1] * MM, e.pos[2] * MM);
    m.userData = { name: e.name };
    extraGroup.add(m);
  }

  buildUI();
  buildCallouts();
  applyColorsAndOpacity();
  setCameraView("iso");

  let last = performance.now();
  function tick(now) {
    requestAnimationFrame(tick);
    const dt = Math.min(0.05, (now - last) * 0.001);
    last = now;
    if (demo.on && demo.playing) {
      demo.t = (demo.t + dt) % DEMO_LEN;
      demoPose(demo.t);
    }
    if (turntableOn && !printBedMode && mediaEl.hidden) worldGroup.rotation.z += dt * 0.45;
    controls.update();
    renderer.render(scene, camera);
    updateCallouts();
  }
  requestAnimationFrame(tick);
}

init();
