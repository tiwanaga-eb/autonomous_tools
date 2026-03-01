const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");
const banner = document.getElementById("banner");
const metricsBox = document.getElementById("metricsBox");
const statusBox = document.getElementById("statusBox");

const vehicleSelect = document.getElementById("vehicle");
const algorithmInput = document.getElementById("algorithm");
const tolPos = document.getElementById("tolPos");
const tolYaw = document.getElementById("tolYaw");
const timeoutMs = document.getElementById("timeoutMs");
const sampleCount = document.getElementById("sampleCount");
const yawBins = document.getElementById("yawBins");
const straightMargin = document.getElementById("straightMargin");
const seedInput = document.getElementById("seed");
const primXyResInput = document.getElementById("primXyRes");
const primDeltaBinsInput = document.getElementById("primDeltaBins");
const primTInput = document.getElementById("primT");
const primDtInput = document.getElementById("primDt");
const primSpeedLevelsInput = document.getElementById("primSpeedLevels");
const primDeltaDotLevelsInput = document.getElementById("primDeltaDotLevels");
const primVFwdInput = document.getElementById("primVFwd");
const primVRevInput = document.getElementById("primVRev");
const primSwitchDistInput = document.getElementById("primSwitchDist");
const primMaxNodesInput = document.getElementById("primMaxNodes");
const primCorridorWidthInput = document.getElementById("primCorridorWidth");
const primDistIncLimitInput = document.getElementById("primDistIncLimit");
const primAstarWeightInput = document.getElementById("primAstarWeight");
const primCollisionStrideInput = document.getElementById("primCollisionStride");
const ppKappa1Input = document.getElementById("ppKappa1");
const ppKappa2Input = document.getElementById("ppKappa2");
const ppSwitchBuffersInput = document.getElementById("ppSwitchBuffers");
const ppYawOffsetsInput = document.getElementById("ppYawOffsets");
const ppArcStepInput = document.getElementById("ppArcStep");
const ppMaxRevRatioInput = document.getElementById("ppMaxRevRatio");
const ppSwitchYawLimitInput = document.getElementById("ppSwitchYawLimit");
const ppKappaSwitchLimitRatioInput = document.getElementById("ppKappaSwitchLimitRatio");
const ppReverseArcRatioLimitInput = document.getElementById("ppReverseArcRatioLimit");
const ppSwitchSectorDMinInput = document.getElementById("ppSwitchSectorDMin");
const ppSwitchSectorDMaxInput = document.getElementById("ppSwitchSectorDMax");
const ppSwitchSectorAngleDegInput = document.getElementById("ppSwitchSectorAngleDeg");
const ppDeltaNeutralLimitDegInput = document.getElementById("ppDeltaNeutralLimitDeg");
const ppForwardLengthRatioInput = document.getElementById("ppForwardLengthRatio");
const ppEntryAngleLimitDegInput = document.getElementById("ppEntryAngleLimitDeg");
const ppSwitchRankMinObstacleClearanceInput = document.getElementById("ppSwitchRankMinObstacleClearance");
const ppSwitchRankMinEdgeMarginInput = document.getElementById("ppSwitchRankMinEdgeMargin");
const applyMiningPresetBtn = document.getElementById("applyMiningPreset");
const enableSmoothingInput = document.getElementById("enableSmoothing");
const wLen = document.getElementById("wLen");
const wTime = document.getElementById("wTime");
const wRev = document.getElementById("wRev");
const wGoal = document.getElementById("wGoal");
const bgMode = document.getElementById("bgMode");
const originLatInput = document.getElementById("originLat");
const originLonInput = document.getElementById("originLon");
const startYawInput = document.getElementById("startYaw");
const dockYawInput = document.getElementById("dockYaw");
const exitYawInput = document.getElementById("exitYaw");

const modeButtons = Array.from(document.querySelectorAll(".modes button[data-mode]"));

const state = {
  mode: "draw_drivable",
  drivable: [],
  obstacles: [],
  drawingObstacle: [],
  start: null,
  dock: null,
  exit: null,
  plan: null,
  scale: 8,
  tx: 80,
  ty: 80,
  panning: false,
  lastPan: null,
  backgroundMode: "grid",
  tileCache: new Map(),
};

function degToRad(deg) {
  return (deg * Math.PI) / 180.0;
}

function parseCsvNumbers(text) {
  if (!text) return [];
  return text
    .split(",")
    .map((s) => Number(s.trim()))
    .filter((v) => Number.isFinite(v));
}

function localToLatLon(x, y) {
  const lat0 = Number(originLatInput.value) || 0;
  const lon0 = Number(originLonInput.value) || 0;
  const dLat = y / 111320.0;
  const dLon = x / (111320.0 * Math.cos((lat0 * Math.PI) / 180.0));
  return { lat: lat0 + dLat, lon: lon0 + dLon };
}

function latLonToLocal(lat, lon) {
  const lat0 = Number(originLatInput.value) || 0;
  const lon0 = Number(originLonInput.value) || 0;
  const y = (lat - lat0) * 111320.0;
  const x = (lon - lon0) * 111320.0 * Math.cos((lat0 * Math.PI) / 180.0);
  return { x, y };
}

function latLonToTileXY(lat, lon, z) {
  const latRad = (lat * Math.PI) / 180.0;
  const n = 2 ** z;
  const x = ((lon + 180.0) / 360.0) * n;
  const y = ((1.0 - Math.log(Math.tan(latRad) + 1.0 / Math.cos(latRad)) / Math.PI) / 2.0) * n;
  return { x, y };
}

function tileXYToLatLon(x, y, z) {
  const n = 2 ** z;
  const lon = (x / n) * 360.0 - 180.0;
  const latRad = Math.atan(Math.sinh(Math.PI * (1.0 - (2.0 * y) / n)));
  const lat = (latRad * 180.0) / Math.PI;
  return { lat, lon };
}

function estimateTileZoom() {
  const lat0 = Number(originLatInput.value) || 0;
  const metersPerPixel = 1.0 / Math.max(state.scale, 1e-9);
  let bestZ = 12;
  let bestDiff = Infinity;
  for (let z = 0; z <= 20; z += 1) {
    const mpp = (156543.03392 * Math.cos((lat0 * Math.PI) / 180.0)) / (2 ** z);
    const diff = Math.abs(mpp - metersPerPixel);
    if (diff < bestDiff) {
      bestDiff = diff;
      bestZ = z;
    }
  }
  return bestZ;
}

function tileUrl(provider, z, x, y) {
  if (provider === "osm") {
    return `https://tile.openstreetmap.org/${z}/${x}/${y}.png`;
  }
  if (provider === "esri") {
    return `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${z}/${y}/${x}`;
  }
  return null;
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}

window.addEventListener("resize", resizeCanvas);

function worldToScreen(p) {
  return {
    x: p.x * state.scale + state.tx,
    y: canvas.clientHeight - (p.y * state.scale + state.ty),
  };
}

function screenToWorld(x, y) {
  return {
    x: (x - state.tx) / state.scale,
    y: ((canvas.clientHeight - y) - state.ty) / state.scale,
  };
}

function setMode(nextMode) {
  state.mode = nextMode;
  modeButtons.forEach((btn) => btn.classList.toggle("active", btn.dataset.mode === nextMode));
  statusBox.textContent = `mode: ${nextMode}`;
}

modeButtons.forEach((btn) => {
  btn.addEventListener("click", () => setMode(btn.dataset.mode));
});

function drawPolygon(points, style = "#000", dashed = false) {
  if (points.length === 0) return;
  ctx.save();
  ctx.strokeStyle = style;
  ctx.lineWidth = 1.5;
  ctx.setLineDash(dashed ? [6, 4] : []);
  ctx.beginPath();
  points.forEach((p, i) => {
    const s = worldToScreen({ x: p[0], y: p[1] });
    if (i === 0) ctx.moveTo(s.x, s.y);
    else ctx.lineTo(s.x, s.y);
  });
  if (points.length > 2) {
    const s0 = worldToScreen({ x: points[0][0], y: points[0][1] });
    ctx.lineTo(s0.x, s0.y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawPose(pose, color) {
  if (!pose) return;
  const s = worldToScreen(pose);
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(s.x, s.y, 5, 0, Math.PI * 2);
  ctx.fill();
  const len = 18;
  ctx.strokeStyle = color;
  ctx.beginPath();
  ctx.moveTo(s.x, s.y);
  ctx.lineTo(s.x + Math.cos(pose.yaw) * len, s.y - Math.sin(pose.yaw) * len);
  ctx.stroke();
  ctx.restore();
}

function drawPlan() {
  if (!state.plan || !state.plan.segments) return;

  const roadWidth = Number(state.plan.road_width || 0);
  if (roadWidth > 0) {
    state.plan.segments.forEach((seg) => {
      ctx.save();
      ctx.strokeStyle = "rgba(30, 64, 175, 0.15)";
      ctx.lineWidth = Math.max(1, roadWidth * state.scale);
      ctx.setLineDash([]);
      ctx.beginPath();
      seg.states.forEach((st, i) => {
        const s = worldToScreen(st);
        if (i === 0) ctx.moveTo(s.x, s.y);
        else ctx.lineTo(s.x, s.y);
      });
      ctx.stroke();
      ctx.restore();
    });
  }

  state.plan.segments.forEach((seg) => {
    ctx.save();
    ctx.strokeStyle = "#1d4ed8";
    ctx.lineWidth = 2;
    ctx.setLineDash(seg.gear === "R" ? [8, 5] : []);
    ctx.beginPath();
    seg.states.forEach((st, i) => {
      const s = worldToScreen(st);
      if (i === 0) ctx.moveTo(s.x, s.y);
      else ctx.lineTo(s.x, s.y);
    });
    ctx.stroke();
    ctx.restore();
  });

  // Overlay ramp regions for smoothing visibility.
  state.plan.segments.forEach((seg) => {
    const ramps = seg.states.filter((s) => s.is_ramp);
    if (ramps.length < 2) return;
    ctx.save();
    ctx.strokeStyle = "#f97316";
    ctx.lineWidth = 2;
    ctx.setLineDash([2, 4]);
    ctx.beginPath();
    ramps.forEach((st, i) => {
      const s = worldToScreen(st);
      if (i === 0) ctx.moveTo(s.x, s.y);
      else ctx.lineTo(s.x, s.y);
    });
    ctx.stroke();
    ctx.restore();
  });

  if (state.plan.switch_pose) {
    drawPose(state.plan.switch_pose, "#f59e0b");
  }
}

function drawGrid() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  const step = Math.max(20, state.scale);
  ctx.save();
  ctx.strokeStyle = "#f0f0f0";
  ctx.lineWidth = 1;
  for (let x = state.tx % step; x < w; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = (h - state.ty) % step; y < h; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
  ctx.restore();
}

async function drawBackgroundTiles() {
  if (state.backgroundMode === "grid") {
    drawGrid();
    return;
  }

  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  const corners = [
    screenToWorld(0, 0),
    screenToWorld(w, 0),
    screenToWorld(0, h),
    screenToWorld(w, h),
  ];
  const lls = corners.map((p) => localToLatLon(p.x, p.y));
  const minLat = Math.min(...lls.map((p) => p.lat));
  const maxLat = Math.max(...lls.map((p) => p.lat));
  const minLon = Math.min(...lls.map((p) => p.lon));
  const maxLon = Math.max(...lls.map((p) => p.lon));

  const z = estimateTileZoom();
  const a = latLonToTileXY(maxLat, minLon, z);
  const b = latLonToTileXY(minLat, maxLon, z);
  const minTx = Math.floor(Math.min(a.x, b.x));
  const maxTx = Math.floor(Math.max(a.x, b.x));
  const minTy = Math.floor(Math.min(a.y, b.y));
  const maxTy = Math.floor(Math.max(a.y, b.y));
  const n = 2 ** z;

  for (let tx = minTx; tx <= maxTx; tx += 1) {
    for (let ty = minTy; ty <= maxTy; ty += 1) {
      const xWrap = ((tx % n) + n) % n;
      if (ty < 0 || ty >= n) continue;
      const key = `${state.backgroundMode}:${z}:${xWrap}:${ty}`;
      if (!state.tileCache.has(key)) {
        const url = tileUrl(state.backgroundMode, z, xWrap, ty);
        if (!url) continue;
        const img = new Image();
        img.crossOrigin = "anonymous";
        img.src = url;
        img.onload = () => draw();
        state.tileCache.set(key, img);
      }
      const img = state.tileCache.get(key);
      if (!img || !img.complete) continue;

      const nw = tileXYToLatLon(tx, ty, z);
      const se = tileXYToLatLon(tx + 1, ty + 1, z);
      const pNw = latLonToLocal(nw.lat, nw.lon);
      const pSe = latLonToLocal(se.lat, se.lon);
      const sNw = worldToScreen(pNw);
      const sSe = worldToScreen(pSe);
      const dx = sNw.x;
      const dy = sNw.y;
      const dw = sSe.x - sNw.x;
      const dh = sSe.y - sNw.y;
      ctx.drawImage(img, dx, dy, dw, dh);
    }
  }
}

async function draw() {
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  await drawBackgroundTiles();
  drawPolygon(state.drivable, "#111");
  state.obstacles.forEach((o) => drawPolygon(o, "#777"));
  drawPolygon(state.drawingObstacle, "#777", true);
  drawPose(state.start, "#16a34a");
  drawPose(state.dock, "#dc2626");
  drawPose(state.exit, "#2563eb");
  drawPlan();
}

function setBanner(msg) {
  if (!msg) {
    banner.classList.add("hidden");
    banner.textContent = "";
    return;
  }
  banner.classList.remove("hidden");
  banner.textContent = msg;
}

function setMetrics(planResp) {
  if (!planResp || !planResp.metrics) {
    metricsBox.textContent = "No plan metrics";
    return;
  }
  const m = planResp.metrics;
  const samples = planResp.debug ? planResp.debug.candidate_generated : 0;
  const nodesExpanded = planResp.debug ? planResp.debug.nodes_expanded || 0 : 0;
  const primitivesTested = planResp.debug ? planResp.debug.primitives_tested || 0 : 0;
  const prunedCorridor = planResp.debug ? planResp.debug.pruned_by_corridor || 0 : 0;
  const prunedDom = planResp.debug ? planResp.debug.pruned_by_dominance || 0 : 0;
  const prunedDistInc = planResp.debug ? planResp.debug.pruned_by_distance_increase || 0 : 0;
  const ratioBehaviorReject = planResp.debug ? planResp.debug.ratio_behavior_reject || 0 : 0;
  const ratioCollisionReject = planResp.debug ? planResp.debug.ratio_collision_reject || 0 : 0;
  const ratioCostPruned = planResp.debug ? planResp.debug.ratio_cost_pruned || 0 : 0;
  const stageUsed = planResp.debug ? planResp.debug.stage_used || 0 : 0;
  metricsBox.innerHTML = [
    `status: ${planResp.status}`,
    `length: ${m.total_length.toFixed(2)} m`,
    `time: ${m.total_time.toFixed(2)} s`,
    `compute: ${m.compute_time_ms.toFixed(1)} ms`,
    `samples: ${samples}`,
    `nodes_expanded: ${nodesExpanded}`,
    `primitives_tested: ${primitivesTested}`,
    `pruned_corridor: ${prunedCorridor}`,
    `pruned_dominance: ${prunedDom}`,
    `pruned_distance_inc: ${prunedDistInc}`,
    `ratio_behavior_reject: ${ratioBehaviorReject.toFixed(2)}`,
    `ratio_collision_reject: ${ratioCollisionReject.toFixed(2)}`,
    `ratio_cost_pruned: ${ratioCostPruned.toFixed(2)}`,
    `stage_used: ${stageUsed}`,
    `collision_free: ${m.collision_free}`,
    `dynamic_feasible: ${m.dynamic_feasible}`,
    `smoothing_applied: ${m.smoothing_applied}`,
    `added_smoothing_length: ${m.added_smoothing_length.toFixed(2)} m`,
    `max_kappa_rate: ${m.max_kappa_rate.toFixed(3)} 1/m^2`,
    `road_width: ${(planResp.road_width || 0).toFixed(2)} m`,
  ].join("<br>");
}

canvas.addEventListener("contextmenu", (e) => e.preventDefault());

canvas.addEventListener("mousedown", (e) => {
  if (e.button === 2) {
    state.panning = true;
    state.lastPan = { x: e.clientX, y: e.clientY };
    return;
  }

  const rect = canvas.getBoundingClientRect();
  const p = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);

  if (state.mode === "draw_drivable") {
    state.drivable.push([p.x, p.y]);
    state.plan = null;
  } else if (state.mode === "draw_obstacle") {
    state.drawingObstacle.push([p.x, p.y]);
    state.plan = null;
  } else if (state.mode === "set_start") {
    state.start = { x: p.x, y: p.y, yaw: degToRad(Number(startYawInput.value) || 0) };
  } else if (state.mode === "set_dock") {
    state.dock = { x: p.x, y: p.y, yaw: degToRad(Number(dockYawInput.value) || 0) };
  } else if (state.mode === "set_exit") {
    state.exit = { x: p.x, y: p.y, yaw: degToRad(Number(exitYawInput.value) || 0) };
  }
  draw();
});

canvas.addEventListener("mousemove", (e) => {
  if (!state.panning) return;
  const dx = e.clientX - state.lastPan.x;
  const dy = e.clientY - state.lastPan.y;
  state.tx += dx;
  state.ty -= dy;
  state.lastPan = { x: e.clientX, y: e.clientY };
  draw();
});

window.addEventListener("mouseup", () => {
  state.panning = false;
});

canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left;
  const sy = e.clientY - rect.top;
  const before = screenToWorld(sx, sy);
  const factor = e.deltaY < 0 ? 1.1 : 0.9;
  state.scale = Math.max(0.02, Math.min(120, state.scale * factor));
  const after = worldToScreen(before);
  state.tx += sx - after.x;
  state.ty += (canvas.clientHeight - sy) - (canvas.clientHeight - after.y);
  draw();
}, { passive: false });

document.getElementById("finishPolygon").addEventListener("click", () => {
  if (state.mode === "draw_drivable") {
    if (state.drivable.length < 3) {
      setBanner("Drivable polygon needs at least 3 points");
      return;
    }
    setBanner("");
  }
  if (state.mode === "draw_obstacle") {
    if (state.drawingObstacle.length >= 3) {
      state.obstacles.push([...state.drawingObstacle]);
      state.drawingObstacle = [];
      setBanner("");
    } else {
      setBanner("Obstacle polygon needs at least 3 points");
    }
  }
  draw();
});

document.getElementById("clearAll").addEventListener("click", () => {
  state.drivable = [];
  state.obstacles = [];
  state.drawingObstacle = [];
  state.start = null;
  state.dock = null;
  state.exit = null;
  state.plan = null;
  setBanner("");
  metricsBox.textContent = "No plan metrics";
  draw();
});

async function loadVehicles() {
  try {
    const res = await fetch("/api/vehicles");
    const data = await res.json();
    vehicleSelect.innerHTML = "";
    data.vehicles.forEach((v) => {
      const opt = document.createElement("option");
      opt.value = v;
      opt.textContent = v;
      vehicleSelect.appendChild(opt);
    });
    if (data.vehicles.includes("HD785-7")) {
      vehicleSelect.value = "HD785-7";
    }
  } catch (e) {
    statusBox.textContent = `vehicle list error: ${e}`;
  }
}

async function generatePath() {
  setBanner("");
  if (state.drawingObstacle.length >= 3) {
    state.obstacles.push([...state.drawingObstacle]);
    state.drawingObstacle = [];
  }
  if (state.drivable.length < 3) {
    setBanner("Please draw drivable polygon first");
    return;
  }
  if (!state.start || !state.dock) {
    setBanner("Please set both start and dock poses");
    return;
  }

  const payload = {
    vehicle_id: vehicleSelect.value,
    start_pose: state.start,
    dock_pose: state.dock,
    exit_pose: state.exit,
    drivable_polygon: state.drivable,
    obstacles: state.obstacles,
    planner_params: {
      algorithm: algorithmInput.value,
      timeout_ms: Number(timeoutMs.value),
      tolerances: {
        pos: Number(tolPos.value),
        yaw: degToRad(Number(tolYaw.value)),
      },
      weights: {
        w_len: Number(wLen.value),
        w_time: Number(wTime.value),
        w_rev: Number(wRev.value),
        w_goal: Number(wGoal.value),
      },
      sampling_params: {
        switch_sample_count: Number(sampleCount.value),
        yaw_bins: Number(yawBins.value),
        straight_margin: Number(straightMargin.value),
        enable_smoothing: enableSmoothingInput.checked,
      },
      primitives_params: {
        xy_res: Number(primXyResInput.value),
        yaw_bins: Number(yawBins.value),
        delta_bins: Number(primDeltaBinsInput.value),
        T: Number(primTInput.value),
        dt: Number(primDtInput.value),
        speed_levels: Number(primSpeedLevelsInput.value),
        delta_dot_levels: Number(primDeltaDotLevelsInput.value),
        v_fwd: Number(primVFwdInput.value),
        v_rev: Number(primVRevInput.value),
        switch_distance_threshold: Number(primSwitchDistInput.value),
        max_nodes_expanded: Number(primMaxNodesInput.value),
        corridor_width: Number(primCorridorWidthInput.value),
        distance_increase_limit: Number(primDistIncLimitInput.value),
        astar_weight: Number(primAstarWeightInput.value),
        collision_check_stride: Number(primCollisionStrideInput.value),
      },
      dubins_pp_params: {
        kappa1_ratios: parseCsvNumbers(ppKappa1Input.value),
        kappa2_ratios: parseCsvNumbers(ppKappa2Input.value),
        switch_buffer_lengths: parseCsvNumbers(ppSwitchBuffersInput.value),
        switch_yaw_offsets_deg: parseCsvNumbers(ppYawOffsetsInput.value),
        arc_sample_step: Number(ppArcStepInput.value),
        max_reverse_length_ratio: Number(ppMaxRevRatioInput.value),
        switch_yaw_limit_deg: Number(ppSwitchYawLimitInput.value),
        kappa_switch_limit_ratio: Number(ppKappaSwitchLimitRatioInput.value),
      },
      dubins_pp_behavioral_params: {
        reverse_arc_ratio_limit: Number(ppReverseArcRatioLimitInput.value),
        switch_sector_d_min: Number(ppSwitchSectorDMinInput.value),
        switch_sector_d_max: Number(ppSwitchSectorDMaxInput.value),
        switch_sector_angle_deg: Number(ppSwitchSectorAngleDegInput.value),
        delta_neutral_limit_deg: Number(ppDeltaNeutralLimitDegInput.value),
        forward_length_ratio: Number(ppForwardLengthRatioInput.value),
        entry_angle_limit_deg: Number(ppEntryAngleLimitDegInput.value),
        switch_rank_min_obstacle_clearance: Number(ppSwitchRankMinObstacleClearanceInput.value),
        switch_rank_min_edge_margin: Number(ppSwitchRankMinEdgeMarginInput.value),
      },
      seed: Number(seedInput.value),
    },
  };

  statusBox.textContent = "planning...";
  try {
    const res = await fetch("/api/plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });

    if (!res.ok) {
      const err = await res.json();
      setBanner(`Error: ${err.detail || res.statusText}`);
      statusBox.textContent = "request failed";
      return;
    }

    const data = await res.json();
    state.plan = data;
    setMetrics(data);

    if (data.status === "NO_FEASIBLE_PATH") {
      setBanner("NO_FEASIBLE_PATH");
    } else if (data.status === "TIMEOUT") {
      setBanner("TIMEOUT");
    } else {
      setBanner("");
    }

    draw();
    statusBox.textContent = `done: ${data.status}`;
  } catch (e) {
    setBanner(`Network error: ${e}`);
    statusBox.textContent = "network error";
  }
}

document.getElementById("generateBtn").addEventListener("click", () => {
  generatePath();
});

applyMiningPresetBtn.addEventListener("click", () => {
  ppKappa1Input.value = "0.8,1.0";
  ppKappa2Input.value = "0.8,1.0";
  ppSwitchBuffersInput.value = "0,1,2";
  ppYawOffsetsInput.value = "-10,0,10";
  ppArcStepInput.value = "0.2";
  ppMaxRevRatioInput.value = "1.5";
  ppSwitchYawLimitInput.value = "30";
  ppKappaSwitchLimitRatioInput.value = "0.2";
  ppReverseArcRatioLimitInput.value = "0.3";
  ppSwitchSectorDMinInput.value = "5.0";
  ppSwitchSectorDMaxInput.value = "20.0";
  ppSwitchSectorAngleDegInput.value = "30";
  ppDeltaNeutralLimitDegInput.value = "10";
  ppForwardLengthRatioInput.value = "1.2";
  ppEntryAngleLimitDegInput.value = "20";
  ppSwitchRankMinObstacleClearanceInput.value = "0.5";
  ppSwitchRankMinEdgeMarginInput.value = "0.5";
});

bgMode.addEventListener("change", () => {
  state.backgroundMode = bgMode.value;
  state.tileCache.clear();
  draw();
});

originLatInput.addEventListener("change", () => {
  state.tileCache.clear();
  draw();
});
originLonInput.addEventListener("change", () => {
  state.tileCache.clear();
  draw();
});

setMode("draw_drivable");
loadVehicles();
resizeCanvas();
metricsBox.textContent = "No plan metrics";
