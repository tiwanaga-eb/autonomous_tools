/**
 * Backhoe Simulator — Three.js frontend
 *
 * Coordinate system note
 * ----------------------
 * MuJoCo uses Z-up (X forward, Y left, Z up).
 * Three.js uses Y-up (default).
 *
 * Conversion: mj(x,y,z) → three(x, z, -y)
 * For quaternions (w,x,y,z) → conjugate by Rx(-90°):
 *   q_three = Rx(-90°) * q_mj * Rx(+90°)
 *
 * All joint rotations are applied to the kinematic group hierarchy so
 * we can also update directly from q[] without needing xpos/xquat.
 */

import GUI from 'lil-gui'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'

// 接続先WSは ?ws=<url|port> クエリで上書き可（既定 :8765）。
// 例: localhost:3000/?ws=8766  /  ?ws=ws://localhost:8766
const _wsParam = new URLSearchParams(location.search).get('ws')
const WS_URL = _wsParam
  ? (/^wss?:\/\//.test(_wsParam) ? _wsParam : `ws://localhost:${_wsParam}`)
  : 'ws://localhost:8765'

// ─── geometry constants (mirror config.py GEOM) ──────────────────────────────
const G = {
  baseHalf:     [1.15, 0.75, 0.50] as const,
  swingPivot:   [0,    0,    1.1 ] as const,
  boomPivot:    [0.60, 0,    1.4 ] as const,
  boomLen: 4.5,
  armLen:  2.2,
  bucketLen: 1.1,
  tipLocal: [1.1, 0, -0.30] as const,
}

// ─── colour palette ───────────────────────────────────────────────────────────
const MAT_STEEL  = new THREE.MeshStandardMaterial({ color: 0xcc9f20 })
const MAT_DARK   = new THREE.MeshStandardMaterial({ color: 0x353840 })
const MAT_GROUND = new THREE.MeshStandardMaterial({
  color: 0x404550, wireframe: false, side: THREE.DoubleSide,
})
const MAT_TERRAIN = new THREE.MeshStandardMaterial({
  color: 0x8b7355, wireframe: false, side: THREE.DoubleSide,
  roughness: 0.9,
})
const MAT_TERRAIN_WIRE = new THREE.MeshBasicMaterial({
  color: 0x5a4a2a, wireframe: true,
})
const MAT_TIP = new THREE.MeshStandardMaterial({ color: 0xff3322 })

// ─── helpers ──────────────────────────────────────────────────────────────────
/** MuJoCo pos (x,y,z) → Three.js Vector3 */
function mp(x: number, y: number, z: number): THREE.Vector3 {
  return new THREE.Vector3(x, z, -y)
}

/** Capsule approximation: box along X axis in Three.js */
function capsuleBox(len: number, r: number): THREE.BoxGeometry {
  return new THREE.BoxGeometry(len, r * 2, r * 2)
}

// ─── Three.js setup ───────────────────────────────────────────────────────────
const canvas = document.getElementById('canvas') as HTMLCanvasElement
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true })
renderer.setPixelRatio(devicePixelRatio)
renderer.shadowMap.enabled = true
renderer.shadowMap.type = THREE.PCFSoftShadowMap

const scene = new THREE.Scene()
scene.background = new THREE.Color(0x1a1c20)
scene.fog = new THREE.FogExp2(0x1a1c20, 0.018)

const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 200)
camera.position.set(-8, 6, 12)
camera.lookAt(3, 0, 0)

const controls = new OrbitControls(camera, renderer.domElement)
controls.target.set(3, 1, 0)
controls.enableDamping = true
controls.dampingFactor = 0.08

// Lights
const ambient = new THREE.AmbientLight(0xffffff, 0.4)
scene.add(ambient)
const sun = new THREE.DirectionalLight(0xffffff, 1.2)
sun.position.set(10, 20, 8)
sun.castShadow = true
sun.shadow.mapSize.set(2048, 2048)
sun.shadow.camera.near = 0.5
sun.shadow.camera.far = 80
sun.shadow.camera.left = -20
sun.shadow.camera.right = 20
sun.shadow.camera.top = 20
sun.shadow.camera.bottom = -20
scene.add(sun)

// Grid ground
const gridHelper = new THREE.GridHelper(80, 80, 0x333333, 0x2a2a2a)
gridHelper.position.y = -0.01
scene.add(gridHelper)

// ─── machine kinematic chain ──────────────────────────────────────────────────
const machineRoot = new THREE.Group()
scene.add(machineRoot)

// base (static)
const baseGroup = new THREE.Group()
machineRoot.add(baseGroup)
const baseMesh = new THREE.Mesh(
  new THREE.BoxGeometry(G.baseHalf[0] * 2, G.baseHalf[2] * 2, G.baseHalf[1] * 2),
  MAT_DARK,
)
baseMesh.position.copy(mp(0, 0, G.baseHalf[2]))
baseMesh.castShadow = true
baseMesh.receiveShadow = true
baseGroup.add(baseMesh)

// swing group — rotates about Y (MuJoCo Z)
const swingGroup = new THREE.Group()
swingGroup.position.copy(mp(G.swingPivot[0], G.swingPivot[1], G.swingPivot[2]))
machineRoot.add(swingGroup)

const cabMesh = new THREE.Mesh(
  new THREE.BoxGeometry(1.0 * 2, 0.65 * 2, 0.85 * 2),
  MAT_STEEL,
)
cabMesh.position.copy(mp(-0.3, 0, 0.7))
cabMesh.castShadow = true
swingGroup.add(cabMesh)

// boom group — pivot inside swingGroup
const boomGroup = new THREE.Group()
boomGroup.position.copy(mp(G.boomPivot[0], G.boomPivot[1], G.boomPivot[2]))
swingGroup.add(boomGroup)

const boomMesh = new THREE.Mesh(capsuleBox(G.boomLen, 0.22), MAT_STEEL)
boomMesh.position.set(G.boomLen / 2, 0, 0)
boomMesh.castShadow = true
boomGroup.add(boomMesh)

// arm group — pivot at boom tip
const armGroup = new THREE.Group()
armGroup.position.set(G.boomLen, 0, 0)
boomGroup.add(armGroup)

const armMesh = new THREE.Mesh(capsuleBox(G.armLen, 0.17), MAT_STEEL)
armMesh.position.set(G.armLen / 2, 0, 0)
armMesh.castShadow = true
armGroup.add(armMesh)

// bucket group — pivot at arm tip
const bucketGroup = new THREE.Group()
bucketGroup.position.set(G.armLen, 0, 0)
armGroup.add(bucketGroup)

const bucketBackMesh = new THREE.Mesh(
  new THREE.BoxGeometry(1.0, 0.16, 0.96),
  MAT_STEEL,
)
bucketBackMesh.position.set(0.5, -0.10, 0)
bucketBackMesh.rotation.z = 0.5   // matches model.xml euler
bucketBackMesh.castShadow = true
bucketGroup.add(bucketBackMesh)

const bucketEdgeMesh = new THREE.Mesh(
  new THREE.BoxGeometry(0.20, 0.10, 0.96),
  MAT_DARK,
)
bucketEdgeMesh.position.set(1.05, -0.30, 0)
bucketGroup.add(bucketEdgeMesh)

// tip marker
const tipMesh = new THREE.Mesh(new THREE.SphereGeometry(0.06, 8, 8), MAT_TIP)
tipMesh.position.set(G.tipLocal[0], G.tipLocal[2], -G.tipLocal[1])
bucketGroup.add(tipMesh)

// ─── terrain mesh ─────────────────────────────────────────────────────────────
let terrainMesh: THREE.Mesh | null = null
let terrainWire: THREE.Mesh | null = null

interface TerrainMsg {
  origin: [number, number]
  res: number
  size_x: number
  size_y: number
  grid: number[][]
}

function buildOrUpdateTerrain(msg: TerrainMsg) {
  const nx = Math.round(msg.size_x / msg.res)  // columns (x)
  const ny = Math.round(msg.size_y / msg.res)  // rows (y)

  // PlaneGeometry: width along X, height along Z in Three.js Y-up
  const geo = new THREE.PlaneGeometry(msg.size_x, msg.size_y, nx - 1, ny - 1)
  geo.rotateX(-Math.PI / 2)  // lay flat

  // Position: centre of the grid in Three.js coords
  // MuJoCo origin is (x_min, y_min); centre = origin + half-size
  const cx = msg.origin[0] + msg.size_x / 2
  const cy = msg.origin[1] + msg.size_y / 2

  // Set vertex heights from grid
  // After rotateX the vertex order: row-major, (i=col along +X, j=row along +Z)
  // Three.js PlaneGeometry: vertices are row-major left-to-right, then front-to-back.
  // After rotateX(-PI/2): +X stays +X, -Z becomes +Y, +Y becomes +Z.
  // Our terrain: grid[i][j] = height at MuJoCo (x = origin_x + i*res, y = origin_y + j*res)
  const pos = geo.attributes.position
  const cols = nx
  const rows = ny
  for (let j = 0; j < rows; j++) {
    for (let i = 0; i < cols; i++) {
      const idx = j * cols + i
      // grid[i][j]: i=x-index, j=y-index
      const h = (msg.grid[i] && msg.grid[i][j] != null) ? msg.grid[i][j] : 0
      pos.setY(idx, h)
    }
  }
  pos.needsUpdate = true
  geo.computeVertexNormals()

  if (terrainMesh) {
    scene.remove(terrainMesh)
    scene.remove(terrainWire!)
    terrainMesh.geometry.dispose()
    terrainWire!.geometry.dispose()
  }

  terrainMesh = new THREE.Mesh(geo, MAT_TERRAIN)
  terrainMesh.position.set(cx, 0, -cy)  // MuJoCo y → Three.js -z
  terrainMesh.receiveShadow = true
  scene.add(terrainMesh)

  terrainWire = new THREE.Mesh(geo.clone(), MAT_TERRAIN_WIRE)
  terrainWire.position.copy(terrainMesh.position)
  scene.add(terrainWire)
}

// ─── trajectory visualization ─────────────────────────────────────────────────
const trajectoryGeom = new THREE.BufferGeometry()
const trajectoryPositions: number[] = []
const trajectoryLine = new THREE.Line(
  trajectoryGeom,
  new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 1, transparent: true, opacity: 0.7 })
)
scene.add(trajectoryLine)

// ─── performance graphs ───────────────────────────────────────────────────────
const graphData = {
  payload: [] as number[],
  force: [] as number[],
  safety: [] as number[],
  maxPoints: 200,
}

function updateGraphs(msg: StateMsg) {
  // Accumulate data
  graphData.payload.push(msg.payload * 1000)  // liters
  graphData.force.push(Math.sqrt(msg.tau[0]**2 + msg.tau[1]**2 + msg.tau[2]**2 + msg.tau[3]**2) / 1000)  // normalized
  graphData.safety.push(msg.safety.min_h)

  if (graphData.payload.length > graphData.maxPoints) {
    graphData.payload.shift()
    graphData.force.shift()
    graphData.safety.shift()
  }

  // Draw payload graph
  drawGraph('graph-payload', graphData.payload, '#0ff', 'Payload (L)', 0, 200)
  // Draw force graph
  drawGraph('graph-force', graphData.force, '#f80', 'Force (kN)', 0, 100)
  // Draw safety margin graph
  drawGraph('graph-safety', graphData.safety, graphData.safety[graphData.safety.length-1] < 0 ? '#f00' : '#4c4', 'Min-H (m)', -2, 2)
}

function drawGraph(canvasId: string, data: number[], color: string, title: string, minY: number, maxY: number) {
  const canvas = $(canvasId) as HTMLCanvasElement
  const ctx = canvas.getContext('2d')!
  const w = canvas.width
  const h = canvas.height

  ctx.fillStyle = 'rgba(0,0,0,0.7)'
  ctx.fillRect(0, 0, w, h)

  // Title
  ctx.fillStyle = '#aaa'
  ctx.font = '10px monospace'
  ctx.fillText(title, 4, 12)

  if (data.length < 2) return

  // Plot
  ctx.strokeStyle = color
  ctx.lineWidth = 1.5
  ctx.beginPath()

  for (let i = 0; i < data.length; i++) {
    const x = (i / (data.length - 1)) * (w - 8) + 4
    const norm = (data[i] - minY) / (maxY - minY)
    const y = h - 20 - Math.max(0, Math.min(1, norm)) * (h - 24)

    if (i === 0) ctx.moveTo(x, y)
    else ctx.lineTo(x, y)
  }
  ctx.stroke()

  // Axes
  ctx.strokeStyle = '#444'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(4, h - 20)
  ctx.lineTo(w - 4, h - 20)
  ctx.stroke()
}

// ─── update trajectory ────────────────────────────────────────────────────────
function updateTrajectory(tip: number[]) {
  trajectoryPositions.push(tip[0])
  trajectoryPositions.push(tip[2])
  trajectoryPositions.push(-tip[1])

  if (trajectoryPositions.length > 600) {  // max ~200 points (600 / 3)
    trajectoryPositions.splice(0, 3)
  }

  trajectoryGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(trajectoryPositions), 3))
}
function applyJointAngles(q: number[]) {
  // swing: MuJoCo hinge about Z → Three.js rotation about Y (same sign)
  swingGroup.rotation.y = q[0]
  // boom/arm/bucket: MuJoCo hinge about Y (+q → tip down)
  //   in Three.js (after coord transform): rotation.z = -q
  boomGroup.rotation.z = -q[1]
  armGroup.rotation.z  = -q[2]
  bucketGroup.rotation.z = -q[3]
}

// ─── HUD update ───────────────────────────────────────────────────────────────
const $ = (id: string) => document.getElementById(id)!

function updateHUD(msg: StateMsg) {
  const r3 = (n: number) => n.toFixed(3)
  const deg = (n: number) => (n * 180 / Math.PI).toFixed(1) + '°'
  ;($('h-mode')    as HTMLElement).textContent = msg.mode
  ;($('h-phase')   as HTMLElement).textContent = msg.fsm_phase
  ;($('h-tip')     as HTMLElement).textContent = msg.tip.map(r3).join(' ')
  ;($('h-payload') as HTMLElement).textContent = (msg.payload * 1000).toFixed(1) + ' L'
  ;($('h-q0')      as HTMLElement).textContent = deg(msg.q[0])
  ;($('h-q1')      as HTMLElement).textContent = deg(msg.q[1])
  ;($('h-q2')      as HTMLElement).textContent = deg(msg.q[2])
  ;($('h-q3')      as HTMLElement).textContent = deg(msg.q[3])
  const safeEl = $('h-safe') as HTMLElement
  const safe = !msg.safety.geofence && !msg.safety.cbf && !msg.safety.active
  safeEl.textContent = safe ? '✓ OK' : '⚠ ACTIVE'
  safeEl.className = safe ? 'safe' : 'warn'
  ;($('h-minh') as HTMLElement).textContent = msg.safety.min_h.toFixed(3)
}

// ─── WebSocket ────────────────────────────────────────────────────────────────
interface StateMsg {
  type: 'state'
  t: number
  q: number[]
  qd: number[]
  tip: number[]
  mode: string
  fsm_phase: string
  payload: number
  tau: number[]
  safety: { geofence: boolean; cbf: boolean; active: boolean; min_h: number }
  xpos: Record<string, number[]>
  xquat: Record<string, number[]>
  paused: boolean
}

let ws: WebSocket | null = null
let reconnectTimer: ReturnType<typeof setTimeout> | null = null
const badge = document.getElementById('conn-badge')!

function connect() {
  ws = new WebSocket(WS_URL)

  ws.onopen = () => {
    badge.textContent = '● CONNECTED'
    badge.classList.add('connected')
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null }
  }

  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data)
    if (msg.type === 'state') {
      applyJointAngles(msg.q)
      updateHUD(msg)
      updateTrajectory(msg.tip)
      updateGraphs(msg)
    } else if (msg.type === 'terrain') {
      buildOrUpdateTerrain(msg)
    }
  }

  ws.onclose = () => {
    badge.textContent = '● DISCONNECTED'
    badge.classList.remove('connected')
    reconnectTimer = setTimeout(connect, 2000)
  }

  ws.onerror = () => ws?.close()
}

function send(obj: object) {
  if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj))
}

connect()

// ─── virtual joystick ────────────────────────────────────────────────────────
interface JsState { x: number; y: number; active: boolean; pointerId: number | null }

function makeJoystick(canvasId: string): () => JsState {
  const el = document.getElementById(canvasId) as HTMLCanvasElement
  const ctx = el.getContext('2d')!
  const R = el.width / 2
  const state: JsState = { x: 0, y: 0, active: false, pointerId: null }

  function draw() {
    ctx.clearRect(0, 0, el.width, el.height)
    ctx.beginPath()
    ctx.arc(R, R, R - 2, 0, Math.PI * 2)
    ctx.strokeStyle = 'rgba(255,255,255,0.2)'
    ctx.stroke()
    // thumb
    const tx = R + state.x * (R - 16)
    const ty = R + state.y * (R - 16)
    ctx.beginPath()
    ctx.arc(tx, ty, 14, 0, Math.PI * 2)
    ctx.fillStyle = state.active ? 'rgba(100,200,255,0.8)' : 'rgba(200,200,200,0.4)'
    ctx.fill()
  }

  function fromEvent(e: PointerEvent) {
    const rect = el.getBoundingClientRect()
    const nx = ((e.clientX - rect.left) / rect.width  - 0.5) * 2
    const ny = ((e.clientY - rect.top)  / rect.height - 0.5) * 2
    const len = Math.sqrt(nx * nx + ny * ny)
    const clamped = Math.min(1, len)
    state.x = len > 0 ? nx / len * clamped : 0
    state.y = len > 0 ? ny / len * clamped : 0
  }

  el.addEventListener('pointerdown', (e) => {
    state.active = true
    state.pointerId = e.pointerId
    el.setPointerCapture(e.pointerId)
    fromEvent(e); draw()
  })
  el.addEventListener('pointermove', (e) => {
    if (!state.active) return
    fromEvent(e); draw()
  })
  const release = () => { state.active = false; state.x = 0; state.y = 0; draw() }
  el.addEventListener('pointerup', release)
  el.addEventListener('pointercancel', release)

  draw()
  return () => ({ ...state })
}

const getLeft  = makeJoystick('js-left')
const getRight = makeJoystick('js-right')

// ─── WASD keyboard fallback ───────────────────────────────────────────────────
const keys: Record<string, boolean> = {}
window.addEventListener('keydown', (e) => { keys[e.key.toLowerCase()] = true })
window.addEventListener('keyup',   (e) => { keys[e.key.toLowerCase()] = false })

function keyAxes(): number[] {
  // ISO pattern: left=[bucket,boom], right=[swing,arm]
  const lx = (keys['a'] ? -1 : 0) + (keys['d'] ? 1 : 0)
  const ly = (keys['w'] ? -1 : 0) + (keys['s'] ? 1 : 0)
  const rx = (keys['arrowleft'] ? -1 : 0) + (keys['arrowright'] ? 1 : 0)
  const ry = (keys['arrowup']   ? -1 : 0) + (keys['arrowdown']  ? 1 : 0)
  return [lx, ly, rx, ry]
}

// ─── Gamepad API ──────────────────────────────────────────────────────────────
function gamepadAxes(): number[] | null {
  const pads = navigator.getGamepads()
  for (const pad of pads) {
    if (pad && pad.axes.length >= 4) {
      return Array.from(pad.axes.slice(0, 4))
    }
  }
  return null
}

// Joystick command pump (10 Hz is fine — server side rate-limits)
let jsInterval: ReturnType<typeof setInterval> | null = null

function startJsPump() {
  if (jsInterval) return
  jsInterval = setInterval(() => {
    const gp = gamepadAxes()
    let axes: number[]
    if (gp) {
      axes = gp
    } else {
      const L = getLeft()
      const R = getRight()
      const kd = keyAxes()
      axes = [
        L.active ? L.x : kd[0],
        L.active ? L.y : kd[1],
        R.active ? R.x : kd[2],
        R.active ? R.y : kd[3],
      ]
    }
    if (ws?.readyState === WebSocket.OPEN) {
      send({ cmd: 'joystick', axes, buttons: [] })
    }
  }, 50)  // 20 Hz
}
startJsPump()

// ─── lil-gui controls ────────────────────────────────────────────────────────
const gui = new GUI({ title: 'Backhoe Sim' })
gui.domElement.style.right = '0'
gui.domElement.style.top = '0'

const modes = { mode: 'AUTO' }
gui.add(modes, 'mode', ['AUTO', 'MANUAL', 'MANUAL_ASSIST', 'AUTO_OVERRIDE'])
  .name('Mode').onChange((v: string) => send({ cmd: 'mode', mode: v }))

const man = { name: 'dig', cycle: false }
const manFolder = gui.addFolder('Maneuvers')
manFolder.add(man, 'name', ['dig', 'load', 'trench', 'grade', 'slope', 'compact'])
  .name('Maneuver')
manFolder.add(man, 'cycle').name('Cycle (load)')
manFolder.add({ run: () => send({ cmd: 'maneuver', name: man.name, cycle: man.cycle }) }, 'run')
  .name('▶ Run')
manFolder.add({ stop: () => send({ cmd: 'mode', mode: 'AUTO' }) }, 'stop').name('■ Stop')

gui.add({ reset: () => send({ cmd: 'reset' }) }, 'reset').name('Reset')

const simCtrl = { pause: () => send({ cmd: 'pause' }), resume: () => send({ cmd: 'resume' }) }
const simFolder = gui.addFolder('Sim Control')
simFolder.add(simCtrl, 'pause').name('Pause')
simFolder.add(simCtrl, 'resume').name('Resume')

// Joint sliders (JOINT mode)
const jointFolder = gui.addFolder('Joints (deg)')
jointFolder.close()
const jointVals = { swing: 0, boom: 18, arm: -110, bucket: -140 }
const JOINT_LIMITS: Record<string, [number, number]> = {
  swing: [-180, 180], boom: [-60, 65], arm: [-150, 0], bucket: [-175, 5],
}
for (const [name, [lo, hi]] of Object.entries(JOINT_LIMITS)) {
  jointFolder.add(jointVals, name as keyof typeof jointVals, lo, hi, 0.5)
    .name(name).onChange((v: number) => {
      send({ cmd: 'mode', mode: 'JOINT' })
      send({ cmd: 'set_joint', name, value: v * Math.PI / 180 })
    })
}

// Safety overlay toggle
const viz = { terrain: true, safety: true }
const vizFolder = gui.addFolder('Visualisation')
vizFolder.add(viz, 'terrain').name('Terrain').onChange((v: boolean) => {
  if (terrainMesh) terrainMesh.visible = v
  if (terrainWire) terrainWire.visible = v
})

// ─── resize handler ───────────────────────────────────────────────────────────
function resize() {
  const w = window.innerWidth
  const h = window.innerHeight
  renderer.setSize(w, h)
  camera.aspect = w / h
  camera.updateProjectionMatrix()
}
window.addEventListener('resize', resize)
resize()

// ─── render loop ──────────────────────────────────────────────────────────────
function animate() {
  requestAnimationFrame(animate)
  controls.update()
  renderer.render(scene, camera)
}
animate()
