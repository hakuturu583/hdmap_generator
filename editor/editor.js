// The roadgen editor.
//
// The page owns the document and every interaction; the WebAssembly engine
// (`roadgen-editor-wasm`) owns everything about roads. After each change the
// document goes across as JSON and comes back as what the generator made of it —
// lanes, markings, junction connectors — drawn by `roadgen-irview`, with every
// problem pinned to the road or link it is about. Nothing here knows how a lane is
// laid out or how a junction is connected: what is drawn is the IR, never a guess
// at it.

import init, {
  buildDocument,
  exampleDocument,
  exportDocument,
  formats,
  importFile,
} from './pkg/roadgen_editor_wasm.js'

const STORAGE_KEY = 'roadgen-editor/document'
const SNAP_PX = 10
const HANDLE_PX = 7
const LANE_TYPES = ['driving', 'shoulder', 'border', 'sidewalk', 'biking', 'parking', 'restricted', 'none']
const ROAD_TYPES = ['town', 'rural', 'motorway', 'low_speed', 'pedestrian']
const MARKINGS = ['none', 'solid', 'broken', 'solid solid', 'broken solid', 'solid broken', 'curb']
const COLORS = ['white', 'yellow']

const LANE_FILL = {
  driving: '#4a4f57',
  biking: '#7a4d4d',
  sidewalk: '#b8b2a7',
  shoulder: '#6c6f73',
  parking: '#5a6270',
  border: '#7d7a74',
  restricted: '#7d7a74',
  none: '#8c8a86',
}

// --- State -------------------------------------------------------------------------

const state = {
  doc: null,
  built: null,
  selection: null, // {kind: 'road', id} | {kind: 'junction', id} | {kind: 'link', index}
  point: null, // index of the selected control point of the selected road
  lane: null, // index of the lane being edited in the inspector
  tool: 'select',
  view: { cx: 0, cy: 0, scale: 3 },
  drawing: null, // {points: [[x,y,z]], startSnap, endSnap}
  linkFrom: null, // {road, end}
  drag: null,
  cursor: null, // world [x, y]
  formats: [],
}
const undoStack = []
const redoStack = []

const $ = (id) => document.getElementById(id)
const canvas = $('canvas')
const ctx = canvas.getContext('2d')

// --- Document helpers --------------------------------------------------------------

const clone = (value) => JSON.parse(JSON.stringify(value))
const roadKey = (irId) => (irId?.startsWith('road/') ? irId.slice(5) : irId)
const junctionKey = (irId) => (irId?.startsWith('junction/') ? irId.slice(9) : irId)
const findRoad = (id) => state.doc.roads.find((road) => road.id === id)
const endPoint = (road, end) => (end === 'start' ? road.points[0] : road.points[road.points.length - 1])
const endIndex = (road, end) => (end === 'start' ? 0 : road.points.length - 1)

function uniqueId(prefix, taken) {
  for (let i = 1; ; i++) {
    const id = `${prefix}${i}`
    if (!taken.has(id)) return id
  }
}

function junctionNames() {
  return [...new Set(state.doc.links.map((link) => link.junction).filter(Boolean))]
}

function defaultLanes() {
  return [lane(3.5, 'forward'), lane(3.5, 'backward')]
}

function lane(width, direction, laneType = 'driving') {
  return {
    width,
    direction,
    lane_type: laneType,
    left_marking: { kind: 'solid', color: 'white' },
    right_marking: { kind: 'solid', color: 'white' },
  }
}

function emptyDocument() {
  return { version: 1, name: 'untitled', handedness: 'right', roads: [], links: [] }
}

// --- History -----------------------------------------------------------------------

/// Applies a change to the document as one undoable step.
function commit(change, { select } = {}) {
  undoStack.push(JSON.stringify(state.doc))
  if (undoStack.length > 200) undoStack.shift()
  redoStack.length = 0
  change(state.doc)
  if (select !== undefined) state.selection = select
  changed()
}

/// Puts a whole new document in place, as one undoable step.
function replace(doc) {
  undoStack.push(JSON.stringify(state.doc))
  redoStack.length = 0
  state.doc = { ...emptyDocument(), ...doc }
  state.selection = null
  changed()
}

/// Records the document as it was before a change made in place (a drag).
function checkpoint(snapshot) {
  undoStack.push(snapshot)
  redoStack.length = 0
}

function undo() {
  if (!undoStack.length) return
  redoStack.push(JSON.stringify(state.doc))
  state.doc = JSON.parse(undoStack.pop())
  changed()
}

function redo() {
  if (!redoStack.length) return
  undoStack.push(JSON.stringify(state.doc))
  state.doc = JSON.parse(redoStack.pop())
  changed()
}

function changed() {
  validateSelection()
  rebuild()
  save()
  renderPanels()
  render()
}

function validateSelection() {
  const s = state.selection
  if (!s) return
  if (s.kind === 'road' && !findRoad(s.id)) state.selection = null
  if (s.kind === 'junction' && !junctionNames().includes(s.id)) state.selection = null
  if (s.kind === 'link' && !state.doc.links[s.index]) state.selection = null
  if (state.selection?.kind !== 'road') state.point = null
  const road = state.selection?.kind === 'road' && findRoad(state.selection.id)
  if (road && state.point != null && state.point >= road.points.length) state.point = null
}

let saveTimer = null
function save() {
  clearTimeout(saveTimer)
  saveTimer = setTimeout(() => {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(state.doc))
    } catch {
      // Storage is a convenience; a private window simply does not remember.
    }
  }, 300)
}

function restore() {
  try {
    const saved = localStorage.getItem(STORAGE_KEY)
    if (saved) return JSON.parse(saved)
  } catch {
    // Fall through to the example.
  }
  return null
}

// --- Building ----------------------------------------------------------------------

function rebuild() {
  try {
    state.built = JSON.parse(buildDocument(JSON.stringify(state.doc)))
  } catch (error) {
    state.built = {
      scene: { lanes: [], lines: [], roads: [], junctions: [], objects: [], areas: [] },
      problems: [{ severity: 'error', road: null, link: null, message: String(error) }],
      valid: false,
      stats: {},
    }
  }
  renderStatus()
}

let pendingFrame = 0
/// Rebuilds at most once a frame, for changes that arrive with every mouse move.
function rebuildSoon() {
  if (pendingFrame) return
  pendingFrame = requestAnimationFrame(() => {
    pendingFrame = 0
    rebuild()
    render()
  })
}

function problemRoads() {
  const roads = new Set()
  for (const problem of state.built?.problems ?? []) {
    if (problem.road) roads.add(problem.road)
    if (problem.link != null) {
      const link = state.doc.links[problem.link]
      if (link) {
        roads.add(link.from)
        roads.add(link.to)
      }
    }
  }
  return roads
}

// --- View --------------------------------------------------------------------------

let dpr = 1
function resize() {
  dpr = window.devicePixelRatio || 1
  const { width, height } = canvas.getBoundingClientRect()
  canvas.width = Math.max(1, Math.round(width * dpr))
  canvas.height = Math.max(1, Math.round(height * dpr))
  render()
}

const size = () => ({ w: canvas.width / dpr, h: canvas.height / dpr })

function toScreen([x, y]) {
  const { w, h } = size()
  const { cx, cy, scale } = state.view
  return [(x - cx) * scale + w / 2, h / 2 - (y - cy) * scale]
}

function toWorld([sx, sy]) {
  const { w, h } = size()
  const { cx, cy, scale } = state.view
  return [(sx - w / 2) / scale + cx, cy - (sy - h / 2) / scale]
}

function fit() {
  const points = []
  for (const road of state.doc.roads) points.push(...road.points)
  const b = state.built?.scene?.bounds
  if (b) points.push([b.min_x, b.min_y], [b.max_x, b.max_y])
  if (!points.length) {
    state.view = { cx: 0, cy: 0, scale: 3 }
    render()
    return
  }
  const xs = points.map((p) => p[0])
  const ys = points.map((p) => p[1])
  const [minX, maxX, minY, maxY] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)]
  const { w, h } = size()
  const scale = Math.min((w - 80) / Math.max(maxX - minX, 10), (h - 80) / Math.max(maxY - minY, 10))
  state.view = { cx: (minX + maxX) / 2, cy: (minY + maxY) / 2, scale: Math.max(0.05, Math.min(scale, 60)) }
  render()
}

// --- Drawing -----------------------------------------------------------------------

function render() {
  if (!state.doc) return
  const { w, h } = size()
  const { cx, cy, scale } = state.view
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = '#24301f'
  ctx.fillRect(0, 0, w, h)

  // World space: metres, y up. A pixel is `px` metres.
  ctx.setTransform(dpr * scale, 0, 0, -dpr * scale, dpr * (w / 2 - cx * scale), dpr * (h / 2 + cy * scale))
  const px = 1 / scale
  const scene = state.built?.scene ?? {}
  const selectedRoad = state.selection?.kind === 'road' ? state.selection.id : null
  const selectedJunction = state.selection?.kind === 'junction' ? state.selection.id : null
  const broken = problemRoads()

  drawGrid(px)

  for (const area of scene.areas ?? []) {
    polygon(area.outline)
    ctx.fillStyle = area.category === 'building' ? '#9c8f80' : '#5b5040'
    ctx.fill()
  }
  for (const junction of scene.junctions ?? []) {
    if (junction.outline.length < 3) continue
    polygon(junction.outline)
    ctx.fillStyle = '#4a4f57'
    ctx.fill()
    if (junctionKey(junction.id) === selectedJunction) {
      ctx.strokeStyle = '#4ea1ff'
      ctx.lineWidth = 2 * px
      ctx.stroke()
    }
  }

  const lanes = [...(scene.lanes ?? [])].sort((a, b) => a.connector - b.connector)
  for (const lane of lanes) {
    polygon(lane.outline)
    ctx.fillStyle = LANE_FILL[lane.lane_type] ?? '#8c8a86'
    ctx.fill()
    const key = roadKey(lane.road)
    if (key === selectedRoad) {
      ctx.fillStyle = state.lane === lane.index ? '#4ea1ff66' : '#4ea1ff26'
      ctx.fill()
    }
    if (broken.has(key)) {
      ctx.strokeStyle = '#f85149'
      ctx.lineWidth = 1.5 * px
      ctx.stroke()
    }
  }

  for (const line of scene.lines ?? []) drawMarking(line, px)

  if (scale > 1.5) {
    ctx.fillStyle = '#ffffff70'
    for (const lane of scene.lanes ?? []) {
      if (!lane.drivable) continue
      const arrow = arrowOn(lane.centerline, Math.min(Math.max(lane.width, 1), 4))
      if (!arrow) continue
      polygon(arrow)
      ctx.fill()
    }
  }

  for (const object of scene.objects ?? []) {
    ctx.strokeStyle = ctx.fillStyle = object.kind === 'traffic_light' ? '#ff5a4f' : '#ffffff'
    if (object.points.length === 1) {
      ctx.beginPath()
      ctx.arc(object.points[0][0], object.points[0][1], 4 * px, 0, Math.PI * 2)
      ctx.fill()
    } else {
      path(object.points, object.closed)
      ctx.lineWidth = Math.max(0.4, px)
      object.closed ? ctx.fill() : ctx.stroke()
    }
  }

  // The document on top: reference lines, joints and handles.
  for (const road of state.doc.roads) {
    const selected = road.id === selectedRoad
    const irRoad = scene.roads?.find((r) => r.id === `road/${road.id}`)
    const line = irRoad?.reference_line ?? road.points
    path(line, false)
    ctx.setLineDash(selected ? [] : [4 * px, 3 * px])
    ctx.strokeStyle = selected ? '#4ea1ff' : '#5fb4ff99'
    ctx.lineWidth = (selected ? 2 : 1) * px
    ctx.stroke()
    ctx.setLineDash([])
  }

  const selectedLink = state.selection?.kind === 'link' ? state.selection.index : null
  state.doc.links.forEach((link, index) => {
    const a = findRoad(link.from)
    const b = findRoad(link.to)
    if (!a || !b) return
    const pa = endPoint(a, link.from_end)
    const pb = endPoint(b, link.to_end)
    const highlighted = index === selectedLink || (selectedJunction && link.junction === selectedJunction)
    if (link.junction) {
      ctx.beginPath()
      ctx.moveTo(pa[0], pa[1])
      ctx.lineTo(pb[0], pb[1])
      ctx.setLineDash([2 * px, 4 * px])
      ctx.strokeStyle = highlighted ? '#4ea1ff' : '#ffffff40'
      ctx.lineWidth = (highlighted ? 2 : 1) * px
      ctx.stroke()
      ctx.setLineDash([])
    } else {
      ctx.beginPath()
      ctx.arc(pa[0], pa[1], (highlighted ? 6 : 4) * px, 0, Math.PI * 2)
      ctx.fillStyle = highlighted ? '#4ea1ff' : '#3fb950'
      ctx.fill()
    }
  })

  // Back to screen space for handles, which keep their size at any zoom.
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  for (const road of state.doc.roads) {
    const selected = road.id === selectedRoad
    road.points.forEach((point, i) => {
      const isEnd = i === 0 || i === road.points.length - 1
      if (!selected && !(isEnd && (state.tool !== 'select' || scale > 0.8))) return
      const [sx, sy] = toScreen(point)
      ctx.beginPath()
      if (selected) {
        ctx.rect(sx - 4.5, sy - 4.5, 9, 9)
        ctx.fillStyle = i === state.point ? '#4ea1ff' : '#ffffff'
      } else {
        ctx.arc(sx, sy, 3.5, 0, Math.PI * 2)
        ctx.fillStyle = '#c9d1d9'
      }
      ctx.fill()
      ctx.strokeStyle = '#0d1117'
      ctx.lineWidth = 1.5
      ctx.stroke()
    })
  }

  if (state.linkFrom) {
    const road = findRoad(state.linkFrom.road)
    if (road) {
      const [sx, sy] = toScreen(endPoint(road, state.linkFrom.end))
      ctx.beginPath()
      ctx.arc(sx, sy, 8, 0, Math.PI * 2)
      ctx.strokeStyle = '#4ea1ff'
      ctx.lineWidth = 2
      ctx.stroke()
      if (state.cursor) {
        const [tx, ty] = toScreen(state.cursor)
        ctx.beginPath()
        ctx.moveTo(sx, sy)
        ctx.lineTo(tx, ty)
        ctx.setLineDash([5, 4])
        ctx.stroke()
        ctx.setLineDash([])
      }
    }
  }

  if (state.drawing) {
    const points = [...state.drawing.points]
    if (state.cursor) points.push(snapped(state.cursor).point)
    ctx.beginPath()
    points.forEach((p, i) => {
      const [sx, sy] = toScreen(p)
      i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy)
    })
    ctx.strokeStyle = '#ffd33d'
    ctx.lineWidth = 2
    ctx.stroke()
    for (const p of state.drawing.points) {
      const [sx, sy] = toScreen(p)
      ctx.fillStyle = '#ffd33d'
      ctx.fillRect(sx - 3.5, sy - 3.5, 7, 7)
    }
  }

  const snap = state.cursor && (state.tool !== 'select' || state.drag) ? snapped(state.cursor, state.drag?.road) : null
  if (snap?.end) {
    const [sx, sy] = toScreen(snap.point)
    ctx.beginPath()
    ctx.arc(sx, sy, 9, 0, Math.PI * 2)
    ctx.strokeStyle = '#ffd33d'
    ctx.lineWidth = 2
    ctx.stroke()
  }
}

function drawGrid(px) {
  const { w, h } = size()
  const [x0, y1] = toWorld([0, 0])
  const [x1, y0] = toWorld([w, h])
  let step = 10
  while (step * state.view.scale < 40) step *= 5
  while (step * state.view.scale > 200) step /= 5
  ctx.beginPath()
  for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) {
    ctx.moveTo(x, y0)
    ctx.lineTo(x, y1)
  }
  for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) {
    ctx.moveTo(x0, y)
    ctx.lineTo(x1, y)
  }
  ctx.strokeStyle = '#ffffff0d'
  ctx.lineWidth = px
  ctx.stroke()
}

function drawMarking(line, px) {
  const color = line.marking === 'curb' ? '#d8d4cc' : line.color === 'yellow' ? '#f2c230' : '#f4f4f2'
  const width = Math.max(line.marking === 'curb' ? 0.25 : 0.15, px)
  path(line.points, false)
  ctx.strokeStyle = color
  if (line.marking.includes(' ')) {
    // Two lines: a wide stroke with the road painted back down its middle.
    ctx.setLineDash(line.marking.startsWith('broken') ? [3, 6] : [])
    ctx.lineWidth = width * 3
    ctx.stroke()
    ctx.setLineDash(line.marking.endsWith('broken') ? [3, 6] : [])
    ctx.strokeStyle = LANE_FILL.driving
    ctx.lineWidth = width
    ctx.stroke()
  } else {
    ctx.setLineDash(line.marking === 'broken' ? [3, 6] : [])
    ctx.lineWidth = width
    ctx.stroke()
  }
  ctx.setLineDash([])
}

function polygon(points) {
  path(points, true)
}

function path(points, closed) {
  ctx.beginPath()
  points.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)))
  if (closed) ctx.closePath()
}

function arrowOn(line, size) {
  let total = 0
  for (let i = 1; i < line.length; i++) total += dist(line[i - 1], line[i])
  if (total < size * 2) return null
  let remaining = total / 2
  for (let i = 1; i < line.length; i++) {
    const [a, b] = [line[i - 1], line[i]]
    const length = dist(a, b)
    if (length <= 0) continue
    if (remaining <= length) {
      const [dx, dy] = [(b[0] - a[0]) / length, (b[1] - a[1]) / length]
      const t = remaining / length
      const tip = [a[0] + (b[0] - a[0]) * t + (dx * size) / 2, a[1] + (b[1] - a[1]) * t + (dy * size) / 2]
      const base = [tip[0] - dx * size, tip[1] - dy * size]
      const half = size * 0.35
      return [tip, [base[0] - dy * half, base[1] + dx * half], [base[0] + dy * half, base[1] - dx * half]]
    }
    remaining -= length
  }
  return null
}

const dist = (a, b) => Math.hypot(b[0] - a[0], b[1] - a[1])

// --- Hit testing -------------------------------------------------------------------

function screenDistance(world, screen) {
  const [sx, sy] = toScreen(world)
  return Math.hypot(sx - screen[0], sy - screen[1])
}

/// The control point of the selected road under the pointer.
function handleAt(screen) {
  if (state.selection?.kind !== 'road') return null
  const road = findRoad(state.selection.id)
  if (!road) return null
  let best = null
  road.points.forEach((point, index) => {
    const d = screenDistance(point, screen)
    if (d <= HANDLE_PX && (!best || d < best.d)) best = { road: road.id, index, d }
  })
  return best
}

/// The road end nearest the pointer, within reach; `except` is a road to ignore.
function endAt(screen, except = null, reach = SNAP_PX) {
  let best = null
  for (const road of state.doc.roads) {
    if (road.id === except) continue
    for (const end of ['start', 'end']) {
      const d = screenDistance(endPoint(road, end), screen)
      if (d <= reach && (!best || d < best.d)) best = { road: road.id, end, d }
    }
  }
  return best
}

/// Where a point lands once snapped to a road end.
function snapped(world, except = null) {
  const end = endAt(toScreen(world), except)
  if (!end) return { point: [world[0], world[1], 0], end: null }
  const road = findRoad(end.road)
  return { point: [...endPoint(road, end.end)], end }
}

function inside(ring, [x, y]) {
  let result = false
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const [xi, yi] = ring[i]
    const [xj, yj] = ring[j]
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) result = !result
  }
  return result
}

/// The lane under a world point, a connector winning over the road it crosses.
function laneAt(world) {
  let found = null
  for (const lane of state.built?.scene?.lanes ?? []) {
    if (inside(lane.outline, world) && (!found || (lane.connector && !found.connector))) found = lane
  }
  return found
}

/// What a click on a lane selects: its road, or the junction a connector is in.
function selectionForLane(lane) {
  if (!lane) return null
  if (lane.connector) {
    const road = state.built.scene.roads.find((r) => r.id === lane.road)
    if (road?.junction) return { kind: 'junction', id: junctionKey(road.junction) }
  }
  return { kind: 'road', id: roadKey(lane.road) }
}

/// The segment of a road's reference line nearest a world point, for inserting.
function nearestSegment(road, world) {
  let best = null
  for (let i = 1; i < road.points.length; i++) {
    const [a, b] = [road.points[i - 1], road.points[i]]
    const [dx, dy] = [b[0] - a[0], b[1] - a[1]]
    const t = Math.max(0, Math.min(1, ((world[0] - a[0]) * dx + (world[1] - a[1]) * dy) / (dx * dx + dy * dy || 1)))
    const p = [a[0] + t * dx, a[1] + t * dy, a[2] + t * (b[2] - a[2])]
    const d = dist(p, world)
    if (!best || d < best.d) best = { index: i, point: p, d }
  }
  return best
}

// --- Pointer -----------------------------------------------------------------------

const pointer = (event) => {
  const rect = canvas.getBoundingClientRect()
  return [event.clientX - rect.left, event.clientY - rect.top]
}

let spaceDown = false

canvas.addEventListener('pointerdown', (event) => {
  if (!state.doc) return
  canvas.focus()
  const screen = pointer(event)
  const world = toWorld(screen)
  if (event.button === 1 || event.button === 2 || spaceDown) {
    startPan(event, screen)
    return
  }
  if (state.tool === 'road') return addDrawingPoint(world)
  if (state.tool === 'link') return pickLinkEnd(screen)

  const handle = handleAt(screen)
  if (handle) {
    state.point = handle.index
    state.drag = {
      kind: 'point',
      road: handle.road,
      index: handle.index,
      snapshot: JSON.stringify(state.doc),
      moved: false,
    }
    canvas.setPointerCapture(event.pointerId)
    renderPanels()
    render()
    return
  }
  const end = endAt(screen, null, HANDLE_PX)
  const lane = laneAt(world)
  const selection = end ? { kind: 'road', id: end.road } : selectionForLane(lane)
  if (selection) {
    const same = JSON.stringify(selection) === JSON.stringify(state.selection)
    state.selection = selection
    state.point = null
    if (lane && selection.kind === 'road' && roadKey(lane.road) === selection.id) state.lane = lane.index
    else if (!same) state.lane = null
    renderPanels()
    render()
    return
  }
  state.selection = null
  state.point = null
  renderPanels()
  startPan(event, screen)
})

function startPan(event, screen) {
  state.drag = { kind: 'pan', from: screen, view: { ...state.view } }
  canvas.setPointerCapture(event.pointerId)
}

canvas.addEventListener('pointermove', (event) => {
  if (!state.doc) return
  const screen = pointer(event)
  const world = toWorld(screen)
  state.cursor = world
  $('cursor').textContent = `x ${world[0].toFixed(2)}  y ${world[1].toFixed(2)}  ·  ${state.view.scale.toFixed(2)} px/m`
  const drag = state.drag
  if (drag?.kind === 'pan') {
    state.view.cx = drag.view.cx - (screen[0] - drag.from[0]) / state.view.scale
    state.view.cy = drag.view.cy + (screen[1] - drag.from[1]) / state.view.scale
    render()
    return
  }
  if (drag?.kind === 'point') {
    movePoint(drag, world, event.shiftKey)
    return
  }
  if (state.tool !== 'select') render()
})

canvas.addEventListener('pointerup', (event) => {
  const drag = state.drag
  state.drag = null
  if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId)
  if (drag?.kind === 'point' && drag.moved) {
    checkpoint(drag.snapshot)
    changed()
  }
})

canvas.addEventListener('dblclick', (event) => {
  if (!state.doc) return
  const screen = pointer(event)
  const world = toWorld(screen)
  if (state.tool === 'road') {
    finishDrawing()
    return
  }
  if (state.tool !== 'select' || state.selection?.kind !== 'road') return
  const road = findRoad(state.selection.id)
  const segment = road && nearestSegment(road, world)
  if (!segment || segment.d * state.view.scale > 25) return
  commit((doc) => {
    doc.roads.find((r) => r.id === road.id).points.splice(segment.index, 0, segment.point)
  })
  state.point = segment.index
  renderPanels()
  render()
})

canvas.addEventListener('contextmenu', (event) => event.preventDefault())

canvas.addEventListener(
  'wheel',
  (event) => {
    event.preventDefault()
    const screen = pointer(event)
    const before = toWorld(screen)
    const factor = Math.exp(-event.deltaY * 0.0015)
    state.view.scale = Math.max(0.02, Math.min(200, state.view.scale * factor))
    const after = toWorld(screen)
    state.view.cx += before[0] - after[0]
    state.view.cy += before[1] - after[1]
    render()
  },
  { passive: false },
)

/// Moves a control point; an end that is joined directly to another road takes the
/// other road's end with it, so the joint holds. Shift turns snapping off.
function movePoint(drag, world, free) {
  const road = findRoad(drag.road)
  if (!road) return
  const isEnd = drag.index === 0 || drag.index === road.points.length - 1
  const old = road.points[drag.index]
  let target = [world[0], world[1], old[2]]
  if (isEnd && !free) {
    const snap = snapped(world, road.id)
    if (snap.end) target = snap.point
  }
  road.points[drag.index] = target
  if (isEnd) {
    const end = drag.index === 0 ? 'start' : 'end'
    for (const link of state.doc.links) {
      if (link.junction) continue
      const partner =
        link.from === road.id && link.from_end === end
          ? { road: link.to, end: link.to_end }
          : link.to === road.id && link.to_end === end
            ? { road: link.from, end: link.from_end }
            : null
      const other = partner && findRoad(partner.road)
      if (other && other !== road) other.points[endIndex(other, partner.end)] = [...target]
    }
  }
  drag.moved = true
  rebuildSoon()
  renderPanelsSoon()
}

// --- Tools -------------------------------------------------------------------------

function setTool(tool) {
  state.tool = tool
  state.drawing = tool === 'road' ? { points: [], startSnap: null, endSnap: null } : null
  state.linkFrom = null
  for (const button of document.querySelectorAll('.tool')) {
    button.classList.toggle('active', button.dataset.tool === tool)
  }
  $('hint').textContent = {
    select: '',
    road: 'Click to place points · double-click or Enter to finish · Esc to cancel · ends snap to other roads',
    link: 'Click a road end, then another · empty “via junction” joins them directly',
  }[tool]
  render()
}

function addDrawingPoint(world) {
  const snap = snapped(world)
  const drawing = state.drawing
  const last = drawing.points[drawing.points.length - 1]
  if (last && dist(last, snap.point) < 0.05) return // the second click of a double-click
  if (!drawing.points.length) drawing.startSnap = snap.end
  drawing.endSnap = snap.end
  drawing.points.push(snap.point)
  render()
}

function finishDrawing() {
  const drawing = state.drawing
  if (!drawing || drawing.points.length < 2) {
    toast('A road needs two points at least')
    return
  }
  const id = uniqueId('road', new Set(state.doc.roads.map((r) => r.id)))
  const { points, startSnap, endSnap } = drawing
  commit(
    (doc) => {
      doc.roads.push({ id, points, shape: 'smooth', lanes: defaultLanes(), road_type: 'town' })
      if (startSnap) doc.links.push({ from: startSnap.road, from_end: startSnap.end, to: id, to_end: 'start' })
      if (endSnap && endSnap !== startSnap) {
        doc.links.push({ from: id, from_end: 'end', to: endSnap.road, to_end: endSnap.end })
      }
    },
    { select: { kind: 'road', id } },
  )
  setTool('select')
}

function pickLinkEnd(screen) {
  const end = endAt(screen)
  if (!end) {
    state.linkFrom = null
    render()
    return
  }
  if (!state.linkFrom) {
    state.linkFrom = { road: end.road, end: end.end }
    render()
    return
  }
  const from = state.linkFrom
  state.linkFrom = null
  if (from.road === end.road && from.end === end.end) return render()
  const junction = $('junction-name').value.trim() || undefined
  const exists = state.doc.links.some(
    (l) =>
      ((l.from === from.road && l.from_end === from.end && l.to === end.road && l.to_end === end.end) ||
        (l.to === from.road && l.to_end === from.end && l.from === end.road && l.from_end === end.end)) &&
      (l.junction ?? undefined) === junction,
  )
  if (exists) {
    toast('Those two ends are joined already')
    return render()
  }
  commit(
    (doc) => {
      if (!junction) {
        // A direct joint is one point: bring the second end onto the first.
        const a = doc.roads.find((r) => r.id === from.road)
        const b = doc.roads.find((r) => r.id === end.road)
        b.points[endIndex(b, end.end)] = [...endPoint(a, from.end)]
      }
      doc.links.push({ from: from.road, from_end: from.end, to: end.road, to_end: end.end, junction })
    },
    { select: { kind: 'link', index: state.doc.links.length } },
  )
}

function deleteSelection() {
  const s = state.selection
  if (!s) return
  if (s.kind === 'road' && state.point != null) {
    const road = findRoad(s.id)
    if (road.points.length <= 2) return toast('A road keeps two points at least')
    const index = state.point
    state.point = null
    return commit((doc) => doc.roads.find((r) => r.id === s.id).points.splice(index, 1))
  }
  if (s.kind === 'road') {
    return commit(
      (doc) => {
        doc.roads = doc.roads.filter((r) => r.id !== s.id)
        doc.links = doc.links.filter((l) => l.from !== s.id && l.to !== s.id)
      },
      { select: null },
    )
  }
  if (s.kind === 'link') return commit((doc) => doc.links.splice(s.index, 1), { select: null })
  if (s.kind === 'junction') {
    return commit((doc) => (doc.links = doc.links.filter((l) => l.junction !== s.id)), { select: null })
  }
}

// --- Keyboard ----------------------------------------------------------------------

window.addEventListener('keydown', (event) => {
  if (!state.doc) return
  const typing = /^(INPUT|SELECT|TEXTAREA)$/.test(document.activeElement?.tagName)
  const mod = event.ctrlKey || event.metaKey
  if (mod && event.key.toLowerCase() === 'z') {
    if (typing) return
    event.preventDefault()
    event.shiftKey ? redo() : undo()
    return
  }
  if (mod && event.key.toLowerCase() === 'y') {
    if (typing) return
    event.preventDefault()
    redo()
    return
  }
  if (typing) return
  if (event.key === ' ') spaceDown = true
  if (event.key === 'Escape') setTool('select')
  if (event.key === 'Enter' && state.tool === 'road') finishDrawing()
  if (event.key === 'Delete' || event.key === 'Backspace') {
    event.preventDefault()
    deleteSelection()
  }
  const tools = { v: 'select', r: 'road', l: 'link' }
  if (!mod && tools[event.key.toLowerCase()]) setTool(tools[event.key.toLowerCase()])
  if (!mod && event.key.toLowerCase() === 'f') fit()
})
window.addEventListener('keyup', (event) => {
  if (event.key === ' ') spaceDown = false
})

// --- Panels ------------------------------------------------------------------------

let panelFrame = 0
function renderPanelsSoon() {
  if (panelFrame) return
  panelFrame = requestAnimationFrame(() => {
    panelFrame = 0
    renderPanels()
  })
}

function h(tag, attributes = {}, ...children) {
  const element = document.createElement(tag)
  for (const [key, value] of Object.entries(attributes)) {
    if (value == null || value === false) continue
    if (key.startsWith('on')) element.addEventListener(key.slice(2), value)
    else if (key === 'class') element.className = value
    else if (key in element && key !== 'list') element[key] = value
    else element.setAttribute(key, value === true ? '' : value)
  }
  for (const child of children.flat()) {
    if (child == null || child === false) continue
    element.append(child instanceof Node ? child : document.createTextNode(String(child)))
  }
  return element
}

function select(options, value, onchange) {
  return h(
    'select',
    { onchange: (event) => onchange(event.target.value) },
    options.map((option) => {
      const [v, label] = Array.isArray(option) ? option : [option, option]
      return h('option', { value: v, selected: v === value }, label)
    }),
  )
}

function numberInput(value, onchange, attributes = {}) {
  return h('input', {
    type: 'number',
    value: value ?? '',
    step: 'any',
    ...attributes,
    onchange: (event) => onchange(event.target.value === '' ? null : Number(event.target.value)),
  })
}

function field(label, control) {
  return h('label', { class: 'field' }, h('span', { class: 'muted' }, label), control)
}

function renderPanels() {
  if (!state.doc) return
  renderInspector()
  renderLinks()
  renderProblems()
  renderMapTab()
}

function renderInspector() {
  const tab = $('tab-inspect')
  tab.replaceChildren()
  const s = state.selection
  if (!s) {
    tab.append(
      h('p', { class: 'muted' }, 'Click a road to edit it, or pick the Road tool (R) to draw one.'),
      h('p', { class: 'muted' }, 'Drag the square handles to reshape the selected road; double-click its line to add a point; Delete removes the selected point or road.'),
    )
    return
  }
  if (s.kind === 'road') return renderRoadInspector(tab, findRoad(s.id))
  if (s.kind === 'junction') return renderJunctionInspector(tab, s.id)
  if (s.kind === 'link') return renderLinkInspector(tab, s.index)
}

function editRoad(id, change) {
  commit((doc) => change(doc.roads.find((r) => r.id === id), doc))
}

function renderRoadInspector(tab, road) {
  if (!road) return
  const id = road.id
  tab.append(
    h('h3', {}, `Road ${id}`),
    field(
      'id',
      h('input', {
        value: id,
        onchange: (event) => renameRoad(id, event.target.value.trim()),
      }),
    ),
    field('shape', select([['smooth', 'smooth curve'], ['polyline', 'straight segments']], road.shape ?? 'smooth', (v) => editRoad(id, (r) => (r.shape = v)))),
    field('road type', select(ROAD_TYPES, road.road_type ?? 'town', (v) => editRoad(id, (r) => (r.road_type = v)))),
    field(
      'speed limit km/h',
      numberInput(road.speed_limit, (v) => editRoad(id, (r) => (v == null ? delete r.speed_limit : (r.speed_limit = v))), { min: 1, placeholder: 'none' }),
    ),
  )

  tab.append(h('h4', {}, `Lanes (${road.lanes.length})`), h('p', { class: 'muted' }, 'Listed outwards from the reference line on each side.'))
  road.lanes.forEach((lane, index) => tab.append(laneCard(road, lane, index)))
  tab.append(
    h(
      'div',
      { class: 'row' },
      h('button', { onclick: () => editRoad(id, (r) => r.lanes.push(lane(3.5, 'forward'))) }, '+ forward lane'),
      h('button', { onclick: () => editRoad(id, (r) => r.lanes.push(lane(3.5, 'backward'))) }, '+ backward lane'),
      h('button', { onclick: () => editRoad(id, (r) => r.lanes.push(lane(2, 'forward', 'sidewalk'), lane(2, 'backward', 'sidewalk'))) }, '+ pavements'),
    ),
  )

  tab.append(h('h4', {}, 'Points (x, y, z metres)'))
  const table = h('table', { class: 'points' })
  road.points.forEach((point, index) => {
    const coordinate = (axis) =>
      numberInput(point[axis].toFixed(3), (v) => v != null && editRoad(id, (r) => (r.points[index][axis] = v)))
    table.append(
      h(
        'tr',
        { class: index === state.point ? 'selected' : '' },
        h('td', { class: 'muted' }, index),
        h('td', {}, coordinate(0)),
        h('td', {}, coordinate(1)),
        h('td', {}, coordinate(2)),
        h(
          'td',
          {},
          h('button', {
            title: 'Remove this point',
            disabled: road.points.length <= 2,
            onclick: () => editRoad(id, (r) => r.points.splice(index, 1)),
          }, '×'),
        ),
      ),
    )
  })
  tab.append(table)

  const links = state.doc.links.map((link, index) => ({ link, index })).filter(({ link }) => link.from === id || link.to === id)
  tab.append(h('h4', {}, `Links (${links.length})`), linkList(links))
  tab.append(h('h4', {}, ''), h('button', { onclick: deleteSelection }, 'Delete road'))
}

function laneCard(road, laneDoc, index) {
  const id = road.id
  const edit = (change) => editRoad(id, (r) => change(r.lanes[index]))
  const markingControls = (which) =>
    h(
      'span',
      { class: 'row' },
      select(MARKINGS, laneDoc[which]?.kind ?? 'solid', (v) => edit((l) => (l[which] = { ...(l[which] ?? { color: 'white' }), kind: v }))),
      select(COLORS, laneDoc[which]?.color ?? 'white', (v) => edit((l) => (l[which] = { ...(l[which] ?? { kind: 'solid' }), color: v }))),
    )
  const move = (delta) =>
    editRoad(id, (r) => {
      const [moved] = r.lanes.splice(index, 1)
      r.lanes.splice(index + delta, 0, moved)
    })
  return h(
    'div',
    {
      class: `lane${state.lane === index ? ' selected' : ''}`,
      onclick: (event) => {
        if (state.lane === index) return
        state.lane = index
        render()
        for (const card of document.querySelectorAll('.lane')) card.classList.remove('selected')
        event.currentTarget.classList.add('selected')
      },
    },
    h(
      'div',
      { class: 'head' },
      h('strong', {}, `lane ${index}`),
      h(
        'span',
        { class: 'row' },
        h('button', { title: 'Move inwards', disabled: index === 0, onclick: () => move(-1) }, '↑'),
        h('button', { title: 'Move outwards', disabled: index === road.lanes.length - 1, onclick: () => move(1) }, '↓'),
        h('button', { title: 'Remove lane', disabled: road.lanes.length <= 1, onclick: () => editRoad(id, (r) => r.lanes.splice(index, 1)) }, '×'),
      ),
    ),
    h(
      'div',
      { class: 'grid' },
      h('span', { class: 'muted' }, 'width'),
      numberInput(laneDoc.width, (v) => v != null && edit((l) => (l.width = v)), { min: 0.1, step: 0.1 }),
      h('span', { class: 'muted' }, 'type'),
      select(LANE_TYPES, laneDoc.lane_type ?? 'driving', (v) => edit((l) => (l.lane_type = v))),
      h('span', { class: 'muted' }, 'travel'),
      select(['forward', 'backward'], laneDoc.direction ?? 'forward', (v) => edit((l) => (l.direction = v))),
      h('span', { class: 'muted' }, 'side'),
      select([['', 'by handedness'], 'left', 'right'], laneDoc.side ?? '', (v) => edit((l) => (v ? (l.side = v) : delete l.side))),
    ),
    h('div', { class: 'row' }, h('span', { class: 'muted' }, 'left edge'), markingControls('left_marking')),
    h('div', { class: 'row' }, h('span', { class: 'muted' }, 'right edge'), markingControls('right_marking')),
  )
}

function renameRoad(from, to) {
  if (!to || to === from) return renderPanels()
  if (findRoad(to)) {
    toast(`There is a road called ${to} already`)
    return renderPanels()
  }
  if (!/^[A-Za-z0-9_.-]+$/.test(to)) {
    toast('A road id is letters, digits, “_”, “.” and “-”')
    return renderPanels()
  }
  commit(
    (doc) => {
      doc.roads.find((r) => r.id === from).id = to
      for (const link of doc.links) {
        if (link.from === from) link.from = to
        if (link.to === from) link.to = to
      }
    },
    { select: { kind: 'road', id: to } },
  )
}

function renderJunctionInspector(tab, name) {
  const links = state.doc.links.map((link, index) => ({ link, index })).filter(({ link }) => link.junction === name)
  const ir = state.built?.scene?.junctions?.find((j) => junctionKey(j.id) === name)
  tab.append(
    h('h3', {}, `Junction ${name}`),
    field(
      'name',
      h('input', {
        value: name,
        onchange: (event) => {
          const to = event.target.value.trim()
          if (!to || to === name) return
          commit(
            (doc) => doc.links.forEach((l) => l.junction === name && (l.junction = to)),
            { select: { kind: 'junction', id: to } },
          )
        },
      }),
    ),
    h('p', { class: 'muted' }, `${ir?.connecting_roads?.length ?? 0} connector roads generated through it.`),
    h('h4', {}, `Movements (${links.length})`),
    linkList(links),
    h('h4', {}, ''),
    h('button', { onclick: deleteSelection }, 'Delete junction and its links'),
  )
}

function renderLinkInspector(tab, index) {
  const link = state.doc.links[index]
  if (!link) return
  const roads = state.doc.roads.map((r) => r.id)
  const edit = (change) => commit((doc) => change(doc.links[index]))
  tab.append(
    h('h3', {}, `Link ${index}`),
    field('from road', select(roads, link.from, (v) => edit((l) => (l.from = v)))),
    field('from end', select(['start', 'end'], link.from_end, (v) => edit((l) => (l.from_end = v)))),
    field('to road', select(roads, link.to, (v) => edit((l) => (l.to = v)))),
    field('to end', select(['start', 'end'], link.to_end, (v) => edit((l) => (l.to_end = v)))),
    field(
      'via junction',
      h('input', {
        value: link.junction ?? '',
        placeholder: 'direct',
        onchange: (event) => edit((l) => (event.target.value.trim() ? (l.junction = event.target.value.trim()) : delete l.junction)),
      }),
    ),
    h('p', { class: 'muted' }, link.junction ? 'A connector is generated for every movement between the two ends.' : 'The two ends must coincide; smooth roads are bent to meet without a corner.'),
    h('button', { onclick: deleteSelection }, 'Delete link'),
  )
}

function linkLabel(link) {
  const via = link.junction ? ` via ${link.junction}` : ''
  return `${link.from}.${link.from_end} → ${link.to}.${link.to_end}${via}`
}

function linkList(entries) {
  if (!entries.length) return h('p', { class: 'muted' }, 'None.')
  return h(
    'ul',
    { class: 'list' },
    entries.map(({ link, index }) =>
      h(
        'li',
        { onclick: () => selectAndShow({ kind: 'link', index }) },
        h('span', { class: 'what' }, linkLabel(link)),
        h(
          'button',
          {
            title: 'Delete link',
            onclick: (event) => {
              event.stopPropagation()
              commit((doc) => doc.links.splice(index, 1))
            },
          },
          '×',
        ),
      ),
    ),
  )
}

function renderLinks() {
  const tab = $('tab-links')
  tab.replaceChildren(
    h('p', { class: 'muted' }, 'Pick the Link tool (L) and click two road ends to join them. Name a junction in the toolbar to join through it.'),
    linkList(state.doc.links.map((link, index) => ({ link, index }))),
  )
}

function renderProblems() {
  const problems = state.built?.problems ?? []
  $('problem-count').textContent = problems.length || ''
  const tab = $('tab-problems')
  if (!problems.length) {
    tab.replaceChildren(h('p', { class: 'muted' }, 'No problems: the map validates and every format can be written.'))
    return
  }
  tab.replaceChildren(
    h(
      'ul',
      { class: 'list' },
      problems.map((problem) =>
        h(
          'li',
          {
            class: problem.severity,
            onclick: () => {
              if (problem.road) selectAndShow({ kind: 'road', id: problem.road })
              else if (problem.link != null) selectAndShow({ kind: 'link', index: problem.link })
            },
          },
          h('span', { class: 'what' }, problem.road ? `${problem.road}: ` : problem.link != null ? `link ${problem.link}: ` : '', problem.message),
        ),
      ),
    ),
  )
}

let importNotes = ''
function renderMapTab() {
  const doc = state.doc
  const origin = doc.origin ?? [null, null, null]
  const setOrigin = (axis) => (value) =>
    commit((d) => {
      const next = [...(d.origin ?? [0, 0, 0])]
      next[axis] = value ?? 0
      d.origin = next
    })
  const stats = state.built?.stats ?? {}
  $('tab-map').replaceChildren(
    h('h3', {}, 'Map'),
    field('name', h('input', { value: doc.name ?? '', onchange: (e) => commit((d) => (d.name = e.target.value)) })),
    field('traffic', select([['right', 'right-hand'], ['left', 'left-hand']], doc.handedness ?? 'right', (v) => commit((d) => (d.handedness = v)))),
    field('origin latitude', numberInput(origin[0], setOrigin(0), { placeholder: 'default' })),
    field('origin longitude', numberInput(origin[1], setOrigin(1), { placeholder: 'default' })),
    field('origin altitude', numberInput(origin[2], setOrigin(2), { placeholder: '0' })),
    h('h4', {}, 'Generated'),
    h(
      'p',
      { class: 'muted' },
      `${stats.roads ?? 0} roads, ${stats.connectors ?? 0} connectors, ${stats.lanes ?? 0} lanes, ` +
        `${stats.junctions ?? 0} junctions, ${stats.connections ?? 0} lane connections.`,
    ),
    importNotes ? h('h4', {}, 'Last import') : null,
    importNotes ? h('p', { class: 'muted notes' }, importNotes) : null,
    h('h4', {}, 'Keys'),
    h(
      'p',
      { class: 'muted notes' },
      'V select · R draw road · L link · F fit · Esc cancel\n' +
        'Delete removes the selected point, road, link or junction\n' +
        'Shift while dragging: no snapping · Space or right-drag: pan · wheel: zoom\n' +
        'Ctrl+Z undo · Ctrl+Shift+Z redo',
    ),
  )
}

function selectAndShow(selection) {
  state.selection = selection
  state.point = null
  showTab('inspect')
  renderPanels()
  render()
}

function showTab(name) {
  for (const button of document.querySelectorAll('.tabs button')) button.classList.toggle('active', button.dataset.tab === name)
  for (const tab of document.querySelectorAll('.tab')) tab.classList.toggle('active', tab.id === `tab-${name}`)
}

function renderStatus() {
  const valid = state.built?.valid
  const badge = $('validity')
  badge.textContent = valid ? 'valid' : 'not valid'
  badge.className = `badge ${valid ? 'ok' : 'bad'}`
  const stats = state.built?.stats ?? {}
  $('stats').textContent = `${state.doc.roads.length} roads · ${state.doc.links.length} links · ${stats.lanes ?? 0} lanes generated`
  for (const option of $('format').options) {
    const format = state.formats.find((f) => f.id === option.value)
    option.disabled = Boolean(format?.needs_valid_map && !valid)
  }
}

// --- Files -------------------------------------------------------------------------

function download(name, bytes) {
  const url = URL.createObjectURL(new Blob([bytes]))
  const a = h('a', { href: url, download: name })
  document.body.append(a)
  a.click()
  a.remove()
  setTimeout(() => URL.revokeObjectURL(url), 1000)
}

async function exportSelected() {
  const format = $('format').value
  try {
    const today = new Date().toISOString().slice(0, 10)
    const result = exportDocument(JSON.stringify(state.doc), format, today)
    const files = []
    for (let i = 0; i < result.count(); i++) files.push({ name: result.name(i), bytes: result.bytes(i) })
    result.free()
    for (const file of files) {
      download(file.name, file.bytes)
      // Several downloads at once are throttled by some browsers.
      if (files.length > 1) await new Promise((resolve) => setTimeout(resolve, 150))
    }
    window.dispatchEvent(new CustomEvent('roadgen-export', { detail: files }))
    toast(`Wrote ${files.map((f) => f.name).join(', ')}`)
  } catch (error) {
    toast(String(error.message ?? error))
  }
}

async function openFile(file) {
  try {
    const bytes = new Uint8Array(await file.arrayBuffer())
    const imported = JSON.parse(importFile(file.name, bytes))
    importNotes = imported.notes.length ? `${file.name}:\n${imported.notes.join('\n')}` : `${file.name}: nothing was left out.`
    replace(imported.document)
    fit()
    toast(imported.notes.length ? `Opened ${file.name} — see Map for what was left out` : `Opened ${file.name}`)
  } catch (error) {
    toast(String(error.message ?? error))
  }
}

let toastTimer = null
function toast(message) {
  const element = $('toast')
  element.textContent = message
  element.classList.add('show')
  clearTimeout(toastTimer)
  toastTimer = setTimeout(() => element.classList.remove('show'), 3500)
}

// --- Start -------------------------------------------------------------------------

function wire() {
  for (const button of document.querySelectorAll('.tool')) button.addEventListener('click', () => setTool(button.dataset.tool))
  for (const button of document.querySelectorAll('.tabs button')) button.addEventListener('click', () => showTab(button.dataset.tab))
  $('undo').addEventListener('click', undo)
  $('redo').addEventListener('click', redo)
  $('fit').addEventListener('click', fit)
  $('new-junction').addEventListener('click', () => {
    $('junction-name').value = uniqueId('j', new Set(junctionNames()))
  })
  $('new-doc').addEventListener('click', () => {
    replace(emptyDocument())
    fit()
  })
  $('example-doc').addEventListener('click', () => {
    replace(JSON.parse(exampleDocument()))
    fit()
  })
  $('open-file').addEventListener('change', (event) => {
    const [file] = event.target.files
    if (file) openFile(file)
    event.target.value = ''
  })
  $('export').addEventListener('click', exportSelected)
  canvas.addEventListener('dragover', (event) => event.preventDefault())
  canvas.addEventListener('drop', (event) => {
    event.preventDefault()
    const [file] = event.dataTransfer.files
    if (file) openFile(file)
  })
  new ResizeObserver(resize).observe(canvas)
}

async function start() {
  wire()
  try {
    await init()
  } catch (error) {
    $('loading').textContent = `The roadgen engine did not load: ${error}`
    throw error
  }
  state.formats = JSON.parse(formats())
  $('format').replaceChildren(...state.formats.map((f) => h('option', { value: f.id }, f.label)))
  state.doc = restore() ?? JSON.parse(exampleDocument())
  rebuild()
  resize()
  fit()
  setTool('select')
  renderPanels()
  $('loading').hidden = true
  // For the smoke test, and for anyone poking at the page from the console.
  window.roadgenEditor = {
    state,
    document: () => clone(state.doc),
    load: (doc) => replace(doc),
    built: () => state.built,
    exportFormat: (format) => {
      const result = exportDocument(JSON.stringify(state.doc), format, new Date().toISOString().slice(0, 10))
      const files = []
      for (let i = 0; i < result.count(); i++) files.push({ name: result.name(i), size: result.bytes(i).length })
      result.free()
      return files
    },
    toScreen,
  }
  document.body.dataset.ready = 'true'
}

start()
