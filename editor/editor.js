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

const SURFACE_KINDS = ['rut', 'pothole', 'puddle', 'bump', 'roughness']
const BUMP_KINDS = ['bump', 'hump', 'table', 'cushion', 'heave']
const BUMP_PROFILES = [['', 'usual for its kind'], 'circular', 'parabolic', 'sinusoidal', 'flat']
const ROUGHNESS_CLASSES = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
const SURFACE_FILL = {
  puddle: ['#3b82c4', 0.75],
  pothole: ['#16181b', 0.9],
  bump: ['#e0a030', 0.8],
  rut: ['#2c3036', 0.55],
  roughness: ['#b07d5a', 0.35],
}

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
  surface: null, // index of the selected surface feature of the selected road
  structure: null, // index of the selected bridge or tunnel of the selected road
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
const isAligned = (road) => road.shape === 'alignment'
const endIndex = (road, end) => (end === 'start' || isAligned(road) ? 0 : road.points.length - 1)
const builtRoad = (id) => state.built?.scene?.roads?.find((r) => r.id === `road/${id}`)

/// Where one end of a road is. An alignment's far end is wherever its pieces take
/// it, which only the built road knows.
function endPoint(road, end) {
  if (end === 'start') return road.points[0]
  if (!isAligned(road)) return road.points[road.points.length - 1]
  return builtRoad(road.id)?.end?.point ?? road.points[0]
}

/// Puts one end of a road at `point`. An alignment's far end cannot be put
/// anywhere by itself, so the whole road moves to bring it there.
function setEndPoint(road, end, point) {
  if (!isAligned(road) || end === 'start') {
    road.points[endIndex(road, end)] = [...point]
    return
  }
  const now = endPoint(road, 'end')
  const start = road.points[0]
  road.points[0] = [start[0] + point[0] - now[0], start[1] + point[1] - now[1], start[2] + point[2] - now[2]]
}

/// The direction an alignment starts in, as a unit vector in plan.
const headingVector = (road) => [Math.cos(((road.heading ?? 0) * Math.PI) / 180), Math.sin(((road.heading ?? 0) * Math.PI) / 180)]
const HEADING_PX = 46

/// A smooth road's handle at point `i`: the one set there, or the one the curve
/// works out — along the chord between the neighbours, a third of the shorter
/// chord either side long. `[x, y, set]`.
function handleOf(road, i) {
  const set = road.handles?.[i]
  if (set) return [set[0], set[1], true]
  const p = road.points
  const [a, b] = [p[Math.max(0, i - 1)], p[Math.min(p.length - 1, i + 1)]]
  const [dx, dy] = [b[0] - a[0], b[1] - a[1]]
  const norm = Math.hypot(dx, dy) || 1
  const chords = [i > 0 ? dist(p[i - 1], p[i]) : Infinity, i < p.length - 1 ? dist(p[i], p[i + 1]) : Infinity]
  const reach = Math.min(...chords) / 3
  return [(dx / norm) * reach, (dy / norm) * reach, false]
}

/// Keeps a road's handles in step with its points when one is put in or taken out.
function spliceHandles(road, index, removed, ...added) {
  if (!road.handles?.length) return
  while (road.handles.length < index) road.handles.push(null)
  road.handles.splice(index, removed, ...added.map(() => null))
  while (road.handles.length && road.handles[road.handles.length - 1] == null) road.handles.pop()
}

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
  if (s.kind === 'land' && !state.doc.terrain?.[s.group]?.[s.index]) state.selection = null
  if (state.selection?.kind !== 'road') state.point = null
  const road = state.selection?.kind === 'road' && findRoad(state.selection.id)
  if (road && state.point != null && state.point >= road.points.length) state.point = null
  if (!road || state.surface >= (road.surface?.length ?? 0)) state.surface = null
  if (!road || state.structure >= (road.structures?.length ?? 0)) state.structure = null
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

/// Builds the document. With `land` false — while something is dragged — the
/// land is not generated again, and the land last generated stays on the map.
function rebuild(land = true) {
  const before = state.built
  try {
    state.built = JSON.parse(buildDocument(JSON.stringify(state.doc), land))
    if (land) {
      state.landCache = landCache(state.built.scene)
    } else if (state.doc.terrain && before?.scene) {
      state.built.scene.terrain = before.scene.terrain
      state.built.scene.contours = before.scene.contours
      state.built.land_notes = before.land_notes
    }
    if (!state.doc.terrain) state.landCache = null
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
    rebuild(false)
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

  drawLand(px)
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

  // A bridge's deck is wider than its road, so it goes under it, piers first.
  for (const structure of (scene.structures ?? []).filter((s) => s.kind === 'bridge')) {
    for (const support of structure.supports) {
      polygon(support)
      ctx.fillStyle = '#1f2328'
      ctx.fill()
    }
    polygon(structure.outline)
    ctx.fillStyle = '#8a96a3'
    ctx.fill()
    ctx.strokeStyle = isSelectedStructure(structure) ? '#4ea1ff' : '#d0d7de'
    ctx.lineWidth = (isSelectedStructure(structure) ? 3 : 2) * px
    ctx.stroke()
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

  // A tunnel goes over the road it holds: a shade, its portals drawn hard.
  for (const structure of (scene.structures ?? []).filter((s) => s.kind === 'tunnel')) {
    polygon(structure.outline)
    ctx.fillStyle = '#0b0d1073'
    ctx.fill()
    ctx.setLineDash([6 * px, 4 * px])
    ctx.strokeStyle = isSelectedStructure(structure) ? '#4ea1ff' : '#9aa3ad'
    ctx.lineWidth = (isSelectedStructure(structure) ? 3 : 1.5) * px
    ctx.stroke()
    ctx.setLineDash([])
    for (const portal of structure.portals) {
      path(portal, false)
      ctx.strokeStyle = '#d0d7de'
      ctx.lineWidth = Math.max(0.8, 3 * px)
      ctx.stroke()
    }
  }

  for (const surface of scene.surfaces ?? []) {
    const [fill, alpha] = SURFACE_FILL[surface.kind] ?? SURFACE_FILL.roughness
    polygon(surface.outline)
    ctx.globalAlpha = alpha
    ctx.fillStyle = fill
    ctx.fill()
    ctx.globalAlpha = 1
    for (const trough of surface.troughs) {
      path(trough, false)
      ctx.strokeStyle = '#1d2024'
      ctx.lineWidth = Math.max(0.3, px)
      ctx.lineCap = 'round'
      ctx.stroke()
      ctx.lineCap = 'butt'
    }
    if (isSelectedSurface(surface)) {
      polygon(surface.outline)
      ctx.strokeStyle = '#4ea1ff'
      ctx.lineWidth = 2 * px
      ctx.stroke()
    }
  }

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
    if (selected && isAligned(road) && irRoad) drawElementTicks(road, line, px)
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

  drawLandShapes(px)

  // Back to screen space for handles, which keep their size at any zoom.
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  const chosen = selectedRoad && findRoad(selectedRoad)
  if (chosen && isAligned(chosen)) {
    // The heading: a line out of the start, its knob to turn the road by.
    const [sx, sy] = toScreen(chosen.points[0])
    const [ux, uy] = headingVector(chosen)
    const [kx, ky] = [sx + ux * HEADING_PX, sy - uy * HEADING_PX]
    ctx.beginPath()
    ctx.moveTo(sx, sy)
    ctx.lineTo(kx, ky)
    ctx.strokeStyle = '#ffd33d'
    ctx.lineWidth = 2
    ctx.stroke()
    ctx.beginPath()
    ctx.arc(kx, ky, 5.5, 0, Math.PI * 2)
    ctx.fillStyle = '#ffd33d'
    ctx.fill()
    // Where each piece ends: a knob to drag along the road to change its length.
    const dragging = state.drag?.kind === 'length' && state.drag.road === chosen.id ? state.drag.index : null
    for (const end of elementEnds(chosen)) {
      const [ex, ey] = toScreen(end.point)
      ctx.beginPath()
      ctx.moveTo(ex, ey - 6.5)
      ctx.lineTo(ex + 6.5, ey)
      ctx.lineTo(ex, ey + 6.5)
      ctx.lineTo(ex - 6.5, ey)
      ctx.closePath()
      ctx.fillStyle = end.index === dragging ? '#ffffff' : '#ffd33d'
      ctx.fill()
      ctx.strokeStyle = '#0d1117'
      ctx.lineWidth = 1.5
      ctx.stroke()
      if (end.index === dragging) {
        const element = chosen.elements[end.index]
        const label = `#${end.index} ${element.kind === 'spiral' ? 'clothoid' : element.kind === 'line' ? 'straight' : 'arc'} ${element.length.toFixed(1)} m`
        ctx.font = '12px system-ui, sans-serif'
        const width = ctx.measureText(label).width
        ctx.fillStyle = '#000000cc'
        ctx.fillRect(ex + 10, ey - 22, width + 10, 18)
        ctx.fillStyle = '#ffffff'
        ctx.fillText(label, ex + 15, ey - 9)
      }
    }
    // The middle of each bend: a knob to drag sideways to change its radius.
    const bending = state.drag?.kind === 'radius' && state.drag.road === chosen.id ? state.drag.index : null
    for (const knob of radiusKnobs(chosen)) {
      const [rx, ry] = toScreen(knob.point)
      ctx.beginPath()
      ctx.arc(rx, ry, 6, 0, Math.PI * 2)
      ctx.fillStyle = knob.index === bending ? '#ffffff' : '#fb923c'
      ctx.fill()
      ctx.strokeStyle = '#0d1117'
      ctx.lineWidth = 1.5
      ctx.stroke()
      ctx.fillStyle = '#0d1117'
      ctx.font = 'bold 8px system-ui, sans-serif'
      ctx.fillText('R', rx - 2.6, ry + 3)
      if (knob.index === bending) {
        const element = chosen.elements[knob.index]
        const radius = element.radius ? `${element.radius > 0 ? 'left' : 'right'} R ${Math.abs(element.radius).toFixed(1)} m` : 'to straight'
        const label = `#${knob.index} ${element.kind === 'arc' ? 'arc' : 'clothoid, ends'} ${radius}`
        ctx.font = '12px system-ui, sans-serif'
        const width = ctx.measureText(label).width
        ctx.fillStyle = '#000000cc'
        ctx.fillRect(rx + 10, ry - 22, width + 10, 18)
        ctx.fillStyle = '#ffffff'
        ctx.fillText(label, rx + 15, ry - 9)
      }
    }
  } else if (chosen && chosen.shape !== 'polyline' && state.point != null && chosen.points[state.point]) {
    // The selected point's tangent handle, either side of it: solid where it was
    // set, hollow where the curve works it out.
    const [hx, hy, set] = handleOf(chosen, state.point)
    const [sx, sy] = toScreen(chosen.points[state.point])
    const p = chosen.points[state.point]
    for (const sign of [1, -1]) {
      const [kx, ky] = toScreen([p[0] + sign * hx, p[1] + sign * hy])
      ctx.beginPath()
      ctx.moveTo(sx, sy)
      ctx.lineTo(kx, ky)
      ctx.strokeStyle = '#c084fc'
      ctx.lineWidth = 1.5
      ctx.stroke()
      ctx.beginPath()
      ctx.arc(kx, ky, 5, 0, Math.PI * 2)
      ctx.fillStyle = set ? '#c084fc' : '#1d2127'
      ctx.fill()
      ctx.strokeStyle = '#c084fc'
      ctx.stroke()
    }
  }
  if (state.selection?.kind === 'land') {
    const shape = state.doc.terrain?.[state.selection.group]?.[state.selection.index]
    if (shape) {
      landPoints(state.selection.group, shape).forEach((point, i, all) => {
        const [sx, sy] = toScreen(point)
        ctx.beginPath()
        if (state.selection.group === 'heaps' && i === 1) ctx.arc(sx, sy, 5.5, 0, Math.PI * 2)
        else ctx.rect(sx - 4.5, sy - 4.5, 9, 9)
        ctx.fillStyle = all.length > 1 && state.selection.group === 'heaps' && i === 1 ? '#f0e2c0' : '#ffffff'
        ctx.fill()
        ctx.strokeStyle = '#0d1117'
        ctx.lineWidth = 1.5
        ctx.stroke()
      })
    }
  }
  if (chosen) {
    // Each bridge and tunnel of the selected road ends in a knob either end, to
    // drag along the road.
    const moving = ['structureEnd', 'structure'].includes(state.drag?.kind) && state.drag.road === chosen.id ? state.drag.index : null
    for (const knob of structureKnobs(chosen)) {
      const [qx, qy] = toScreen(knob.point)
      ctx.beginPath()
      ctx.rect(qx - 5, qy - 5, 10, 10)
      ctx.fillStyle = knob.index === moving ? '#ffffff' : '#22d3ee'
      ctx.fill()
      ctx.strokeStyle = '#0d1117'
      ctx.lineWidth = 1.5
      ctx.stroke()
      if (knob.index === moving && knob.which === 'from') {
        const structure = chosen.structures[knob.index]
        const label = `#${knob.index} ${structure.kind} ${structure.from.toFixed(1)}–${structure.to.toFixed(1)} m (${(structure.to - structure.from).toFixed(1)} m)`
        ctx.font = '12px system-ui, sans-serif'
        const width = ctx.measureText(label).width
        ctx.fillStyle = '#000000cc'
        ctx.fillRect(qx + 10, qy - 22, width + 10, 18)
        ctx.fillStyle = '#ffffff'
        ctx.fillText(label, qx + 15, qy - 9)
      }
    }
  }
  for (const road of state.doc.roads) {
    const selected = road.id === selectedRoad
    const ends = isAligned(road) ? [road.points[0], endPoint(road, 'end')] : road.points
    ends.forEach((point, i) => {
      const isEnd = i === 0 || i === ends.length - 1
      // A selected alignment's far end is its last piece's knob, drawn above.
      if (isAligned(road) && i > 0 && selected) return
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

/// A tick across an alignment where each piece gives way to the next, so its
/// pieces can be told apart on the map.
function drawElementTicks(road, line, px) {
  let station = 0
  ctx.strokeStyle = '#ffd33d'
  ctx.lineWidth = 2 * px
  for (const element of road.elements ?? []) {
    station += element.length
    const at = alongLine(line, station)
    if (!at) continue
    const [x, y, ux, uy] = at
    const half = 6 * px
    ctx.beginPath()
    ctx.moveTo(x - uy * half, y + ux * half)
    ctx.lineTo(x + uy * half, y - ux * half)
    ctx.stroke()
  }
}

/// Where each piece of an alignment ends on the built road: its index, the
/// station, and the point.
function elementEnds(road) {
  const line = builtRoad(road.id)?.reference_line
  if (!line) return []
  let station = 0
  return (road.elements ?? []).flatMap((element, index) => {
    station += element.length
    const at = alongLine(line, station)
    return at ? [{ index, station, point: [at[0], at[1]] }] : []
  })
}

/// The curvature a piece of an alignment ends with: an arc's, a clothoid's end
/// radius's, nothing for a straight. Positive turns left.
function curvatureAfter(element) {
  if (element.kind === 'line') return 0
  return element.radius ? 1 / element.radius : 0
}

/// Where each bending piece of an alignment is grabbed by its radius: its middle,
/// with where the piece starts — point, direction and the curvature it starts
/// with — for working the new radius out from the pointer.
function radiusKnobs(road) {
  const line = builtRoad(road.id)?.reference_line
  if (!line) return []
  let station = 0
  let curvature = 0
  // The heading where each piece starts, exactly: the alignment's own, turned
  // by every piece before — a straight not at all, an arc by L·k, a clothoid by
  // L·(k0 + k1)/2. Read off the drawn line instead it would be a chord's.
  let heading = ((road.heading ?? 0) * Math.PI) / 180
  const knobs = []
  ;(road.elements ?? []).forEach((element, index) => {
    const begins = station
    const middle = alongLine(line, begins + element.length / 2)
    const start = alongLine(line, begins)
    if (element.kind !== 'line' && middle && start) {
      knobs.push({ index, point: [middle[0], middle[1]], start: [start[0], start[1]], along: [Math.cos(heading), Math.sin(heading)], curvature })
    }
    const after = curvatureAfter(element)
    heading += element.kind === 'line' ? 0 : element.kind === 'arc' ? element.length * after : (element.length * (curvature + after)) / 2
    station += element.length
    curvature = after
  })
  return knobs
}

/// The radius a bending piece takes for the pointer at `world`, from where it
/// starts: for an arc, the circle leaving the start in its direction through the
/// pointer — exactly; for a clothoid, the end curvature that puts the pointer on
/// it, by the clothoid's small-angle offset `k0·u²/2 + (k1−k0)·u³/(6L)`. `null`
/// for straight, `undefined` when the pointer says nothing (behind the start).
function radiusFor(drag, element, world) {
  const [dx, dy] = [world[0] - drag.start[0], world[1] - drag.start[1]]
  const [ax, ay] = drag.along
  const u = dx * ax + dy * ay
  const v = ax * dy - ay * dx
  if (u < 0.5) return undefined
  const k =
    element.kind === 'arc'
      ? (2 * v) / (u * u + v * v)
      : drag.curvature + ((v - (drag.curvature * u * u) / 2) * 6 * element.length) / (u * u * u)
  if (Math.abs(k) < 1e-4) return element.kind === 'arc' ? Math.sign(k || 1) * 10000 : null
  const radius = Math.round((1 / k) * 10) / 10
  return Math.abs(radius) < 1 ? Math.sign(radius) : radius
}

/// How far along a polyline a point is, metres: the station of the nearest point
/// on it — carried on past either end along the line's first or last segment, so
/// that a road's end can be dragged further than the road goes.
function stationOn(line, [x, y]) {
  let best = null
  let along = 0
  for (let i = 1; i < line.length; i++) {
    const [a, b] = [line[i - 1], line[i]]
    const [dx, dy] = [b[0] - a[0], b[1] - a[1]]
    const length = Math.hypot(dx, dy)
    if (length === 0) continue
    let t = ((x - a[0]) * dx + (y - a[1]) * dy) / (length * length)
    if (!(i === 1 && t < 0) && !(i === line.length - 1 && t > 1)) t = Math.max(0, Math.min(1, t))
    const d = Math.hypot(x - (a[0] + t * dx), y - (a[1] + t * dy))
    if (!best || d < best.d) best = { d, station: along + t * length }
    along += length
  }
  return best?.station ?? 0
}

/// The point `station` metres along a polyline, and the direction there.
function alongLine(line, station) {
  let travelled = 0
  for (let i = 1; i < line.length; i++) {
    const [a, b] = [line[i - 1], line[i]]
    const length = dist(a, b)
    if (length === 0) continue
    if (travelled + length >= station - 1e-9 || i === line.length - 1) {
      const t = Math.max(0, Math.min(1, (station - travelled) / length))
      const [ux, uy] = [(b[0] - a[0]) / length, (b[1] - a[1]) / length]
      return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, ux, uy]
    }
    travelled += length
  }
  return null
}

/// The land as a few paths to fill — its triangles gathered into bands of
/// height, each one colour — and its contours as two, minor and major: drawn
/// every frame, built once a build.
function landCache(scene) {
  const terrain = scene?.terrain ?? []
  if (!terrain.length) return null
  const low = Math.min(...terrain.map((t) => t.heights[0]))
  const high = Math.max(...terrain.map((t) => t.heights[1]))
  const BANDS = 24
  const bands = Array.from({ length: BANDS }, (_, i) => ({ color: landColor((i + 0.5) / BANDS), path: new Path2D() }))
  for (const surface of terrain) {
    for (const [a, b, c] of surface.triangles) {
      const [p, q, r] = [surface.nodes[a], surface.nodes[b], surface.nodes[c]]
      const share = high > low ? ((p[2] + q[2] + r[2]) / 3 - low) / (high - low) : 0
      const band = bands[Math.min(BANDS - 1, Math.floor(share * BANDS))]
      band.path.moveTo(p[0], p[1])
      band.path.lineTo(q[0], q[1])
      band.path.lineTo(r[0], r[1])
      band.path.closePath()
    }
  }
  const minor = new Path2D()
  const major = new Path2D()
  for (const contour of scene.contours ?? []) {
    const into = contour.major ? major : minor
    for (const [a, b] of contour.segments) {
      into.moveTo(a[0], a[1])
      into.lineTo(b[0], b[1])
    }
  }
  return { bands, minor, major, low, high, interval: scene.contour_interval }
}

/// Green low ground through olive to pale brown high ground: the same ramp the
/// SVG uses.
function landColor(share) {
  const stops = [
    [0, [44, 70, 42]],
    [0.4, [86, 104, 56]],
    [0.75, [134, 118, 78]],
    [1, [190, 176, 150]],
  ]
  let i = 0
  while (i < stops.length - 2 && stops[i + 1][0] <= share) i++
  const [[a, from], [b, to]] = [stops[i], stops[i + 1]]
  const t = Math.max(0, Math.min(1, (share - a) / (b - a)))
  return `rgb(${from.map((v, k) => Math.round(v + (to[k] - v) * t)).join(',')})`
}

function drawLand(px) {
  const land = state.landCache
  if (!land) return
  for (const band of land.bands) {
    ctx.fillStyle = band.color
    ctx.fill(band.path)
    // Hairline seams between triangles of one band are the fill's own edges.
    ctx.strokeStyle = band.color
    ctx.lineWidth = px
    ctx.stroke(band.path)
  }
  ctx.strokeStyle = '#1b241455'
  ctx.lineWidth = 0.8 * px
  ctx.stroke(land.minor)
  ctx.strokeStyle = '#1b2414aa'
  ctx.lineWidth = 1.4 * px
  ctx.stroke(land.major)
}

/// The document's land shapes, over the land they make: a hill's crest, a
/// river's course at its width, a heap's toe, a pad's outline. The selected
/// one's points are drawn later, as handles.
function drawLandShapes(px) {
  const terrain = state.doc.terrain
  if (!terrain) return
  const chosen = state.selection?.kind === 'land' ? state.selection : null
  const is = (group, index) => chosen?.group === group && chosen.index === index
  ;(terrain.rivers ?? []).forEach((river, index) => {
    path(river.centre, false)
    ctx.strokeStyle = '#3b82c466'
    ctx.lineWidth = river.width
    ctx.lineCap = 'round'
    ctx.stroke()
    ctx.lineCap = 'butt'
    ctx.strokeStyle = is('rivers', index) ? '#4ea1ff' : '#9cc8f0'
    ctx.lineWidth = (is('rivers', index) ? 2.5 : 1.5) * px
    ctx.stroke()
  })
  ;(terrain.hills ?? []).forEach((hill, index) => {
    if (hill.crest.length > 1) {
      path(hill.crest, false)
    } else if (hill.crest.length === 1) {
      ctx.beginPath()
      ctx.arc(hill.crest[0][0], hill.crest[0][1], 3 * px, 0, Math.PI * 2)
    }
    ctx.setLineDash([5 * px, 3 * px])
    ctx.strokeStyle = is('hills', index) ? '#4ea1ff' : '#f0e2c0'
    ctx.lineWidth = (is('hills', index) ? 2.5 : 1.5) * px
    ctx.stroke()
    ctx.setLineDash([])
  })
  ;(terrain.heaps ?? []).forEach((heap, index) => {
    ctx.beginPath()
    ctx.arc(heap.centre[0], heap.centre[1], heap.radius, 0, Math.PI * 2)
    ctx.setLineDash([5 * px, 3 * px])
    ctx.strokeStyle = is('heaps', index) ? '#4ea1ff' : '#f0e2c0'
    ctx.lineWidth = (is('heaps', index) ? 2.5 : 1.5) * px
    ctx.stroke()
    ctx.setLineDash([])
  })
  ;(terrain.pads ?? []).forEach((pad, index) => {
    if (pad.outline.length < 2) return
    polygon(pad.outline)
    ctx.setLineDash([5 * px, 3 * px])
    ctx.strokeStyle = is('pads', index) ? '#4ea1ff' : '#f0e2c0'
    ctx.lineWidth = (is('pads', index) ? 2.5 : 1.5) * px
    ctx.stroke()
    ctx.setLineDash([])
  })
}

/// The points of a land shape a user drags: a crest's, a course's, an outline's,
/// a heap's centre and the point on its toe that sets its radius.
function landPoints(group, shape) {
  if (group === 'hills') return shape.crest
  if (group === 'rivers') return shape.centre
  if (group === 'pads') return shape.outline
  return [shape.centre, [shape.centre[0] + shape.radius, shape.centre[1], shape.centre[2]]]
}

/// The land shape under a world point: near a crest or a course, inside a heap's
/// toe or a pad's outline.
function landAt(world, screen) {
  const terrain = state.doc.terrain
  if (!terrain) return null
  const nearLine = (line, reach) => {
    for (let i = 1; i < line.length; i++) {
      const [a, b] = [line[i - 1], line[i]]
      const [dx, dy] = [b[0] - a[0], b[1] - a[1]]
      const t = Math.max(0, Math.min(1, ((world[0] - a[0]) * dx + (world[1] - a[1]) * dy) / (dx * dx + dy * dy || 1)))
      if (Math.hypot(world[0] - a[0] - t * dx, world[1] - a[1] - t * dy) <= reach) return true
    }
    return line.length === 1 && screenDistance(line[0], screen) <= SNAP_PX
  }
  const px = 1 / state.view.scale
  for (const [index, river] of (terrain.rivers ?? []).entries()) if (nearLine(river.centre, Math.max(river.width / 2, 6 * px))) return { kind: 'land', group: 'rivers', index }
  for (const [index, hill] of (terrain.hills ?? []).entries()) if (nearLine(hill.crest, 8 * px)) return { kind: 'land', group: 'hills', index }
  for (const [index, pad] of (terrain.pads ?? []).entries()) if (pad.outline.length > 2 && inside(pad.outline, world)) return { kind: 'land', group: 'pads', index }
  for (const [index, heap] of (terrain.heaps ?? []).entries()) if (dist(heap.centre, world) <= heap.radius) return { kind: 'land', group: 'heaps', index }
  return null
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

/// The tangent or heading handle of the selected road under the pointer.
function tangentAt(screen) {
  if (state.selection?.kind !== 'road') return null
  const road = findRoad(state.selection.id)
  if (!road) return null
  if (isAligned(road)) {
    const [sx, sy] = toScreen(road.points[0])
    const [ux, uy] = headingVector(road)
    if (Math.hypot(sx + ux * HEADING_PX - screen[0], sy - uy * HEADING_PX - screen[1]) <= HANDLE_PX) {
      return { kind: 'heading', road: road.id }
    }
    const knob = radiusKnobs(road).find((knob) => screenDistance(knob.point, screen) <= HANDLE_PX)
    if (knob) {
      // Measured from where the piece starts as it was when the drag began; the
      // pieces before it do not change, so neither does that.
      return { kind: 'radius', road: road.id, index: knob.index, start: knob.start, along: knob.along, curvature: knob.curvature }
    }
    const end = elementEnds(road).find((end) => screenDistance(end.point, screen) <= HANDLE_PX)
    if (!end) return null
    const line = builtRoad(road.id).reference_line
    return {
      kind: 'length',
      road: road.id,
      index: end.index,
      // The road as it was when the drag began: a piece that changes length can
      // change shape before its end too — a clothoid does — so the pointer is
      // measured along the road it grabbed, not the one it is reshaping.
      line,
      begins: end.station - road.elements[end.index].length,
    }
  }
  if (road.shape === 'polyline' || state.point == null || !road.points[state.point]) return null
  const [hx, hy] = handleOf(road, state.point)
  const p = road.points[state.point]
  for (const sign of [1, -1]) {
    if (screenDistance([p[0] + sign * hx, p[1] + sign * hy], screen) <= HANDLE_PX) {
      return { kind: 'tangent', road: road.id, index: state.point, sign }
    }
  }
  return null
}

/// The control point of the selected road under the pointer.
function handleAt(screen) {
  if (state.selection?.kind !== 'road') return null
  const road = findRoad(state.selection.id)
  if (!road) return null
  let best = null
  ;(isAligned(road) ? road.points.slice(0, 1) : road.points).forEach((point, index) => {
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

/// Which of its road's document features a drawn surface feature is. The IR names
/// a feature `surface/<road>/<kind>/<n>`, `n` counting that kind on that road in
/// the order the document lists them.
function surfaceIndex(view) {
  const road = findRoad(roadKey(view.road))
  if (!road) return null
  const n = Number(view.id.split('/').pop())
  let seen = -1
  for (let i = 0; i < (road.surface ?? []).length; i++) {
    if (road.surface[i].kind === view.kind && ++seen === n) return { road: road.id, index: i }
  }
  return null
}

/// Which of its road's document structures a drawn structure is: the one over the
/// same stretch of the same road.
function structureIndex(view) {
  const span = view.carries[0]
  const road = span && findRoad(roadKey(span.road))
  if (!road) return null
  const index = (road.structures ?? []).findIndex(
    (s) => s.kind === view.kind && Math.abs(s.from - span.from) < 1e-6 && Math.abs(s.to - span.to) < 1e-6,
  )
  return index < 0 ? null : { road: road.id, index }
}

function isSelectedStructure(view) {
  if (state.selection?.kind !== 'road' || state.structure == null) return false
  const found = structureIndex(view)
  return found?.road === state.selection.id && found.index === state.structure
}

/// The bridge or tunnel under a world point.
function structureAt(world) {
  for (const view of state.built?.scene?.structures ?? []) {
    if (inside(view.outline, world)) return structureIndex(view)
  }
  return null
}

/// The two ends of each of a road's structures, on the built road.
function structureKnobs(road) {
  const line = builtRoad(road.id)?.reference_line
  if (!line) return []
  return (road.structures ?? []).flatMap((structure, index) =>
    ['from', 'to'].flatMap((which) => {
      const at = alongLine(line, structure[which])
      return at ? [{ index, which, point: [at[0], at[1]] }] : []
    }),
  )
}

function isSelectedSurface(view) {
  if (state.selection?.kind !== 'road' || state.surface == null) return false
  const found = surfaceIndex(view)
  return found?.road === state.selection.id && found.index === state.surface
}

/// The surface feature under a world point, the smallest when several overlap.
function surfaceAt(world) {
  let found = null
  let area = Infinity
  for (const view of state.built?.scene?.surfaces ?? []) {
    if (!inside(view.outline, world)) continue
    const size = (view.stations[1] - view.stations[0]) * (view.laterals[1] - view.laterals[0])
    if (size < area) {
      found = view
      area = size
    }
  }
  return found && surfaceIndex(found)
}

/// A world point in a road's own coordinates: station along its reference line
/// and metres to the left of it, read off the drawn reference line.
function project(roadId, [x, y]) {
  const line = state.built?.scene?.roads?.find((r) => r.id === `road/${roadId}`)?.reference_line
  if (!line || line.length < 2) return null
  let best = null
  let along = 0
  for (let i = 1; i < line.length; i++) {
    const [a, b] = [line[i - 1], line[i]]
    const [dx, dy] = [b[0] - a[0], b[1] - a[1]]
    const length = Math.hypot(dx, dy)
    if (length === 0) continue
    const t = Math.max(0, Math.min(1, ((x - a[0]) * dx + (y - a[1]) * dy) / (length * length)))
    const [px, py] = [a[0] + t * dx, a[1] + t * dy]
    const d = Math.hypot(x - px, y - py)
    if (!best || d < best.d) best = { d, s: along + t * length, t: (dx * (y - a[1]) - dy * (x - a[0])) / length }
    along += length
  }
  return best && { s: best.s, t: best.t }
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

  if (state.selection?.kind === 'land') {
    const { group, index } = state.selection
    const shape = state.doc.terrain?.[group]?.[index]
    const at = shape && landPoints(group, shape).findIndex((point) => screenDistance(point, screen) <= HANDLE_PX)
    if (at != null && at >= 0) {
      state.drag = { kind: 'landPoint', group, index, point: at, snapshot: JSON.stringify(state.doc), moved: false }
      canvas.setPointerCapture(event.pointerId)
      return
    }
  }
  const tangent = tangentAt(screen)
  if (tangent) {
    state.drag = { ...tangent, snapshot: JSON.stringify(state.doc), moved: false }
    canvas.setPointerCapture(event.pointerId)
    return
  }
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
  const chosen = state.selection?.kind === 'road' && findRoad(state.selection.id)
  const knob = chosen && structureKnobs(chosen).find((knob) => screenDistance(knob.point, screen) <= HANDLE_PX)
  if (knob) {
    state.structure = knob.index
    state.drag = {
      kind: 'structureEnd',
      road: chosen.id,
      index: knob.index,
      which: knob.which,
      line: builtRoad(chosen.id).reference_line,
      snapshot: JSON.stringify(state.doc),
      moved: false,
    }
    canvas.setPointerCapture(event.pointerId)
    renderPanels()
    render()
    return
  }
  const end = endAt(screen, null, HANDLE_PX)
  const hit = !end && surfaceAt(world)
  if (hit) {
    const feature = findRoad(hit.road).surface[hit.index]
    state.selection = { kind: 'road', id: hit.road }
    state.surface = hit.index
    state.structure = null
    state.point = null
    state.drag = {
      kind: 'surface',
      road: hit.road,
      index: hit.index,
      from: project(hit.road, world),
      original: JSON.parse(JSON.stringify(feature)),
      snapshot: JSON.stringify(state.doc),
      moved: false,
    }
    canvas.setPointerCapture(event.pointerId)
    renderPanels()
    render()
    return
  }
  const held = !end && structureAt(world)
  if (held) {
    const structure = findRoad(held.road).structures[held.index]
    const line = builtRoad(held.road).reference_line
    state.selection = { kind: 'road', id: held.road }
    state.structure = held.index
    state.surface = null
    state.point = null
    state.drag = {
      kind: 'structure',
      road: held.road,
      index: held.index,
      line,
      grab: stationOn(line, world),
      original: { from: structure.from, to: structure.to },
      snapshot: JSON.stringify(state.doc),
      moved: false,
    }
    canvas.setPointerCapture(event.pointerId)
    renderPanels()
    render()
    return
  }
  const lane = laneAt(world)
  const shape = !end && !lane && landAt(world, screen)
  if (shape) {
    state.selection = shape
    state.point = null
    state.surface = null
    state.structure = null
    showTab('inspect')
    renderPanels()
    render()
    startPan(event, screen)
    return
  }
  const selection = end ? { kind: 'road', id: end.road } : selectionForLane(lane)
  if (selection) {
    const same = JSON.stringify(selection) === JSON.stringify(state.selection)
    state.selection = selection
    state.point = null
    state.surface = null
    state.structure = null
    if (lane && selection.kind === 'road' && roadKey(lane.road) === selection.id) state.lane = lane.index
    else if (!same) state.lane = null
    renderPanels()
    render()
    return
  }
  state.selection = null
  state.point = null
  state.surface = null
  state.structure = null
  renderPanels()
  render()
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
  if (drag?.kind === 'surface') {
    moveSurface(drag, world)
    return
  }
  if (drag?.kind === 'landPoint') {
    const shape = state.doc.terrain[drag.group][drag.index]
    if (drag.group === 'heaps') {
      if (drag.point === 0) shape.centre = [world[0], world[1], shape.centre[2]]
      else shape.radius = Math.max(1, Math.round(dist(shape.centre, world) * 10) / 10)
    } else {
      const points = landPoints(drag.group, shape)
      points[drag.point] = [world[0], world[1], points[drag.point][2] ?? 0]
    }
    drag.moved = true
    rebuildSoon()
    renderPanelsSoon()
    return
  }
  if (drag?.kind === 'structure' || drag?.kind === 'structureEnd') {
    moveStructure(drag, world)
    return
  }
  if (['tangent', 'heading', 'length', 'radius'].includes(drag?.kind)) {
    turnHandle(drag, world)
    return
  }
  if (state.tool !== 'select') render()
})

canvas.addEventListener('pointerup', (event) => {
  const drag = state.drag
  state.drag = null
  if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId)
  if (['point', 'surface', 'tangent', 'heading', 'length', 'radius', 'structure', 'structureEnd', 'landPoint'].includes(drag?.kind) && drag.moved) {
    checkpoint(drag.snapshot)
    changed()
  } else if (drag) {
    render()
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
  if (!road || isAligned(road)) return
  const segment = nearestSegment(road, world)
  if (!segment || segment.d * state.view.scale > 25) return
  commit((doc) => {
    const r = doc.roads.find((r) => r.id === road.id)
    r.points.splice(segment.index, 0, segment.point)
    spliceHandles(r, segment.index, 0, null)
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
  // An alignment moves whole when its start does, so its far end moves too —
  // by as much as the start has since the drag began, because the built road
  // the far end is read from lags a frame behind.
  drag.origin = drag.origin ?? { start: [...old], far: isAligned(road) ? [...endPoint(road, 'end')] : null }
  road.points[drag.index] = target
  const moved = isEnd ? [[drag.index === 0 ? 'start' : 'end', target]] : []
  const { start, far } = drag.origin
  if (far) moved.push(['end', far.map((v, k) => v + target[k] - start[k])])
  for (const [end, point] of moved) {
    for (const link of state.doc.links) {
      if (link.junction) continue
      const partner =
        link.from === road.id && link.from_end === end
          ? { road: link.to, end: link.to_end }
          : link.to === road.id && link.to_end === end
            ? { road: link.from, end: link.from_end }
            : null
      const other = partner && findRoad(partner.road)
      if (other && other !== road && !isAligned(other)) setEndPoint(other, partner.end, point)
    }
  }
  drag.moved = true
  rebuildSoon()
  renderPanelsSoon()
}

/// Slides a bridge or tunnel along its road, or one of its ends, by the station
/// the pointer is at on the road as it was when the drag began. Lengths keep to a
/// metre at least, and stations to the decimetre.
function moveStructure(drag, world) {
  const structure = findRoad(drag.road)?.structures?.[drag.index]
  if (!structure) return
  const station = stationOn(drag.line, world)
  const round = (v) => Math.round(v * 10) / 10
  if (drag.kind === 'structure') {
    const shift = round(station - drag.grab)
    const from = Math.max(0, drag.original.from + shift)
    structure.to = round(from + drag.original.to - drag.original.from)
    structure.from = round(from)
  } else if (drag.which === 'from') {
    structure.from = round(Math.max(0, Math.min(station, structure.to - 1)))
  } else {
    structure.to = round(Math.max(station, structure.from + 1))
  }
  drag.moved = true
  rebuildSoon()
  renderPanelsSoon()
}

/// Turns a handle: a tangent handle sets its point's handle to the pointer
/// (mirrored for the one behind the point), the heading knob points the
/// alignment at the pointer.
function turnHandle(drag, world) {
  const road = findRoad(drag.road)
  if (!road) return
  if (drag.kind === 'radius') {
    const element = road.elements[drag.index]
    const radius = radiusFor(drag, element, world)
    if (radius === undefined) return
    if (radius === null) delete element.radius
    else element.radius = radius
  } else if (drag.kind === 'length') {
    const length = Math.round((stationOn(drag.line, world) - drag.begins) * 10) / 10
    road.elements[drag.index].length = Math.max(0.5, length)
  } else if (drag.kind === 'heading') {
    const start = road.points[0]
    const degrees = (Math.atan2(world[1] - start[1], world[0] - start[0]) * 180) / Math.PI
    road.heading = Math.round(degrees * 10) / 10
  } else {
    const p = road.points[drag.index]
    const handle = [drag.sign * (world[0] - p[0]), drag.sign * (world[1] - p[1])]
    if (Math.hypot(...handle) < 0.05) return
    road.handles = road.handles ?? []
    while (road.handles.length <= drag.index) road.handles.push(null)
    road.handles[drag.index] = handle.map((v) => Math.round(v * 1000) / 1000)
  }
  drag.moved = true
  rebuildSoon()
  renderPanelsSoon()
}

/// Slides a surface feature along and across its road, by how far the pointer has
/// moved in the road's own coordinates. A rut along a path takes its path along.
function moveSurface(drag, world) {
  const at = project(drag.road, world)
  const feature = findRoad(drag.road)?.surface?.[drag.index]
  if (!at || !drag.from || !feature) return
  const [ds, dt] = [at.s - drag.from.s, at.t - drag.from.t]
  if (feature.path) {
    feature.path.knots = drag.original.path.knots.map(([s, t]) => [s + ds, t + dt])
  } else {
    feature.station = Math.max(0, drag.original.station + ds)
    feature.offset = drag.original.offset + dt
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
        if (isAligned(b) && !isAligned(a)) setEndPoint(a, from.end, endPoint(b, end.end))
        else setEndPoint(b, end.end, endPoint(a, from.end))
      }
      doc.links.push({ from: from.road, from_end: from.end, to: end.road, to_end: end.end, junction })
    },
    { select: { kind: 'link', index: state.doc.links.length } },
  )
}

function deleteSelection() {
  const s = state.selection
  if (!s) return
  if (s.kind === 'land') {
    return commit((doc) => doc.terrain[s.group].splice(s.index, 1), { select: null })
  }
  if (s.kind === 'road' && state.structure != null) {
    const index = state.structure
    state.structure = null
    return commit((doc) => doc.roads.find((r) => r.id === s.id).structures.splice(index, 1))
  }
  if (s.kind === 'road' && state.surface != null) {
    const index = state.surface
    state.surface = null
    return commit((doc) => doc.roads.find((r) => r.id === s.id).surface.splice(index, 1))
  }
  if (s.kind === 'road' && state.point != null) {
    const road = findRoad(s.id)
    if (road.points.length <= 2) return toast('A road keeps two points at least')
    const index = state.point
    state.point = null
    return commit((doc) => {
      const r = doc.roads.find((r) => r.id === s.id)
      r.points.splice(index, 1)
      spliceHandles(r, index, 1)
    })
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
  if (s.kind === 'land') return renderLandInspector(tab, s.group, s.index)
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
    field(
      'shape',
      select(
        [['smooth', 'smooth curve through points'], ['polyline', 'straight segments'], ['alignment', 'alignment: straights, arcs, clothoids']],
        road.shape ?? 'smooth',
        (v) => editRoad(id, (r) => changeShape(r, v)),
      ),
    ),
    field('road type', select(ROAD_TYPES, road.road_type ?? 'town', (v) => editRoad(id, (r) => (r.road_type = v)))),
    field(
      'speed limit km/h',
      numberInput(road.speed_limit, (v) => editRoad(id, (r) => (v == null ? delete r.speed_limit : (r.speed_limit = v))), { min: 1, placeholder: 'none' }),
    ),
    state.doc.terrain
      ? field(
          'with the land',
          select(
            [['', 'land meets the road'], ['fitted', 'land cut and filled to it'], ['structured', 'bridges and tunnels from the land']],
            road.ground ?? '',
            (v) => editRoad(id, (r) => (v ? (r.ground = v) : delete r.ground)),
          ),
        )
      : null,
  )

  // The shape first: it is what the road is.
  if (isAligned(road)) {
    renderAlignmentSection(tab, road)
  } else {
    renderPointsSection(tab, road)
  }

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

  renderSurfaceSection(tab, road)
  renderStructureSection(tab, road)

  const links = state.doc.links.map((link, index) => ({ link, index })).filter(({ link }) => link.from === id || link.to === id)
  tab.append(h('h4', {}, `Links (${links.length})`), linkList(links))
  tab.append(h('h4', {}, ''), h('button', { onclick: deleteSelection }, 'Delete road'))
}

/// Switches how a road's shape is said. Into an alignment, the road starts where
/// it did, the way it did, as one straight as long as it was — ready to be given
/// its bends. Out of one, its points are taken off the built road.
function changeShape(road, shape) {
  const built = builtRoad(road.id)
  if (shape === 'alignment' && !isAligned(road)) {
    const [a, b] = road.points
    road.heading = Math.round((Math.atan2(b[1] - a[1], b[0] - a[0]) * 1800) / Math.PI) / 10
    if (!road.elements?.length) {
      const length = built?.length ?? road.points.slice(1).reduce((sum, p, i) => sum + dist(road.points[i], p), 0)
      road.elements = [{ kind: 'line', length: Math.round(length * 100) / 100, rise: 0 }]
    }
  } else if (shape !== 'alignment' && isAligned(road) && built) {
    const line = built.reference_line
    const count = Math.min(line.length, Math.max(2, Math.round(built.length / 25) + 1))
    const z = road.points[0][2]
    road.points = Array.from({ length: count }, (_, i) => {
      const [x, y] = line[Math.round((i * (line.length - 1)) / (count - 1))]
      return [x, y, z]
    })
    road.handles = []
  }
  road.shape = shape
}

const ELEMENT_KINDS = [['line', 'straight'], ['arc', 'arc'], ['spiral', 'clothoid']]

function renderAlignmentSection(tab, road) {
  const id = road.id
  const start = road.points[0]
  const total = (road.elements ?? []).reduce((sum, e) => sum + (e.length || 0), 0)
  tab.append(
    h('h4', {}, `Alignment (${(road.elements ?? []).length} pieces, ${total.toFixed(1)} m)`),
    h('p', { class: 'muted' }, 'From its start, at its heading, each piece where the last one ended. Radius in metres: positive turns left, negative right. A clothoid eases from the curvature before it to the radius it ends at — blank for straight. Drag the square to move the road, the round knob to turn it, a diamond along the road to change the length of the piece that ends there, and an R knob sideways to change the radius of a bend.'),
    ...['x', 'y', 'z'].map((axis, k) =>
      field(`start ${axis} m`, numberInput(start[k].toFixed(3), (v) => v != null && editRoad(id, (r) => (r.points[0][k] = v)))),
    ),
    field('heading °', numberInput(road.heading ?? 0, (v) => v != null && editRoad(id, (r) => (r.heading = v)), { step: 0.1 })),
  )
  ;(road.elements ?? []).forEach((element, index) => tab.append(elementCard(road, element, index)))
  const add = (element) => editRoad(id, (r) => (r.elements = [...(r.elements ?? []), element]))
  tab.append(
    h(
      'div',
      { class: 'row' },
      h('button', { onclick: () => add({ kind: 'line', length: 30, rise: 0 }) }, '+ straight'),
      h('button', { onclick: () => add({ kind: 'arc', length: 30, radius: 60, rise: 0 }) }, '+ arc'),
      h('button', { onclick: () => add({ kind: 'spiral', length: 20, radius: 60, rise: 0 }) }, '+ clothoid'),
      h(
        'button',
        {
          title: 'Ease in, bend, ease out: a clothoid into an arc and a clothoid back to straight',
          onclick: () =>
            editRoad(id, (r) => {
              r.elements = [
                ...(r.elements ?? []),
                { kind: 'spiral', length: 20, radius: 60, rise: 0 },
                { kind: 'arc', length: 30, radius: 60, rise: 0 },
                { kind: 'spiral', length: 20, rise: 0 },
              ]
            }),
        },
        '+ curve set',
      ),
    ),
  )
}

function elementCard(road, element, index) {
  const id = road.id
  const edit = (change) => editRoad(id, (r) => change(r.elements[index]))
  const move = (delta) =>
    editRoad(id, (r) => {
      const [moved] = r.elements.splice(index, 1)
      r.elements.splice(index + delta, 0, moved)
    })
  const radius = element.kind === 'line'
    ? []
    : [
        h('span', { class: 'muted' }, element.kind === 'arc' ? 'radius m' : 'end radius m'),
        numberInput(element.radius ?? '', (v) => edit((e) => (v == null || v === 0 ? delete e.radius : (e.radius = v))), {
          placeholder: element.kind === 'arc' ? 'required' : 'straight',
        }),
      ]
  return h(
    'div',
    { class: 'lane' },
    h(
      'div',
      { class: 'head' },
      h(
        'span',
        { class: 'row' },
        h('strong', {}, `#${index}`),
        select(ELEMENT_KINDS, element.kind, (v) =>
          edit((e) => {
            e.kind = v
            if (v === 'arc' && !e.radius) e.radius = 60
          }),
        ),
      ),
      h(
        'span',
        { class: 'row' },
        h('button', { title: 'Earlier', disabled: index === 0, onclick: () => move(-1) }, '↑'),
        h('button', { title: 'Later', disabled: index === road.elements.length - 1, onclick: () => move(1) }, '↓'),
        h('button', { title: 'Remove this piece', disabled: road.elements.length <= 1, onclick: () => editRoad(id, (r) => r.elements.splice(index, 1)) }, '×'),
      ),
    ),
    h(
      'div',
      { class: 'grid' },
      h('span', { class: 'muted' }, 'length m'),
      numberInput(element.length, (v) => v != null && v > 0 && edit((e) => (e.length = v)), { min: 0.1, step: 1 }),
      ...radius,
      h('span', { class: 'muted' }, 'rise m'),
      numberInput(element.rise ?? 0, (v) => edit((e) => (e.rise = v ?? 0)), { step: 0.1 }),
    ),
  )
}

function renderPointsSection(tab, road) {
  const id = road.id
  tab.append(
    h('h4', {}, 'Points (x, y, z metres)'),
    road.shape === 'polyline'
      ? null
      : h('p', { class: 'muted' }, 'Select a point to see its tangent handles and drag them to bend the road there; “auto” is the handle the curve works out by itself.'),
  )
  const table = h('table', { class: 'points' })
  road.points.forEach((point, index) => {
    const coordinate = (axis) =>
      numberInput(point[axis].toFixed(3), (v) => v != null && editRoad(id, (r) => (r.points[index][axis] = v)))
    table.append(
      h(
        'tr',
        {
          class: index === state.point ? 'selected' : '',
          title: 'Select this point',
          onclick: (event) => {
            if (event.target.tagName !== 'TD' || state.point === index) return
            state.point = index
            renderPanels()
            render()
          },
        },
        h('td', { class: 'muted' }, index),
        h('td', {}, coordinate(0)),
        h('td', {}, coordinate(1)),
        h('td', {}, coordinate(2)),
        road.shape === 'polyline'
          ? null
          : h(
              'td',
              {},
              road.handles?.[index]
                ? h('button', { title: 'Let the curve work this handle out again', onclick: () => editRoad(id, (r) => (r.handles[index] = null)) }, 'reset')
                : h('span', { class: 'muted' }, 'auto'),
            ),
        h(
          'td',
          {},
          h('button', {
            title: 'Remove this point',
            disabled: road.points.length <= 2,
            onclick: () =>
              editRoad(id, (r) => {
                r.points.splice(index, 1)
                spliceHandles(r, index, 1)
              }),
          }, '×'),
        ),
      ),
    )
  })
  tab.append(table)
}

/// Where the middle of a lane is, metres to the left of the reference line, from
/// the document's widths: what a new feature is centred on.
function laneCentre(road, index) {
  const handedness = state.doc.handedness ?? 'right'
  const sideOf = (lane) =>
    lane.side ?? ((lane.direction === 'forward') === (handedness === 'right') ? 'right' : 'left')
  const reach = { left: 0, right: 0 }
  for (let i = 0; i < road.lanes.length; i++) {
    const lane = road.lanes[i]
    const side = sideOf(lane)
    if (i === index) {
      const middle = reach[side] + lane.width / 2
      return side === 'left' ? middle : -middle
    }
    reach[side] += lane.width
  }
  return 0
}

/// How far the carriageway reaches either side: [right edge, left edge].
function carriageway(road) {
  let [right, left] = [0, 0]
  road.lanes.forEach((lane, i) => {
    const centre = laneCentre(road, i)
    right = Math.min(right, centre - lane.width / 2)
    left = Math.max(left, centre + lane.width / 2)
  })
  return [right, left]
}

/// A new feature of `kind`, halfway along the road, sized like the real thing.
function newSurface(road, kind) {
  const length = state.built?.scene?.roads?.find((r) => r.id === `road/${road.id}`)?.length ?? 20
  const middle = length / 2
  const lane = laneCentre(road, 0)
  const [right, left] = carriageway(road)
  const across = { offset: (right + left) / 2, width: left - right }
  const base = { kind, depth: 0.05, height: 0.075, track: 1.8, trough: 0.4, bump: 'hump', profile: '', ramp: 1, class: 'C', seed: 0 }
  const footprint = {
    rut: { station: Math.max(0, middle - 10), length: 20, offset: lane, width: 2.2, depth: 0.02 },
    pothole: { station: middle, length: 1.2, offset: lane, width: 0.9, depth: 0.06 },
    puddle: { station: middle, length: 3, offset: lane, width: 2, depth: 0.03 },
    bump: { station: middle, length: 3.7, ...across },
    roughness: { station: Math.max(0, middle - 10), length: 20, ...across },
  }[kind]
  return { ...base, ...footprint }
}

function renderSurfaceSection(tab, road) {
  const id = road.id
  const features = road.surface ?? []
  tab.append(
    h('h4', {}, `Surface (${features.length})`),
    h('p', { class: 'muted' }, 'Where the road departs from its design. Station along the road, offset to the left of its reference line, in metres; drag one on the map to move it.'),
  )
  features.forEach((feature, index) => tab.append(surfaceCard(road, feature, index)))
  tab.append(
    h(
      'div',
      { class: 'row' },
      SURFACE_KINDS.map((kind) =>
        h(
          'button',
          {
            onclick: () => {
              editRoad(id, (r) => (r.surface = [...(r.surface ?? []), newSurface(r, kind)]))
              state.surface = (findRoad(id).surface ?? []).length - 1
              renderPanels()
              render()
            },
          },
          `+ ${kind}`,
        ),
      ),
    ),
  )
}

const BRIDGE_FORMS = [['', 'bridge'], 'viaduct', 'overpass']
const TUNNEL_FORMS = [['', 'bored'], ['cut_and_cover', 'cut and cover'], 'underpass']
const MATERIALS = ['concrete', 'steel', 'masonry', 'timber']

function renderStructureSection(tab, road) {
  const id = road.id
  const structures = road.structures ?? []
  tab.append(
    h('h4', {}, `Bridges and tunnels (${structures.length})`),
    h('p', { class: 'muted' }, 'Over a stretch of the road, from and to in metres along it. Drag one on the map to slide it, or its cyan knobs to move an end.'),
  )
  structures.forEach((structure, index) => tab.append(structureCard(road, structure, index)))
  const add = (kind) => {
    const length = builtRoad(id)?.length ?? 60
    const from = Math.round(length * 0.3 * 10) / 10
    const to = Math.round(length * 0.7 * 10) / 10
    const base = { kind, from, to, material: 'concrete', deck_depth: 2.5, pier_spacing: 40, curb: 0.6, clear_height: 5, lining: 0.6, lighting: true, walkway: 0.75 }
    editRoad(id, (r) => (r.structures = [...(r.structures ?? []), base]))
    state.structure = (findRoad(id).structures ?? []).length - 1
    renderPanels()
    render()
  }
  tab.append(h('div', { class: 'row' }, h('button', { onclick: () => add('bridge') }, '+ bridge'), h('button', { onclick: () => add('tunnel') }, '+ tunnel')))
}

function structureCard(road, structure, index) {
  const id = road.id
  const edit = (change) => editRoad(id, (r) => change(r.structures[index]))
  const number = (key, label, attributes = {}) => [
    h('span', { class: 'muted' }, label),
    numberInput(structure[key], (v) => v != null && edit((s) => (s[key] = v)), { step: 0.1, ...attributes }),
  ]
  const bridge = structure.kind === 'bridge'
  const fields = bridge
    ? [
        h('span', { class: 'muted' }, 'material'),
        select(MATERIALS, structure.material ?? 'concrete', (v) => edit((s) => (s.material = v))),
        ...number('deck_depth', 'deck m', { min: 0.1 }),
        ...number('pier_spacing', 'pier spacing m', { min: 1, step: 1 }),
        ...number('curb', 'kerb m', { min: 0 }),
      ]
    : [
        ...number('clear_height', 'clear height m', { min: 0.1 }),
        ...number('lining', 'lining m', { min: 0.01 }),
        ...number('walkway', 'walkway m', { min: 0 }),
        h('span', { class: 'muted' }, 'lit'),
        h('input', { type: 'checkbox', checked: structure.lighting !== false, onchange: (e) => edit((s) => (s.lighting = e.target.checked)) }),
      ]
  return h(
    'div',
    {
      class: `lane${state.structure === index ? ' selected' : ''}`,
      onclick: (event) => {
        if (state.structure === index) return
        state.structure = index
        render()
        for (const card of document.querySelectorAll('.lane')) card.classList.remove('selected')
        event.currentTarget.classList.add('selected')
      },
    },
    h(
      'div',
      { class: 'head' },
      h(
        'span',
        { class: 'row' },
        h('strong', {}, `#${index}`),
        select(['bridge', 'tunnel'], structure.kind, (v) =>
          edit((s) => {
            s.kind = v
            s.form = ''
          }),
        ),
        select(bridge ? BRIDGE_FORMS : TUNNEL_FORMS, structure.form ?? '', (v) => edit((s) => (v ? (s.form = v) : delete s.form))),
      ),
      h(
        'button',
        {
          title: 'Remove this structure',
          onclick: (event) => {
            event.stopPropagation()
            state.structure = null
            editRoad(id, (r) => r.structures.splice(index, 1))
          },
        },
        '×',
      ),
    ),
    field('name', h('input', { value: structure.name ?? '', placeholder: 'none', onchange: (e) => edit((s) => (e.target.value.trim() ? (s.name = e.target.value.trim()) : delete s.name)) })),
    h('div', { class: 'grid' }, ...number('from', 'from m', { min: 0 }), ...number('to', 'to m', { min: 0 }), ...fields),
  )
}

function surfaceCard(road, feature, index) {
  const id = road.id
  const edit = (change) => editRoad(id, (r) => change(r.surface[index]))
  const number = (key, label, attributes = {}) => [
    h('span', { class: 'muted' }, label),
    numberInput(feature[key], (v) => v != null && edit((f) => (f[key] = v)), { step: 0.01, ...attributes }),
  ]
  const cm = (key, label) => [
    h('span', { class: 'muted' }, `${label} cm`),
    numberInput(Number(((feature[key] ?? 0) * 100).toFixed(2)), (v) => v != null && edit((f) => (f[key] = v / 100)), { step: 0.5, min: 0.1 }),
  ]
  const choose = (key, label, options) => [
    h('span', { class: 'muted' }, label),
    select(options, feature[key] ?? '', (v) => edit((f) => (f[key] = v))),
  ]
  const fields = {
    rut: [...cm('depth', 'depth'), ...number('track', 'track m'), ...number('trough', 'trough m')],
    pothole: [...cm('depth', 'depth'), ...number('seed', 'seed', { step: 1, min: 0 })],
    puddle: [...cm('depth', 'depth')],
    bump: [
      ...choose('bump', 'kind', BUMP_KINDS),
      ...cm('height', 'height'),
      ...choose('profile', 'profile', BUMP_PROFILES),
      ...(feature.profile === 'flat' ? number('ramp', 'ramp m') : []),
    ],
    roughness: [...choose('class', 'ISO 8608', ROUGHNESS_CLASSES), ...number('seed', 'seed', { step: 1, min: 0 })],
  }[feature.kind] ?? []
  const footprint = feature.path
    ? [h('p', { class: 'muted' }, `Follows a path through lane ${feature.path.lane}; its footprint is fitted to the path.`)]
    : [
        h(
          'div',
          { class: 'grid' },
          ...number('station', 'station'),
          ...number('length', 'length', { min: 0.01 }),
          ...number('offset', 'offset'),
          ...number('width', 'width', { min: 0.01 }),
        ),
      ]
  return h(
    'div',
    {
      class: `lane${state.surface === index ? ' selected' : ''}`,
      onclick: (event) => {
        if (state.surface === index) return
        state.surface = index
        render()
        for (const card of document.querySelectorAll('.lane')) card.classList.remove('selected')
        event.currentTarget.classList.add('selected')
      },
    },
    h(
      'div',
      { class: 'head' },
      h('span', { class: 'row' }, h('strong', {}, `#${index}`), select(SURFACE_KINDS, feature.kind, (v) => edit((f) => (f.kind = v)))),
      h(
        'button',
        {
          title: 'Remove this feature',
          onclick: (event) => {
            event.stopPropagation()
            state.surface = null
            editRoad(id, (r) => r.surface.splice(index, 1))
          },
        },
        '×',
      ),
    ),
    ...footprint,
    h('div', { class: 'grid' }, ...fields),
  )
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
              if (problem.road) {
                selectAndShow({ kind: 'road', id: problem.road })
                if (problem.surface != null || problem.structure != null) {
                  state.surface = problem.surface ?? null
                  state.structure = problem.structure ?? null
                  renderPanels()
                  render()
                }
              }
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
    ...landSection(),
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

const LAND_GROUPS = [
  ['hills', 'hill', 'A swell of the country along its crest: one point for a hill, more for a ridge.'],
  ['rivers', 'river', 'A channel along its course.'],
  ['heaps', 'heap', 'A cone of tipped material.'],
  ['pads', 'pad', 'Land levelled flat inside its outline.'],
]

/// A new land shape of `group`, in the middle of the view, sized like the real thing.
function newLandShape(group, taken) {
  const [x, y] = [state.view.cx, state.view.cy]
  const name = uniqueId(group.slice(0, -1), new Set(taken.map((shape) => shape.name)))
  if (group === 'hills') return { name, crest: [[x, y, 0]], width: 120, height: 20 }
  if (group === 'rivers') return { name, centre: [[x - 80, y, 0], [x + 80, y, 0]], width: 20, depth: 3, bank_slope: 2 }
  if (group === 'heaps') return { name, kind: 'stockpile', centre: [x, y, 0], radius: 40, height: 12, repose: 37 }
  return { name, outline: [[x - 20, y - 20, 0], [x + 20, y - 20, 0], [x + 20, y + 20, 0], [x - 20, y + 20, 0]] }
}

function landSection() {
  const terrain = state.doc.terrain
  const ground = (key, label, attributes = {}) =>
    field(label, numberInput(terrain[key], (v) => v != null && commit((d) => (d.terrain[key] = v)), { step: 1, ...attributes }))
  const header = [
    h('h4', {}, 'Land'),
    h(
      'label',
      { class: 'row' },
      h('input', {
        type: 'checkbox',
        checked: Boolean(terrain),
        onchange: (e) => commit((d) => (e.target.checked ? (d.terrain = d.terrain ?? { margin: 150, cell: 10, spacing: 5, relief: 6, wavelength: 180, seed: 0, hills: [], rivers: [], heaps: [], pads: [] }) : delete d.terrain)),
      }),
      'generate land round the roads',
    ),
  ]
  if (!terrain) return [...header, h('p', { class: 'muted' }, 'Without it the map states no land, and each format lays its own ground, if any.')]
  const notes = state.built?.land_notes ?? []
  return [
    ...header,
    h('p', { class: 'muted' }, `Contours every ${state.landCache?.interval ?? '—'} m, from ${state.landCache ? state.landCache.low.toFixed(1) : '—'} to ${state.landCache ? state.landCache.high.toFixed(1) : '—'} m. Each road says how it meets the land in its inspector.`),
    ground('relief', 'country relief m', { step: 0.5, min: 0 }),
    ground('wavelength', 'over m', { min: 1 }),
    ground('seed', 'seed', { min: 0 }),
    ground('margin', 'margin m', { min: 0 }),
    ground('cell', 'cell m', { min: 1 }),
    ground('spacing', 'spacing m', { min: 0.5, step: 0.5 }),
    ...LAND_GROUPS.flatMap(([group, noun]) => [
      h('h4', {}, `${group} (${(terrain[group] ?? []).length})`),
      (terrain[group] ?? []).length
        ? h(
            'ul',
            { class: 'list' },
            terrain[group].map((shape, index) =>
              h('li', { onclick: () => selectAndShow({ kind: 'land', group, index }) }, h('span', { class: 'what' }, shape.name || `${noun} ${index}`)),
            ),
          )
        : null,
      h(
        'button',
        {
          onclick: () => {
            const shape = newLandShape(group, terrain[group] ?? [])
            commit((d) => (d.terrain[group] = [...(d.terrain[group] ?? []), shape]), { select: { kind: 'land', group, index: (terrain[group] ?? []).length } })
            showTab('inspect')
          },
        },
        `+ ${noun}`,
      ),
    ]),
    notes.length ? h('h4', {}, 'The generator noted') : null,
    notes.length ? h('p', { class: 'muted notes' }, notes.join('\n')) : null,
  ]
}

function renderLandInspector(tab, group, index) {
  const shape = state.doc.terrain?.[group]?.[index]
  if (!shape) return
  const [, noun, about] = LAND_GROUPS.find(([g]) => g === group)
  const edit = (change) => commit((d) => change(d.terrain[group][index]))
  const number = (key, label, attributes = {}) =>
    field(label, numberInput(shape[key], (v) => v != null && edit((s) => (s[key] = v)), { step: 0.5, ...attributes }))
  tab.append(
    h('h3', {}, `${noun[0].toUpperCase()}${noun.slice(1)} ${shape.name}`),
    h('p', { class: 'muted' }, `${about} Drag its square handles on the map.`),
    field('name', h('input', { value: shape.name ?? '', onchange: (e) => edit((s) => (s.name = e.target.value.trim() || s.name)) })),
    ...{
      hills: [number('height', 'height m', { min: 0.5 }), number('width', 'width at foot m', { min: 1, step: 5 })],
      rivers: [number('width', 'width m', { min: 0.5 }), number('depth', 'depth m', { min: 0.1 }), number('bank_slope', 'bank slope (h:v)', { min: 0.1 })],
      heaps: [number('height', 'height m', { min: 0.5 }), number('radius', 'radius m', { min: 1 }), number('repose', 'repose °', { min: 5, max: 60 }), field('kind', h('input', { value: shape.kind ?? '', onchange: (e) => edit((s) => (s.kind = e.target.value.trim() || s.kind)) }))],
      pads: [],
    }[group],
  )
  if (group !== 'heaps') {
    const key = { hills: 'crest', rivers: 'centre', pads: 'outline' }[group]
    const points = shape[key]
    tab.append(h('h4', {}, `Points (${points.length})`))
    const table = h('table', { class: 'points' })
    points.forEach((point, i) =>
      table.append(
        h(
          'tr',
          {},
          h('td', { class: 'muted' }, i),
          h('td', {}, numberInput(point[0].toFixed(2), (v) => v != null && edit((s) => (s[key][i][0] = v)))),
          h('td', {}, numberInput(point[1].toFixed(2), (v) => v != null && edit((s) => (s[key][i][1] = v)))),
          group === 'pads' ? h('td', {}, numberInput(point[2].toFixed(2), (v) => v != null && edit((s) => (s[key][i][2] = v)))) : null,
          h('td', {}, h('button', { title: 'Remove this point', disabled: points.length <= (group === 'pads' ? 3 : group === 'rivers' ? 2 : 1), onclick: () => edit((s) => s[key].splice(i, 1)) }, '×')),
        ),
      ),
    )
    tab.append(
      table,
      h(
        'button',
        {
          onclick: () =>
            edit((s) => {
              const last = s[key][s[key].length - 1]
              const before = s[key][s[key].length - 2] ?? [last[0] - 30, last[1], last[2]]
              s[key].push([2 * last[0] - before[0], 2 * last[1] - before[1], last[2]])
            }),
        },
        '+ point',
      ),
      group === 'pads' ? h('p', { class: 'muted' }, 'A pad is levelled at the mean height of its points.') : null,
    )
  }
  tab.append(h('h4', {}, ''), h('button', { onclick: deleteSelection }, `Delete ${noun}`))
}

function selectAndShow(selection) {
  state.selection = selection
  state.point = null
  state.surface = null
  state.structure = null
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
    view: (cx, cy, scale) => {
      state.view = { cx, cy, scale }
      render()
    },
  }
  document.body.dataset.ready = 'true'
}

start()
