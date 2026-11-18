// Loads the built editor in a real browser and checks it works.
//
//     node smoke.mjs [site-directory]
//
// The engine is a wasm32 build that nothing on the build machine runs, so whether
// it loads, builds a document and writes every format is a question for a browser.
// This opens `site/` in headless Chromium and uses the page the way a person would:
// selects a road, drags a point, draws a road, joins it, undoes, and exports every
// format the page offers.

import fs from 'node:fs'
import http from 'node:http'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

import { chromium } from 'playwright'

const here = path.dirname(fileURLToPath(import.meta.url))
const site = path.resolve(process.argv[2] ?? path.join(here, 'site'))

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.wasm': 'application/wasm',
}

if (!fs.existsSync(path.join(site, 'index.html'))) fail(`${site} has no index.html — run ./build.sh first`)

const server = http.createServer((request, response) => {
  let file = decodeURIComponent(new URL(request.url, 'http://x').pathname)
  if (file.endsWith('/')) file += 'index.html'
  const full = path.join(site, file)
  if (!full.startsWith(site) || !fs.existsSync(full) || !fs.statSync(full).isFile()) {
    response.writeHead(404)
    response.end()
    return
  }
  response.writeHead(200, { 'content-type': TYPES[path.extname(full)] ?? 'application/octet-stream' })
  response.end(fs.readFileSync(full))
})
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve))
const origin = `http://127.0.0.1:${server.address().port}`

const browser = await chromium.launch()
const page = await browser.newPage({ viewport: { width: 1400, height: 900 }, acceptDownloads: true })
const thrown = []
page.on('pageerror', (error) => thrown.push(String(error)))
page.on('console', (message) => message.type() === 'error' && thrown.push(message.text()))

try {
  await page.goto(`${origin}/`)
  await page.waitForSelector('body[data-ready=true]', { timeout: 60_000 })
  const editor = (fn, arg) => page.evaluate(fn, arg)

  // The example is valid and generated connectors through its junction.
  let built = await editor(() => window.roadgenEditor.built())
  check(built.valid, `the example does not validate: ${JSON.stringify(built.problems)}`)
  check(built.stats.connectors > 0, 'no connectors were generated through the junction')
  check(built.scene.lanes.length > 10, `only ${built.scene.lanes.length} lanes were drawn`)
  check((await page.locator('#validity').textContent()) === 'valid', 'the status bar does not say valid')

  await shoot('overview')

  const box = await page.locator('#canvas').boundingBox()
  const at = async (world) => {
    const [x, y] = await editor((p) => window.roadgenEditor.toScreen(p), world)
    return [box.x + x, box.y + y]
  }

  // The example's worn road: every feature drawn, and a pothole dragged along.
  const kinds = built.scene.surfaces.map((s) => s.kind).sort()
  check(JSON.stringify(kinds) === JSON.stringify(['bump', 'pothole', 'puddle', 'rut']), `surface features drawn: ${kinds}`)
  const pothole = built.scene.surfaces.find((s) => s.kind === 'pothole')
  const centre = pothole.outline.reduce((sum, p) => [sum[0] + p[0] / pothole.outline.length, sum[1] + p[1] / pothole.outline.length], [0, 0])
  const before = (await editor(() => window.roadgenEditor.document())).roads[0].surface[1].station
  const grab = await at(centre)
  await page.mouse.move(...grab)
  await page.mouse.down()
  await page.mouse.move(grab[0] + 40, grab[1], { steps: 6 })
  await page.mouse.up()
  const after = (await editor(() => window.roadgenEditor.document())).roads[0].surface[1].station
  check(after > before + 2, `dragging the pothole moved it from station ${before} to ${after}`)
  const dragged = (await editor(() => window.roadgenEditor.built())).scene.surfaces.find((s) => s.kind === 'pothole')
  check(dragged.stations[0] > pothole.stations[0] + 2, 'the drawn pothole did not follow the document')
  await page.keyboard.press('Control+z')

  // A close look at the worn road, with its surface in the inspector.
  await page.mouse.click(...(await at([-70, -1.75])))
  await editor(() => window.roadgenEditor.view(-46, 0, 16))
  await page.locator('#tab-inspect h4', { hasText: 'Surface' }).scrollIntoViewIfNeeded()
  await shoot('surface')
  await page.keyboard.press('f')

  // Select `west` by clicking one of its lanes, then drag its far end.
  const west = (await editor(() => window.roadgenEditor.document())).roads.find((r) => r.id === 'west')
  const [a, b] = west.points
  await page.mouse.click(...(await at([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2 - 1.75])))
  const selection = await editor(() => window.roadgenEditor.state.selection)
  check(selection?.kind === 'road' && selection.id === 'west', `clicking west selected ${JSON.stringify(selection)}`)
  const lanesBefore = (await editor(() => window.roadgenEditor.built())).scene.lanes.find((l) => l.id === 'lane/west/0').outline
  const handle = await at(a)
  await page.mouse.move(...handle)
  await page.mouse.down()
  await page.mouse.move(handle[0], handle[1] - 80, { steps: 8 })
  await page.mouse.up()
  const moved = (await editor(() => window.roadgenEditor.document())).roads.find((r) => r.id === 'west').points[0]
  check(moved[1] > a[1] + 5, `dragging the end of west moved it to ${moved}`)
  const lanesAfter = (await editor(() => window.roadgenEditor.built())).scene.lanes.find((l) => l.id === 'lane/west/0').outline
  check(JSON.stringify(lanesBefore) !== JSON.stringify(lanesAfter), 'the lanes did not follow the dragged point')
  await page.keyboard.press('Control+z')
  const undone = (await editor(() => window.roadgenEditor.document())).roads.find((r) => r.id === 'west').points[0]
  check(JSON.stringify(undone) === JSON.stringify(a), `undo left west's end at ${undone}`)

  // Draw a road off the end of `avenue`: its start snaps on and a link is made.
  const avenue = (await editor(() => window.roadgenEditor.document())).roads.find((r) => r.id === 'avenue')
  const tip = avenue.points[avenue.points.length - 1]
  await page.keyboard.press('r')
  await page.mouse.click(...(await at(tip)))
  // Back over the map rather than past its edge: the avenue ends at the right of it.
  await page.mouse.click(...(await at([tip[0] - 10, tip[1] + 15])))
  await page.mouse.dblclick(...(await at([tip[0] - 25, tip[1] + 25])))
  const doc = await editor(() => window.roadgenEditor.document())
  const drawn = doc.roads.find((r) => r.id === 'road1')
  check(drawn && drawn.points.length === 3, `drawing made ${JSON.stringify(drawn)}`)
  check(
    doc.links.some((l) => l.from === 'avenue' && l.to === 'road1' && !l.junction),
    `no link from avenue to the new road: ${JSON.stringify(doc.links)}`,
  )
  built = await editor(() => window.roadgenEditor.built())
  check(built.scene.lanes.some((l) => l.road === 'road/road1'), 'the new road was not built')

  // Add a lane from the inspector.
  await page.getByRole('button', { name: '+ forward lane' }).click()
  const lanes = (await editor(() => window.roadgenEditor.document())).roads.find((r) => r.id === 'road1').lanes
  check(lanes.length === 3, `the new road has ${lanes.length} lanes after adding one`)
  await page.keyboard.press('Control+z')

  // A tangent handle: select east's middle point and bend the road there.
  await editor(() => window.roadgenEditor.view(60, 10, 7))
  let east = (await editor(() => window.roadgenEditor.document())).roads.find((r) => r.id === 'east')
  const mid = east.points[1]
  await page.mouse.click(...(await at([36, -1.75])))
  await page.mouse.click(...(await at(mid)))
  check((await editor(() => window.roadgenEditor.state.point)) === 1, 'clicking east\'s middle point did not select it')
  const [p0, p2] = [east.points[0], east.points[2]]
  const chord = Math.hypot(p2[0] - p0[0], p2[1] - p0[1])
  const reach = Math.min(Math.hypot(mid[0] - p0[0], mid[1] - p0[1]), Math.hypot(p2[0] - mid[0], p2[1] - mid[1])) / 3
  const knob = await at([mid[0] + ((p2[0] - p0[0]) / chord) * reach, mid[1] + ((p2[1] - p0[1]) / chord) * reach])
  const bentBefore = JSON.stringify((await editor(() => window.roadgenEditor.built())).scene.roads.find((r) => r.id === 'road/east').reference_line)
  await page.mouse.move(...knob)
  await page.mouse.down()
  await page.mouse.move(knob[0] + 10, knob[1] + 70, { steps: 8 })
  await page.mouse.up()
  east = (await editor(() => window.roadgenEditor.document())).roads.find((r) => r.id === 'east')
  check(east.handles?.[1], `dragging the handle set none: ${JSON.stringify(east.handles)}`)
  built = await editor(() => window.roadgenEditor.built())
  check(built.valid, `the bent road does not validate: ${JSON.stringify(built.problems)}`)
  check(JSON.stringify(built.scene.roads.find((r) => r.id === 'road/east').reference_line) !== bentBefore, 'the handle did not bend east')

  // An alignment: north, as a straight, then given a curve set.
  await editor(() => window.roadgenEditor.view(0, 60, 4))
  await page.mouse.click(...(await at([1.75, 40])))
  await page.locator('#tab-inspect select').first().selectOption('alignment')
  const north = (await editor(() => window.roadgenEditor.document())).roads.find((r) => r.id === 'north')
  check(north.shape === 'alignment' && north.elements.length === 1 && Math.abs(north.heading - 90) < 1e-6, `north as an alignment: ${JSON.stringify(north)}`)
  await page.getByRole('button', { name: '+ curve set' }).click()
  built = await editor(() => window.roadgenEditor.built())
  check(built.valid, `the aligned road does not validate: ${JSON.stringify(built.problems)}`)
  const aligned = built.scene.roads.find((r) => r.id === 'road/north')
  check(Math.abs(aligned.length - 128) < 0.01 && aligned.end.point[0] < -20, `north's alignment ends at ${aligned.end.point}, ${aligned.length} m`)
  await shoot('alignment')
  for (let i = 0; i < 3; i++) await page.keyboard.press('Control+z')
  const restored = await editor(() => window.roadgenEditor.document())
  check(restored.roads.find((r) => r.id === 'north').shape === 'smooth' && !restored.roads.find((r) => r.id === 'east').handles?.length, 'undo did not take the handle and the alignment back')
  await page.keyboard.press('f')

  // Every tab draws without throwing.
  for (const tab of ['Links', 'Problems', 'Map', 'Inspect']) await page.locator('.tabs button', { hasText: tab }).click()

  // Every format writes, from the engine directly and through the download button.
  const formats = await page.locator('#format option').evaluateAll((options) => options.map((o) => o.value))
  check(formats.length >= 9, `only ${formats.length} formats are offered`)
  built = await editor(() => window.roadgenEditor.built())
  check(built.valid, `the edited map does not validate: ${JSON.stringify(built.problems)}`)
  for (const format of formats) {
    const files = await editor((f) => window.roadgenEditor.exportFormat(f), format)
    check(files.length > 0 && files.every((f) => f.size > 0), `${format} wrote ${JSON.stringify(files)}`)
    if (format === 'opendrive') {
      check(files.some((f) => f.name.endsWith('.crg')), `OpenDRIVE wrote no OpenCRG for the surface: ${JSON.stringify(files)}`)
    }
    console.log(`  ${format}: ${files.map((f) => `${f.name} (${f.size} B)`).join(', ')}`)
  }
  await page.selectOption('#format', 'opendrive')
  const download = page.waitForEvent('download')
  await page.click('#export')
  const saved = await download
  check(saved.suggestedFilename().endsWith('.xodr'), `the OpenDRIVE download is called ${saved.suggestedFilename()}`)

  // What was written opens again.
  const xodr = await saved.path()
  await page.setInputFiles('#open-file', { name: 'roundtrip.xodr', mimeType: 'application/xml', buffer: fs.readFileSync(xodr) })
  // An imported road is its reference line as straight segments.
  await page.waitForFunction(() => window.roadgenEditor.document().roads.every((r) => r.shape === 'polyline'), null, { timeout: 30_000 })
  const reopened = await editor(() => window.roadgenEditor.document())
  check(reopened.roads.length === 6, `the OpenDRIVE file opened with ${reopened.roads.length} roads`)

  await shoot('final')
} catch (error) {
  thrown.push(String(error?.stack ?? error))
} finally {
  await browser.close()
  server.close()
}

if (thrown.length) fail(thrown.join('\n'))
console.log('editor smoke test passed')

/// Saves what the page looks like now as `smoke-<name>.png`. With
/// EDITOR_PRINT_SCREENSHOTS set, also prints it to the log as base64 JPEG between
/// marker lines, for a reader who can see the log but not the artifacts.
async function shoot(name) {
  await page.screenshot({ path: path.join(here, `smoke-${name}.png`) })
  if (!process.env.EDITOR_PRINT_SCREENSHOTS) return
  const jpeg = await page.screenshot({ type: 'jpeg', quality: 70 })
  const text = jpeg.toString('base64')
  console.log(`--- screenshot ${name} begin ---`)
  for (let i = 0; i < text.length; i += 4000) console.log(text.slice(i, i + 4000))
  console.log(`--- screenshot ${name} end ---`)
}

function check(condition, message) {
  if (!condition) throw new Error(message)
}

function fail(message) {
  console.error(message)
  process.exit(1)
}
