// Loads the built page in a real browser and checks it works.
//
//     node smoke.mjs [site-directory]
//
// The page is the one deliverable no unit test can stand in for: the wheel is a
// wasm32 build that nothing on the build machine can import, and whether it loads is
// a question about a browser. So this opens `site/` in headless Chromium, runs each
// example, and clicks through every tab — because a format that is drawn only when
// you look at it is a format that has to be looked at to be checked.
//
// Tiles are not fetched: a smoke test that needs the network to pass is a smoke test
// that fails for reasons that are not about this repository.

import fs from 'node:fs'
import http from 'node:http'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

import { chromium } from 'playwright'

const here = path.dirname(fileURLToPath(import.meta.url))
const site = path.resolve(process.argv[2] ?? path.join(here, 'site'))

/// Every format the page is meant to draw. Nothing less than all of them is a pass:
/// the failure this is looking for is one exporter quietly writing nothing.
const FORMATS = ['OpenDRIVE', 'Lanelet2', 'OpenStreetMap', 'SUMO', 'ClipGT', 'GPUDrive']

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json',
  '.py': 'text/plain; charset=utf-8',
  '.wasm': 'application/wasm',
  '.whl': 'application/octet-stream',
  '.zip': 'application/zip',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
}

if (!fs.existsSync(path.join(site, 'index.html'))) {
  fail(`${site} has no index.html in it — run \`node build.mjs --wheel …\` first`)
}

const server = http.createServer(serve)
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve))
const origin = `http://127.0.0.1:${server.address().port}`

const browser = await chromium.launch()
const page = await browser.newPage({ viewport: { width: 1400, height: 1000 } })

const thrown = []
page.on('pageerror', (error) => thrown.push(String(error)))
page.on('console', (message) => {
  if (message.type() !== 'error') return
  // A tile that did not load is the network being absent, not the page being wrong.
  if (/tile\.openstreetmap|ERR_(TUNNEL|NAME|INTERNET|CONNECTION)/.test(message.text())) return
  thrown.push(message.text())
})

await page.goto(`${origin}/index.html`)
// The wheel and the Pyodide runtime are megabytes; a slow runner is not a failure.
await page.waitForSelector('.tab', { timeout: 300_000 })

const examples = await page.$$eval('#example option', (options) =>
  options.map((option) => option.textContent),
)
if (examples.length === 0) fail('the page offers no examples')

for (const [index, name] of examples.entries()) {
  if (index > 0) {
    await page.selectOption('#example', String(index))
    await page.click('#run')
    await page.waitForTimeout(1500)
  }
  await check(name)
}

console.log(`${examples.length} examples, ${FORMATS.length} formats each: all drawn`)
await browser.close()
server.close()

async function check(example) {
  const offered = await page.$$eval('.tab', (tabs) => tabs.map((tab) => tab.textContent))
  for (const format of FORMATS) {
    if (!offered.includes(format)) fail(`${example}: no ${format} tab`)
  }

  for (const format of offered) {
    await page.click(`.tab:text-is("${format}")`)
    // Leaflet resizes itself when its panel is shown, and refitting takes a frame.
    await page.waitForTimeout(250)

    const panel = await page.$$eval(
      '.panel:not([hidden])',
      (nodes) => {
        const node = nodes[0]
        if (!node) return null
        return {
          title: node.querySelector('h2')?.textContent ?? '',
          bad: node.querySelector('.bad')?.textContent ?? '',
          // A Leaflet map draws its features into an overlay pane; a viewer panel is
          // an SVG of its own. Either way, an empty one is the thing worth failing on.
          drawn:
            node.querySelectorAll('.figure svg polyline, .figure svg polygon').length +
            node.querySelectorAll('.leaflet-overlay-pane path').length,
          files: node.querySelectorAll('.chip').length,
        }
      },
    )

    if (!panel) fail(`${example}: choosing ${format} showed nothing`)
    if (panel.bad) fail(`${example}: ${format} said "${panel.bad}"`)
    if (panel.drawn === 0) fail(`${example}: the ${format} panel is empty`)
    if (panel.files === 0) fail(`${example}: the ${format} panel lists no files`)

    const shown = await page.$$eval('.panel:not([hidden])', (nodes) => nodes.length)
    if (shown !== 1) fail(`${example}: ${shown} panels are visible at once, not 1`)
  }

  const console_ = await page.$eval('#console', (node) => node.textContent)
  if (/Traceback/.test(console_)) {
    fail(`${example}: the script raised\n${console_}`)
  }
  if (thrown.length > 0) {
    fail(`${example}: the page threw\n  ${thrown.join('\n  ')}`)
  }
  console.log(`  ${example}: ${offered.length} tabs`)
}

function serve(request, response) {
  const asked = decodeURIComponent(new URL(request.url, origin).pathname)
  const file = path.join(site, asked === '/' ? '/index.html' : asked)
  // Nothing outside the site directory is served, symlink or `..` notwithstanding.
  if (!file.startsWith(site) || !fs.existsSync(file) || !fs.statSync(file).isFile()) {
    response.writeHead(404).end('not found')
    return
  }
  response.writeHead(200, {
    'Content-Type': TYPES[path.extname(file)] ?? 'application/octet-stream',
  })
  fs.createReadStream(file).pipe(response)
}

function fail(message) {
  console.error(`smoke test failed: ${message}`)
  process.exit(1)
}
