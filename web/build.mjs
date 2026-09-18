// Assembles the demo page into `site/`.
//
// Everything the page needs is copied in: the Pyodide runtime, the three JavaScript
// libraries, and roadgen's own wheel. The published page makes no request to anything
// it was not built with — no CDN, no tile of code fetched at runtime — which is what
// makes a deploy reproducible and what makes the page work behind a proxy that has
// never heard of jsdelivr.
//
//     node build.mjs --wheel ../dist/roadgen-0.1.0-cp313-cp313-pyodide_2025_0_wasm32.whl
//
// The Pyodide tarball is cached under `.cache/`, so a second build is offline.

import { createHash } from 'node:crypto'
import { spawnSync } from 'node:child_process'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))

// The Pyodide the wheel was built against. It is not a floating version: a wheel
// carries the ABI tag of the cross-build environment it came out of, and a runtime
// that does not match refuses to load it. The GitHub workflow pins the same number.
const PYODIDE = '0.28.3'
const PYODIDE_URL =
  `https://github.com/pyodide/pyodide/releases/download/${PYODIDE}/pyodide-core-${PYODIDE}.tar.bz2`

const site = path.join(here, 'site')
const cache = path.join(here, '.cache')

const wheel = argument('--wheel')

fs.rmSync(site, { recursive: true, force: true })
fs.mkdirSync(site, { recursive: true })

copyStatic()
copyVendor()
await copyPyodide()
copyWheel()

console.log(`site built in ${path.relative(process.cwd(), site)}`)

function argument(name) {
  const index = process.argv.indexOf(name)
  if (index < 0) return undefined
  const value = process.argv[index + 1]
  if (value === undefined || value.startsWith('--')) {
    throw new Error(`${name} needs a value`)
  }
  return value
}

function copyStatic() {
  fs.cpSync(path.join(here, 'src'), site, { recursive: true })
}

/// The three libraries the page does not write itself, taken from the packages npm
/// resolved rather than from a CDN, so the lockfile is what says which code ships.
function copyVendor() {
  const vendor = path.join(site, 'vendor')
  fs.mkdirSync(vendor, { recursive: true })

  const files = [
    // Leaflet draws the two OSM exports. BSD-2-Clause.
    ['leaflet/dist/leaflet.js', 'leaflet.js'],
    ['leaflet/dist/leaflet.css', 'leaflet.css'],
    ['leaflet/LICENSE', 'leaflet.LICENSE.txt'],
    // osmtogeojson turns OSM XML into something Leaflet can add. MIT. The
    // `osmtogeojson.js` in the package is the browser bundle: it reads the DOM the
    // browser already has rather than pulling an XML parser in with it.
    ['osmtogeojson/osmtogeojson.js', 'osmtogeojson.js'],
    ['osmtogeojson/LICENSE', 'osmtogeojson.LICENSE.txt'],
    // CodeMirror is the editor. MIT.
    ['codemirror/lib/codemirror.js', 'codemirror.js'],
    ['codemirror/lib/codemirror.css', 'codemirror.css'],
    ['codemirror/mode/python/python.js', 'codemirror-python.js'],
    ['codemirror/LICENSE', 'codemirror.LICENSE.txt'],
  ]
  for (const [from, to] of files) {
    fs.copyFileSync(path.join(here, 'node_modules', from), path.join(vendor, to))
  }

  // Leaflet's stylesheet asks for its marker and layer images by relative path.
  fs.cpSync(
    path.join(here, 'node_modules/leaflet/dist/images'),
    path.join(vendor, 'images'),
    { recursive: true },
  )
}

async function copyPyodide() {
  fs.mkdirSync(cache, { recursive: true })
  const tarball = path.join(cache, `pyodide-core-${PYODIDE}.tar.bz2`)
  if (!fs.existsSync(tarball)) {
    console.log(`fetching ${PYODIDE_URL}`)
    const response = await fetch(PYODIDE_URL)
    if (!response.ok) {
      throw new Error(`${PYODIDE_URL}: ${response.status} ${response.statusText}`)
    }
    fs.writeFileSync(tarball, Buffer.from(await response.arrayBuffer()))
  }
  console.log(`pyodide ${PYODIDE}: ${digest(tarball)}`)

  const unpacked = path.join(cache, `pyodide-${PYODIDE}`)
  if (!fs.existsSync(unpacked)) {
    fs.mkdirSync(unpacked, { recursive: true })
    run('tar', ['-xjf', tarball, '-C', unpacked])
  }
  fs.cpSync(path.join(unpacked, 'pyodide'), path.join(site, 'pyodide'), {
    recursive: true,
  })
}

/// The wheel, and a one-line manifest naming it — so the page asks the build which
/// file to load rather than having a version hard-coded in it.
function copyWheel() {
  if (!wheel) {
    throw new Error(
      'no --wheel given: build one with `pyodide build` and pass the .whl. ' +
        'The page is roadgen running, so there is nothing to publish without it.',
    )
  }
  const name = path.basename(wheel)
  if (!name.includes('wasm32')) {
    throw new Error(
      `${name} is not a Pyodide wheel. A native wheel loads in the browser only as ` +
        'far as the first import.',
    )
  }
  fs.copyFileSync(wheel, path.join(site, name))
  fs.writeFileSync(
    path.join(site, 'build.json'),
    `${JSON.stringify({ wheel: name, pyodide: PYODIDE }, null, 2)}\n`,
  )
  console.log(`wheel ${name}: ${digest(wheel)}`)
}

function digest(file) {
  return `sha256-${createHash('sha256').update(fs.readFileSync(file)).digest('base64')}`
}

function run(command, args) {
  const result = spawnSync(command, args, { stdio: 'inherit' })
  if (result.status !== 0) {
    throw new Error(`${command} ${args.join(' ')} failed`)
  }
}
