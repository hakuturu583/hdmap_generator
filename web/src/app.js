// The page.
//
// There are three jobs here and nothing else: start Python, hand it what was typed,
// and lay out what comes back. No road, lane or file format is understood on this
// side — the panels are built from whatever `driver.py` says it found, so adding a
// format to roadgen adds a panel here without this file changing.

import { examples } from './examples.js'

const ui = {
  example: document.querySelector('#example'),
  run: document.querySelector('#run'),
  editor: document.querySelector('#editor'),
  console: document.querySelector('#console'),
  status: document.querySelector('#status'),
  results: document.querySelector('#results'),
  build: document.querySelector('#build-line'),
}

let pyodide = null
let driver = null
let editor = null
// Leaflet keeps hold of the element it was given, so a map from a previous run has to
// be taken down before its panel is thrown away — otherwise it goes on listening to a
// window it is no longer in.
let maps = []

boot()

async function boot() {
  editor = CodeMirror(ui.editor, {
    value: examples[0].code,
    mode: 'python',
    lineNumbers: true,
    indentUnit: 4,
    // The pane is a column of a two-column page rather than a window, so a long line
    // wraps instead of asking for the page to be scrolled sideways to read it.
    lineWrapping: true,
    extraKeys: { 'Ctrl-Enter': run, 'Cmd-Enter': run },
  })

  for (const [index, example] of examples.entries()) {
    const option = document.createElement('option')
    option.value = String(index)
    option.textContent = example.name
    option.title = example.description
    ui.example.append(option)
  }
  ui.example.addEventListener('change', () => {
    editor.setValue(examples[Number(ui.example.value)].code)
    editor.focus()
  })
  ui.run.addEventListener('click', run)

  try {
    const build = await (await fetch('./build.json')).json()
    ui.build.textContent =
      `Built from ${build.wheel}, on Pyodide ${build.pyodide}.`

    say('Starting Python…')
    pyodide = await loadPyodide({ indexURL: './pyodide/' })

    say(`Loading ${build.wheel}…`)
    await pyodide.loadPackage(`./${build.wheel}`)

    // The driver is a file rather than a string in here so that it is Python being
    // edited when it is edited, with everything that comes with that.
    const source = await (await fetch('./driver.py')).text()
    pyodide.FS.writeFile('/driver.py', source)
    // The script the driver runs chdirs into its own directory, so the driver cannot
    // be imported from the working directory: it is put at the root and the root is
    // put on the path.
    pyodide.runPython('import sys; sys.path.insert(0, "/")')
    driver = pyodide.pyimport('driver')

    const version = pyodide.runPython('import roadgen; roadgen.__version__')
    say(`roadgen ${version} is ready. Ctrl+Enter runs.`)
    ui.run.disabled = false
    await run()
  } catch (error) {
    say(`Python would not start: ${error}`, true)
  }
}

async function run() {
  if (!driver || ui.run.disabled) return
  ui.run.disabled = true
  say('Running…')
  try {
    const answer = JSON.parse(driver.run(editor.getValue()))
    report(answer)
    show(answer)
  } catch (error) {
    say(String(error), true)
  } finally {
    ui.run.disabled = false
  }
}

function say(message, bad = false) {
  ui.console.replaceChildren(element('div', { class: bad ? 'status bad' : 'status' }, message))
}

/// What the script printed, what it raised, and how long it took.
function report({ stdout, error, seconds, files }) {
  const written = files.length
  const lines = [
    element(
      'div',
      { class: 'status' },
      `${written} file${written === 1 ? '' : 's'} written in ${seconds}s`,
    ),
  ]
  if (stdout.trim()) lines.push(element('div', {}, stdout.trimEnd()))
  if (error) lines.push(element('div', { class: 'bad' }, error.trimEnd()))
  ui.console.replaceChildren(...lines)
}

function show({ views, files, error }) {
  for (const map of maps) map.remove()
  maps = []

  if (views.length === 0) {
    ui.results.replaceChildren(
      element(
        'p',
        { class: 'placeholder' },
        error
          ? 'Nothing was drawn: the script stopped before it wrote anything.'
          : 'Nothing was drawn. Call one of the export_* methods and run again.',
      ),
    )
    return
  }

  const sizes = new Map(files.map((file) => [file.path, file.size]))
  ui.results.replaceChildren(...views.map((view) => card(view, sizes)))
}

function card(view, sizes) {
  const body = view.error
    ? element('p', { class: 'bad' }, `${view.format} would not draw: ${view.error}`)
    : view.svg
      ? figure(view.svg)
      : mapOf(view.xml)

  return element('article', { class: 'card' }, [
    element('header', {}, [
      element('span', { class: 'badge' }, view.format),
      element('h2', {}, view.title),
      note(view),
    ]),
    element('div', { class: 'chips' }, view.files.map((path) => chip(path, sizes))),
    body,
  ])
}

/// The picture's own `<desc>`: what the reader found in the file, and what the format
/// could not carry. It is written by the crate that drew the picture, so it says the
/// same thing on the page as it does in a notebook.
function note(view) {
  if (!view.svg) {
    return element(
      'p',
      { class: 'note' },
      'Drawn by Leaflet, over OpenStreetMap tiles: this export carries latitudes and ' +
        'longitudes, so it has a place on Earth rather than only a shape.',
    )
  }
  const description = new DOMParser()
    .parseFromString(view.svg, 'image/svg+xml')
    .querySelector('desc')
  return element('p', { class: 'note' }, description ? description.textContent : '')
}

function figure(svg) {
  const holder = element('div', { class: 'figure' })
  // The SVG comes from the wheel this page shipped with, not from anything a visitor
  // supplied, and it has to become live elements to be styled by the page's custom
  // properties.
  holder.innerHTML = svg
  return holder
}

function mapOf(xml) {
  const holder = element('div', { class: 'map' })
  // Leaflet measures the element it is given, so it can only be set up once the
  // element is in the document.
  queueMicrotask(() => {
    const document_ = new DOMParser().parseFromString(xml, 'text/xml')
    const geojson = onlyWays(osmtogeojson(document_))

    const map = L.map(holder, { scrollWheelZoom: false })
    maps.push(map)
    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20,
      attribution:
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    }).addTo(map)

    const layer = L.geoJSON(geojson, {
      style: { color: '#2563eb', weight: 2 },
      pointToLayer: (_, at) => L.circleMarker(at, { radius: 3, color: '#f97316' }),
      onEachFeature: (feature, target) => {
        const tags = Object.entries(feature.properties ?? {})
          .filter(([key]) => !key.startsWith('@'))
          .map(([key, value]) => `${key} = ${value}`)
        if (tags.length) target.bindPopup(tags.join('<br>'))
      },
    }).addTo(map)

    const bounds = layer.getBounds()
    if (bounds.isValid()) {
      map.fitBounds(bounds, { padding: [24, 24] })
    } else {
      map.setView([0, 0], 2)
    }
  })
  return holder
}

/// Ways, and the nodes that say something on their own.
///
/// A Lanelet2 map is mostly relations — a lanelet is a pair of ways and a relation
/// tying them together — and osmtogeojson turns each of those into a feature covering
/// the same ground as the ways inside it. Drawing both stacks every boundary twice
/// and makes a popup that hits the relation rather than the way under the pointer, so
/// only the ways are kept.
function onlyWays(geojson) {
  return {
    type: 'FeatureCollection',
    features: geojson.features.filter((feature) => {
      const id = feature.id ?? ''
      return id.startsWith('way/') || id.startsWith('node/')
    }),
  }
}

function chip(path, sizes) {
  const size = sizes.get(path)
  const view = element('button', { type: 'button', 'aria-pressed': 'false' }, 'source')
  view.addEventListener('click', () => toggleSource(view, path))

  const save = element('a', { href: '#', download: path.split('/').pop() }, 'save')
  save.addEventListener('click', (event) => {
    event.preventDefault()
    download(path)
  })

  return element('span', { class: 'chip' }, [
    element('span', {}, path),
    element('span', { class: 'size' }, size === undefined ? '' : bytes(size)),
    view,
    save,
  ])
}

function toggleSource(button, path) {
  const card = button.closest('.card')
  const open = card.querySelector(`.source[data-path="${cssEscape(path)}"]`)
  for (const shown of card.querySelectorAll('.source')) shown.remove()
  for (const other of card.querySelectorAll('.chip button')) {
    other.setAttribute('aria-pressed', 'false')
  }
  if (open) return

  button.setAttribute('aria-pressed', 'true')
  const raw = pyodide.FS.readFile(`/work/${path}`)
  const text = looksBinary(raw)
    ? `${path} is ${bytes(raw.length)} of binary — Parquet, in this case. Save it and ` +
      'open it with pyarrow or pandas.'
    : new TextDecoder().decode(raw)
  card.append(element('pre', { class: 'source', 'data-path': path }, text))
}

function download(path) {
  const raw = pyodide.FS.readFile(`/work/${path}`)
  const url = URL.createObjectURL(new Blob([raw], { type: 'application/octet-stream' }))
  const link = element('a', { href: url, download: path.split('/').pop() })
  document.body.append(link)
  link.click()
  link.remove()
  setTimeout(() => URL.revokeObjectURL(url), 1000)
}

/// A NUL byte in the first kilobyte. Crude, and right about every file roadgen
/// writes: five of the six formats are text and the sixth is Parquet, which opens
/// with `PAR1` and is full of them.
function looksBinary(raw) {
  return raw.subarray(0, 1024).includes(0)
}

function bytes(count) {
  if (count < 1024) return `${count} B`
  if (count < 1024 * 1024) return `${(count / 1024).toFixed(1)} kB`
  return `${(count / 1024 / 1024).toFixed(1)} MB`
}

function cssEscape(value) {
  return window.CSS && CSS.escape ? CSS.escape(value) : value.replace(/["\\]/g, '\\$&')
}

function element(tag, attributes = {}, children = []) {
  const node = document.createElement(tag)
  for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, value)
  for (const child of [children].flat()) {
    if (child === null || child === undefined) continue
    node.append(child)
  }
  return node
}
