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
  results: document.querySelector('#results'),
  build: document.querySelector('#build-line'),
}

let pyodide = null
let driver = null
let editor = null
// Where the driver ran the script. It says so with every answer rather than the page
// spelling the directory a second time.
let work = null
// Leaflet keeps hold of the element it was given, so a map from a previous run has to
// be taken down before its panel is thrown away — otherwise it goes on listening to a
// window it is no longer in.
let maps = []
// Which view the viewer is showing, as its format and the file it was drawn from. It
// outlives a run: change the script, press Run again, and you are still looking at
// what you were looking at before, which is the whole point of being able to compare
// them. The file is remembered as well as the format because a script may write two
// exports of one format, and then the format alone does not say which.
let showing = null

// A vendor script that did not arrive leaves its global undefined, and the first line
// of `boot` then throws before there is anywhere to say so: the page would sit there
// with the console still reading "Starting Python…" and the Run button disabled, which
// looks like the page being broken rather than telling you what broke.
boot().catch((error) => say(`The page would not start: ${error}`, true))

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
    // The runtime is megabytes and the two files beside it are a few hundred bytes;
    // asking for them in front of it, one after the other, would add two round trips
    // to the slowest part of the page. Nothing here needs the answers yet.
    const described = fetch('./build.json').then((answer) => answer.json())
    // The driver is a file rather than a string in here so that it is Python being
    // edited when it is edited, with everything that comes with that.
    const source = fetch('./driver.py').then((answer) => answer.text())

    say('Starting Python…')
    const starting = loadPyodide({ indexURL: './pyodide/' })

    const build = await described
    ui.build.textContent = `Built from ${build.wheel}, on Pyodide ${build.pyodide}.`
    pyodide = await starting

    say(`Loading ${build.wheel}…`)
    await pyodide.loadPackage(`./${build.wheel}`)

    pyodide.FS.writeFile('/driver.py', await source)
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
  say(`${written} file${written === 1 ? '' : 's'} written in ${seconds}s`)
  if (stdout.trim()) ui.console.append(element('div', {}, stdout.trimEnd()))
  if (error) ui.console.append(element('div', { class: 'bad' }, error.trimEnd()))
}

function show({ views, work: directory, error }) {
  for (const map of maps) map.remove()
  maps = []
  work = directory

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

  // A tab and a panel per thing that was written, kept together: everything that
  // selects one of them works off this list rather than going back to the document.
  const viewers = views.map(viewer)
  const bar = element('div', { class: 'tabs', role: 'tablist', 'aria-label': 'Format' })
  bar.append(...viewers.map((entry) => entry.tab))
  listen(viewers)

  // Every panel is built now and all but one is hidden; the pictures are already in
  // the answer, so this is an SVG becoming elements rather than any file being read
  // again. The one thing worth deferring — a Leaflet map, which cannot measure a
  // hidden element anyway — waits until its panel is shown.
  ui.results.replaceChildren(bar, ...viewers.map((entry) => entry.node))

  // The same file if it is still there, else the same format, else the first thing
  // written.
  const wanted =
    viewers.find((entry) => same(entry, showing)) ??
    viewers.find((entry) => entry.format === showing?.format) ??
    viewers[0]
  select(viewers, wanted.key)
}

function same(entry, showing) {
  return entry.format === showing?.format && entry.title === showing?.title
}

/// One view: the button that chooses it and the panel it shows.
///
/// The key is the view's position in the answer rather than its format, because two
/// exports of one format are two views — and a format used as an identity would make
/// them one tab that shows two panels.
function viewer(view, index) {
  const body = view.error
    ? { node: element('p', { class: 'bad' }, `${view.format} would not draw: ${view.error}`) }
    : view.svg
      ? figure(view.svg)
      : mapOf(view.title)

  const key = String(index)
  const node = element(
    'section',
    {
      class: 'panel',
      id: `panel-${key}`,
      role: 'tabpanel',
      'aria-labelledby': `tab-${key}`,
      tabindex: '0',
    },
    [
      element('header', {}, [
        element('h2', {}, view.title),
        // Whatever the picture or the driver says about this export. The page has no
        // opinion of its own about any format, so there is nothing to write here.
        element('p', { class: 'note' }, body.note ?? view.note ?? ''),
      ]),
      element('div', { class: 'chips' }, view.files.map(chip)),
      body.node,
    ],
  )

  const tab = element(
    'button',
    {
      class: 'tab',
      type: 'button',
      role: 'tab',
      id: `tab-${key}`,
      'aria-controls': `panel-${key}`,
      // Two exports of one format are two tabs with one label; the file each was
      // drawn from is what tells them apart.
      title: view.title,
    },
    view.format,
  )
  return { key, format: view.format, title: view.title, tab, node, shown: body.shown }
}

function listen(viewers) {
  for (const [index, entry] of viewers.entries()) {
    entry.tab.addEventListener('click', () => select(viewers, entry.key))
    // A tab strip is one stop in the tab order and the arrow keys move within it,
    // which is what a screen reader and a keyboard both expect of one.
    entry.tab.addEventListener('keydown', (event) => {
      const wanted = {
        ArrowRight: index + 1,
        ArrowLeft: index - 1,
        Home: 0,
        End: viewers.length - 1,
      }[event.key]
      if (wanted === undefined) return
      event.preventDefault()
      const next = viewers[Math.min(viewers.length - 1, Math.max(0, wanted))]
      select(viewers, next.key)
      next.tab.focus()
    })
  }
}

/// Shows one view and hides the rest. The only writer of "which one is showing".
function select(viewers, key) {
  for (const entry of viewers) {
    const chosen = entry.key === key
    if (chosen) showing = { format: entry.format, title: entry.title }
    entry.tab.setAttribute('aria-selected', String(chosen))
    entry.tab.tabIndex = chosen ? 0 : -1
    entry.node.hidden = !chosen
    // Leaflet measured a hidden element as nothing at all, so a map is built and
    // refitted the first time its panel is shown.
    if (chosen) entry.shown?.()
  }
}

/// The drawing, and the `<desc>` it carries: what the reader found in the file, and
/// what the format could not hold. Both are written by the crate that drew the
/// picture, so the page says what a notebook says.
function figure(svg) {
  const holder = element('div', { class: 'figure' })
  // The SVG comes from the wheel this page shipped with, not from anything a visitor
  // supplied, and it has to become live elements to be styled by the page's custom
  // properties.
  holder.innerHTML = svg
  const description = holder.querySelector('desc')
  return { node: holder, note: description ? description.textContent : '' }
}

/// An OSM export, drawn by Leaflet from the file itself.
///
/// The map is built the first time its panel is shown, which is also the first time
/// Leaflet can measure the element it is given.
function mapOf(path) {
  const holder = element('div', { class: 'map' })
  let fit = null
  return {
    node: holder,
    shown: () => {
      if (!fit) fit = draw(holder, path)
      fit()
    },
  }
}

function draw(holder, path) {
  const xml = new TextDecoder().decode(read(path))
  const geojson = onlyWays(osmtogeojson(new DOMParser().parseFromString(xml, 'text/xml')))

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

  // Called again every time the panel is shown: until then the map has been measuring
  // an element of no size.
  return () => {
    map.invalidateSize()
    const bounds = layer.getBounds()
    if (bounds.isValid()) {
      map.fitBounds(bounds, { padding: [24, 24] })
    } else {
      map.setView([0, 0], 2)
    }
  }
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

/// One written file: what it is called, how big it is, and the two things you can do
/// with it.
function chip({ path, size, text }) {
  // Read and decoded once. A multi-megabyte export re-read on every click is a hitch
  // on a button that should feel instant.
  let source = null

  const show = element('button', { type: 'button', 'aria-pressed': 'false' }, 'source')
  show.addEventListener('click', () => {
    const panel = show.closest('.panel')
    const open = show.getAttribute('aria-pressed') === 'true'
    // One source at a time: the panel has room for one, and two would be a pile.
    for (const other of panel.querySelectorAll('.chip button[aria-pressed="true"]')) {
      other.setAttribute('aria-pressed', 'false')
    }
    for (const shown of panel.querySelectorAll('.source')) shown.hidden = true
    if (open) return

    if (!source) {
      source = element(
        'pre',
        { class: 'source' },
        text
          ? new TextDecoder().decode(read(path))
          : `${path} is ${bytes(size)} of binary. Save it and open it with a reader for ` +
              'the format.',
      )
      panel.append(source)
    }
    source.hidden = false
    show.setAttribute('aria-pressed', 'true')
  })

  const save = element('button', { type: 'button' }, 'save')
  save.addEventListener('click', () => download(path))

  return element('span', { class: 'chip' }, [
    element('span', {}, path),
    element('span', { class: 'size' }, size === undefined ? '' : bytes(size)),
    show,
    save,
  ])
}

function read(path) {
  return pyodide.FS.readFile(`${work}/${path}`)
}

function download(path) {
  const url = URL.createObjectURL(new Blob([read(path)], { type: 'application/octet-stream' }))
  const link = element('a', { href: url, download: path.split('/').pop() })
  document.body.append(link)
  link.click()
  link.remove()
  setTimeout(() => URL.revokeObjectURL(url), 1000)
}

function bytes(count) {
  if (count < 1024) return `${count} B`
  if (count < 1024 * 1024) return `${(count / 1024).toFixed(1)} kB`
  return `${(count / 1024 / 1024).toFixed(1)} MB`
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
