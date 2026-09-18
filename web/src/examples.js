// The scripts the page starts you off with.
//
// Every one of them exports every format. That is not for completeness: the point of
// the page is that one description becomes six files, so an example that wrote only
// one would be hiding the thing it is meant to show.

export const examples = [
  {
    name: 'Two roads, joined',
    description: 'The example from the README: a bend and a climb.',
    code: `# The map from roadgen's README: two roads, joined end to start,
# with a bend and a climb.
import roadgen

def two_way():
    return [
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="backward"),
    ]

# The origin is what puts the map somewhere on Earth. Without it the
# Lanelet2 and OpenStreetMap exports come out in the Gulf of Guinea.
m = roadgen.Map(name="joined", origin=(35.6586, 139.7454, 0.0))

a = m.add_road(start=(0.0, 0.0, 10.0), end=(100.0, 0.0, 12.0),
               lanes=two_way(), name="a")
b = m.add_road(start=(100.0, 0.0, 12.0), end=(200.0, 50.0, 15.0),
               lanes=two_way(), name="b")
m.connect(a, b)

m.validate()
for issue in m.issues():
    print("issue:", issue)

m.export_opendrive("map.xodr")
m.export_lanelet2("lanelet2.osm")
m.export_osm("openstreetmap.osm")
m.export_sumo("sumo/")
m.export_clipgt("clip/")
m.export_gpudrive("scene.json")

print("lanes:", ", ".join(m.lane_ids()))
`,
  },
  {
    name: 'A signalised crossroads',
    description: 'Four approaches, every turn, a light and a crossing.',
    code: `# A four-way crossroads: four two-way approaches meeting at a junction,
# with a stop line, a traffic light that governs it, and a crossing.
import roadgen

def two_way():
    return [
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="backward"),
    ]

m = roadgen.Map(name="demo_town", origin=(35.6586, 139.7454, 0.0))

# Each arm points at the middle, stopping short of it: the junction is the
# gap, and the connectors roadgen generates are what fill it.
north = m.add_road(start=(0.0, 70.0, 0.0), end=(0.0, 14.0, 0.0),
                   lanes=two_way(), name="north")
south = m.add_road(start=(0.0, -14.0, 0.0), end=(0.0, -70.0, 0.0),
                   lanes=two_way(), name="south")
east = m.add_road(start=(14.0, 0.0, 0.0), end=(70.0, 0.0, 0.0),
                  lanes=two_way(), name="east")
west = m.add_road(start=(-70.0, 0.0, 0.0), end=(-14.0, 0.0, 0.0),
                  lanes=two_way(), name="west")

x = m.add_junction(name="x")
m.connect(north, south, junction=x)   # straight on
m.connect(north, east, junction=x)    # and the left turn
m.connect(west, east, junction=x)
m.connect(west, south, junction=x)

stop = m.add_stop_line(north.lane(0))
light = m.add_traffic_light(north.lane(0))
m.add_traffic_light_rule([light], [north.lane(0)], stop_line=stop)
m.add_crosswalk(east, fraction=0.15)

m.validate()
print(len(m.lane_ids()), "lanes, of which",
      sum(1 for lane in m.lane_ids() if "/x/" in lane), "are through the junction")

m.export_opendrive("map.xodr")
m.export_lanelet2("lanelet2.osm")
m.export_osm("openstreetmap.osm")
m.export_sumo("sumo/")
m.export_clipgt("clip/")
m.export_gpudrive("scene.json")
`,
  },
  {
    name: 'A bend, entered properly',
    description: 'Straight, clothoid, arc, clothoid, straight — and banked.',
    code: `# A road that enters its bend the way a real one does: a clothoid
# transition either side of the arc, so the curvature ramps rather than
# steps. These reach OpenDRIVE as <spiral> elements.
import roadgen

al = roadgen.Alignment(start=(0.0, 0.0, 4.0), heading=0.0)
al.line(80.0, rise=1.0)
al.spiral(60.0, curvature_end=1 / 120, rise=1.0)
al.arc(140.0, curvature=1 / 120, rise=2.0)
al.spiral(60.0, curvature_end=0.0, rise=1.0)
al.line(80.0, rise=1.0)

m = roadgen.Map(name="sweep", origin=(35.6586, 139.7454, 0.0), sampling=1.0)
m.add_road(
    lanes=[
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="forward", left_marking="broken"),
        roadgen.Lane(width=3.5, direction="backward"),
    ],
    alignment=al,
    name="sweep",
    # Flat on the approach, rolled through the bend, flat again on the way out.
    superelevation=[(0.0, 0.0), (80.0, 0.0), (170.0, -0.06),
                    (280.0, -0.06), (360.0, 0.0)],
)

m.validate()

m.export_opendrive("map.xodr")
m.export_lanelet2("lanelet2.osm")
m.export_osm("openstreetmap.osm")
m.export_sumo("sumo/")
m.export_clipgt("clip/")
m.export_gpudrive("scene.json")

# The pictures are plan views, so the climb and the banking are in the files
# rather than on the screen. This is where they went:
print("the road rises", 4.0, "->", round(al.point[2], 1), "metres")
`,
  },
  {
    name: 'A lane that ends',
    description: 'Three lanes tapering to two, as a second cross-section.',
    code: `# Two different things at once. The outer lane *narrows*, which is a
# width profile — it stays one lane while it does it. Then it *ends*,
# which is a new cross-section, because past that point the road has one
# lane fewer.
import roadgen

left = roadgen.Lane(width=3.5, direction="forward")
middle = roadgen.Lane(width=3.5, direction="forward", left_marking="broken")
outer = roadgen.Lane(
    width=3.5,
    direction="forward",
    left_marking="broken",
    width_profile=[(0.0, 3.5), (120.0, 3.5), (200.0, 2.2)],
    taper="smooth",
)

m = roadgen.Map(name="lane_drop", origin=(35.6586, 139.7454, 0.0))
m.add_road(
    lanes=[left, middle, outer],
    cross_sections=[(200.0, [left, middle])],
    start=(0.0, 0.0, 0.0),
    end=(300.0, 0.0, 0.0),
    name="trunk",
    type_="motorway",
)

m.validate()

m.export_opendrive("map.xodr")
m.export_lanelet2("lanelet2.osm")
m.export_osm("openstreetmap.osm")
m.export_sumo("sumo/")
m.export_clipgt("clip/")
m.export_gpudrive("scene.json")

for warning in m.format_warnings():
    print("note:", warning)
`,
  },
]
