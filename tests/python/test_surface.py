"""Surface features from Python: ruts, potholes, puddles, humps and roughness on a
road, written as OpenDRIVE with OpenCRG files beside it, and kept through the IR's
own Protocol Buffers document."""

import roadgen


def worn_road():
    m = roadgen.Map(name="worn")
    road = m.add_road(
        start=(0.0, 0.0, 0.0),
        end=(100.0, 0.0, 0.0),
        name="main",
        lanes=[
            roadgen.Lane(width=3.5, direction="forward"),
            roadgen.Lane(width=3.5, direction="backward"),
        ],
    )
    ids = [
        m.add_surface_feature(road, "rut", station=5.0, length=30.0, offset=-1.75),
        m.add_surface_feature(
            road, "pothole", station=40.0, length=0.8, offset=1.5, width=0.6, depth=0.06
        ),
        m.add_surface_feature(
            "main", "puddle", station=50.0, length=2.0, offset=-1.75, width=1.2
        ),
        m.add_surface_feature(
            road, "bump", station=70.0, length=3.7, width=7.0, bump="hump", height=0.08
        ),
        m.add_surface_feature(
            road,
            "roughness",
            station=80.0,
            length=15.0,
            width=7.0,
            roughness_class="D",
            seed=3,
        ),
    ]
    return m, ids


def test_features_are_kept_and_measured():
    m, ids = worn_road()
    assert ids[0] == "surface/main/rut/0"
    assert [f[0] for f in m.surface_features()] == ids
    rut = m.surface_features()[0]
    assert abs(rut[6] - 2.4) < 1e-12  # track + trough
    depth, water = m.surface_displacement("road/main", 40.4, 1.5)
    assert abs(depth + 0.06) < 1e-9 and water == 0.0
    level, water = m.surface_displacement("road/main", 51.0, -1.75)
    assert level < 0.0 and abs(water + level) < 1e-12
    crest, _ = m.surface_displacement("road/main", 71.85, 0.0)
    assert abs(crest - 0.08) < 1e-9
    assert any("surface features" in w for w in m.format_warnings())


def test_opendrive_with_opencrg(tmp_path):
    m, _ = worn_road()
    path = tmp_path / "map.xodr"
    m.export_opendrive(str(path), crg=True, crg_increment=0.1)
    xml = path.read_text()
    assert 'mode="attached"' in xml and 'file="main.elevation.crg"' in xml
    elevation = (tmp_path / "main.elevation.crg").read_text()
    assert elevation.startswith("$CT") and "#:LRFI" in elevation
    assert (tmp_path / "main.friction.crg").exists()
    # And without asking, no grid.
    plain = tmp_path / "plain.xodr"
    m.export_opendrive(str(plain))
    assert "<CRG" not in plain.read_text()


def test_features_survive_the_ir_document(tmp_path):
    m, ids = worn_road()
    path = tmp_path / "map.roadgen.pb"
    m.export_proto(str(path))
    back = roadgen.read_proto(str(path))
    assert [f[0] for f in back.surface_features()] == ids
    assert back.surface_displacement("road/main", 40.4, 1.5) == m.surface_displacement(
        "road/main", 40.4, 1.5
    )
