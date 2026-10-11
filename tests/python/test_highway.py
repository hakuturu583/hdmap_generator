"""Motorways: bridges, tunnels and viaducts, the corridor that plans them, and a
motorway meeting a town."""

import pytest

import roadgen


def street_lanes():
    return [
        roadgen.Lane(width=3.5, direction="backward"),
        roadgen.Lane(width=2.5, direction="backward", type_="sidewalk"),
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=2.5, direction="forward", type_="sidewalk"),
    ]


def two_way():
    return [
        roadgen.Lane(width=3.5, direction="forward"),
        roadgen.Lane(width=3.5, direction="backward"),
    ]


def test_a_motorway_bridged_over_a_street_is_a_grade_separation():
    m = roadgen.Map(name="crossing")
    m.add_road(start=(-300.0, 0.0, 9.0), end=(300.0, 0.0, 9.0), lanes=two_way(),
               name="motorway", type_="motorway", form="motorway")
    m.add_road(start=(0.0, -200.0, 0.0), end=(0.0, 200.0, 0.0), lanes=two_way(), name="street")
    m.add_bridge([("motorway", 200.0, 400.0)], form="viaduct", name="Street Viaduct")
    m.validate()
    assert m.structure_ids() == ["structure/street_viaduct"]
    bridge = m.structure("structure/street_viaduct")
    assert bridge["form"] == "viaduct"
    assert [s["kind"] for s in bridge["supports"]].count("abutment") == 2
    assert m.road_form("motorway") == "motorway"
    assert m.road_structures("motorway") == [(200.0, 400.0, "structure/street_viaduct")]
    [separation] = m.grade_separations()
    assert separation["under"][0] == "road/street"


def test_a_motorway_crossing_a_street_on_the_level_is_refused():
    m = roadgen.Map()
    m.add_road(start=(-300.0, 0.0, 0.0), end=(300.0, 0.0, 0.0), lanes=two_way(),
               name="motorway", form="motorway")
    m.add_road(start=(0.0, -200.0, 0.0), end=(0.0, 200.0, 0.0), lanes=two_way(), name="street")
    with pytest.raises(ValueError, match="crosses nothing on the level"):
        m.validate()


def test_two_streets_crossing_on_the_level_are_still_a_map():
    m = roadgen.Map()
    m.add_road(start=(-100.0, 0.0, 0.0), end=(100.0, 0.0, 0.0), lanes=two_way(), name="a")
    m.add_road(start=(0.0, -100.0, 0.0), end=(0.0, 100.0, 0.0), lanes=two_way(), name="b")
    m.validate()


def test_a_tunnel_has_portals_and_the_hill_covers_it():
    m = roadgen.Map(name="through")
    m.add_road(start=(0.0, 0.0, 0.0), end=(1200.0, 0.0, 0.0), lanes=two_way(), name="road")
    m.add_tunnel([("road", 400.0, 800.0)], form="bored", name="Kita Tunnel")
    m.add_hill(crest=[(600.0, 0.0, 0.0)], width=900.0, height=60.0, name="Kita")
    m.validate()
    tunnel = m.structure("structure/kita_tunnel")
    assert [p[1] for p in tunnel["portals"]] == [400.0, 800.0]
    assert m.terrain_height(600.0, 0.0) > 5.6


def urban_corridor(m, **extra):
    for name, x in (("first", -100.0), ("second", 200.0)):
        m.add_road(start=(x, -300.0, 0.0), end=(x, 300.0, 0.0), lanes=street_lanes(), name=name)
    corridor = m.add_corridor("Shuto", [(-800.0, 0.0, 0.0), (1300.0, 0.0, 0.0)],
                              design_speed=60.0, **extra)
    return corridor


def test_an_elevated_corridor_crosses_a_town_on_viaducts():
    m = roadgen.Map(name="shuto")
    corridor = urban_corridor(m, elevated=[(200.0, 1400.0)])
    corridor.add_ramp("West Off", at=150.0, end=(-450.0, -70.0, 0.0), side="right")
    corridor.add_interchange("East", at=1700.0, crossroad=[(900.0, -250.0, 0.0), (900.0, 250.0, 0.0)],
                             crossroad_lanes=street_lanes(), kind="half_diamond", side="left",
                             control="yield", terminal=60.0, reach=180.0)
    plan = corridor.plan()
    assert plan.notes == []
    names = [road["name"] for road in plan.roads]
    assert "west_off" in names and "east_left_off" in names
    # The elevated stretch is a viaduct on each carriageway, said in the plan.
    assert [form for form, _ in plan.structures].count("viaduct") == 2
    m.add_plan(plan)
    m.generate_buildings()
    m.validate()
    assert m.generation_notes() == []
    forms = {m.structure(s)["form"] for s in m.structure_ids()}
    assert {"viaduct", "overpass"} <= forms
    under = [g["under"][0] for g in m.grade_separations() if isinstance(g["under"], tuple)]
    assert under.count("road/first") == 2 and under.count("road/second") == 2
    assert m.road_form("west_off") == "slip_road"
    [interchange] = m.interchange_ids()
    assert m.interchange(interchange)["kind"] == "half_diamond"
    assert m.site("site/shuto")["kind"] == "motorway"
    # The town never faces the motorway.
    for building in m.building_ids():
        road = m.building_frontage(building)[0]
        assert m.road_form(road) not in ("motorway", "slip_road")


def test_an_elevated_corridor_on_a_shared_deck_is_one_viaduct_on_piers_down_the_median():
    m = roadgen.Map(name="shuto")
    corridor = urban_corridor(m, elevated=[(200.0, 1400.0)], shared_deck=True)
    plan = corridor.plan()
    # Both carriageways on one viaduct, said in the plan.
    [(form, spans)] = [s for s in plan.structures if s[0] == "viaduct"]
    assert sorted(road for road, _, _ in spans) == ["shuto_left_0", "shuto_right_0"]
    m.add_plan(plan)
    m.validate()
    assert m.generation_notes() == []
    [viaduct] = [s for s in m.structure_ids() if m.structure(s)["form"] == "viaduct"]
    deck = m.structure(viaduct)
    assert len(deck["carries"]) == 2
    piers = [s for s in deck["supports"] if s["kind"] == "pier"]
    assert piers
    # The median is 3 m at 60 km/h: every pier stands within it.
    for pier in piers:
        assert all(abs(y) <= 1.5 for _, y, _ in pier["footprint"])


def test_a_sunken_corridor_runs_under_the_town():
    m = roadgen.Map(name="shuto")
    corridor = urban_corridor(m, sunken=[(200.0, 1400.0)])
    m.add_plan(corridor.plan())
    m.validate()
    forms = [m.structure(s)["form"] for s in m.structure_ids()]
    assert forms.count("cut_and_cover") == 2


def test_an_urban_corridor_finds_the_town_and_crosses_it_on_viaducts():
    m = roadgen.Map(name="shuto")
    corridor = urban_corridor(m, urban="elevated")
    corridor.add_ramp("West Off", at=150.0, end=(-450.0, -70.0, 0.0), side="right")
    corridor.add_interchange("East", at=1700.0, crossroad=[(900.0, -250.0, 0.0), (900.0, 250.0, 0.0)],
                             crossroad_lanes=street_lanes(), kind="half_diamond", side="left",
                             control="yield", terminal=60.0, reach=180.0)
    plan = corridor.plan()
    assert plan.notes == []
    # No stretch given by hand: the viaducts are where the streets are.
    assert [form for form, _ in plan.structures].count("viaduct") == 2
    m.add_plan(plan)
    m.validate()
    assert m.generation_notes() == []
    under = [g["under"][0] for g in m.grade_separations() if isinstance(g["under"], tuple)]
    assert under.count("road/first") == 2 and under.count("road/second") == 2
    [interchange] = m.interchange_ids()
    assert m.interchange(interchange)["kind"] == "half_diamond"


def test_an_urban_corridor_can_be_sunk_under_the_town_instead():
    m = roadgen.Map(name="shuto")
    corridor = urban_corridor(m, urban="sunken")
    m.add_plan(corridor.plan())
    m.validate()
    forms = [m.structure(s)["form"] for s in m.structure_ids()]
    assert forms.count("cut_and_cover") == 2 and "viaduct" not in forms


def test_an_urban_corridor_is_elevated_or_sunken():
    m = roadgen.Map()
    with pytest.raises(ValueError, match="elevated or sunken"):
        m.add_corridor("Shuto", [(0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)], urban="floating")


def test_a_plan_is_put_on_the_map_whole_or_not_at_all():
    m = roadgen.Map(name="shuto")
    corridor = urban_corridor(m)
    plan = corridor.plan()
    m.add_plan(plan)
    with pytest.raises(ValueError, match="already"):
        m.add_plan(plan)


def test_a_corridor_motorway_with_a_river_a_ridge_and_a_diamond():
    m = roadgen.Map(name="expressway", handedness="lht")
    m.add_river(centre=[(-1450.0, -600.0, 0.0), (-1350.0, 600.0, 0.0)], width=80.0, depth=6.0,
                name="Ara River")
    m.add_hill(crest=[(1650.0, 1350.0, 0.0), (2350.0, -650.0, 0.0)], width=700.0, height=50.0,
               name="Kita Ridge")
    e1 = m.add_corridor(
        "E1", [(-2000.0, 0.0, 0.0), (1000.0, 0.0, 0.0), (3000.0, 700.0, 0.0)],
        design_speed=100.0,
        profile=[(0.0, 0.0), (500.0, 10.0), (800.0, 10.0), (1250.0, 1.0), (4300.0, 4.0), (4900.0, 0.0)],
        end_transition=400.0,
    )
    e1.add_interchange("Kita IC", at=1700.0, crossroad=[(-300.0, -450.0, 0.0), (-300.0, 450.0, 0.0)],
                       crossroad_lanes=street_lanes(), terminal=80.0, reach=320.0)
    assert e1.standard["min_radius"] == 460.0
    m.add_plan(e1.plan())
    m.validate()
    forms = sorted(m.structure(s)["form"] for s in m.structure_ids())
    assert forms.count("bored") == 2
    assert forms.count("bridge") == 2
    assert forms.count("overpass") == 1
    rivers = [g for g in m.grade_separations() if g["under"] == "landcover/land/ara_river"]
    assert len(rivers) == 2
    for structure in m.structure_ids():
        for support in m.structure(structure).get("supports", []):
            assert support["base"] is not None


@pytest.mark.parametrize("kind,side,loops", [("parclo", None, 2), ("trumpet", "right", 1)])
def test_a_parclo_and_a_trumpet_turn_on_by_loops(kind, side, loops):
    m = roadgen.Map(name="loops")
    e5 = m.add_corridor("E5", [(-1500.0, 0.0, 0.0), (1500.0, 0.0, 0.0)], design_speed=100.0)
    e5.add_interchange("Minami IC", at=1500.0, crossroad=[(0.0, -500.0, 0.0), (0.0, 500.0, 0.0)],
                       crossroad_lanes=street_lanes(), kind=kind, side=side, terminal=100.0)
    plan = e5.plan()
    assert plan.notes == []
    m.add_plan(plan)
    m.validate()
    names = [road["name"] for road in plan.roads]
    assert sum(name.endswith("_loop") for name in names) == loops
    [interchange] = m.interchange_ids()
    assert m.interchange(interchange)["kind"] == kind


@pytest.mark.parametrize("kind,side,slip_roads", [
    ("trumpet", "right", 4),
    # Four slip roads a side, and a collector-distributor road in five pieces.
    ("cloverleaf", None, 18),
])
def test_a_trumpet_and_a_cloverleaf_flow_freely(kind, side, slip_roads):
    m = roadgen.Map(name="free")
    e5 = m.add_corridor("E5", [(-1500.0, 0.0, 0.0), (1500.0, 0.0, 0.0)], design_speed=100.0)
    e5.add_interchange("Minami IC", at=1500.0, crossroad=[(0.0, -500.0, 0.0), (0.0, 500.0, 0.0)],
                       crossroad_lanes=street_lanes(), kind=kind, side=side)
    plan = e5.plan()
    assert plan.notes == []
    assert sum(road["form"] == "slip_road" for road in plan.roads) == slip_roads
    m.add_plan(plan)
    m.validate()
    [interchange] = m.interchange_ids()
    assert m.interchange(interchange)["kind"] == kind
    assert [s["form"] for s in map(m.structure, m.structure_ids())].count("overpass") == 1


def test_a_directional_t_ends_another_motorway_on_flyovers():
    m = roadgen.Map(name="junction")
    e5 = m.add_corridor("E5", [(-1500.0, 0.0, 0.0), (1500.0, 0.0, 0.0)], design_speed=100.0)
    e5.add_interchange("Minami JCT", at=1500.0, crossroad=[(0.0, 700.0, 0.0), (0.0, -20.0, 0.0)],
                       crossroad_lanes=street_lanes(), kind="directional", side="left")
    plan = e5.plan()
    assert plan.notes == []
    forms = {road["name"]: road["form"] for road in plan.roads}
    assert forms["minami_jct_branch_in"] == "motorway"
    assert sum(form == "slip_road" for form in forms.values()) == 4
    assert [form for form, _ in plan.structures].count("viaduct") == 2
    m.add_plan(plan)
    m.validate()
    [interchange] = m.interchange_ids()
    assert m.interchange(interchange)["kind"] == "directional"
    with pytest.raises(ValueError, match="side="):
        e5.add_interchange("T", at=1500.0, crossroad=[(0.0, 700.0, 0.0), (0.0, -20.0, 0.0)],
                           crossroad_lanes=street_lanes(), kind="directional")


def test_a_trumpet_says_which_side_it_stops_on():
    corridor = roadgen.Map().add_corridor("E5", [(0.0, 0.0, 0.0), (3000.0, 0.0, 0.0)])
    with pytest.raises(ValueError, match="side="):
        corridor.add_interchange("T", at=1500.0, crossroad=[(1500.0, -500.0, 0.0), (1500.0, 500.0, 0.0)],
                                 crossroad_lanes=street_lanes(), kind="trumpet")


def ordinary_road_across_a_valley_and_through_a_hill(**extra):
    m = roadgen.Map(name="pass")
    m.add_river(centre=[(0.0, -400.0, 0.0), (0.0, 400.0, 0.0)], width=60.0, depth=6.0,
                name="Naka")
    m.add_hill(crest=[(1600.0, 0.0, 0.0)], width=900.0, height=60.0, name="Kobu")
    m.add_road(points=[(-600.0, 0.0, 0.0), (-200.0, 0.0, 12.0), (200.0, 0.0, 12.0),
                       (600.0, 0.0, 0.0), (2400.0, 0.0, 0.0)],
               lanes=street_lanes(), name="pass", **extra)
    return m


def test_an_ordinary_road_asked_to_have_its_structures_chosen_bridges_and_tunnels():
    m = ordinary_road_across_a_valley_and_through_a_hill(structures="auto")
    m.validate()
    assert m.generation_notes() == []
    forms = sorted(m.structure(s)["form"] for s in m.structure_ids())
    assert forms == ["bored", "bridge"]
    [tunnel] = [s for s in m.structure_ids() if m.structure(s)["form"] == "bored"]
    portals = [p[1] for p in m.structure(tunnel)["portals"]]
    # The hill stands about 2200 m along the road.
    assert 1750.0 < portals[0] < portals[1] < 2650.0
    assert any(g["under"] == "landcover/land/naka" for g in m.grade_separations())
    assert m.road_form("pass") != "motorway"
    kinds = {m.land_cover(c)["kind"] for c in m.land_cover_ids()}
    assert {"embankment", "cutting"} <= kinds
    assert m.site_ids() == []


def test_an_ordinary_road_not_asked_to_have_its_structures_chosen_gets_none():
    m = ordinary_road_across_a_valley_and_through_a_hill()
    m.validate()
    assert m.structure_ids() == []


def test_a_corridors_road_has_its_structures_chosen_by_its_corridor():
    m = roadgen.Map()
    with pytest.raises(ValueError, match="by its corridor"):
        m.add_road(start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0), lanes=two_way(),
                   corridor="E1", structures="auto")
    with pytest.raises(ValueError, match="structures is auto"):
        m.add_road(start=(0.0, 0.0, 0.0), end=(100.0, 0.0, 0.0), lanes=two_way(),
                   structures="always")
