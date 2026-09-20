//! Street furniture: traffic lights and signs, as meshes, and where each one stands.
//!
//! The IR holds a traffic light as a bar over a lane at a height, and a sign as the
//! same bar lower down. That is what a light *means*. What it *is* — a pole on the
//! pavement, a mast arm reaching over the carriageway, a head hung from the arm
//! over each lane it governs — has to be built, and built to the cross-section,
//! because the length of the arm is the distance from where a pole can stand to
//! where the lanes are, and no two roads agree on it.
//!
//! # Why this is not just meshes
//!
//! CARLA spawns traffic lights of its own. `ATrafficLightManager` reads every
//! `<signal>` in the `.xodr` and, for each light, puts its `BP_TLOpenDrive` at the
//! signal's position — which, for a signal written where the IR put it, is five
//! metres over the middle of the lane. A pole in the carriageway. And a light mesh
//! of our own in the map's FBX would be dropped before it was placed, because
//! `ValidateStaticMesh` rejects any mesh whose name holds `light` or `sign`.
//!
//! So a light is three things that have to agree, and this module makes all three:
//!
//! 1. **A prop.** The mesh goes into a props FBX, declared in the descriptor with
//!    the tag `TrafficLight`, so it imports into `/Game/<Package>/Static/TrafficLight/`
//!    — which is what makes a segmentation camera report it as one — and is placed
//!    in the level by the package's script rather than by CARLA's importer, at the
//!    foot of the pole, facing the traffic.
//! 2. **A signal.** The `.xodr` signal is written with the pole's position as its
//!    `<positionInertial>` (see [`roadgen_opendrive::SignalPlacement`]), so CARLA's
//!    own idea of where the signal is *is* the foot of the pole, to the centimetre.
//! 3. **A `map_logic.json`.** Beside the `.xodr` — carried in the package as
//!    `map_logic.carla`, since `Import.py` would take a `.json` for a package, and
//!    copied into place by the script. CARLA's `InitializeTrafficLights` finds it
//!    and takes the other branch: it spawns no lights of its own, and
//!    `UMapLogicParser::ApplyLaneIdsFromMapLogic` turns whatever actor stands within
//!    fifty centimetres of each listed signal into an `ADigitalTwinsTrafficLight` —
//!    a working light, with the signal's id, in the junction's controller, with the
//!    stop boxes built from the signal's lanes, and with its lamps driven through
//!    any material named `TrafficLight` that has an `Emissive Intensity`. The
//!    package's script gives the lamps such a material.
//!
//! That conversion copies the actor's mesh components into a new actor and
//! destroys the old one, and the copies are made after CARLA has tagged the level,
//! so they segment as nothing. Hence a light is **two meshes**: the post — pole, arm
//! and the heads' housings — which stands in the level untouched and segments as a
//! traffic light, and the three lamps of each head, a small mesh of their own that
//! is the actor CARLA finds and converts. Only the lamps go unlabelled, and only
//! the lamps need to be part of the light that switches.
//!
//! The signs work the same way as far as CARLA lets them. There is no
//! `map_logic.json` for signs; CARLA's `SpawnSignals` looks for an
//! `ATrafficSignBase` within five metres whose state matches the signal's type and
//! attaches its sign component to that, and spawns its own blueprint only when it
//! finds none. So the script places, at the foot of each sign it knows the CARLA
//! state for, a bare `ATrafficSignBase` in that state, and CARLA adopts it.
//!
//! # Where a pole stands
//!
//! On the pavement, [`FurnitureConfig::kerb_setback`] in from the kerb, on the side
//! the governed traffic keeps to — the driver's right in right-hand traffic. A road
//! with no pavement puts it that far out on the verge instead. The arm runs from
//! there, perpendicular to the road, to [`FurnitureConfig::arm_overhang`] past the
//! middle of the farthest lane the light governs, at the height the IR gave the bar;
//! a head hangs under it over the middle of every governed lane.
//!
//! A sign's post stands [`FurnitureConfig::sign_setback_along`] before the line it
//! applies to, and moves further back if another post is already within
//! [`FurnitureConfig::clearance`] of it — which is also what keeps a stop sign's post
//! out of the fifty centimetres CARLA searches around a light on the same corner.

use std::f64::consts::{PI, TAU};

use roadgen_core::geometry::{Point3, Vector3};
use roadgen_core::id::{LaneId, ObjectId, RoadId};
use roadgen_core::map::{Map, Road};
use roadgen_core::semantics::{MapObject, MapObjectKind, ObjectGeometry, TrafficRule};
use roadgen_core::topology::{Direction, LateralSide};
use roadgen_opendrive::road_coordinates;
use roadgen_opendrive::{SignalCatalogue, SignalPlacement};

use crate::materials;
use crate::mesh::Mesh;
use crate::surfaces::{Layout, Ordinals, SurfaceConfig};
use crate::tags::{mesh_name, Role};

/// How furniture is built. Every distance is in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FurnitureConfig {
    /// How far in from the kerb face a post stands on the pavement — or, with no
    /// pavement, how far out from the carriageway's edge it stands on the verge.
    pub kerb_setback: f64,
    pub pole_radius: f64,
    pub arm_radius: f64,
    /// How far the mast arm reaches past the middle of the farthest governed lane.
    pub arm_overhang: f64,
    /// A head's width across the arm and its depth along the road.
    pub head_width: f64,
    pub head_depth: f64,
    /// The diameter of each of the three lamps; the head is tall enough for them.
    pub lamp_diameter: f64,
    pub sign_post_radius: f64,
    /// The size of a sign's plate: across an octagon or a circle, the side of a
    /// square or a triangle.
    pub plate_size: f64,
    /// How far before the line it applies to a sign's post stands.
    pub sign_setback_along: f64,
    /// The least distance between two posts. A post that would stand closer than
    /// this to one already placed moves back along the road until it does not.
    pub clearance: f64,
    /// The seconds a light spends in each state, written to `map_logic.json` for
    /// CARLA's controllers to run.
    pub timing: Timing,
}

/// How long a light shows each colour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timing {
    pub green: f64,
    pub amber: f64,
    pub red: f64,
}

impl Default for FurnitureConfig {
    fn default() -> Self {
        FurnitureConfig {
            kerb_setback: 0.6,
            pole_radius: 0.14,
            arm_radius: 0.07,
            arm_overhang: 0.5,
            head_width: 0.38,
            head_depth: 0.30,
            lamp_diameter: 0.30,
            sign_post_radius: 0.04,
            plate_size: 0.7,
            sign_setback_along: 2.0,
            clearance: 1.0,
            // CARLA's own defaults, from `FTrafficLightTiming`.
            timing: Timing {
                green: 10.0,
                amber: 3.0,
                red: 2.0,
            },
        }
    }
}

/// What a sign's catalogue code turns out to mean, as far as CARLA is concerned.
///
/// The IR does not define a sign catalogue — a code is whatever the caller wrote —
/// so this recognises the spellings a caller is likely to have used: plain words,
/// the German catalogue (`206`, `de205`, `de274-60`), the MUTCD (`R1-1`), and
/// nothing else. CARLA itself knows three: it is the German catalogue's numbers
/// that `carla::road::SignalType` compares against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignKind {
    Stop,
    Yield,
    /// km/h, as a whole number.
    SpeedLimit(u32),
    /// Anything else: written with the caller's own code, and never one CARLA acts
    /// on.
    Other,
}

impl SignKind {
    pub fn parse(code: &str) -> SignKind {
        let code = code.trim().to_ascii_lowercase().replace([' ', '_'], "-");
        // The German catalogue, with or without a country prefix: `206`, `de206`,
        // `de-274-60`.
        let bare = code
            .strip_prefix("de-")
            .or_else(|| code.strip_prefix("de"))
            .filter(|rest| rest.starts_with(|character: char| character.is_ascii_digit()))
            .unwrap_or(&code);
        let (head, tail) = match bare.split_once('-') {
            Some((head, tail)) => (head, Some(tail)),
            None => (bare, None),
        };
        // The limit is the last number in the tail: `limit-60`, `1-60`.
        let limit =
            tail.and_then(|tail| tail.rsplit('-').find_map(|part| part.parse::<u32>().ok()));
        match (head, tail) {
            ("206" | "stop", _) | ("r1", Some("1")) => SignKind::Stop,
            ("205" | "yield" | "give", _) | ("r1", Some("2")) => SignKind::Yield,
            ("274" | "speed" | "maxspeed" | "limit", _) => {
                limit.map_or(SignKind::Other, SignKind::SpeedLimit)
            }
            ("r2", Some(tail)) if tail.starts_with("1-") => {
                limit.map_or(SignKind::Other, SignKind::SpeedLimit)
            }
            _ => SignKind::Other,
        }
    }

    /// The catalogue entry CARLA recognises, or `None` for a sign it has no word
    /// for, which is written with the caller's own code.
    pub fn catalogue(self) -> Option<SignalCatalogue> {
        let germany = Some("DE".to_owned());
        Some(match self {
            SignKind::Stop => SignalCatalogue {
                country: germany,
                kind: "206".into(),
                subtype: "-1".into(),
                speed_kph: None,
            },
            SignKind::Yield => SignalCatalogue {
                country: germany,
                kind: "205".into(),
                subtype: "-1".into(),
                speed_kph: None,
            },
            SignKind::SpeedLimit(kph) => SignalCatalogue {
                country: germany,
                kind: "274".into(),
                subtype: kph.to_string(),
                speed_kph: Some(f64::from(kph)),
            },
            SignKind::Other => return None,
        })
    }

    /// The `ETrafficSignState` CARLA's `SpawnSignals` will match this sign's signal
    /// against, spelled the way Unreal's Python spells the enumerator — or `None`
    /// for a sign CARLA has no state for, beside which it will spawn nothing.
    ///
    /// Two of these are deliberately wrong. `MatchSignalAndActor` compares a 70
    /// against `SpeedLimit_60` and an 80 against `SpeedLimit_90`, so a marker in the
    /// state CARLA *checks for* is the one it adopts; a marker in the right state
    /// would be ignored and a second plate spawned beside it.
    pub fn carla_state(self) -> Option<&'static str> {
        Some(match self {
            SignKind::Stop => "STOP_SIGN",
            SignKind::Yield => "YIELD_SIGN",
            SignKind::SpeedLimit(30) => "SPEED_LIMIT_30",
            SignKind::SpeedLimit(40) => "SPEED_LIMIT_40",
            SignKind::SpeedLimit(50) => "SPEED_LIMIT_50",
            SignKind::SpeedLimit(60 | 70) => "SPEED_LIMIT_60",
            SignKind::SpeedLimit(80 | 90) => "SPEED_LIMIT_90",
            SignKind::SpeedLimit(100) => "SPEED_LIMIT_100",
            SignKind::SpeedLimit(120) => "SPEED_LIMIT_120",
            SignKind::SpeedLimit(130) => "SPEED_LIMIT_130",
            SignKind::SpeedLimit(_) | SignKind::Other => return None,
        })
    }

    /// Whether CARLA will spawn a plate of its own for this sign whatever we place:
    /// it has a model for the limit but no state to match a marker against.
    pub fn doubled_by_carla(self) -> bool {
        matches!(self, SignKind::SpeedLimit(110))
    }
}

/// One light or sign: its mesh, in its own frame, and where that frame stands.
#[derive(Debug, Clone, PartialEq)]
pub struct Placed {
    pub object: ObjectId,
    pub role: Role,
    /// The mesh, with the foot of the post at the origin, `+x` the way it faces
    /// and `z` up. Placing it is a translation and a yaw.
    pub mesh: Mesh,
    /// A light's lamps, in the same frame, as the mesh CARLA turns into the light
    /// that switches. `None` for a sign.
    pub lamps: Option<Mesh>,
    /// The foot of the post.
    pub position: Point3,
    /// The direction it faces — towards the traffic it governs, which is against
    /// that traffic's travel — radians anticlockwise from east.
    pub heading: f64,
    /// The road the signal is written against.
    pub road: RoadId,
    /// The lanes it governs on that road.
    pub lanes: Vec<LaneId>,
    /// What the sign is; `Other` for a light.
    pub kind: SignKind,
    /// How far the mast arm reaches; zero for a sign.
    pub arm_length: f64,
    /// What the `.xodr` is told.
    pub signal: SignalPlacement,
}

/// Every light and every sign a map has, built and placed.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Furniture {
    pub lights: Vec<Placed>,
    pub signs: Vec<Placed>,
}

impl Furniture {
    pub fn is_empty(&self) -> bool {
        self.lights.is_empty() && self.signs.is_empty()
    }

    /// Everything, lights first, in the order it was placed.
    pub fn iter(&self) -> impl Iterator<Item = &Placed> {
        self.lights.iter().chain(self.signs.iter())
    }
}

/// Builds and places the map's furniture.
pub fn build(
    map: &Map,
    map_name: &str,
    surfaces: &SurfaceConfig,
    config: &FurnitureConfig,
) -> Furniture {
    let mut furniture = Furniture::default();
    let mut ordinals = Ordinals::default();
    let mut posts: Vec<Point3> = Vec::new();
    // Lights first, so that a sign on the same corner is the one that moves.
    for object in map.objects.iter() {
        if object.kind != MapObjectKind::TrafficLight {
            continue;
        }
        if let Some(placed) = light(
            map,
            object,
            map_name,
            surfaces,
            config,
            &mut ordinals,
            &posts,
        ) {
            posts.push(placed.position);
            furniture.lights.push(placed);
        }
    }
    for object in map.objects.iter() {
        let MapObjectKind::TrafficSign { code } = &object.kind else {
            continue;
        };
        let kind = SignKind::parse(code);
        if let Some(placed) = sign(
            map,
            object,
            kind,
            map_name,
            surfaces,
            config,
            &mut ordinals,
            &posts,
        ) {
            posts.push(placed.position);
            furniture.signs.push(placed);
        }
    }
    furniture
}

/// Where an object applies: its road, the station of its bar, the lanes it
/// governs there, and how high the bar is above the road surface.
struct Site<'a> {
    road: &'a Road,
    station: f64,
    lanes: Vec<&'a roadgen_core::map::Lane>,
    /// The bar's height above the road surface, in the road's banked frame.
    height: f64,
    /// The side of the reference line the governed traffic keeps to.
    kerb: LateralSide,
    direction: Direction,
}

fn site<'a>(map: &'a Map, object: &MapObject) -> Option<Site<'a>> {
    let first = map.lanes.get(object.lanes.first()?)?;
    let road = map.road(&first.road)?;
    let lanes: Vec<_> = object
        .lanes
        .iter()
        .filter_map(|id| map.lanes.get(id))
        .filter(|lane| lane.road == road.id)
        .collect();
    let centre = match &object.geometry {
        ObjectGeometry::Point(position) => *position,
        ObjectGeometry::Line(curve) => curve.start_point().lerp(curve.end_point(), 0.5),
        ObjectGeometry::Band { left, right } => left.start_point().lerp(right.end_point(), 0.5),
    };
    let position = road_coordinates::locate(map, road, centre)?;
    Some(Site {
        road,
        station: position.s,
        lanes,
        height: position.height,
        kerb: map.metadata.handedness.side_for(first.direction),
        direction: first.direction,
    })
}

/// The road's frame at `station`, and the direction a signal there faces.
fn frame_and_facing(
    map: &Map,
    site: &Site,
    station: f64,
) -> Option<(roadgen_core::geometry::Frame3, f64)> {
    let frame = site.road.frame_at(station, map.metadata.sampling).ok()?;
    let tangent = frame.tangent.get();
    let along = tangent.y.atan2(tangent.x);
    // Facing the traffic is facing against its travel: a forward lane's traffic
    // comes from behind the frame, so its light looks back down the road.
    let facing = match site.direction {
        Direction::Forward => along + PI,
        Direction::Backward => along,
    };
    Some((frame, wrap(facing)))
}

/// Where a post ends up: its station along the road, its foot, the way it faces
/// and how far above the road plane the foot is.
struct Stand {
    station: f64,
    foot: Point3,
    heading: f64,
    rise: f64,
}

/// A post beside the road at `station`, or as near it as the posts already placed
/// allow: one that would stand within the clearance of another moves back along
/// the road, against the traffic, until it is clear or the road runs out.
fn stand(
    map: &Map,
    site: &Site,
    surfaces: &SurfaceConfig,
    config: &FurnitureConfig,
    station: f64,
    posts: &[Point3],
) -> Option<Stand> {
    let length = site.road.horizontal_length().ok()?;
    let back = match site.direction {
        Direction::Forward => -1.0,
        Direction::Backward => 1.0,
    };
    let mut wanted = station;
    let mut found = None;
    for _ in 0..8 {
        let station = wanted.clamp(0.0, length);
        let section = site.road.section_at(station)?;
        let layout = Layout::of(map, site.road, section, surfaces)?;
        let (lateral, rise) = layout.post_position(station, site.kerb, config.kerb_setback)?;
        let (frame, heading) = frame_and_facing(map, site, station)?;
        let foot = frame.to_global([0.0, lateral, rise]);
        let clear = posts
            .iter()
            .all(|post| horizontal_distance(*post, foot) >= config.clearance);
        let at_end = (station - wanted).abs() > 1e-9;
        found = Some(Stand {
            station,
            foot,
            heading,
            rise,
        });
        if clear || at_end {
            break;
        }
        wanted += back * config.clearance * 1.5;
    }
    found
}

#[allow(clippy::too_many_arguments)]
fn light(
    map: &Map,
    object: &MapObject,
    map_name: &str,
    surfaces: &SurfaceConfig,
    config: &FurnitureConfig,
    ordinals: &mut Ordinals,
    posts: &[Point3],
) -> Option<Placed> {
    let site = site(map, object)?;
    let stand = stand(map, &site, surfaces, config, site.station, posts)?;
    let section = site.road.section_at(stand.station)?;
    let layout = Layout::of(map, site.road, section, surfaces)?;
    let frame = site
        .road
        .frame_at(stand.station, map.metadata.sampling)
        .ok()?;
    let local = Local::new(stand.foot, stand.heading);

    // The arm is at the bar's height above the road, which is where the IR put
    // the bar; the heads hang below it over the middle of each governed lane.
    let arm_height = (site.height - stand.rise).max(config.lamp_diameter * 4.0);
    let mut heads: Vec<[f64; 3]> = Vec::new();
    for lane in &site.lanes {
        let Some(centre) = layout.lane_centre(stand.station, &lane.id) else {
            continue;
        };
        let over = frame.to_global([0.0, centre, 0.0]);
        let mut point = local.of(over);
        point[2] = arm_height;
        heads.push(point);
    }
    if heads.is_empty() {
        return None;
    }
    // The arm runs from the pole to the farthest head and a little past it. The
    // heads are all abeam of the pole, so the arm's direction is theirs.
    let farthest = heads
        .iter()
        .max_by(|a, b| a[1].abs().total_cmp(&b[1].abs()))
        .copied()
        .unwrap_or([0.0, 1.0, 0.0]);
    let side = if farthest[1] < 0.0 { -1.0 } else { 1.0 };
    let arm_length = farthest[1].abs() + config.arm_overhang;

    let ordinal = ordinals.take(Role::TrafficLight);
    let mut mesh = Mesh::new(
        mesh_name(map_name, Role::TrafficLight, ordinal),
        Role::TrafficLight,
        materials::STEEL,
    );
    let steel = mesh.slot_for(materials::STEEL);
    let housing = mesh.slot_for(materials::HOUSING);
    let mut lamps = Mesh::new(
        lamps_name(map_name, ordinal),
        Role::TrafficLight,
        materials::LAMP_RED,
    );
    let lamp_slots = [
        lamps.slot_for(materials::LAMP_RED),
        lamps.slot_for(materials::LAMP_AMBER),
        lamps.slot_for(materials::LAMP_GREEN),
    ];
    let top = arm_height + config.pole_radius * 3.0;
    cylinder(
        &mut mesh,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, top],
        config.pole_radius,
        12,
        steel,
    );
    cylinder(
        &mut mesh,
        [0.0, 0.0, arm_height],
        [0.0, side * arm_length, arm_height],
        config.arm_radius,
        8,
        steel,
    );
    let head_height = config.lamp_diameter * 3.0 + 0.15;
    for head in &heads {
        let y = head[1];
        let head_top = arm_height - config.arm_radius - 0.05;
        // The bracket the head hangs by.
        cylinder(
            &mut mesh,
            [0.0, y, head_top],
            [0.0, y, arm_height],
            config.arm_radius * 0.6,
            6,
            steel,
        );
        let centre_z = head_top - head_height / 2.0;
        cuboid(
            &mut mesh,
            [0.0, y, centre_z],
            [config.head_depth, config.head_width, head_height],
            housing,
        );
        // Three lamps on the face towards the traffic, red at the top.
        let face_x = config.head_depth / 2.0 + 0.005;
        for (row, slot) in lamp_slots.iter().enumerate() {
            let z =
                head_top - 0.075 - config.lamp_diameter / 2.0 - row as f64 * config.lamp_diameter;
            disc(
                &mut lamps,
                [face_x, y, z],
                config.lamp_diameter / 2.0,
                12,
                *slot,
            );
        }
    }

    Some(Placed {
        object: object.id.clone(),
        role: Role::TrafficLight,
        mesh,
        lamps: Some(lamps),
        position: stand.foot,
        heading: stand.heading,
        road: site.road.id.clone(),
        lanes: site.lanes.iter().map(|lane| lane.id.clone()).collect(),
        kind: SignKind::Other,
        arm_length,
        signal: SignalPlacement {
            position: stand.foot,
            applies_at: stop_line_of(map, &object.id),
            heading: Some(stand.heading),
            catalogue: None,
        },
    })
}

#[allow(clippy::too_many_arguments)]
fn sign(
    map: &Map,
    object: &MapObject,
    kind: SignKind,
    map_name: &str,
    surfaces: &SurfaceConfig,
    config: &FurnitureConfig,
    ordinals: &mut Ordinals,
    posts: &[Point3],
) -> Option<Placed> {
    let site = site(map, object)?;
    // Before the line, which is against the traffic's travel.
    let back = match site.direction {
        Direction::Forward => -1.0,
        Direction::Backward => 1.0,
    };
    let stand = stand(
        map,
        &site,
        surfaces,
        config,
        site.station + back * config.sign_setback_along,
        posts,
    )?;

    let mut mesh = Mesh::new(
        mesh_name(
            map_name,
            Role::TrafficSign,
            ordinals.take(Role::TrafficSign),
        ),
        Role::TrafficSign,
        materials::STEEL,
    );
    let steel = mesh.slot_for(materials::STEEL);
    let face = mesh.slot_for(match kind {
        SignKind::Stop => materials::PLATE_RED,
        SignKind::Yield | SignKind::SpeedLimit(_) => materials::PLATE_WHITE,
        SignKind::Other => materials::PLATE_BLUE,
    });
    let centre_z = (site.height - stand.rise).max(config.plate_size);
    let top = centre_z + config.plate_size / 2.0;
    cylinder(
        &mut mesh,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, top],
        config.sign_post_radius,
        8,
        steel,
    );
    let outline = plate_outline(kind, config.plate_size);
    let front = config.sign_post_radius + 0.03;
    plate(
        &mut mesh,
        &outline,
        centre_z,
        front - 0.02,
        front,
        face,
        steel,
    );

    Some(Placed {
        object: object.id.clone(),
        role: Role::TrafficSign,
        mesh,
        lamps: None,
        position: stand.foot,
        heading: stand.heading,
        road: site.road.id.clone(),
        lanes: site.lanes.iter().map(|lane| lane.id.clone()).collect(),
        kind,
        arm_length: 0.0,
        signal: SignalPlacement {
            position: stand.foot,
            applies_at: None,
            heading: Some(stand.heading),
            catalogue: kind.catalogue(),
        },
    })
}

/// Where a light applies: the middle of the stop line of the first rule that
/// names it, if any. CARLA builds a light's stop boxes from the signal's `s`, so
/// a light whose rule has a stop line stops traffic there rather than under its
/// own heads.
fn stop_line_of(map: &Map, light: &ObjectId) -> Option<Point3> {
    let stop_line = map.rules.iter().find_map(|rule| match rule {
        TrafficRule::TrafficLight {
            lights,
            stop_line: Some(stop_line),
            ..
        } if lights.contains(light) => Some(stop_line),
        _ => None,
    })?;
    let object = map.objects.get(stop_line)?;
    Some(match &object.geometry {
        ObjectGeometry::Point(position) => *position,
        ObjectGeometry::Line(curve) => curve.start_point().lerp(curve.end_point(), 0.5),
        ObjectGeometry::Band { left, right } => left.start_point().lerp(right.end_point(), 0.5),
    })
}

/// The name of a light's lamps mesh: the post's name with `Lamps` for `Post`, so
/// the two sort together and each says which it is.
fn lamps_name(map_name: &str, ordinal: usize) -> String {
    let (kind, _) = Role::TrafficLight.name_parts();
    format!("{map_name}_{kind}_Lamps_{ordinal}")
}

/// The shape of a sign's plate, as a ring in the plate's own `(y, z)` plane about
/// its centre, anticlockwise seen from the front.
fn plate_outline(kind: SignKind, size: f64) -> Vec<[f64; 2]> {
    match kind {
        SignKind::Stop => regular_polygon(8, size / 2.0, PI / 8.0),
        SignKind::SpeedLimit(_) => regular_polygon(24, size / 2.0, 0.0),
        // Point down, as a give-way triangle does.
        SignKind::Yield => {
            let height = size * 3.0_f64.sqrt() / 2.0;
            vec![
                [-size / 2.0, height / 2.0],
                [0.0, -height / 2.0],
                [size / 2.0, height / 2.0],
            ]
        }
        SignKind::Other => {
            let half = size / 2.0;
            vec![[-half, -half], [half, -half], [half, half], [-half, half]]
        }
    }
}

fn regular_polygon(sides: usize, radius: f64, phase: f64) -> Vec<[f64; 2]> {
    (0..sides)
        .map(|index| {
            let angle = phase + TAU * index as f64 / sides as f64;
            [radius * angle.cos(), radius * angle.sin()]
        })
        .collect()
}

/// A frame with the foot of a post at its origin and `+x` the way the post faces.
struct Local {
    origin: Point3,
    facing: Vector3,
    left: Vector3,
}

impl Local {
    fn new(origin: Point3, heading: f64) -> Local {
        Local {
            origin,
            facing: Vector3::new(heading.cos(), heading.sin(), 0.0),
            left: Vector3::new(-heading.sin(), heading.cos(), 0.0),
        }
    }

    fn of(&self, point: Point3) -> [f64; 3] {
        let offset = point - self.origin;
        [offset.dot(self.facing), offset.dot(self.left), offset.z]
    }
}

fn horizontal_distance(a: Point3, b: Point3) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

fn wrap(angle: f64) -> f64 {
    let wrapped = angle.rem_euclid(TAU);
    if wrapped > PI {
        wrapped - TAU
    } else {
        wrapped
    }
}

fn point(local: [f64; 3]) -> Point3 {
    Point3::new(local[0], local[1], local[2])
}

/// A closed tube between two points, with outward normals. Smooth round the
/// tube, since it is one strip; open at both ends, since a post's ends are in the
/// ground and in a cap nobody looks at.
fn cylinder(
    mesh: &mut Mesh,
    from: [f64; 3],
    to: [f64; 3],
    radius: f64,
    segments: usize,
    slot: usize,
) {
    let axis = point(to) - point(from);
    let Ok(axis) = axis.normalize() else {
        return;
    };
    let axis = axis.get();
    // Any two vectors across the axis, taken from whichever world axis it leans
    // on least.
    let seed = if axis.z.abs() < 0.9 {
        Vector3::new(0.0, 0.0, 1.0)
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    let Ok(u) = axis.cross(seed).normalize() else {
        return;
    };
    let u = u.get();
    let v = axis.cross(u);
    let ring = |centre: [f64; 3]| -> Vec<Point3> {
        (0..=segments)
            .map(|index| {
                let angle = TAU * (index % segments) as f64 / segments as f64;
                point(centre) + u * (radius * angle.cos()) + v * (radius * angle.sin())
            })
            .collect()
    };
    // Going round `u` towards `v` is anticlockwise seen from the `to` end, so the
    // `to` ring on the left of the `from` ring faces outwards.
    mesh.strip(&ring(to), &ring(from), slot);
}

/// An axis-aligned box about `centre`, faces outwards.
fn cuboid(mesh: &mut Mesh, centre: [f64; 3], size: [f64; 3], slot: usize) {
    let [cx, cy, cz] = centre;
    let [hx, hy, hz] = [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0];
    let corner = |sx: f64, sy: f64, sz: f64| Point3::new(cx + sx * hx, cy + sy * hy, cz + sz * hz);
    // Each ring anticlockwise seen from outside the box.
    let faces: [[Point3; 4]; 6] = [
        // +x
        [
            corner(1.0, -1.0, -1.0),
            corner(1.0, 1.0, -1.0),
            corner(1.0, 1.0, 1.0),
            corner(1.0, -1.0, 1.0),
        ],
        // -x
        [
            corner(-1.0, 1.0, -1.0),
            corner(-1.0, -1.0, -1.0),
            corner(-1.0, -1.0, 1.0),
            corner(-1.0, 1.0, 1.0),
        ],
        // +y
        [
            corner(1.0, 1.0, -1.0),
            corner(-1.0, 1.0, -1.0),
            corner(-1.0, 1.0, 1.0),
            corner(1.0, 1.0, 1.0),
        ],
        // -y
        [
            corner(-1.0, -1.0, -1.0),
            corner(1.0, -1.0, -1.0),
            corner(1.0, -1.0, 1.0),
            corner(-1.0, -1.0, 1.0),
        ],
        // +z
        [
            corner(-1.0, -1.0, 1.0),
            corner(1.0, -1.0, 1.0),
            corner(1.0, 1.0, 1.0),
            corner(-1.0, 1.0, 1.0),
        ],
        // -z
        [
            corner(-1.0, 1.0, -1.0),
            corner(1.0, 1.0, -1.0),
            corner(1.0, -1.0, -1.0),
            corner(-1.0, -1.0, -1.0),
        ],
    ];
    for face in &faces {
        mesh.face(face, slot);
    }
}

/// A flat disc in the `yz` plane at `centre`, facing `+x`.
fn disc(mesh: &mut Mesh, centre: [f64; 3], radius: f64, segments: usize, slot: usize) {
    let ring: Vec<Point3> = regular_polygon(segments, radius, 0.0)
        .into_iter()
        .map(|[y, z]| Point3::new(centre[0], centre[1] + y, centre[2] + z))
        .collect();
    mesh.face(&ring, slot);
}

/// A plate: `outline` extruded from `x_back` to `x_front`, its front in `face`
/// and its back and rim in `side`.
fn plate(
    mesh: &mut Mesh,
    outline: &[[f64; 2]],
    centre_z: f64,
    x_back: f64,
    x_front: f64,
    face: usize,
    side: usize,
) {
    let at = |x: f64, [y, z]: [f64; 2]| Point3::new(x, y, centre_z + z);
    let front: Vec<Point3> = outline.iter().map(|corner| at(x_front, *corner)).collect();
    let mut back: Vec<Point3> = outline.iter().map(|corner| at(x_back, *corner)).collect();
    mesh.face(&front, face);
    back.reverse();
    mesh.face(&back, side);
    for index in 0..outline.len() {
        let next = (index + 1) % outline.len();
        // Outwards: the outline is anticlockwise seen from the front, so the rim
        // between two corners faces away from the plate's middle when its ring
        // runs front-to-back on the second corner.
        mesh.face(
            &[
                at(x_front, outline[index]),
                at(x_back, outline[index]),
                at(x_back, outline[next]),
                at(x_front, outline[next]),
            ],
            side,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sign_code_is_read_the_way_a_caller_is_likely_to_have_written_it() {
        for code in ["stop", "STOP", "Stop_Sign", "R1-1", "206", "de206"] {
            assert_eq!(SignKind::parse(code), SignKind::Stop, "{code}");
        }
        for code in ["yield", "give way", "R1-2", "205", "de205"] {
            assert_eq!(SignKind::parse(code), SignKind::Yield, "{code}");
        }
        for code in [
            "de274-60",
            "274-60",
            "speed_limit_60",
            "maxspeed-60",
            "R2-1-60",
        ] {
            assert_eq!(SignKind::parse(code), SignKind::SpeedLimit(60), "{code}");
        }
        for code in ["", "274", "no_entry", "de267"] {
            assert_eq!(SignKind::parse(code), SignKind::Other, "{code}");
        }
    }

    #[test]
    fn a_sign_carla_knows_is_written_in_the_catalogue_carla_reads() {
        let limit = SignKind::SpeedLimit(50).catalogue().unwrap();
        assert_eq!((limit.kind.as_str(), limit.subtype.as_str()), ("274", "50"));
        assert_eq!(limit.speed_kph, Some(50.0));
        assert_eq!(SignKind::Stop.catalogue().unwrap().kind, "206");
        assert!(SignKind::Other.catalogue().is_none());
    }

    #[test]
    fn a_marker_is_in_the_state_carla_checks_for_not_the_state_that_is_right() {
        // The two `MatchSignalAndActor` gets wrong, matched to what it does rather
        // than to what it should.
        assert_eq!(
            SignKind::SpeedLimit(70).carla_state(),
            Some("SPEED_LIMIT_60")
        );
        assert_eq!(
            SignKind::SpeedLimit(80).carla_state(),
            Some("SPEED_LIMIT_90")
        );
        assert_eq!(SignKind::SpeedLimit(110).carla_state(), None);
        assert!(SignKind::SpeedLimit(110).doubled_by_carla());
        assert_eq!(SignKind::Other.carla_state(), None);
    }

    #[test]
    fn a_cylinder_faces_outwards() {
        let mut mesh = Mesh::new("t", Role::TrafficLight, 0);
        cylinder(&mut mesh, [0.0, 0.0, 0.0], [0.0, 0.0, 5.0], 0.1, 8, 0);
        for (position, normal) in mesh.positions.iter().zip(mesh.normals()) {
            let radial = Vector3::new(position.x, position.y, 0.0);
            assert!(radial.dot(normal) > 0.0, "{position:?} faces {normal:?}");
        }
    }

    #[test]
    fn a_box_and_a_plate_face_outwards() {
        let mut mesh = Mesh::new("t", Role::TrafficLight, 0);
        cuboid(&mut mesh, [1.0, 2.0, 3.0], [0.4, 0.4, 1.0], 0);
        let centre = Point3::new(1.0, 2.0, 3.0);
        for (position, normal) in mesh.positions.iter().zip(mesh.normals()) {
            assert!(
                (*position - centre).dot(normal) > 0.0,
                "{position:?} faces {normal:?}"
            );
        }
        let mut mesh = Mesh::new("t", Role::TrafficSign, 0);
        plate(
            &mut mesh,
            &regular_polygon(8, 0.35, 0.0),
            2.0,
            0.05,
            0.07,
            0,
            0,
        );
        let centre = Point3::new(0.06, 0.0, 2.0);
        for (position, normal) in mesh.positions.iter().zip(mesh.normals()) {
            assert!(
                (*position - centre).dot(normal) > -1e-9,
                "{position:?} faces {normal:?}"
            );
        }
    }
}
