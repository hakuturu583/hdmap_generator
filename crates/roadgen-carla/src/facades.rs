//! Windows and doors, cut into the walls a building's solid gives.
//!
//! The IR's building is a massing model: an outline, walls that rise from it and a
//! roof. It says how many storeys a part has and which street it faces, and nothing
//! about openings — a facade's windows are, by design, neither parts nor roofs, and
//! no format the IR is written to has a word for them. A camera does not care about
//! that. To a camera a wall with no windows is a warehouse, and a street of them is
//! a street of warehouses whatever the grammar called them.
//!
//! So the openings are laid out here, from what the IR does say. A wall is divided
//! into bays at the pitch its building kind uses, each storey of each bay gets a
//! window sized for that kind, and the ground storey of the wall that faces the
//! building's street gets a door in its middle bay. Each opening is two quads
//! standing a hair proud of the wall — a frame, and the glazing inside it — so it
//! needs no boolean cut and leaves the wall's own quad, and the building's semantic
//! tag, exactly as they were.

use roadgen_core::buildings::{Building, BuildingPart};
use roadgen_core::geometry::{Point3, Vector3};
use roadgen_core::map::Map;

use crate::materials;

/// How a kind of building arranges its openings, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rhythm {
    /// Distance between window centres along a wall.
    pub bay: f64,
    /// Wall left blank at each end.
    pub margin: f64,
    /// Width and height of a window above the ground storey.
    pub window: (f64, f64),
    /// Height of a window's sill above its storey's floor.
    pub sill: f64,
    /// Width and height of a ground-storey window, which a shop makes into a
    /// shopfront and a house keeps the same as the rest.
    pub ground_window: (f64, f64),
    pub ground_sill: f64,
    /// Width of the door on the street-facing wall; its height is
    /// [`DOOR_HEIGHT`].
    pub door: f64,
}

pub const DOOR_HEIGHT: f64 = 2.1;
/// How far a frame stands off its wall, and the glazing off the frame.
const RELIEF: f64 = 0.02;
/// How much wider and taller than its glazing a frame is, all round.
const FRAME: f64 = 0.06;
/// Clear wall a window has to leave above and below itself.
const HEADROOM: f64 = 0.25;

const HOUSE: Rhythm = Rhythm {
    bay: 3.0,
    margin: 0.7,
    window: (1.2, 1.3),
    sill: 0.9,
    ground_window: (1.2, 1.3),
    ground_sill: 0.9,
    door: 1.0,
};

const RETAIL: Rhythm = Rhythm {
    bay: 3.4,
    margin: 0.6,
    window: (1.6, 1.5),
    sill: 0.9,
    // A shopfront: glass from just above the pavement to just below the fascia.
    ground_window: (2.6, 2.4),
    ground_sill: 0.4,
    door: 1.6,
};

const BLOCK: Rhythm = Rhythm {
    bay: 3.0,
    margin: 0.8,
    window: (1.6, 1.5),
    sill: 0.9,
    ground_window: (1.6, 1.5),
    ground_sill: 0.9,
    door: 1.4,
};

const INDUSTRIAL: Rhythm = Rhythm {
    bay: 4.5,
    margin: 1.0,
    // Clerestory windows: a high strip of glass and blank wall below it.
    window: (2.4, 0.9),
    sill: 1.8,
    ground_window: (2.4, 0.9),
    ground_sill: 2.0,
    door: 3.0,
};

/// The rhythm a building kind is built to.
///
/// Chosen by the words the shape grammar's presets emit, with a house's rhythm for
/// anything else, because a kind the grammar made up is still somewhere people
/// live.
pub fn rhythm_of(kind: &str) -> Rhythm {
    match kind {
        "retail" | "commercial" => RETAIL,
        "apartments" | "office" => BLOCK,
        "industrial" | "warehouse" => INDUSTRIAL,
        _ => HOUSE,
    }
}

/// One opening: its ring, wound to face out of the wall, and its material.
pub struct Opening {
    pub ring: [Point3; 4],
    pub material: usize,
}

/// The openings of one wall.
///
/// `wall` is the quad `Solid::shell` gives for it — base edge first, then the eaves
/// edge back — and `levels` its part's storey count. `door` asks for a door in the
/// middle bay of the ground storey, and `kind` chooses the rhythm.
pub fn openings(wall: &[Point3], levels: u32, kind: &str, door: bool) -> Vec<Opening> {
    let mut openings = Vec::new();
    if wall.len() != 4 {
        return openings;
    }
    let (a, b, top_b, top_a) = (wall[0], wall[1], wall[2], wall[3]);
    let length = a.horizontal_distance_to(b);
    if length < 1e-6 {
        return openings;
    }
    // Outwards is to the right of the base edge walked from `a` to `b`, which is
    // the side the quad faces when the outline runs anticlockwise.
    let along = b - a;
    let out = Vector3::new(along.y, -along.x, 0.0) * (1.0 / length);
    let height = (top_a.z - a.z).min(top_b.z - b.z);
    let storeys = levels.max(1) as usize;
    let storey = height / storeys as f64;
    let rhythm = rhythm_of(kind);

    let usable = length - 2.0 * rhythm.margin;
    let bays = (usable / rhythm.bay).floor() as usize;
    if bays == 0 || storey <= 0.0 {
        return openings;
    }
    let pitch = usable / bays as f64;
    let door_bay = if door { Some(bays / 2) } else { None };

    // A point on the wall's plane: `x` metres along it from `a`, `z` metres above
    // the base there, `relief` metres out of it.
    let at = |x: f64, z: f64, relief: f64| -> Point3 {
        a.lerp(b, x / length) + out * relief + Vector3::UP * z
    };
    // An opening between `x0..x1` along and `z0..z1` up: its frame, and inside
    // that its pane. A door's frame stops at the ground rather than going under it.
    let mut opening = |x0: f64, x1: f64, z0: f64, z1: f64, pane: usize, framed_below: bool| {
        let sill = if framed_below { z0 - FRAME } else { z0 };
        for (x0, x1, z0, z1, relief, material) in [
            (
                x0 - FRAME,
                x1 + FRAME,
                sill,
                z1 + FRAME,
                RELIEF,
                materials::FRAME,
            ),
            (x0, x1, z0, z1, 2.0 * RELIEF, pane),
        ] {
            openings.push(Opening {
                ring: [
                    at(x0, z0, relief),
                    at(x1, z0, relief),
                    at(x1, z1, relief),
                    at(x0, z1, relief),
                ],
                material,
            });
        }
    };

    for bay in 0..bays {
        let centre = rhythm.margin + (bay as f64 + 0.5) * pitch;
        for level in 0..storeys {
            let floor = level as f64 * storey;
            // What this cell holds: the door, a shopfront, or a window — as a
            // width, a bottom and a top above the storey's floor, and a pane.
            let (width, bottom, top, pane) = if level == 0 && door_bay == Some(bay) {
                (rhythm.door, 0.0, DOOR_HEIGHT, materials::DOOR)
            } else {
                let ((width, tall), sill) = if level == 0 {
                    (rhythm.ground_window, rhythm.ground_sill)
                } else {
                    (rhythm.window, rhythm.sill)
                };
                (width, sill, sill + tall, materials::GLAZING)
            };
            let half = (width / 2.0).min(pitch / 2.0 - FRAME);
            // An opening that will not fit its storey is shortened, and one that
            // would not leave a wall above or below it is left out.
            let top = top.min(storey - HEADROOM);
            if top - bottom < 0.4 || half <= 0.1 {
                continue;
            }
            opening(
                centre - half,
                centre + half,
                floor + bottom,
                floor + top,
                pane,
                pane != materials::DOOR,
            );
        }
    }
    openings
}

/// Which wall of a part faces the building's street: the one whose middle is
/// nearest the point on the road the building fronts. `None` when the building
/// fronts no road, or a road the map has not got.
pub fn street_wall(map: &Map, building: &Building, part: &BuildingPart) -> Option<usize> {
    let frontage = building.frontage.as_ref()?;
    let road = map.road(&frontage.road)?;
    let there = road
        .reference_line
        .sample_at(frontage.station, map.metadata.sampling)
        .ok()?
        .point;
    let base = part.solid.footprint.points();
    (0..base.len()).min_by(|&i, &j| {
        let middle = |k: usize| base[k].lerp(base[(k + 1) % base.len()], 0.5);
        middle(i)
            .horizontal_distance_to(there)
            .partial_cmp(&middle(j).horizontal_distance_to(there))
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wall(length: f64, height: f64) -> Vec<Point3> {
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(length, 0.0, 0.0),
            Point3::new(length, 0.0, height),
            Point3::new(0.0, 0.0, height),
        ]
    }

    #[test]
    fn a_house_wall_gets_a_window_per_bay_per_storey_and_a_door() {
        let openings = openings(&wall(12.0, 6.0), 2, "house", true);
        // 12 m less two 0.7 m margins is 10.6 m: three 3 m bays. Two storeys, one
        // door in the middle bay of the ground storey: 3 * 2 - 1 windows + 1 door,
        // each a frame and a pane.
        let panes = openings
            .iter()
            .filter(|o| o.material == materials::GLAZING)
            .count();
        let doors = openings
            .iter()
            .filter(|o| o.material == materials::DOOR)
            .count();
        let frames = openings
            .iter()
            .filter(|o| o.material == materials::FRAME)
            .count();
        assert_eq!((panes, doors, frames), (5, 1, 6));
    }

    #[test]
    fn openings_stand_just_off_the_wall_and_face_out_of_it() {
        // The wall runs +x with the building to its +y side, so out is -y.
        for opening in openings(&wall(12.0, 6.0), 2, "house", true) {
            for point in &opening.ring {
                assert!(
                    point.y < 0.0 && point.y > -0.05,
                    "{point:?} is not just off the wall"
                );
                assert!(point.x > 0.0 && point.x < 12.0 && point.z >= 0.0 && point.z < 6.0);
            }
            let [p, q, r, _] = [
                opening.ring[0],
                opening.ring[1],
                opening.ring[2],
                opening.ring[3],
            ];
            let normal = (q - p).cross(r - p);
            assert!(normal.y < 0.0, "an opening faces into the wall");
        }
    }

    #[test]
    fn a_wall_too_short_for_a_bay_stays_blank() {
        assert!(openings(&wall(3.0, 6.0), 2, "house", true).is_empty());
    }

    #[test]
    fn a_shop_has_a_shopfront_and_no_window_where_its_door_is() {
        let openings = openings(&wall(10.0, 7.2), 2, "retail", true);
        let ground_panes: Vec<&Opening> = openings
            .iter()
            .filter(|o| o.material == materials::GLAZING && o.ring[0].z < 1.0)
            .collect();
        // Two bays on the ground storey, one of them the door.
        assert_eq!(ground_panes.len(), 1);
        let pane = ground_panes[0];
        let tall = pane.ring[2].z - pane.ring[0].z;
        assert!(tall > 2.0, "a shopfront {tall} m tall");
    }

    #[test]
    fn an_unknown_kind_is_built_like_a_house() {
        assert_eq!(rhythm_of("bandstand"), HOUSE);
    }
}
