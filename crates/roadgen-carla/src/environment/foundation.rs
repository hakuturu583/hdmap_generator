//! Foundations: what a building stands on where the land falls away under it.
//!
//! A building is generated level, at the height of the road it faces, and the land
//! is generated afterwards — round the roads, shaped by the parts, rolling away from
//! everything. Where the land under a building ends up lower than the building's
//! floor, the building would float. It is given a foundation instead: a part of its
//! own, of kind `foundation`, from the lowest point of the land under its footprint
//! up to the floor. Where the land is higher, the building is simply set into it,
//! the way a building on a slope is.

use roadgen_core::buildings::{BuildingPart, Footprint, Solid};
use roadgen_core::geometry::Point3;
use roadgen_core::id::{BuildingPartId, TinSurfaceId};
use roadgen_core::map::Map;

/// Below this, a gap under a building is no gap: the land's own triangles bend
/// that much between their nodes.
const SMALLEST: f64 = 0.2;

/// Gives every building that stands above `surface` a foundation down to it.
/// Returns how many it gave one.
pub fn lay(map: &mut Map, surface: &TinSurfaceId) -> Result<usize, super::land::Error> {
    let Some(land) = map.tin_surfaces.get(surface) else {
        return Ok(0);
    };
    let locator = land.tin.locator();
    let mut laid: Vec<(BuildingPart, roadgen_core::id::BuildingId)> = Vec::new();
    for building in map.buildings.iter() {
        let parts: Vec<&BuildingPart> = building
            .parts
            .iter()
            .filter_map(|id| map.building_parts.get(id))
            .collect();
        let floor = parts
            .iter()
            .map(|part| part.solid.footprint.lowest())
            .fold(f64::INFINITY, f64::min);
        for part in parts {
            let ring = part.solid.footprint.points();
            let base = part.solid.footprint.lowest();
            // Only what stands on the ground: a tower on a podium stands on the
            // podium.
            if base > floor + 0.05 {
                continue;
            }
            let centre = part.solid.footprint.centroid();
            let low = ring
                .iter()
                .copied()
                .chain([centre])
                .filter_map(|p| locator.height_at(p.x, p.y))
                .fold(f64::INFINITY, f64::min);
            if !low.is_finite() || base - low < SMALLEST {
                continue;
            }
            let top = part.solid.footprint.highest();
            let footprint = Footprint::new(ring.iter().map(|p| Point3::new(p.x, p.y, low)))?;
            let key = part
                .id
                .as_str()
                .strip_prefix("part/")
                .unwrap_or(part.id.as_str())
                .to_owned();
            laid.push((
                BuildingPart {
                    id: BuildingPartId::new(format!("{key}/foundation")),
                    building: building.id.clone(),
                    solid: Solid::prism(footprint, top - low),
                    kind: Some("foundation".into()),
                    levels: 1,
                },
                building.id.clone(),
            ));
        }
    }
    let count = laid.len();
    for (part, building) in laid {
        let id = part.id.clone();
        map.building_parts
            .insert(id.clone(), part)
            .map_err(|duplicate| super::land::Error::Duplicate(duplicate.0.to_string()))?;
        if let Some(entry) = map.buildings.get_mut(&building) {
            entry.parts.push(id);
        }
    }
    Ok(count)
}
