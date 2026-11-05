//! The network's geo-reference: the `<location>` element.
//!
//! A SUMO network carries its tie to the globe in one element, and the rule it states
//! is short: a network position is the projected position *plus* `netOffset`,
//!
//! ```text
//! (x, y) = proj(lon, lat) + netOffset
//! ```
//!
//! where `proj` is the PROJ definition in `projParameter`. That is the whole of what
//! `sumolib`'s `convertXY2LonLat` and SUMO's own GUI undo to turn a position back
//! into a longitude and latitude. `convBoundary` and `origBoundary` are the extent of
//! the network in its own metres and in degrees, and are informational.
//!
//! # Where it is written, and why there
//!
//! In the `.nod.xml`, as its first element. netconvert reads a `<location>` in a
//! plain node file and carries it into the network it builds: the nodes' `x` and `y`
//! are taken as positions *in the network's frame* — not reprojected, not shifted —
//! and the offset and projection are written alongside them. With the offset
//! normalisation the configuration already turns off, the built network's
//! coordinates are therefore exactly the ones written here, which are the IR's own,
//! and its `<location>` is this one. No post-processing step, and nothing for the
//! user to pass on the command line: `netconvert -c` is the whole build.
//!
//! (`convBoundary` is recomputed by netconvert from what it built, so the one
//! written here only has to be well formed. `origBoundary` is passed through as
//! written, which is why it is worked out properly rather than left at zero.)
//!
//! # Which projection
//!
//! The map's x and y are metres about its origin whatever its projection; what the
//! projection decides is which frame those metres are read against, and the string
//! has to describe *that* frame. It is chosen to agree with the other exports of
//! the same IR:
//!
//! * **local Cartesian** (and **MGRS**, whose metres are local Cartesian too — MGRS
//!   only changes the grid position the Lanelet2 export reports beside them) is the
//!   transverse Mercator centred on the origin, with a zero offset. PROJ has no
//!   east/north/up projection to name instead, and SUMO's `<location>` is 2D, so a
//!   map projection has to stand in for the east/north/up frame the Lanelet2 export
//!   projects through — a plane tangent to the ellipsoid *at the origin's altitude*.
//!   A metre on that plane is longer, measured on the ellipsoid beneath it, by
//!   `1 + h/R`: at unit scale a point 2 km out from an origin 2000 m up would come
//!   back 0.63 m short. So the scale is `+k=1+h/R`, with `R` the ellipsoid's mean
//!   radius of curvature `√(MN)` at the origin, which takes that to 1.5 mm (the rest
//!   is the difference between the north–south and east–west radii, which a single
//!   scale cannot take up). At altitude zero the scale is exactly 1 and the string is
//!   character for character the one the OpenDRIVE export writes in its
//!   `<geoReference>`; above it the two differ, because that one has no scale.
//!
//!   What a 2D definition cannot carry is each point's own height above the plane:
//!   a point `z` above it, `d` from the origin, is placed `d·|z|/R` from where the
//!   east/north/up frame puts it — 1.6 cm for 50 m at 2 km. [`height_error`] is that
//!   bound, and `check()` reports it when it reaches a centimetre.
//! * **UTM** is the zone's own `+proj=utm`, the form SUMO itself writes for a network
//!   it imported from OpenStreetMap, with the origin's easting and northing as the
//!   (negated) offset: the map's metres are UTM eastings and northings less the
//!   origin's, which is what the Lanelet2 export's UTM projector reads them as.
//!   South of the equator the northing carries UTM's 10 000 km false northing and the
//!   definition says `+south`, as PROJ expects. The offset is written here to the
//!   millimetre, but netconvert writes it into the network to the centimetre, as it
//!   does every length, so a built UTM network places a point within a centimetre of
//!   where the IR does rather than exactly.

use ll2_projection::{utmups, GpsPoint, Origin, Projector, TransverseMercatorProjector, Utm};
use roadgen_core::geometry::Point3;
use roadgen_core::map::Projection;
use roadgen_core::units::GeoOrigin;
use roadgen_core::ValidatedMap;

use crate::error::ExportError;

/// How the network's metres are tied to the globe: SUMO's `netOffset` and
/// `projParameter`.
#[derive(Debug, Clone, PartialEq)]
pub struct GeoReference {
    /// What is added to a projected position to give the network's own: `(x, y) =
    /// proj(lon, lat) + net_offset`.
    pub net_offset: (f64, f64),
    /// The PROJ definition of `proj`.
    pub proj_parameter: String,
}

/// The geo-reference a SUMO network of `map` is written with. See the module
/// documentation for which projection each of the map's projections becomes.
pub fn geo_reference(map: &ValidatedMap) -> Result<GeoReference, ExportError> {
    let origin = map.metadata.origin;
    Ok(match map.metadata.projection {
        Projection::LocalCartesian | Projection::Mgrs => GeoReference {
            net_offset: (0.0, 0.0),
            proj_parameter: tmerc_about(origin),
        },
        Projection::Utm => {
            let (zone, northern, easting, northing) =
                utmups::forward(origin.latitude(), origin.longitude()).map_err(projection)?;
            GeoReference {
                net_offset: (-easting, -northing),
                proj_parameter: format!(
                    "+proj=utm +zone={zone}{} +ellps=WGS84 +datum=WGS84 +units=m +no_defs",
                    if northern { "" } else { " +south" }
                ),
            }
        }
    })
}

/// The transverse Mercator about `origin`, scaled by [`scale_at`] — at altitude zero
/// character for character the string the OpenDRIVE export writes for the same
/// origin, so the two files can be seen to agree.
fn tmerc_about(origin: GeoOrigin) -> String {
    format!(
        "+proj=tmerc +lat_0={} +lon_0={} +k={} +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs",
        origin.latitude(),
        origin.longitude(),
        scale_at(origin)
    )
}

/// The ellipsoid's mean radius of curvature at `latitude` degrees: `√(MN)`, the
/// geometric mean of the meridian's radius and the prime vertical's.
fn mean_radius(latitude: f64) -> f64 {
    use ll2_projection::wgs84::{A, F};
    let e2 = F * (2.0 - F);
    let w2 = 1.0 - e2 * latitude.to_radians().sin().powi(2);
    let prime_vertical = A / w2.sqrt();
    let meridian = A * (1.0 - e2) / w2.powf(1.5);
    (prime_vertical * meridian).sqrt()
}

/// The transverse Mercator's scale for a local map about `origin`: a metre of the
/// east/north/up plane at the origin's altitude, measured on the ellipsoid beneath
/// it. Exactly 1 at altitude zero.
///
/// Written with `{}`, which round-trips, so the definition SUMO reads and the
/// projector `origBoundary` is worked out with have the very same scale.
fn scale_at(origin: GeoOrigin) -> f64 {
    1.0 + origin.altitude() / mean_radius(origin.latitude())
}

/// How far from where the map's own projection puts it SUMO's `<location>` places a
/// point at `point`, through the height the 2D definition cannot carry: the point's
/// distance from the origin times its height above the origin's plane, over the
/// earth's radius. Zero for a UTM map, whose projection is horizontal only in every
/// export.
pub fn height_error(map: &ValidatedMap, point: &Point3) -> f64 {
    match map.metadata.projection {
        Projection::LocalCartesian | Projection::Mgrs => {
            point.x.hypot(point.y) * point.z.abs() / mean_radius(map.metadata.origin.latitude())
        }
        Projection::Utm => 0.0,
    }
}

/// The projector that takes the network's metres back to latitude and longitude:
/// the same projection the `projParameter` names, with the offset already undone.
///
/// Used for `origBoundary` only, but it is the *same* projection rather than a
/// close one, so the boundary written is the one a reader of the `<location>` would
/// compute.
fn projector(map: &ValidatedMap) -> Result<Box<dyn Projector>, ExportError> {
    let origin = map.metadata.origin;
    let origin = Origin::new(GpsPoint::new(
        origin.latitude(),
        origin.longitude(),
        origin.altitude(),
    ));
    Ok(match map.metadata.projection {
        // A transverse Mercator whose central meridian is the origin's and whose
        // northing is rebased on it — `+lat_0 +lon_0 +k +x_0=0 +y_0=0` exactly.
        Projection::LocalCartesian | Projection::Mgrs => Box::new(
            TransverseMercatorProjector::new(origin, scale_at(map.metadata.origin)),
        ),
        // The origin's easting and northing subtracted, which is the negated offset.
        Projection::Utm => Box::new(Utm::new(origin, true, false).map_err(projection)?),
    })
}

/// The `<location>` element itself: the geo-reference, and the extent of `points` in
/// the network's metres and in degrees.
///
/// A map with nothing in it is given the extent of its origin, so that the element
/// is still well formed.
pub(crate) fn location<'p>(
    map: &ValidatedMap,
    points: impl IntoIterator<Item = &'p Point3>,
) -> Result<Vec<(&'static str, String)>, ExportError> {
    let reference = geo_reference(map)?;
    let projector = projector(map)?;

    let mut metres = Extent::default();
    let mut degrees = Extent::default();
    for point in points {
        metres.add(point.x, point.y);
        let position = projector
            .reverse([point.x, point.y, point.z])
            .map_err(projection)?;
        degrees.add(position.lon, position.lat);
    }
    if metres.is_empty() {
        metres.add(0.0, 0.0);
        let origin = map.metadata.origin;
        degrees.add(origin.longitude(), origin.latitude());
    }

    Ok(vec![
        (
            "netOffset",
            format!(
                "{},{}",
                crate::metres(reference.net_offset.0),
                crate::metres(reference.net_offset.1)
            ),
        ),
        ("convBoundary", metres.render(crate::metres)),
        (
            "origBoundary",
            degrees.render(|value| format!("{value:.9}")),
        ),
        ("projParameter", reference.proj_parameter),
    ])
}

/// A bounding box, as SUMO writes one: `xmin,ymin,xmax,ymax`.
#[derive(Default)]
struct Extent(Option<[f64; 4]>);

impl Extent {
    fn add(&mut self, x: f64, y: f64) {
        self.0 = Some(match self.0 {
            None => [x, y, x, y],
            Some([x0, y0, x1, y1]) => [x0.min(x), y0.min(y), x1.max(x), y1.max(y)],
        });
    }

    fn is_empty(&self) -> bool {
        self.0.is_none()
    }

    fn render(&self, number: impl Fn(f64) -> String) -> String {
        self.0
            .unwrap_or_default()
            .iter()
            .map(|value| number(*value))
            .collect::<Vec<_>>()
            .join(",")
    }
}

fn projection(error: ll2_projection::ProjectionError) -> ExportError {
    ExportError::Projection(error.message().to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;
    use roadgen_core::prelude::*;

    fn map(origin: GeoOrigin, projection: Projection) -> ValidatedMap {
        let mut builder = MapBuilder::new(MapMetadata {
            name: Some("located".into()),
            origin,
            projection,
            ..MapMetadata::default()
        });
        let width = PositiveWidth::new(3.5).unwrap();
        builder
            .add_road(
                RoadSpec::line(
                    Point3::new(-100.0, -50.0, 0.0),
                    Point3::new(300.0, 250.0, 0.0),
                    vec![
                        LaneSpec::new(width, Direction::Forward),
                        LaneSpec::new(width, Direction::Backward),
                    ],
                )
                .unwrap(),
            )
            .unwrap();
        builder.finish().unwrap().validate().unwrap()
    }

    fn tokyo() -> GeoOrigin {
        GeoOrigin::new(35.68, 139.76, 0.0).unwrap()
    }

    /// The `<location>` as the node file has it, by attribute.
    fn written(map: &ValidatedMap) -> std::collections::HashMap<String, String> {
        let nodes = crate::to_plain_xml(map).unwrap().nodes;
        let mut reader = quick_xml::Reader::from_str(&nodes);
        loop {
            match reader.read_event().unwrap() {
                quick_xml::events::Event::Empty(element)
                    if element.name().as_ref() == b"location" =>
                {
                    return element
                        .attributes()
                        .map(|attribute| {
                            let attribute = attribute.unwrap();
                            (
                                String::from_utf8_lossy(attribute.key.as_ref()).into_owned(),
                                String::from_utf8_lossy(&attribute.value).into_owned(),
                            )
                        })
                        .collect();
                }
                // It comes before any node: the schema says so, and netconvert reads
                // it before it reads the positions it describes.
                quick_xml::events::Event::Empty(element) if element.name().as_ref() == b"node" => {
                    panic!("a node before the <location>:\n{nodes}")
                }
                quick_xml::events::Event::Eof => panic!("no <location> in\n{nodes}"),
                _ => {}
            }
        }
    }

    fn numbers(value: &str) -> Vec<f64> {
        value.split(',').map(|part| part.parse().unwrap()).collect()
    }

    #[test]
    fn a_local_map_is_a_transverse_mercator_about_its_origin() {
        for projection in [Projection::LocalCartesian, Projection::Mgrs] {
            let location = written(&map(tokyo(), projection));
            assert_eq!(location["netOffset"], "0.000,0.000");
            assert_eq!(
                location["projParameter"],
                "+proj=tmerc +lat_0=35.68 +lon_0=139.76 +k=1 +x_0=0 +y_0=0 \
                 +datum=WGS84 +units=m +no_defs"
            );
        }
    }

    /// Above sea level the transverse Mercator is scaled, so that it reproduces the
    /// east/north/up frame the Lanelet2 export reads the same metres in — which at
    /// unit scale it misses by more than half a metre 2 km out from an origin 2000 m
    /// up.
    #[test]
    fn a_high_origin_scales_the_transverse_mercator_to_its_own_plane() {
        use ll2_projection::LocalCartesian;

        let high = GeoOrigin::new(35.68, 139.76, 2000.0).unwrap();
        let location = written(&map(high, Projection::LocalCartesian));
        let scale = scale_at(high);
        assert!((scale - 1.000_314).abs() < 1e-6, "{scale}");
        assert_eq!(
            location["projParameter"],
            format!(
                "+proj=tmerc +lat_0=35.68 +lon_0=139.76 +k={scale} +x_0=0 +y_0=0 \
                 +datum=WGS84 +units=m +no_defs"
            )
        );
        // The string round-trips the scale exactly.
        let written_scale: f64 = location["projParameter"]
            .split_whitespace()
            .find_map(|part| part.strip_prefix("+k="))
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(written_scale, scale);

        let gps = |lat, lon| GpsPoint::new(lat, lon, 0.0);
        let enu = LocalCartesian::new(Origin::new(GpsPoint::new(35.68, 139.76, 2000.0)));
        let scaled = TransverseMercatorProjector::new(enu.origin(), scale);
        let unit = TransverseMercatorProjector::new(enu.origin(), 1.0);
        // Metres between two positions near the origin, near enough.
        let apart = |a: GpsPoint, b: GpsPoint| {
            let north = (a.lat - b.lat) * 111_000.0;
            let east = (a.lon - b.lon) * 111_000.0 * 35.68f64.to_radians().cos();
            north.hypot(east)
        };
        let (mut worst_scaled, mut worst_unit) = (0.0f64, 0.0f64);
        for step in 0..16 {
            let angle = f64::from(step) * std::f64::consts::PI / 8.0;
            let point = [2000.0 * angle.cos(), 2000.0 * angle.sin(), 0.0];
            let wanted = enu.reverse(point).unwrap();
            let wanted = gps(wanted.lat, wanted.lon);
            let at = |projector: &TransverseMercatorProjector| {
                let p = projector.reverse(point).unwrap();
                apart(gps(p.lat, p.lon), wanted)
            };
            worst_scaled = worst_scaled.max(at(&scaled));
            worst_unit = worst_unit.max(at(&unit));
        }
        assert!(worst_unit > 0.6, "{worst_unit}");
        assert!(worst_scaled < 0.002, "{worst_scaled}");
    }

    #[test]
    fn the_height_a_location_cannot_carry_is_reported() {
        let flat = map(tokyo(), Projection::LocalCartesian);
        assert!(!crate::check(&flat)
            .iter()
            .any(|line| line.contains("no height")));

        // A road climbing 100 m over 2 km: its far end is 3 cm from where the
        // east/north/up frame puts it.
        let climbing = |projection| {
            let mut builder = MapBuilder::new(MapMetadata {
                origin: GeoOrigin::new(35.68, 139.76, 2000.0).unwrap(),
                projection,
                ..MapMetadata::default()
            });
            let width = PositiveWidth::new(3.5).unwrap();
            builder
                .add_road(
                    RoadSpec::line(
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2000.0, 0.0, 100.0),
                        vec![LaneSpec::new(width, Direction::Forward)],
                    )
                    .unwrap(),
                )
                .unwrap();
            builder.finish().unwrap().validate().unwrap()
        };
        let far_end = Point3::new(2000.0, 0.0, 100.0);
        let local = climbing(Projection::LocalCartesian);
        let error = height_error(&local, &far_end);
        assert!((error - 0.0314).abs() < 0.0002, "{error}");
        let report = crate::check(&local).join("\n");
        assert!(report.contains("up to 0.031 m"), "{report}");
        assert!(report.contains("transverse Mercator's scale"), "{report}");

        // UTM is horizontal in every export, so there is nothing to lose.
        let utm = climbing(Projection::Utm);
        assert_eq!(height_error(&utm, &far_end), 0.0);
        assert!(!crate::check(&utm)
            .iter()
            .any(|line| line.contains("no height")));
    }

    #[test]
    fn a_utm_map_is_its_zone_with_the_origin_as_the_offset() {
        for (origin, zone, south) in [
            (tokyo(), 54, ""),
            (GeoOrigin::new(-33.87, 151.21, 0.0).unwrap(), 56, " +south"),
        ] {
            let location = written(&map(origin, Projection::Utm));
            assert_eq!(
                location["projParameter"],
                format!(
                    "+proj=utm +zone={zone}{south} +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
                )
            );
            // The origin is the network's (0, 0), so the offset is minus where the
            // origin falls in the zone — northing with its false northing.
            let (_, _, easting, northing) =
                utmups::forward(origin.latitude(), origin.longitude()).unwrap();
            let offset = numbers(&location["netOffset"]);
            assert!((offset[0] + easting).abs() < 1e-3, "{offset:?}");
            assert!((offset[1] + northing).abs() < 1e-3, "{offset:?}");
        }
    }

    #[test]
    fn the_boundaries_are_the_extent_of_the_network() {
        for projection in [Projection::LocalCartesian, Projection::Utm] {
            let map = map(tokyo(), projection);
            let location = written(&map);
            // The kerbs reach a lane's width and a half beyond the reference line's
            // ends; nothing reaches much further.
            let [x0, y0, x1, y1] = numbers(&location["convBoundary"])[..] else {
                panic!()
            };
            assert!((-110.0..=-100.0).contains(&x0) && (-60.0..=-50.0).contains(&y0));
            assert!((300.0..310.0).contains(&x1) && (250.0..260.0).contains(&y1));

            // And the same box in degrees is around the origin, the right way up:
            // the network runs from south-west of the origin to north-east of it.
            let [lon0, lat0, lon1, lat1] = numbers(&location["origBoundary"])[..] else {
                panic!()
            };
            assert!(lon0 < 139.76 && 139.76 < lon1, "{lon0} {lon1}");
            assert!(lat0 < 35.68 && 35.68 < lat1, "{lat0} {lat1}");
            // 400 m of easting at 35.68° is about 0.0044° of longitude.
            assert!((lon1 - lon0 - 0.0044).abs() < 0.0003, "{lon0} {lon1}");
        }
    }

    #[test]
    fn an_origin_off_the_utm_grid_is_an_error_not_a_location() {
        // Validation lets a UTM origin through to 84 degrees either way, but UTM
        // proper stops at 80 south — beyond it is UPS, which is not supported.
        let antarctic = map(GeoOrigin::new(-82.0, 0.0, 0.0).unwrap(), Projection::Utm);
        assert!(matches!(
            geo_reference(&antarctic),
            Err(ExportError::Projection(_))
        ));
        assert!(matches!(
            crate::to_plain_xml(&antarctic),
            Err(ExportError::Projection(_))
        ));
    }
}
