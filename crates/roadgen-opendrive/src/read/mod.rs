//! Reading an OpenDRIVE document back into the IR.
//!
//! The lowering in the crate root is a table — road to `<road>`, lane to `<lane>`,
//! connection to `<laneLink>` — and this is that table read the other way. Nothing
//! is regenerated: the reference line is rebuilt piece for piece from `<planView>`,
//! the widths from `<width>`, the movements from the links and the junctions, and
//! the lane boundaries are then laid out by [`roadgen_core::layout`], which is the
//! same computation the builder uses for a road it generated. So a file roadgen
//! wrote comes back with the boundaries it was written from, and a file from
//! somewhere else gets the boundaries roadgen would have given it.
//!
//! # What does not come back
//!
//! The IR is narrower than OpenDRIVE in a few places, and where it is, the reader
//! approximates and says so: every [`Imported::approximations`] entry is one thing
//! the map no longer says exactly. A `<width>` that is a free cubic becomes a
//! straight taper through samples of it; a width that reaches zero is held at a
//! floor; an `<elevation>` that curves within one plan-view piece is split into
//! straight grades; a lane type the IR does not have lands on the nearest it does;
//! a building's roof is flat. None of these is an error, because none of them stops
//! the map being a map — they are the reader's equivalent of an exporter's `check`.
//!
//! What is an error is a document that cannot be read as a road network at all: a
//! reference line with a gap in it too wide to close, a link to a road that is not
//! there, a lane link to a lane the neighbour does not have.
//!
//! # Identifiers
//!
//! Roads and junctions are named after their OpenDRIVE ids, so `road/12` is the
//! file's road 12. A signal, object or building whose `name` is one of the IR's own
//! identifiers — which is what the exporter writes there — gets that identifier
//! back, so a map that goes out and comes in keeps its object names. Lanes are
//! numbered by position, as the builder numbers them.

mod furniture;
mod geometry;
mod lanes;
mod links;

use std::collections::HashMap;
use std::path::Path;

use opendrive::core::OpenDrive;
use opendrive::road::rule::Rule;
use opendrive::road::Road as OdRoad;
use uom::si::length::meter;

use roadgen_core::geometry::{Poly3Piece, Poly3Profile, SamplingConfig};
use roadgen_core::id::{JunctionId, LaneId, RoadId};
use roadgen_core::layout::{self, RoadGeometry, SectionLayout};
use roadgen_core::map::{CrossSection, Map, MapMetadata, Projection, Road, TrafficHandedness};
use roadgen_core::semantics::RoadType;
use roadgen_core::topology::{Junction, RoadEnd};
use roadgen_core::units::{GeoOrigin, SpeedLimit};
use roadgen_core::validation::UnvalidatedMap;

use crate::error::ImportError;

/// What a caller can tell the reader that the document does not.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ReadOptions {
    /// How finely the lane boundaries are sampled. Not in the file: a map read at a
    /// different resolution from the one it was written at has the same reference
    /// lines and different vertices along them.
    pub sampling: SamplingConfig,
    /// Where the map's `(0, 0)` stands, when the caller knows better than the
    /// `<geoReference>` — which may be missing, or in a form the reader does not
    /// recognise.
    pub origin: Option<(GeoOrigin, Projection)>,
}

/// A map read from a document, and what the reading could not keep.
#[derive(Debug, Clone)]
pub struct Imported {
    /// The map, not yet validated: the reader builds what the document says, and
    /// whether that is a consistent road network is validation's question.
    pub map: UnvalidatedMap,
    /// Everything the map says less exactly than the document did, one entry each.
    /// Empty for a document roadgen wrote from a map the IR could hold.
    pub approximations: Vec<String>,
}

/// Reads an OpenDRIVE document from a string.
pub fn from_xml(xml: &str) -> Result<Imported, ImportError> {
    from_xml_with(xml, &ReadOptions::default())
}

/// Reads an OpenDRIVE document from a string, with the caller's options.
pub fn from_xml_with(xml: &str, options: &ReadOptions) -> Result<Imported, ImportError> {
    let document =
        OpenDrive::from_xml_str(xml).map_err(|error| ImportError::Parse(format!("{error:?}")))?;
    from_document(&document, options)
}

/// Reads an OpenDRIVE file.
pub fn read(path: impl AsRef<Path>) -> Result<Imported, ImportError> {
    read_with(path, &ReadOptions::default())
}

/// Reads an OpenDRIVE file, with the caller's options.
pub fn read_with(path: impl AsRef<Path>, options: &ReadOptions) -> Result<Imported, ImportError> {
    let path = path.as_ref();
    let xml = std::fs::read_to_string(path)
        .map_err(|error| ImportError::Io(format!("{}: {error}", path.display())))?;
    from_xml_with(&xml, options)
}

/// Reads a parsed document.
pub fn from_document(document: &OpenDrive, options: &ReadOptions) -> Result<Imported, ImportError> {
    Reader::new(document, options)?.run()
}

/// The road ids and junction ids a document names, so a link can be resolved
/// without knowing which of the two it points at.
struct Names {
    roads: HashMap<String, RoadId>,
    junctions: HashMap<String, JunctionId>,
}

/// The reader, and everything it has worked out so far.
struct Reader<'a> {
    document: &'a OpenDrive,
    options: &'a ReadOptions,
    names: Names,
    map: Map,
    /// The IR lane at each OpenDRIVE (road, lane section, lane id).
    lanes: HashMap<(RoadId, usize, i64), LaneId>,
    approximations: Approximations,
}

/// What the reading could not keep, collected as it goes.
///
/// A note about one lane is one line; a note about a kind of thing that happens
/// forty times is one line that says forty. The second is what most of these are,
/// which is why they are counted here and written out at the end.
#[derive(Default)]
struct Approximations {
    lines: Vec<String>,
    counted: Vec<(String, usize)>,
}

impl Approximations {
    fn note(&mut self, line: impl Into<String>) {
        self.lines.push(line.into());
    }

    /// Notes a kind of thing that will be reported once, with how often it happened.
    fn count(&mut self, key: impl Into<String>) {
        let key = key.into();
        match self.counted.iter_mut().find(|(held, _)| *held == key) {
            Some((_, count)) => *count += 1,
            None => self.counted.push((key, 1)),
        }
    }

    fn finish(self) -> Vec<String> {
        let mut lines = self.lines;
        for (key, count) in self.counted {
            lines.push(key.replace("{n}", &count.to_string()));
        }
        lines
    }
}

impl<'a> Reader<'a> {
    fn new(document: &'a OpenDrive, options: &'a ReadOptions) -> Result<Self, ImportError> {
        let mut approximations = Approximations::default();
        let metadata = metadata(document, options, &mut approximations);
        let names = Names {
            roads: document
                .road
                .iter()
                .map(|road| (road.id.clone(), RoadId::new(&road.id)))
                .collect(),
            junctions: document
                .junction
                .iter()
                .map(|junction| (junction.id.clone(), JunctionId::new(&junction.id)))
                .collect(),
        };
        Ok(Reader {
            document,
            options,
            names,
            map: Map::new(metadata),
            lanes: HashMap::new(),
            approximations,
        })
    }

    fn run(mut self) -> Result<Imported, ImportError> {
        // Junctions first, so that a road's membership can be checked against a
        // junction that exists — and the ones roads name without a `<junction>`
        // element are made on the way, so nothing dangles.
        for junction in &self.document.junction {
            self.map
                .junctions
                .insert(
                    JunctionId::new(&junction.id),
                    Junction::new(JunctionId::new(&junction.id), junction.name.clone()),
                )
                .map_err(|duplicate| {
                    ImportError::Inconsistent(format!("two junctions are numbered {}", duplicate.0))
                })?;
        }
        for road in &self.document.road {
            self.read_road(road)?;
        }
        links::connect(&mut self)?;
        furniture::read(&mut self)?;
        Ok(Imported {
            map: UnvalidatedMap::from_map(self.map),
            approximations: self.approximations.finish(),
        })
    }

    fn sampling(&self) -> SamplingConfig {
        self.options.sampling
    }

    fn handedness(&self) -> TrafficHandedness {
        self.map.metadata.handedness
    }

    /// One `<road>`: its reference line, its profiles, its sections and the lanes
    /// laid out against them.
    fn read_road(&mut self, road: &OdRoad) -> Result<(), ImportError> {
        let id = RoadId::new(&road.id);
        let config = self.sampling();
        let length = road.length.get::<meter>();

        let reference_line = geometry::reference_line(road, config, &mut self.approximations)?;
        let lane_offset = profile(
            road.lanes
                .lane_offset
                .iter()
                .map(|offset| (offset.s, offset.a, offset.b, offset.c, offset.d)),
        )?;
        let superelevation = match &road.lateral_profile {
            Some(lateral) => {
                if !lateral.shape.is_empty() {
                    self.approximations.count(
                        "the IR has no `<shape>`: the lateral shape of {n} roads is dropped and \
                         their surfaces are flat across",
                    );
                }
                profile(
                    lateral
                        .super_elevation
                        .iter()
                        .map(|piece| (piece.s, piece.a, piece.b, piece.c, piece.d)),
                )?
            }
            None => Poly3Profile::default(),
        };

        let (road_type, speed_limit) = road_type(road, &mut self.approximations)?;
        let junction = match road.junction.trim() {
            "" | "-1" => None,
            named => {
                let junction = JunctionId::new(named);
                if !self.map.junctions.contains(&junction) {
                    self.approximations.note(format!(
                        "road {} belongs to junction {named}, which the document does not \
                         describe; the junction is made empty",
                        road.id
                    ));
                    self.map
                        .junctions
                        .insert(junction.clone(), Junction::new(junction.clone(), None))
                        .ok();
                }
                Some(junction)
            }
        };
        let link = links::road_link(road, &self.names, &mut self.approximations)?;

        // The sections, each as the lane specs the layout takes.
        let sections = lanes::sections(
            &id,
            road,
            length,
            self.handedness(),
            config,
            &mut self.approximations,
        )?;

        let required = roadgen_core::map::required_stations(
            sections.iter().map(|section| section.station),
            sections
                .iter()
                .flat_map(|section| section.lanes.iter().map(|lane| &lane.spec.width)),
            config,
        );
        let geometry = RoadGeometry::new(&reference_line, &superelevation, config, &required)?;

        let mut all_lanes = Vec::new();
        let mut cross_sections = Vec::new();
        let mut index_offset = 0;
        for (index, section) in sections.iter().enumerate() {
            let end = sections
                .get(index + 1)
                .map(|next| next.station)
                .unwrap_or(length);
            let specs: Vec<_> = section.lanes.iter().map(|lane| lane.spec.clone()).collect();
            let layout = SectionLayout::new(
                &specs,
                (section.station, end),
                lane_offset.clone(),
                self.handedness(),
            );
            let built = layout::section_lanes(
                &id,
                &geometry,
                &specs,
                index,
                index_offset,
                &layout,
                speed_limit,
            )?;
            for (lane, entry) in built.iter().zip(&section.lanes) {
                self.lanes
                    .insert((id.clone(), index, entry.opendrive_id), lane.id.clone());
            }
            cross_sections.push(CrossSection {
                station: section.station,
                lanes: built.iter().map(|lane| lane.id.clone()).collect(),
            });
            index_offset += built.len();
            all_lanes.extend(built);
        }

        let entry = Road {
            id: id.clone(),
            name: road.name.clone(),
            reference_line,
            lane_offset,
            lanes: all_lanes.iter().map(|lane| lane.id.clone()).collect(),
            sections: cross_sections,
            junction: junction.clone(),
            link,
            road_type,
            speed_limit,
            superelevation,
        };
        self.map
            .roads
            .insert(id.clone(), entry)
            .map_err(|duplicate| {
                ImportError::Inconsistent(format!("two roads are numbered {}", duplicate.0))
            })?;
        for lane in all_lanes {
            self.map
                .lanes
                .insert(lane.id.clone(), lane)
                .map_err(|duplicate| ImportError::Inconsistent(duplicate.0.to_string()))?;
        }
        if let Some(junction) = junction {
            if let Some(entry) = self.map.junctions.get_mut(&junction) {
                entry.connecting_roads.push(id);
            }
        }
        Ok(())
    }

    /// The IR lane at an OpenDRIVE (road, section, lane id), or an error naming
    /// what pointed at it.
    fn lane_at(
        &self,
        road: &RoadId,
        section: usize,
        lane: i64,
        referrer: &str,
    ) -> Result<LaneId, ImportError> {
        self.lanes
            .get(&(road.clone(), section, lane))
            .cloned()
            .ok_or_else(|| {
                ImportError::Inconsistent(format!(
                    "{referrer} names lane {lane} of section {section} of {road}, which has no \
                     such lane"
                ))
            })
    }

    /// The section of a road at one of its ends.
    fn section_at_end(&self, road: &RoadId, end: RoadEnd) -> Option<usize> {
        let entry = self.map.roads.get(road)?;
        Some(match end {
            RoadEnd::Start => 0,
            RoadEnd::End => entry.sections.len() - 1,
        })
    }
}

/// A piecewise cubic from OpenDRIVE's `(s, a, b, c, d)` rows.
fn profile(
    pieces: impl IntoIterator<Item = (f64, f64, f64, f64, f64)>,
) -> Result<Poly3Profile, ImportError> {
    Ok(Poly3Profile::new(
        pieces
            .into_iter()
            .map(|(s, a, b, c, d)| Poly3Piece::new(s, a, b, c, d)),
    )?)
}

/// The road's type and the speed limit its type carries.
///
/// OpenDRIVE lets both change along the road; the IR holds one of each, so the
/// first is taken and the rest reported.
fn road_type(
    road: &OdRoad,
    approximations: &mut Approximations,
) -> Result<(RoadType, Option<SpeedLimit>), ImportError> {
    use opendrive::road::road_type_e::RoadTypeE;
    use opendrive::road::speed::MaxSpeed;

    let Some(first) = road.r#type.first() else {
        return Ok((RoadType::Town, None));
    };
    if road.r#type.len() > 1 {
        approximations.count(
            "a road has one type in the IR, so {n} roads whose `<type>` changes along them \
             keep their first",
        );
    }
    let road_type = match first.r#type {
        RoadTypeE::Rural => RoadType::Rural,
        RoadTypeE::Motorway => RoadType::Motorway,
        RoadTypeE::Town
        | RoadTypeE::TownExpressway
        | RoadTypeE::TownCollector
        | RoadTypeE::TownArterial
        | RoadTypeE::TownPrivate
        | RoadTypeE::TownLocal
        | RoadTypeE::TownPlayStreet
        | RoadTypeE::Unknown => RoadType::Town,
        RoadTypeE::LowSpeed | RoadTypeE::Bicycle => RoadType::LowSpeed,
        RoadTypeE::Pedestrian => RoadType::Pedestrian,
    };
    let exact = matches!(
        first.r#type,
        RoadTypeE::Rural
            | RoadTypeE::Motorway
            | RoadTypeE::Town
            | RoadTypeE::LowSpeed
            | RoadTypeE::Pedestrian
    );
    if !exact {
        approximations.count(format!(
            "the IR has five road types, so {{n}} roads of type {:?} are read as {}",
            first.r#type,
            road_type.as_str()
        ));
    }
    let speed_limit = match &first.speed {
        Some(speed) => match speed.max {
            MaxSpeed::Limit(value) => Some(SpeedLimit::from_mps(lanes::to_mps(
                value,
                speed.unit.as_ref(),
            ))?),
            MaxSpeed::NoLimit | MaxSpeed::Undefined => None,
        },
        None => None,
    };
    Ok((road_type, speed_limit))
}

/// The map's name, origin and handedness, from the header and the roads.
fn metadata(
    document: &OpenDrive,
    options: &ReadOptions,
    approximations: &mut Approximations,
) -> MapMetadata {
    let (origin, projection) = match options.origin {
        Some(given) => given,
        None => georeference(
            document
                .header
                .geo_reference
                .as_ref()
                .and_then(|reference| reference.proj.as_deref()),
            approximations,
        ),
    };

    // The rule is per road in OpenDRIVE and per map in the IR. The first road's is
    // the map's; a road that disagrees is reported, because its lanes will be read
    // as running the other way.
    let mut handedness = TrafficHandedness::RightHand;
    let mut stated: Option<Rule> = None;
    for road in &document.road {
        let Some(rule) = &road.rule else {
            continue;
        };
        match &stated {
            None => {
                stated = Some(rule.clone());
                handedness = match rule {
                    Rule::RightHandTraffic => TrafficHandedness::RightHand,
                    Rule::LeftHandTraffic => TrafficHandedness::LeftHand,
                };
            }
            Some(first) if first != rule => {
                approximations.count(
                    "the IR keeps to one side of the road: {n} roads state the opposite \
                     `rule` from the first and are read with the first's",
                );
            }
            Some(_) => {}
        }
    }

    MapMetadata {
        name: document.header.name.clone(),
        origin,
        projection,
        handedness,
        sampling: options.sampling,
    }
}

/// Where the document's `(0, 0)` stands, from its `<geoReference>`.
///
/// Two forms are read, and they are the two the exporter writes: a transverse
/// Mercator about the origin at unit scale, which describes a local Cartesian map,
/// and the UTM zone's transverse Mercator with the origin's easting and northing
/// folded into the false origin, which describes a UTM map. Anything else places
/// the map at the default origin and says so; a caller who knows better passes
/// [`ReadOptions::origin`].
fn georeference(
    proj: Option<&str>,
    approximations: &mut Approximations,
) -> (GeoOrigin, Projection) {
    let fallback = (GeoOrigin::default(), Projection::LocalCartesian);
    let Some(proj) = proj else {
        approximations.note(
            "the document has no `<geoReference>`, so the map is placed at latitude 0, \
             longitude 0",
        );
        return fallback;
    };
    let mut parameters: HashMap<&str, &str> = HashMap::new();
    for token in proj.split_whitespace() {
        let token = token.trim_start_matches('+');
        match token.split_once('=') {
            Some((key, value)) => {
                parameters.insert(key, value);
            }
            None => {
                parameters.insert(token, "");
            }
        }
    }
    let number = |key: &str| {
        parameters
            .get(key)
            .and_then(|value| value.parse::<f64>().ok())
    };

    let understood = match parameters.get("proj").copied() {
        Some("tmerc") => {
            let (lat_0, lon_0) = (number("lat_0").unwrap_or(0.0), number("lon_0"));
            let k = number("k").or_else(|| number("k_0")).unwrap_or(1.0);
            let (x_0, y_0) = (number("x_0").unwrap_or(0.0), number("y_0").unwrap_or(0.0));
            match lon_0 {
                Some(lon_0) if (k - 1.0).abs() < 1e-9 && x_0 == 0.0 && y_0 == 0.0 => {
                    GeoOrigin::new(lat_0, lon_0, 0.0)
                        .ok()
                        .map(|origin| (origin, Projection::LocalCartesian))
                }
                Some(lon_0) if (k - 0.9996).abs() < 1e-9 && lat_0 == 0.0 => {
                    // The zone's central meridian names the zone; the false origin
                    // is 500 km less the easting and, south of the equator, 10 000 km
                    // less the northing. Undo both and project back.
                    let zone = ((lon_0 + 183.0) / 6.0).round() as i32;
                    let easting = 500_000.0 - x_0;
                    let (northern, northing) = if y_0 <= 0.0 {
                        (true, -y_0)
                    } else {
                        (false, 10_000_000.0 - y_0)
                    };
                    ll2_projection::utmups::reverse(zone, northern, easting, northing)
                        .ok()
                        .and_then(|(lat, lon)| GeoOrigin::new(lat, lon, 0.0).ok())
                        .map(|origin| (origin, Projection::Utm))
                }
                _ => None,
            }
        }
        _ => None,
    };
    match understood {
        Some(found) => {
            approximations.note(
                "a PROJ string carries no altitude, so the origin is read at 0 m; the map's \
                 own heights are unaffected",
            );
            found
        }
        None => {
            approximations.note(format!(
                "the `<geoReference>` {proj:?} is not one of the forms the reader knows, so \
                 the map is placed at latitude 0, longitude 0 in its own metres"
            ));
            fallback
        }
    }
}
