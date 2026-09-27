//! Roads whose two ends run into one junction.
//!
//! Inside a large intersection a Lanelet2 map often has a stretch of ordinary road
//! between two sets of turns — a lane that crosses a median, a short link between
//! two halves of a crossroads — and both of its ends then run into the same
//! junction. That is valid OpenDRIVE, but CARLA misreads it. To find where a lane
//! goes at a road's end CARLA looks, in the junction there, for connecting roads
//! whose predecessor is the road — and also for ones whose *successor* is the road,
//! without asking at which end. A connecting road that enters the road at its start
//! is then taken for one that leaves it at its end, CARLA sees a loop between the
//! two, and drops the movement into the road altogether.
//!
//! So the document cuts every such road in two, joined road to road at a
//! geometry boundary near its middle: the first half keeps the road's id and its
//! start, the second is a road of its own with the road's end, and every link to
//! that end now names the second half. Neither half has both ends in the junction,
//! the junction is untouched, and nothing moves: the second half's profiles, lane
//! widths, lifts, marks and furniture are the road's own, counted from the cut.

use opendrive::core::additional_data::AdditionalData;
use opendrive::core::OpenDrive;
use opendrive::junction::contact_point::ContactPoint;
use opendrive::lane::lane_choice::LaneChoice;
use opendrive::lane::lane_link::LaneLink;
use opendrive::lane::lane_section::LaneSection;
use opendrive::lane::predecessor_successor::PredecessorSuccessor as LaneEnd;
use opendrive::lane::Lane as OdLane;
use opendrive::road::element_type::ElementType;
use opendrive::road::geometry::geometry_type::GeometryType;
use opendrive::road::geometry::param_poly_3::ParamPoly3;
use opendrive::road::geometry::param_poly_3_p_range::ParamPoly3pRange;
use opendrive::road::geometry::Geometry;
use opendrive::road::link::Link;
use opendrive::road::predecessor_successor::PredecessorSuccessor;
use opendrive::road::Road as OdRoad;
use uom::si::angle::radian;
use uom::si::curvature::radian_per_meter;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;
use vec1::Vec1;

/// Steps for measuring the halves of a cubic cut in two, which is how their
/// lengths are found; plenty for curves a few metres long.
const LENGTH_STEPS: usize = 256;

fn metres(length: Length) -> f64 {
    length.get::<meter>()
}

/// The junction both ends of an ordinary road run into, if they run into one.
fn looping(road: &OdRoad) -> Option<String> {
    if road.junction != "-1" {
        return None;
    }
    let link = road.link.as_ref()?;
    let (start, end) = (link.predecessor.as_ref()?, link.successor.as_ref()?);
    let junction = |end: &PredecessorSuccessor| end.element_type == Some(ElementType::Junction);
    (junction(start) && junction(end) && start.element_id == end.element_id)
        .then(|| start.element_id.clone())
}

/// A cubic `a + b·x + c·x² + d·x³` re-expressed about `x = k`.
fn shifted(a: f64, b: f64, c: f64, d: f64, k: f64) -> (f64, f64, f64, f64) {
    (
        a + b * k + c * k * k + d * k * k * k,
        b + 2.0 * c * k + 3.0 * d * k * k,
        c + 3.0 * d * k,
        d,
    )
}

/// Records of a piecewise cubic, each valid from its station on, rebased so the
/// second half starts at zero: the record in force at `cut` becomes the first,
/// re-expressed about the cut, and the later ones move back.
fn rebase<T: Clone>(
    records: &[T],
    cut: f64,
    station: impl Fn(&T) -> f64,
    rebuild: impl Fn(&T, f64, (f64, f64, f64, f64)) -> T,
    coefficients: impl Fn(&T) -> (f64, f64, f64, f64),
) -> (Vec<T>, Vec<T>) {
    let first: Vec<T> = records
        .iter()
        .filter(|record| station(record) < cut)
        .cloned()
        .collect();
    let mut second = Vec::new();
    if let Some(current) = records.iter().rfind(|record| station(record) <= cut) {
        let (a, b, c, d) = coefficients(current);
        second.push(rebuild(
            current,
            0.0,
            shifted(a, b, c, d, cut - station(current)),
        ));
    }
    for record in records.iter().filter(|record| station(record) > cut) {
        second.push(rebuild(record, station(record) - cut, coefficients(record)));
    }
    (first, second)
}

/// Records that hold a value from their station on, rebased the same way.
fn rebase_steps<T: Clone>(
    records: &[T],
    cut: f64,
    station: impl Fn(&T) -> f64,
    moved: impl Fn(&T, f64) -> T,
) -> Vec<T> {
    let mut second = Vec::new();
    if let Some(current) = records.iter().rfind(|record| station(record) <= cut) {
        second.push(moved(current, 0.0));
    }
    for record in records.iter().filter(|record| station(record) > cut) {
        second.push(moved(record, station(record) - cut));
    }
    second
}

/// The planView cut at `cut`, which is a geometry boundary or the middle of a
/// single line, arc or cubic.
fn cut_plan_view(geometry: &[Geometry], cut: f64) -> Option<(Vec<Geometry>, Vec<Geometry>)> {
    let station = |entry: &Geometry| metres(entry.s);
    if let Some(at) = geometry
        .iter()
        .position(|entry| (station(entry) - cut).abs() < 1e-9)
    {
        let mut second: Vec<Geometry> = geometry[at..].to_vec();
        for entry in &mut second {
            entry.s = Length::new::<meter>(station(entry) - cut);
        }
        return Some((geometry[..at].to_vec(), second));
    }
    let [only] = geometry else {
        return None;
    };
    let heading = only.hdg.get::<radian>();
    let (x0, y0) = (metres(only.x), metres(only.y));
    let local = |u: f64, v: f64| {
        let (sin, cos) = heading.sin_cos();
        (x0 + u * cos - v * sin, y0 + u * sin + v * cos)
    };
    let half = |entry: &Geometry, s: f64, x: f64, y: f64, hdg: f64, length: f64, kind| Geometry {
        hdg: Angle::new::<radian>(hdg),
        length: Length::new::<meter>(length),
        s: Length::new::<meter>(s),
        x: Length::new::<meter>(x),
        y: Length::new::<meter>(y),
        r#type: kind,
        additional_data: entry.additional_data.clone(),
    };
    let length = metres(only.length);
    match &only.r#type {
        GeometryType::Line(line) => {
            let (x, y) = local(cut, 0.0);
            Some((
                vec![half(
                    only,
                    0.0,
                    x0,
                    y0,
                    heading,
                    cut,
                    GeometryType::Line(line.clone()),
                )],
                vec![half(
                    only,
                    0.0,
                    x,
                    y,
                    heading,
                    length - cut,
                    GeometryType::Line(line.clone()),
                )],
            ))
        }
        GeometryType::Arc(arc) => {
            let k = arc.curvature.get::<radian_per_meter>();
            let turned = k * cut;
            let (u, v) = if k.abs() < 1e-12 {
                (cut, 0.0)
            } else {
                (turned.sin() / k, (1.0 - turned.cos()) / k)
            };
            let (x, y) = local(u, v);
            Some((
                vec![half(
                    only,
                    0.0,
                    x0,
                    y0,
                    heading,
                    cut,
                    GeometryType::Arc(arc.clone()),
                )],
                vec![half(
                    only,
                    0.0,
                    x,
                    y,
                    heading + turned,
                    length - cut,
                    GeometryType::Arc(arc.clone()),
                )],
            ))
        }
        GeometryType::ParamPoly3(poly) if poly.p_range == ParamPoly3pRange::Normalized => {
            let (first, second, first_length, second_length, (u0, v0), turn) = halve(poly);
            debug_assert!((first_length - cut).abs() < 1e-9);
            debug_assert!((first_length + second_length - length).abs() < 1e-3 * length.max(1.0));
            let (x, y) = local(u0, v0);
            Some((
                vec![half(
                    only,
                    0.0,
                    x0,
                    y0,
                    heading,
                    first_length,
                    GeometryType::ParamPoly3(first),
                )],
                vec![half(
                    only,
                    0.0,
                    x,
                    y,
                    heading + turn,
                    second_length,
                    GeometryType::ParamPoly3(second),
                )],
            ))
        }
        _ => None,
    }
}

/// A normalised cubic cut at its parameter's middle: the two halves, each in its
/// own start frame, their lengths, where the second starts in the first's frame
/// and how far it has turned.
fn halve(poly: &ParamPoly3) -> (ParamPoly3, ParamPoly3, f64, f64, (f64, f64), f64) {
    let u = [poly.a_u, poly.b_u, poly.c_u, poly.d_u];
    let v = [poly.a_v, poly.b_v, poly.c_v, poly.d_v];
    let at = |c: &[f64; 4], p: f64| c[0] + c[1] * p + c[2] * p * p + c[3] * p * p * p;
    let slope = |c: &[f64; 4], p: f64| c[1] + 2.0 * c[2] * p + 3.0 * c[3] * p * p;
    let speed = |p: f64| slope(&u, p).hypot(slope(&v, p));
    let measure = |from: f64, to: f64| {
        let h = (to - from) / LENGTH_STEPS as f64;
        // Simpson's rule.
        (0..LENGTH_STEPS)
            .map(|i| {
                let p = from + h * i as f64;
                h / 6.0 * (speed(p) + 4.0 * speed(p + h / 2.0) + speed(p + h))
            })
            .sum::<f64>()
    };
    // The first half: p = q/2, in the same frame.
    let first_half = |c: &[f64; 4]| [c[0], c[1] / 2.0, c[2] / 4.0, c[3] / 8.0];
    // The second half: p = (1 + q)/2, about the middle.
    let second_half = |c: &[f64; 4]| {
        [
            at(c, 0.5),
            c[1] / 2.0 + c[2] / 2.0 + 3.0 * c[3] / 8.0,
            c[2] / 4.0 + 3.0 * c[3] / 8.0,
            c[3] / 8.0,
        ]
    };
    let (u2, v2) = (second_half(&u), second_half(&v));
    let turn = slope(&v, 0.5).atan2(slope(&u, 0.5));
    let (sin, cos) = turn.sin_cos();
    // The second half in its own frame: moved to its start, turned by its heading.
    let mut su = [0.0; 4];
    let mut sv = [0.0; 4];
    for i in 1..4 {
        su[i] = u2[i] * cos + v2[i] * sin;
        sv[i] = -u2[i] * sin + v2[i] * cos;
    }
    let (fu, fv) = (first_half(&u), first_half(&v));
    let make = |cu: [f64; 4], cv: [f64; 4]| ParamPoly3 {
        a_u: cu[0],
        b_u: cu[1],
        c_u: cu[2],
        d_u: cu[3],
        a_v: cv[0],
        b_v: cv[1],
        c_v: cv[2],
        d_v: cv[3],
        p_range: ParamPoly3pRange::Normalized,
    };
    (
        make(fu, fv),
        make(su, sv),
        measure(0.0, 0.5),
        measure(0.5, 1.0),
        (u2[0], v2[0]),
        turn,
    )
}

/// Where to cut: the geometry boundary nearest the middle, or the middle of the
/// only piece.
fn cut_station(road: &OdRoad) -> Option<f64> {
    let geometry = road.plan_view.geometry.as_slice();
    let length = metres(road.length);
    if geometry.len() > 1 {
        return geometry[1..]
            .iter()
            .map(|entry| metres(entry.s))
            .min_by(|a, b| {
                (a - length / 2.0)
                    .abs()
                    .total_cmp(&(b - length / 2.0).abs())
            });
    }
    match &geometry[0].r#type {
        GeometryType::Line(_) | GeometryType::Arc(_) => Some(length / 2.0),
        GeometryType::ParamPoly3(poly) if poly.p_range == ParamPoly3pRange::Normalized => {
            Some(halve(poly).2)
        }
        _ => None,
    }
}

fn lanes_mut(section: &mut LaneSection) -> impl Iterator<Item = &mut OdLane> {
    section
        .left
        .iter_mut()
        .flat_map(|left| left.lane.iter_mut().map(|lane| &mut lane.base))
        .chain(
            section
                .right
                .iter_mut()
                .flat_map(|right| right.lane.iter_mut().map(|lane| &mut lane.base)),
        )
}

fn lane_ids(section: &LaneSection) -> Vec<i64> {
    section
        .left
        .iter()
        .flat_map(|left| left.lane.iter().map(|lane| lane.id))
        .chain(
            section
                .right
                .iter()
                .flat_map(|right| right.lane.iter().map(|lane| lane.id)),
        )
        .collect()
}

/// A lane's own records rebased by `by` metres along its section.
fn rebase_lane(lane: &mut OdLane, by: f64) {
    let at = |length: Length| metres(length);
    let widths: Vec<_> = lane.choice.clone();
    lane.choice = rebase(
        &widths,
        by,
        |choice| match choice {
            LaneChoice::Width(width) => at(width.s_offset),
            LaneChoice::Border(border) => at(border.s_offset),
        },
        |choice, s, (a, b, c, d)| match choice {
            LaneChoice::Width(width) => {
                let mut width = width.clone();
                (width.a, width.b, width.c, width.d) = (a, b, c, d);
                width.s_offset = Length::new::<meter>(s);
                LaneChoice::Width(width)
            }
            LaneChoice::Border(border) => {
                let mut border = border.clone();
                (border.a, border.b, border.c, border.d) = (a, b, c, d);
                border.s_offset = Length::new::<meter>(s);
                LaneChoice::Border(border)
            }
        },
        |choice| match choice {
            LaneChoice::Width(width) => (width.a, width.b, width.c, width.d),
            LaneChoice::Border(border) => (border.a, border.b, border.c, border.d),
        },
    )
    .1;
    lane.road_mark = rebase_steps(
        &lane.road_mark,
        by,
        |mark| at(mark.s_offset),
        |mark, s| {
            let mut mark = mark.clone();
            mark.s_offset = Length::new::<meter>(s);
            mark
        },
    );
    lane.speed = rebase_steps(
        &lane.speed,
        by,
        |speed| at(speed.s_offset),
        |speed, s| {
            let mut speed = speed.clone();
            speed.s_offset = Length::new::<meter>(s);
            speed
        },
    );
    // A lift runs straight from one entry to the next, so the one at the cut is
    // the two either side of it, interpolated.
    let heights = lane.height.clone();
    let mut rebased = Vec::new();
    let before = heights.iter().rfind(|height| at(height.s_offset) <= by);
    let after = heights.iter().find(|height| at(height.s_offset) > by);
    if let Some(before) = before {
        let mut first = before.clone();
        if let Some(after) = after {
            let span = at(after.s_offset) - at(before.s_offset);
            let t = if span > 0.0 {
                (by - at(before.s_offset)) / span
            } else {
                0.0
            };
            let blend = |a: Length, b: Length| Length::new::<meter>(at(a) + (at(b) - at(a)) * t);
            first.inner = blend(before.inner, after.inner);
            first.outer = blend(before.outer, after.outer);
        }
        first.s_offset = Length::new::<meter>(0.0);
        rebased.push(first);
    }
    for height in heights.iter().filter(|height| at(height.s_offset) > by) {
        let mut height = height.clone();
        height.s_offset = Length::new::<meter>(at(height.s_offset) - by);
        rebased.push(height);
    }
    lane.height = rebased;
}

/// Records valid from their station on, without those that start at or after
/// `end` — but never without the first.
fn kept_before<T: Clone>(records: &[T], end: f64, station: impl Fn(&T) -> f64) -> Vec<T> {
    records
        .iter()
        .enumerate()
        .filter(|(index, record)| *index == 0 || station(record) < end)
        .map(|(_, record)| record.clone())
        .collect()
}

/// A lane's records cut off at `end` along its section, so that nothing in the
/// first half's document lies past its end. A lift that runs on past the cut ends
/// at its value there.
fn truncate_lane(lane: &mut OdLane, end: f64) {
    let at = |length: Length| metres(length);
    lane.choice = kept_before(&lane.choice, end, |choice| match choice {
        LaneChoice::Width(width) => at(width.s_offset),
        LaneChoice::Border(border) => at(border.s_offset),
    });
    lane.road_mark = kept_before(&lane.road_mark, end, |mark| at(mark.s_offset));
    lane.speed = kept_before(&lane.speed, end, |speed| at(speed.s_offset));
    let heights = lane.height.clone();
    let mut kept = kept_before(&heights, end, |height| at(height.s_offset));
    let before = heights.iter().rfind(|height| at(height.s_offset) < end);
    let after = heights.iter().find(|height| at(height.s_offset) >= end);
    if let (Some(before), Some(after)) = (before, after) {
        let span = at(after.s_offset) - at(before.s_offset);
        let t = if span > 0.0 {
            (end - at(before.s_offset)) / span
        } else {
            1.0
        };
        let blend = |a: Length, b: Length| Length::new::<meter>(at(a) + (at(b) - at(a)) * t);
        let mut last = after.clone();
        last.inner = blend(before.inner, after.inner);
        last.outer = blend(before.outer, after.outer);
        last.s_offset = Length::new::<meter>(end);
        kept.push(last);
    }
    lane.height = kept;
}

/// Links every lane of `section` on to the lane of the same id beyond the cut
/// (`successor`) or before it (`predecessor`).
fn link_across(section: &mut LaneSection, successor: bool) {
    let ids = lane_ids(section);
    for (lane, id) in lanes_mut(section).zip(ids) {
        let link = lane.link.get_or_insert_with(|| LaneLink {
            predecessor: Vec::new(),
            successor: Vec::new(),
            additional_data: AdditionalData::default(),
        });
        let across = vec![LaneEnd { id }];
        if successor {
            link.successor = across;
        } else {
            link.predecessor = across;
        }
    }
}

/// Cuts `road` at `cut` into the road up to it and the road from it, `id`.
fn cut_road(road: &OdRoad, cut: f64, id: &str) -> Option<(OdRoad, OdRoad)> {
    let (first_geometry, second_geometry) = cut_plan_view(road.plan_view.geometry.as_slice(), cut)?;
    let mut first = road.clone();
    let mut second = road.clone();
    second.id = id.to_owned();
    let length = metres(road.length);
    first.length = Length::new::<meter>(cut);
    second.length = Length::new::<meter>(length - cut);
    first.plan_view.geometry = Vec1::try_from_vec(first_geometry).ok()?;
    second.plan_view.geometry = Vec1::try_from_vec(second_geometry).ok()?;

    // Joined road to road at the cut.
    let original = road.link.clone().unwrap_or(Link {
        predecessor: None,
        successor: None,
        additional_data: AdditionalData::default(),
    });
    first.link = Some(Link {
        predecessor: original.predecessor.clone(),
        successor: Some(PredecessorSuccessor {
            contact_point: Some(ContactPoint::Start),
            element_dir: None,
            element_id: id.to_owned(),
            element_s: None,
            element_type: Some(ElementType::Road),
        }),
        additional_data: original.additional_data.clone(),
    });
    second.link = Some(Link {
        predecessor: Some(PredecessorSuccessor {
            contact_point: Some(ContactPoint::End),
            element_dir: None,
            element_id: road.id.clone(),
            element_s: None,
            element_type: Some(ElementType::Road),
        }),
        successor: original.successor.clone(),
        additional_data: original.additional_data,
    });

    // The road's own profiles.
    if let Some(profile) = &road.elevation_profile {
        let (a, b) = rebase(
            &profile.elevation,
            cut,
            |e| e.s,
            |e, s, (a, b, c, d)| {
                let mut e = e.clone();
                (e.a, e.b, e.c, e.d, e.s) = (a, b, c, d, s);
                e
            },
            |e| (e.a, e.b, e.c, e.d),
        );
        first.elevation_profile.as_mut()?.elevation = a;
        second.elevation_profile.as_mut()?.elevation = b;
    }
    if let Some(profile) = &road.lateral_profile {
        let (a, b) = rebase(
            &profile.super_elevation,
            cut,
            |e| e.s,
            |e, s, (a, b, c, d)| {
                let mut e = e.clone();
                (e.a, e.b, e.c, e.d, e.s) = (a, b, c, d, s);
                e
            },
            |e| (e.a, e.b, e.c, e.d),
        );
        first.lateral_profile.as_mut()?.super_elevation = a;
        second.lateral_profile.as_mut()?.super_elevation = b;
    }
    let (a, b) = rebase(
        &road.lanes.lane_offset,
        cut,
        |o| o.s,
        |o, s, (a, b, c, d)| {
            let mut o = o.clone();
            (o.a, o.b, o.c, o.d, o.s) = (a, b, c, d, s);
            o
        },
        |o| (o.a, o.b, o.c, o.d),
    );
    first.lanes.lane_offset = a;
    second.lanes.lane_offset = b;
    first.r#type = road
        .r#type
        .iter()
        .filter(|kind| metres(kind.s) < cut)
        .cloned()
        .collect();
    second.r#type = rebase_steps(
        &road.r#type,
        cut,
        |kind| metres(kind.s),
        |kind, s| {
            let mut kind = kind.clone();
            kind.s = Length::new::<meter>(s);
            kind
        },
    );

    // Lane sections: the one in force at the cut goes on in the second road,
    // rebased, and its lanes link across.
    let sections = road.lanes.lane_section.as_slice();
    let current = sections.iter().rposition(|section| section.s <= cut)?;
    let mut before: Vec<LaneSection> = sections[..=current].to_vec();
    let mut after: Vec<LaneSection> = sections[current..].to_vec();
    let by = cut - sections[current].s;
    let ending = before.last_mut()?;
    for lane in lanes_mut(ending) {
        truncate_lane(lane, by);
    }
    for lane in ending.center.lane.iter_mut() {
        lane.base.road_mark = kept_before(&lane.base.road_mark, by, |mark| metres(mark.s_offset));
    }
    link_across(ending, true);
    let continuing = &mut after[0];
    continuing.s = 0.0;
    for lane in lanes_mut(continuing) {
        rebase_lane(lane, by);
    }
    rebase_centre(continuing, by);
    link_across(continuing, false);
    for section in after.iter_mut().skip(1) {
        section.s -= cut;
    }
    first.lanes.lane_section = Vec1::try_from_vec(before).ok()?;
    second.lanes.lane_section = Vec1::try_from_vec(after).ok()?;

    // Furniture goes with the half it stands on.
    if let Some(signals) = &road.signals {
        let (mut a, mut b) = (signals.clone(), signals.clone());
        a.signal.retain(|signal| metres(signal.s) < cut);
        b.signal.retain(|signal| metres(signal.s) >= cut);
        for signal in &mut b.signal {
            signal.s = Length::new::<meter>(metres(signal.s) - cut);
        }
        b.signal_reference.clear();
        first.signals = Some(a);
        second.signals = Some(b);
    }
    if let Some(objects) = &road.objects {
        let (mut a, mut b) = (objects.clone(), objects.clone());
        a.object.retain(|object| metres(object.s) < cut);
        b.object.retain(|object| metres(object.s) >= cut);
        for object in &mut b.object {
            object.s = Length::new::<meter>(metres(object.s) - cut);
        }
        b.object_reference.clear();
        b.tunnel.clear();
        b.bridge.clear();
        first.objects = Some(a);
        second.objects = Some(b);
    }
    Some((first, second))
}

fn rebase_centre(section: &mut LaneSection, by: f64) {
    for lane in section.center.lane.iter_mut() {
        lane.base.road_mark = rebase_steps(
            &lane.base.road_mark,
            by,
            |mark| metres(mark.s_offset),
            |mark, s| {
                let mut mark = mark.clone();
                mark.s_offset = Length::new::<meter>(s);
                mark
            },
        );
    }
}

/// Cuts every ordinary road whose two ends run into one junction, and points
/// every link to such a road's end at its second half. Returns how many were cut,
/// and how many could not be (a road that is one spiral).
pub(crate) fn cut_looping_roads(drive: &mut OpenDrive) -> (usize, usize) {
    let mut next = drive
        .road
        .iter()
        .map(|road| road.id.as_str())
        .chain(drive.junction.iter().map(|junction| junction.id.as_str()))
        .filter_map(|id| id.parse::<u64>().ok())
        .max()
        .map_or(0, |max| max + 1);
    let mut renamed: Vec<(String, String)> = Vec::new();
    let mut added = Vec::new();
    let mut skipped = 0;
    for road in drive.road.iter_mut() {
        if looping(road).is_none() {
            continue;
        }
        let id = next.to_string();
        let Some((first, second)) = cut_station(road).and_then(|cut| cut_road(road, cut, &id))
        else {
            skipped += 1;
            continue;
        };
        next += 1;
        renamed.push((road.id.clone(), id));
        *road = first;
        added.push(second);
    }
    // A link to a cut road's end is a link to its second half.
    let second_of = |id: &str| {
        renamed
            .iter()
            .find(|(road, _)| road == id)
            .map(|(_, second)| second.clone())
    };
    for road in drive.road.iter_mut() {
        let Some(link) = road.link.as_mut() else {
            continue;
        };
        for end in [link.predecessor.as_mut(), link.successor.as_mut()]
            .into_iter()
            .flatten()
        {
            if end.element_type == Some(ElementType::Road)
                && end.contact_point == Some(ContactPoint::End)
            {
                if let Some(second) = second_of(&end.element_id) {
                    end.element_id = second;
                }
            }
        }
    }
    let cut = renamed.len();
    drive.road.extend(added);
    // A connection from a cut road comes from whichever half the connecting road
    // now names.
    let names = |connecting: &str, half: &str| {
        drive
            .road
            .iter()
            .find(|road| road.id == connecting)
            .is_some_and(|road| {
                road.link.as_ref().is_some_and(|link| {
                    [&link.predecessor, &link.successor]
                        .into_iter()
                        .flatten()
                        .any(|end| end.element_id == half)
                })
            })
    };
    let mut moves = Vec::new();
    for (j, junction) in drive.junction.iter().enumerate() {
        for (c, connection) in junction.connection.iter().enumerate() {
            let (Some(incoming), Some(connecting)) =
                (&connection.incoming_road, &connection.connecting_road)
            else {
                continue;
            };
            if let Some(second) = second_of(incoming) {
                if names(connecting, &second) {
                    moves.push((j, c, second));
                }
            }
        }
    }
    for (j, c, second) in moves {
        drive.junction[j].connection[c].incoming_road = Some(second);
    }
    (cut, skipped)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cubic_halved_is_the_same_curve() {
        let poly = ParamPoly3 {
            a_u: 0.0,
            b_u: 9.0,
            c_u: 1.5,
            d_u: -0.5,
            a_v: 0.0,
            b_v: 0.5,
            c_v: 2.0,
            d_v: -0.7,
            p_range: ParamPoly3pRange::Normalized,
        };
        let point = |p: &ParamPoly3, t: f64| {
            (
                p.a_u + p.b_u * t + p.c_u * t * t + p.d_u * t * t * t,
                p.a_v + p.b_v * t + p.c_v * t * t + p.d_v * t * t * t,
            )
        };
        let (first, second, first_length, second_length, (u0, v0), turn) = halve(&poly);
        let (sin, cos) = turn.sin_cos();
        for step in 0..=10 {
            let q = step as f64 / 10.0;
            let (u, v) = point(&poly, q / 2.0);
            let (fu, fv) = point(&first, q);
            assert!((u - fu).abs() < 1e-12 && (v - fv).abs() < 1e-12);
            // The second half, turned and moved back into the whole's frame.
            let (su, sv) = point(&second, q);
            let (bu, bv) = (u0 + su * cos - sv * sin, v0 + su * sin + sv * cos);
            let (u, v) = point(&poly, 0.5 + q / 2.0);
            assert!((u - bu).abs() < 1e-12 && (v - bv).abs() < 1e-12);
        }
        // The second half starts heading along its own u axis.
        assert!(second.b_v.abs() < 1e-12);
        assert!(first_length > 0.0 && second_length > 0.0);
    }
}
