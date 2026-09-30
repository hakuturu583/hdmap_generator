//! A traffic light's lamps, which OpenDRIVE has no element for.
//!
//! A `<signal>` says which light it is by a catalogue code and how big its housing
//! is by `width` and `height`, but not where its lamps are or what each shows. So
//! each lamp is a `<userData>` of the signal: code [`BULB_CODE`], and a value
//! naming its colour, its arrow when it has one, and its position in the
//! document's inertial frame —
//! `color=green;arrow=right;x=12.345;y=-6.789;z=46.382`. A reader that does not
//! know them passes over them, as `<userData>` is meant to be.

use std::fmt::Write;

use opendrive::core::user_data::UserData;

use roadgen_core::geometry::Point3;
use roadgen_core::semantics::{LightArrow, LightBulb, LightColor};

/// The `<userData>` code a lamp is written under.
pub const BULB_CODE: &str = "lightBulb";

/// One `<userData>` per lamp, in order.
pub(crate) fn to_user_data(bulbs: &[LightBulb]) -> Vec<UserData> {
    bulbs
        .iter()
        .map(|bulb| {
            let mut value = format!("color={}", bulb.color.as_str());
            if let Some(arrow) = bulb.arrow {
                let _ = write!(value, ";arrow={}", arrow.as_str());
            }
            let p = bulb.position;
            let _ = write!(value, ";x={:.6};y={:.6};z={:.6}", p.x, p.y, p.z);
            UserData {
                code: BULB_CODE.to_owned(),
                value: Some(value),
                elements: Vec::new(),
            }
        })
        .collect()
}

/// The lamps a signal's `<userData>` describe, and how many it names that cannot
/// be read.
pub(crate) fn from_user_data(user_data: &[UserData]) -> (Vec<LightBulb>, usize) {
    let mut bulbs = Vec::new();
    let mut unreadable = 0;
    for entry in user_data.iter().filter(|entry| entry.code == BULB_CODE) {
        match entry.value.as_deref().and_then(parse) {
            Some(bulb) => bulbs.push(bulb),
            None => unreadable += 1,
        }
    }
    (bulbs, unreadable)
}

fn parse(value: &str) -> Option<LightBulb> {
    let (mut color, mut arrow, mut x, mut y, mut z) = (None, None, None, None, None);
    for pair in value.split(';') {
        let (key, value) = pair.split_once('=')?;
        let value = value.trim();
        match key.trim() {
            "color" => color = Some(LightColor::parse(value)?),
            "arrow" => arrow = Some(LightArrow::parse(value)?),
            "x" => x = value.parse::<f64>().ok(),
            "y" => y = value.parse::<f64>().ok(),
            "z" => z = value.parse::<f64>().ok(),
            _ => {}
        }
    }
    // `f64` parses `inf` and `NaN` too, and a lamp there is nowhere.
    let position = Point3::new(x?, y?, z?);
    if !position.is_finite() {
        return None;
    }
    Some(LightBulb {
        position,
        color: color?,
        arrow,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_lamp_reads_back_as_written() {
        let bulbs = vec![
            LightBulb {
                position: Point3::new(1.5, -2.25, 40.125),
                color: LightColor::Red,
                arrow: None,
            },
            LightBulb {
                position: Point3::new(1.75, -2.25, 40.125),
                color: LightColor::Green,
                arrow: Some(LightArrow::UpRight),
            },
        ];
        assert_eq!(from_user_data(&to_user_data(&bulbs)), (bulbs, 0));
    }

    #[test]
    fn a_lamp_that_is_nowhere_is_not_read() {
        let entry = |value: &str| UserData {
            code: BULB_CODE.to_owned(),
            value: Some(value.to_owned()),
            elements: Vec::new(),
        };
        let (bulbs, unreadable) = from_user_data(&[
            entry("color=red;x=inf;y=0;z=5"),
            entry("color=amber;x=1;y=0;z=NaN"),
        ]);
        assert!(bulbs.is_empty());
        assert_eq!(unreadable, 2);
    }
}
