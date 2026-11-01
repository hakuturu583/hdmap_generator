//! Which of the IR's signs CommonRoad can hold, and as what.
//!
//! The IR keeps a sign's code in whatever catalogue the caller used. A CommonRoad
//! scenario written here is set in Zamunda — the fictional country CommonRoad keeps
//! for scenarios that are nowhere real — whose signs are Germany's, so a code is
//! written when it is one of those: a stop sign however it is spelt, as `206`, and
//! any other German sign by its own number, with or without a `DE` prefix. A speed
//! limit written as `274-50` is `274` with its limit as the additional value, in
//! metres per second, as CommonRoad keeps it. Anything else is dropped, and
//! [`crate::check`] says how many.

use roadgen_core::semantics::is_stop_sign;

use crate::model::{number, TrafficSignElement};

/// Maximum speed.
pub const MAX_SPEED: &str = "274";
/// Give way.
pub const YIELD: &str = "205";
/// Stop.
pub const STOP: &str = "206";

/// The sign ids both CommonRoad's 2020a schema and its reader's Zamunda catalogue
/// know.
const CATALOGUE: &[&str] = &[
    "101", "102", "103-10", "103-20", "108", "114", "123", "124", "125", "131", "133-10", "133-20",
    "138", "142-10", "145-50", "201", "205", "206", "208", "209-10", "209-20", "211-20", "215",
    "220-10", "220-20", "222-10", "222-20", "223.2", "223.2-50", "223.2-51", "224-50", "237",
    "239", "240", "242.1", "242.2", "244.1", "244.2", "245", "250", "251", "253", "254", "255",
    "257-54", "259", "260", "261", "262", "264", "265", "266", "267", "270.1", "270.2", "272",
    "274", "274.1", "274.2", "275", "276", "277", "278", "280", "281", "282", "283-10", "283-30",
    "286-30", "301", "306", "308", "310", "311", "314", "314-10", "314-20", "314-30", "325.1",
    "325.2", "327", "328", "330.1", "330.2", "331.1", "331.2", "332", "332.1", "333", "333-21",
    "333-22", "350", "354", "356", "357", "363", "365-51", "365-52", "365-60", "386.1", "386.2",
    "386.3", "406-50", "418-20", "419-20", "434-50", "430-20", "432-10", "432-20", "434", "438",
    "439", "440", "448", "449", "450-50", "450-51", "450-52", "450-53", "450-54", "450-55", "453",
    "458", "455.1-30", "460-10", "460-12", "460-20", "460-21", "460-22", "460-30", "501-15",
    "501-16", "511-22", "521-30", "521-31", "521-32", "521-33", "531-10", "531-20", "531-21",
    "533-22", "550-20", "600-35", "600-30", "600-31", "600-32", "600-34", "600-38", "605-10",
    "605-11", "605-20", "605-21", "605-31", "625-10", "625-11", "625-12", "625-13", "625-20",
    "625-21", "625-22", "625-23", "626-10", "626-20", "626-30", "626-31", "628-10", "629-10",
    "629-20", "720", "1000", "1000-10", "1000-11", "1000-20", "1000-21", "1000-30", "1000-31",
    "1001-30", "1001-31", "1002-10", "1002-11", "1002-12", "1002-13", "1002-14", "1002-20",
    "1002-21", "1002-22", "1002-23", "1002-24", "1004-30", "1004-31", "1004-32", "1004-33",
    "1004-34", "1004-35", "1006-30", "1006-31", "1006-32", "1006-33", "1006-34", "1006-35",
    "1006-36", "1006-37", "1006-38", "1006-39", "1007-31", "1010-10", "1010-11", "1010-12",
    "1010-13", "1010-14", "1012-30", "1012-31", "1012-32", "1012-33", "1012-34", "1012-35",
    "1012-36", "1012-37", "1012-38", "1020-30", "1022-10", "1024-10", "1026-36", "1026-37",
    "1026-38", "1031-52", "1040-30", "1048-12", "1049-13", "1052-31", "1053-33", "1053-34",
    "1053-35", "2113",
];

/// The sign element a code is written as, or `None` when CommonRoad has no such
/// sign.
pub(crate) fn element(code: &str) -> Option<TrafficSignElement> {
    if is_stop_sign(code) || is_stop_sign(strip(code)) {
        return Some(TrafficSignElement {
            sign_id: STOP,
            additional_values: Vec::new(),
        });
    }
    let code = strip(code);
    if let Some(kph) = code
        .strip_prefix("274-")
        .and_then(|kph| kph.parse::<f64>().ok())
        .filter(|kph| kph.is_finite() && *kph > 0.0)
    {
        return Some(TrafficSignElement {
            sign_id: MAX_SPEED,
            additional_values: vec![number(kph / 3.6)],
        });
    }
    // A bare maximum-speed sign says no speed, which is no rule at all.
    if code == MAX_SPEED {
        return None;
    }
    CATALOGUE
        .iter()
        .find(|known| **known == code)
        .map(|known| TrafficSignElement {
            sign_id: known,
            additional_values: Vec::new(),
        })
}

/// A code without the catalogue's country prefix or a subtype after a slash.
fn strip(code: &str) -> &str {
    let code = code.split('/').next().unwrap_or_default().trim();
    code.strip_prefix("DE")
        .or_else(|| code.strip_prefix("de"))
        .map(|rest| rest.trim_start_matches(['_', '-', ' ', '.']))
        .unwrap_or(code)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stop_sign_is_206_however_it_is_spelt() {
        for code in ["stop", "STOP_SIGN", "R1-1", "206", "DE206", "de_206"] {
            assert_eq!(element(code).unwrap().sign_id, STOP, "{code}");
        }
    }

    #[test]
    fn a_speed_limit_carries_its_limit_in_metres_per_second() {
        let element = element("274-36").unwrap();
        assert_eq!(element.sign_id, MAX_SPEED);
        assert_eq!(element.additional_values, vec!["10.0".to_owned()]);
        assert!(super::element("274").is_none());
    }

    #[test]
    fn german_signs_pass_and_others_do_not() {
        assert_eq!(element("DE205").unwrap().sign_id, "205");
        assert_eq!(element("306").unwrap().sign_id, "306");
        assert!(element("R2-1").is_none());
        assert!(element("no-such-sign").is_none());
    }
}
