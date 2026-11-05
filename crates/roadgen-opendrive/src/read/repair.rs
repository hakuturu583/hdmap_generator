//! What older writers leave out, put back before the document is parsed.
//!
//! The parser reads OpenDRIVE 1.7 to the letter, and a document from an earlier
//! revision, or from a tool that read the standard more loosely, can be a perfectly
//! good road network the parser refuses over one attribute. CARLA's exported maps
//! (RoadRunner-authored, OpenDRIVE 1.4) are the case in point: a `<userData>` with
//! elements inside and no `code`, a `<roadMark>` with no `color`, an outline
//! `<cornerLocal>` with no `height`, and an `<object>` whose `type` is `-1`.
//!
//! Each of these has an obvious reading — the standard's own default, or "no
//! type" — so rather than fail, the document is rewritten with it before the
//! parser sees it, and every rewrite that changes what the map says is reported.
//! Nothing else in the document is touched: every other event is copied through
//! byte for byte.

use std::str::FromStr;

use opendrive::object::orientation::ObjectType;
use quick_xml::events::{BytesStart, Event};
use quick_xml::{Reader, Writer};

use super::Approximations;
use crate::error::ImportError;

/// The document with what it leaves out filled in, or the document itself when it
/// leaves nothing out.
pub fn repair(xml: &str, approximations: &mut Approximations) -> Result<String, ImportError> {
    let mut reader = Reader::from_str(xml);
    let mut writer = Writer::new(Vec::with_capacity(xml.len()));
    let mut changed = false;
    loop {
        let event = reader
            .read_event()
            .map_err(|error| ImportError::Parse(error.to_string()))?;
        let event = match event {
            Event::Eof => break,
            Event::Start(element) => match repaired(&element, approximations) {
                Some(fixed) => {
                    changed = true;
                    Event::Start(fixed)
                }
                None => Event::Start(element),
            },
            Event::Empty(element) => match repaired(&element, approximations) {
                Some(fixed) => {
                    changed = true;
                    Event::Empty(fixed)
                }
                None => Event::Empty(element),
            },
            other => other,
        };
        writer
            .write_event(event)
            .map_err(|error| ImportError::Parse(error.to_string()))?;
    }
    if !changed {
        return Ok(xml.to_owned());
    }
    String::from_utf8(writer.into_inner())
        .map_err(|error| ImportError::Parse(format!("the repaired document: {error}")))
}

/// The element with what it leaves out filled in, if it leaves anything out.
fn repaired(
    element: &BytesStart<'_>,
    approximations: &mut Approximations,
) -> Option<BytesStart<'static>> {
    let attribute = |name: &[u8]| {
        element
            .attributes()
            .flatten()
            .find(|attribute| attribute.key.as_ref() == name)
            .map(|attribute| String::from_utf8_lossy(&attribute.value).into_owned())
    };
    match element.name().as_ref() {
        // Ancillary data is free-form; the reader looks for a few codes of its own
        // and passes over the rest, so one with no code is simply one of the rest.
        b"userData" if attribute(b"code").is_none() => Some(with(element, "code", "")),
        b"roadMark" if attribute(b"color").is_none() => {
            approximations.count(
                "{n} `<roadMark>`s give no `color` and are read as `standard`, the colour \
                 OpenDRIVE means by it (white)",
            );
            Some(with(element, "color", "standard"))
        }
        b"cornerLocal" if attribute(b"height").is_none() => {
            approximations.count(
                "{n} outline `<cornerLocal>`s give no `height` and are read as flat on the road",
            );
            Some(with(element, "height", "0"))
        }
        b"object" => {
            let kind = attribute(b"type")?;
            if ObjectType::from_str(&kind).is_ok() {
                return None;
            }
            approximations.count(format!(
                "{{n}} `<object>`s have a `type` OpenDRIVE does not define ({kind:?}) and are \
                 read as `none`"
            ));
            Some(replacing(element, "type", "none"))
        }
        _ => None,
    }
}

/// The element with one attribute more.
fn with(element: &BytesStart<'_>, name: &str, value: &str) -> BytesStart<'static> {
    let mut fixed = element.to_owned();
    fixed.push_attribute((name, value));
    fixed
}

/// The element with one attribute's value replaced.
fn replacing(element: &BytesStart<'_>, name: &str, value: &str) -> BytesStart<'static> {
    let mut fixed = BytesStart::new(String::from_utf8_lossy(element.name().as_ref()).into_owned());
    for attribute in element.attributes().flatten() {
        if attribute.key.as_ref() == name.as_bytes() {
            fixed.push_attribute((name, value));
        } else {
            fixed.push_attribute(attribute);
        }
    }
    fixed
}
