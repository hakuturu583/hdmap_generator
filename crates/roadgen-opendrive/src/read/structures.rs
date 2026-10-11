//! Bridges and tunnels.
//!
//! A `<bridge>` or `<tunnel>` is a stretch of one road, `s` to `s + length`, and a
//! type: what a bridge is made of, whether a tunnel is an underpass. That is the
//! whole of what the document says, and it is enough to put the structure back:
//! [`roadgen_core::structure::place`] lays out a deck or a bore along the stretch
//! from the road's own lanes — its outline, a bridge's abutments and piers, a
//! tunnel's portals — which is how the builder made it in the first place. The
//! outline that comes back is the one the road implies; the piers are where the
//! layout puts them, not where the document's author did, since the document never
//! said. That is noted once for every map that has a bridge.
//!
//! A structure that carries several roads is written once on each, with its name on
//! every element. Elements of one kind that share a name are read as one structure
//! carrying all their stretches, which is what the writer meant by giving them the
//! same name; an element with no name is a structure of its own, so an unnamed deck
//! over both carriageways comes back as two bridges, and that is noted.
//!
//! The finer form — viaduct, overpass, cut and cover — is read from the
//! `<userData>` the writer puts it in ([`crate::STRUCTURE_FORM_CODE`]). A document
//! without one gets a plain bridge, or a bored tunnel for a `standard` one. The
//! depth of a deck and the clear height of a bore are not in the file at all; they
//! are the defaults of [`BridgeSpec`] and [`TunnelSpec`].
//!
//! Once every structure is placed the map's grade separations are derived from
//! them, as the builder derives them.

use opendrive::core::additional_data::AdditionalData;
use opendrive::object::bridge_type::BridgeType;
use opendrive::object::tunnel_type::TunnelType;
use uom::si::length::meter;

use roadgen_core::id::RoadId;
use roadgen_core::structure::{
    self, BridgeForm, BridgeSpec, RoadSpan, StructureMaterial, StructureSpec, TunnelForm,
    TunnelSpec,
};

use super::Reader;
use crate::error::ImportError;
use crate::STRUCTURE_FORM_CODE;

/// One structure to place: the stretches it carries and what it is.
struct Pending {
    spans: Vec<RoadSpan>,
    spec: StructureSpec,
}

impl Pending {
    fn name(&self) -> Option<&str> {
        self.spec.name()
    }

    fn is_bridge(&self) -> bool {
        matches!(self.spec, StructureSpec::Bridge(_))
    }
}

/// The form a `<userData>` names, if one does.
fn form_of(data: &AdditionalData) -> Option<&str> {
    data.user_data
        .iter()
        .find(|entry| entry.code == STRUCTURE_FORM_CODE)
        .and_then(|entry| entry.value.as_deref())
}

/// The name an element carries, when it carries one worth keeping.
fn name_of(name: &Option<String>) -> Option<String> {
    name.as_deref()
        .map(str::trim)
        .filter(|name| !name.is_empty())
        .map(str::to_owned)
}

/// Reads every road's `<bridge>`s and `<tunnel>`s, places them on the map, and
/// derives the grade separations they make.
pub fn read(reader: &mut Reader<'_>) -> Result<(), ImportError> {
    let document = reader.document;
    let mut pending: Vec<Pending> = Vec::new();
    let mut unnamed_bridges = 0;
    let mut bare_tunnels = 0;

    for road in &document.road {
        let Some(objects) = &road.objects else {
            continue;
        };
        let id = RoadId::new(&road.id);
        let span = |s: f64, length: f64| RoadSpan::new(id.clone(), s, s + length);
        for bridge in &objects.bridge {
            let form = form_of(&bridge.additional_data);
            let form = form
                .and_then(BridgeForm::parse)
                .unwrap_or(BridgeForm::Bridge);
            let spec = BridgeSpec {
                name: name_of(&bridge.name),
                material: match bridge.r#type {
                    BridgeType::Concrete => StructureMaterial::Concrete,
                    BridgeType::Steel => StructureMaterial::Steel,
                    BridgeType::Brick => StructureMaterial::Masonry,
                    BridgeType::Wood => StructureMaterial::Timber,
                },
                // An overpass's deck is a shallower one than a bridge's.
                ..BridgeSpec::for_form(form)
            };
            if spec.name.is_none() {
                unnamed_bridges += 1;
            }
            merge(
                &mut pending,
                span(bridge.s.get::<meter>(), bridge.length.get::<meter>()),
                StructureSpec::Bridge(spec),
            );
        }
        for tunnel in &objects.tunnel {
            let form = form_of(&tunnel.additional_data).and_then(TunnelForm::parse);
            if form.is_none() && tunnel.r#type == TunnelType::Standard {
                bare_tunnels += 1;
            }
            let spec = TunnelSpec {
                name: name_of(&tunnel.name),
                form: form.unwrap_or(match tunnel.r#type {
                    TunnelType::Underpass => TunnelForm::Underpass,
                    TunnelType::Standard => TunnelForm::Bored,
                }),
                lighting: tunnel
                    .lighting
                    .map_or(TunnelSpec::default().lighting, |lit| lit > 0.5),
                ..TunnelSpec::default()
            };
            merge(
                &mut pending,
                span(tunnel.s.get::<meter>(), tunnel.length.get::<meter>()),
                StructureSpec::Tunnel(spec),
            );
        }
    }

    if pending.is_empty() {
        return Ok(());
    }
    if pending.iter().any(Pending::is_bridge) {
        reader.approximations.note(
            "a <bridge> is a stretch of road and a type, so each bridge's deck outline, \
             abutments and piers are laid out again from its road rather than read",
        );
    }
    if unnamed_bridges > 1 {
        reader.approximations.note(format!(
            "{unnamed_bridges} bridges have no name to tell which stretches one deck \
             carries, so a deck over several roads comes back as one bridge on each"
        ));
    }
    if bare_tunnels > 0 {
        reader.approximations.note(format!(
            "{bare_tunnels} standard tunnels do not say how they were built, and are read \
             as bored rather than cut and cover"
        ));
    }

    for structure in pending {
        if let Err(error) = structure::place(&mut reader.map, &structure.spans, &structure.spec) {
            let what = if structure.is_bridge() {
                "bridge"
            } else {
                "tunnel"
            };
            let spans: Vec<String> = structure
                .spans
                .iter()
                .map(|span| format!("{} {:.1}–{:.1}", span.road, span.from, span.to))
                .collect();
            reader.approximations.note(format!(
                "the {what} over {} cannot be placed ({error}) and is left out",
                spans.join(", ")
            ));
        }
    }
    structure::derive_separations(&mut reader.map);
    Ok(())
}

/// Adds a stretch to the structure of the same kind and name already met, or starts
/// a structure of its own when it has no name or none matches.
fn merge(pending: &mut Vec<Pending>, span: RoadSpan, spec: StructureSpec) {
    let incoming = Pending {
        spans: vec![span],
        spec,
    };
    if let Some(name) = incoming.name() {
        if let Some(existing) = pending.iter_mut().find(|other| {
            other.name() == Some(name)
                && other.is_bridge() == incoming.is_bridge()
                && other
                    .spans
                    .iter()
                    .all(|held| held.road != incoming.spans[0].road)
        }) {
            existing.spans.extend(incoming.spans);
            return;
        }
    }
    pending.push(incoming);
}
