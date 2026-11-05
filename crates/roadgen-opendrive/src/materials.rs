//! What a lane's `<material>` says about a pavement.
//!
//! OpenDRIVE's lane material carries a friction coefficient that it requires and a
//! roughness it does not, beside a free-form surface code. The IR has neither number:
//! they are properties of an encoding that wants them, not of the road, and a road
//! described as gravel is fully described. So they are nominal values for the surface
//! type, written here and nowhere else.

use roadgen_core::semantics::{PavedSurface, Pavement, UnpavedSurface};

/// `(friction, roughness)` for a surface: a dry tyre-on-surface friction
/// coefficient, and a roughness in metres of a representative texture amplitude.
pub fn nominal(pavement: Pavement) -> (f64, f64) {
    match pavement {
        Pavement::Paved(PavedSurface::Asphalt) => (0.8, 0.003),
        Pavement::Paved(PavedSurface::Concrete) => (0.8, 0.002),
        Pavement::Paved(PavedSurface::Cobblestone) => (0.65, 0.02),
        Pavement::Unpaved(UnpavedSurface::Gravel) => (0.6, 0.05),
        Pavement::Unpaved(UnpavedSurface::Dirt) => (0.55, 0.03),
        Pavement::Unpaved(UnpavedSurface::Sand) => (0.45, 0.02),
        Pavement::Unpaved(UnpavedSurface::Rock) => (0.7, 0.08),
    }
}
